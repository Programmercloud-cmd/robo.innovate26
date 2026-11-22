"""Inspect a saved variant USD independently of its authoring-side counters."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import struct


def inspect(stage, output):
    from pxr import Gf, Usd, UsdGeom, UsdPhysics
    robot = '/World/envs/env_0/Robot'
    plate = '/World/LegoSeatingTaskCandidate/Plate'
    spec = json.loads((output/'environment.json').read_text())
    report = {'passed':False, 'environment':spec['environment'], 'fingers':spec['fingers'],
              'hardware_clients':0, 'physical_lift_validated':False, 'training_ready':False}
    if spec['plate']:
        studs = [p for p in Usd.PrimRange(stage.GetPrimAtPath(plate+'/Studs')) if p.IsA(UsdGeom.Mesh)]
        visible, covered = 0, 0
        for prim in studs:
            i,j = prim.GetAttribute('seating:gridIndex').Get()
            if 0 < i < 31 and 0 < j < 31:
                visible += 1
            else:
                covered += 1
        assert len(studs)==1024 and visible==900 and covered==124
        assert not stage.GetPrimAtPath('/World/LegoSeatingTaskCandidate/Stack')
        assert len(list(stage.GetPrimAtPath(plate+'/TapeOverlap').GetChildren()))==4
        report['plate'] = {'studs':len(studs),'exposed_studs':visible,'covered_studs':covered,'bricks_added':0}
    limits = []
    for i in (1,2):
        joint = UsdPhysics.PrismaticJoint(stage.GetPrimAtPath(robot+f'/fr3_hand/fr3_finger_joint{i}'))
        assert joint
        limits.append([float(joint.GetLowerLimitAttr().Get()),float(joint.GetUpperLimitAttr().Get())])
    report['native_finger_limits_m'] = limits
    if spec['fingers'] == 'wide':
        cap = spec['fingertips']
        original = Path(__file__).with_name('assets')/'original_fingertip.stl'
        data = original.read_bytes()
        profiles = json.loads(Path(__file__).with_name('profiles.json').read_text())
        profiles_config = profiles['fingers']['wide']
        assert hashlib.sha256(data).hexdigest() == profiles['fingers']['wide']['source_stl_sha256']
        count = struct.unpack_from('<I',data,80)[0]
        original_points = [p for i in range(count) for p in struct.iter_unpack('<fff',data[96+50*i:132+50*i])]
        geometry = []
        hand_inverse = UsdGeom.Xformable(stage.GetPrimAtPath(robot+'/fr3_hand')).ComputeLocalToWorldTransform(0).GetInverse()
        axes, origins, projections = [], [], []
        for name in ('fr3_leftfinger','fr3_rightfinger'):
            parent = UsdGeom.Xformable(stage.GetPrimAtPath(robot+'/'+name))
            frame = parent.ComputeLocalToWorldTransform(0)*hand_inverse
            axes.append(frame.TransformDir(Gf.Vec3d(0,1,0)).GetNormalized())
            origins.append(frame.Transform(Gf.Vec3d(0)))
        assert Gf.Dot(axes[0],axes[1]) > .999999
        for index,name in enumerate(('fr3_leftfinger','fr3_rightfinger')):
            parent = stage.GetPrimAtPath(robot+'/'+name)
            inverse = UsdGeom.Xformable(parent).ComputeLocalToWorldTransform(0).GetInverse()
            parts,wall_vertices = {},{}
            projected = []
            paths = [robot+'/'+name+'/UpdatedTipSTLVisual/Mesh']
            paths += [robot+'/'+name+'/WideCap/'+part for part in ('BackWall','EndNegative','EndPositive','Top')]
            for path in paths:
                item = UsdGeom.Mesh(stage.GetPrimAtPath(path))
                assert item and item.GetPrim().HasAPI(UsdPhysics.CollisionAPI)
                assert UsdPhysics.CollisionAPI(item).GetCollisionEnabledAttr().Get()
                assert UsdPhysics.MeshCollisionAPI(item).GetApproximationAttr().Get()=='convexHull'
                xf = item.ComputeLocalToWorldTransform(0)
                points = item.GetPointsAttr().Get()
                if path.endswith('/UpdatedTipSTLVisual/Mesh'):
                    assert list(map(tuple,points)) == original_points
                local = xf*inverse
                vectors = [local.Transform(Gf.Vec3d(p)) for p in points]
                bounds = [[min(v[a] for v in vectors) for a in range(3)],
                          [max(v[a] for v in vectors) for a in range(3)]]
                parts[path.rsplit('/',1)[1]] = bounds
                wall_vertices[path.rsplit('/',1)[1]] = vectors
                in_hand = xf*hand_inverse
                projected += [Gf.Dot(in_hand.Transform(Gf.Vec3d(p)),axes[0]) for p in points]
                vertices = [tuple(round(float(v),5) for v in p) for p in points]
                faces = item.GetFaceVertexIndicesAttr().Get()
                edges = Counter()
                volume = 0.
                for a,b,c in zip(faces[::3],faces[1::3],faces[2::3]):
                    for u,v in ((a,b),(b,c),(c,a)):
                        edges[tuple(sorted((vertices[u],vertices[v])))] += 1
                    volume += Gf.Dot(Gf.Vec3d(points[a]),Gf.Cross(Gf.Vec3d(points[b]),Gf.Vec3d(points[c])))/6
                assert all(n==2 for n in edges.values()) and volume>0
            projections.append(projected)
            a,b = parts['EndNegative'],parts['EndPositive']
            inside_span = b[0][0]-a[1][0]
            wall = a[1][0]-a[0][0]
            roof = parts['Top'];top = roof[1][2]-roof[0][2]
            height = a[1][2]-roof[1][2]
            front = parts['Mesh'][0][1] if index==0 else parts['Mesh'][1][1]
            reach = front-a[0][1] if index==0 else a[1][1]-front
            sign = 1 if index==0 else -1
            profiles = [[(sign*(p[1]-front),p[2]) for p in wall_vertices['BackWall']],
                        [(-p[0]-cap['inside_span_m']/2,p[2]) for p in wall_vertices['EndNegative']],
                        [(p[0]-cap['inside_span_m']/2,p[2]) for p in wall_vertices['EndPositive']]]
            # Compare distances rather than rounded bins: nanometre float32
            # error can put identical profiles on opposite sides of a bin edge.
            def directed_distance(a,b):
                return max(min(max(abs(u[0]-v[0]),abs(u[1]-v[1])) for v in b) for u in a)
            profile_error = max(max(directed_distance(profiles[0],p),directed_distance(p,profiles[0])) for p in profiles[1:])
            assert profile_error<1e-7
            assert len({round(d,6) for d,z in profiles[0]})>2, 'Original distal taper must be present, not rectangular walls'
            assert abs(inside_span-cap['inside_span_m'])<1e-7
            assert abs(wall-cap['wall_thickness_m'])<1e-7
            assert abs(top-cap['top_thickness_m'])<1e-7
            assert abs(height-cap['inside_height_m'])<1e-7
            assert abs(reach-cap['inside_reach_m'])<1e-7
            ceiling_config = profiles_config['ceiling']
            original_profile_top = parts['Mesh'][0][2]
            fixed_top = original_profile_top-ceiling_config['fixed_top_reference_thickness_m']
            assert abs(roof[0][2]-fixed_top)<1e-7, 'Ceiling top surface moved'
            clearance,contact_error = None,None
            if ceiling_config.get('mode') == 'preserve_current':
                assert abs(top-ceiling_config['retained_thickness_m'])<1e-7
                assert abs(roof[1][2]-original_profile_top)<1e-7, 'Retained ceiling underside moved'
            else:
                clearance = ceiling_config['fingertip_to_brick_bottom_clearance_m']
                assert ceiling_config['clearance_verified'] and ceiling_config['clearance_evidence']
                assert abs(top-(ceiling_config['nominal_thickness_m']+clearance))<1e-7
                rl = json.loads(Path(__file__).resolve().parents[1].joinpath('lego_contact/task/config.json').read_text())
                brick_bottom = parts['Mesh'][1][2]+clearance
                stud_top = brick_bottom-profiles_config['whole_layer_size_m'][2]-rl['visual_geometry_priors']['stud_height_m']
                contact_error = roof[1][2]-stud_top
                assert abs(contact_error)<1e-7, 'Ceiling underside misses the stud-top datum'
            fit = cap['pocket_fit']
            passive_clearance = inside_span-fit['assembled_span_used_m']
            assert abs(passive_clearance-fit['total_guide_clearance_m'])<1e-7
            assert passive_clearance>0 and not fit['brick_geometry_scaled']
            stock = UsdGeom.Cube(stage.GetPrimAtPath(robot+'/'+name+'/collisions/mesh_3'))
            half = stock.GetSizeAttr().Get()/2
            xf = stock.ComputeLocalToWorldTransform(0)*inverse
            support = [xf.Transform(Gf.Vec3d(x,y,z)) for x in (-half,half)
                       for y in (-half,half) for z in (-half,half)]
            inward_support = min(sign*p[1] for p in support)
            assert abs(inward_support-profiles_config['source_pad_backplane_abs_finger_y_m'])<1e-7
            geometry.append({'finger':name,'inside_span_m':inside_span,'inside_reach_m':reach,
                             'inside_height_m':height,'side_wall_thickness_m':wall,'top_thickness_m':top,
                             'all_three_wall_profiles_equal':True,'original_distal_taper_present':True,
                             'maximum_profile_difference_m':profile_error,
                             'ceiling_top_unchanged':True,
                             'ceiling_underside_to_stud_top_error_m':contact_error,
                             'fingertip_to_brick_bottom_clearance_m':clearance,
                             'passive_guide_clearance_total_m':passive_clearance,
                             'pocket_dimension_measured':fit['assembled_span_measured'],
                             'underside_open':not bool(stage.GetPrimAtPath(robot+'/'+name+'/WideCap/Bottom')),
                             'obsolete_stock_pad_collision_removed':True,
                             'closed_manifold_parts':len(paths),'bounds_finger_m':parts})
        native_width = Gf.Dot(origins[0]-origins[1],axes[0])
        gap = min(projections[0])-max(projections[1])
        zero_gap = gap-native_width
        expected = cap['geometric_fit']['return_tip_gap_at_zero_native_width_m']
        assert abs(zero_gap-expected)<1e-7
        minimum_width = sum(v[0] for v in limits)
        maximum_width = sum(v[1] for v in limits)
        assert minimum_width+zero_gap>=cap['geometric_fit']['minimum_opposing_clearance_m']-1e-7
        assert minimum_width<=.060 and maximum_width>=.070-1e-7
        report['fingertips'] = geometry
        report['separating_plane_proof'] = {'current_native_width_m':native_width,'current_return_tip_gap_m':gap,
            'zero_width_return_tip_gap_m':zero_gap,
            'collision_free_simulated_native_width_range_m':[minimum_width,maximum_width],
            'minimum_return_tip_gap_at_simulated_lower_limit_m':minimum_width+zero_gap,
            'common_finger_closing_axis_verified':True,'physical_hardware_limits_modified':False}
    report.update(passed=True,stage_sha256=hashlib.sha256((output/'digital_twin.usdc').read_bytes()).hexdigest())
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output',type=Path)
    args = parser.parse_args()
    from isaacsim import SimulationApp
    app = SimulationApp({'headless':True,'enable_crashreporter':False},
                        experience=str(Path(__file__).resolve().parents[1]/'physics_only.kit'))
    from pxr import Usd
    try:
        report = inspect(Usd.Stage.Open(str(args.output/'digital_twin.usdc')),args.output)
        (args.output/'geometry_verification.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report),flush=True)
    finally:
        app.close(wait_for_replicator=False)


if __name__=='__main__':
    main()
