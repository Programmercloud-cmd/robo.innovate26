"""Independent scene additions, reusing the RL plate and supplied fingertip CAD."""
import copy
import hashlib
import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROBOT = '/World/envs/env_0/Robot'


def mesh(stage, path, points, counts, indices, color):
    from pxr import Gf, UsdGeom
    value = UsdGeom.Mesh.Define(stage, path)
    value.CreatePointsAttr([Gf.Vec3f(*p) for p in points])
    value.CreateFaceVertexCountsAttr(counts)
    value.CreateFaceVertexIndicesAttr(indices)
    value.CreateSubdivisionSchemeAttr('none')
    value.CreateDoubleSidedAttr(True)
    value.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    return value


def add_plate(stage, spec):
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics
    from lego_contact.task.scene import ROOT, author, build_spec
    source = HERE / spec['rl_config']
    registration = HERE / spec['rl_registration']
    config = json.loads(source.read_text())
    prepared = build_spec(config, json.loads(registration.read_text()))
    # Reuse the RL plate author, explicitly omitting its example requested stack.
    prepared = copy.deepcopy(prepared)
    prepared['layout']['bricks'] = []
    prepared['world']['poses'] = []
    author(stage, prepared)
    stage.RemovePrim(ROOT + '/Stack')
    pitch = prepared['layout']['stud_pitch_m']
    cols, rows = prepared['layout']['grid_shape']
    radius = prepared['visual_priors']['stud_diameter_m'] / 2
    height = prepared['visual_priors']['stud_height_m']
    # Mesh studs are also visible in the existing mesh-only WebGL exporter.
    for i in range(cols):
        for j in range(rows):
            path = ROOT + f'/Plate/Studs/p_{i}_{j}'
            stage.RemovePrim(path)
            points = [(i*pitch + radius*math.cos(2*math.pi*s/16),
                       j*pitch + radius*math.sin(2*math.pi*s/16), z)
                      for z in (0., height) for s in range(16)]
            faces = [list(range(15, -1, -1)), list(range(16, 32))]
            faces += [[s, (s+1)%16, (s+1)%16+16, s+16] for s in range(16)]
            item = mesh(stage, path, points, list(map(len, faces)),
                        [v for face in faces for v in face], (.95,.95,.95))
            item.GetPrim().CreateAttribute('seating:gridIndex', Sdf.ValueTypeNames.Int2).Set(Gf.Vec2i(i,j))
    border = spec['tape_covered_rows_each_edge']
    if not isinstance(border, int) or not 0 < 2*border < min(cols, rows):
        raise ValueError('Invalid number of tape-covered stud rows')
    x0 = y0 = -pitch/2
    x1, y1 = (cols-.5)*pitch, (rows-.5)*pitch
    width = border*pitch
    bands = {'Left': (x0,y0,x0+width,y1), 'Right': (x1-width,y0,x1,y1),
             'Bottom': (x0+width,y0,x1-width,y0+width),
             'Top': (x0+width,y1-width,x1-width,y1)}
    z = height + spec['tape_surface_drawing_offset_m']
    for name, (a,b,c,d) in bands.items():
        mesh(stage, ROOT+'/Plate/TapeOverlap/'+name,
             [(a,b,z),(c,b,z),(c,d,z),(a,d,z)], [4], [0,1,2,3], (.025,.025,.026))
    root = stage.GetPrimAtPath(ROOT)
    root.CreateAttribute('seating:tapeCoveredRowsPerEdge', Sdf.ValueTypeNames.Int).Set(border)
    root.CreateAttribute('seating:photographedBricksAdded', Sdf.ValueTypeNames.Bool).Set(False)
    prims = list(Usd.PrimRange(root))
    studs = [p for p in prims if p.GetAttribute('seating:gridIndex').HasAuthoredValueOpinion()]
    assert len(studs) == cols*rows
    assert not stage.GetPrimAtPath(ROOT+'/Stack')
    assert not any(p.HasAPI(UsdPhysics.CollisionAPI) for p in prims)
    return {'grid': [cols,rows], 'physical_grid_studs': len(studs),
            'exposed_studs': (cols-2*border)*(rows-2*border),
            'tape_covered_studs': cols*rows-(cols-2*border)*(rows-2*border),
            'added_bricks': 0, 'T_O_plate_display_proxy': prepared['T_O_P_display'],
            'registration_status': prepared['transform_status'],
            'physical_thickness_m': prepared['physical_thickness_m'],
            'contact_enabled': False, 'training_ready': False,
            'tape_extent': spec['tape_extent'],
            'source_sha256': {str(p): hashlib.sha256(p.read_bytes()).hexdigest()
                              for p in (source, registration)}}


def widen_fingers(stage, spec):
    """Attach an operator-dimensioned cap to the unchanged narrow CAD mount."""
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics
    from .ceiling import calculate, thickness
    pocket = spec['pocket_fit']
    measured_span = pocket.get('measured_assembled_span_m')
    if measured_span is not None:
        if (not isinstance(measured_span,(int,float)) or isinstance(measured_span,bool)
                or not math.isfinite(measured_span) or measured_span<=0
                or not pocket.get('measurement_evidence')):
            raise ValueError('Measured assembled span needs a positive value and measurement evidence')
    layer_span = measured_span if measured_span is not None else spec['whole_layer_size_m'][0]
    guide_clearance = pocket['total_guide_clearance_m']
    if not isinstance(guide_clearance,(int,float)) or isinstance(guide_clearance,bool) or not math.isfinite(guide_clearance) or guide_clearance<=0:
        raise ValueError('Positive guide clearance required; passive sides must not squeeze the layer')
    span, reach = layer_span+guide_clearance, spec['inside_reach_m']
    approved_span = pocket.get('approved_clear_span_m')
    if approved_span is not None and (not isinstance(approved_span,(int,float)) or isinstance(approved_span,bool)
                                      or not math.isfinite(approved_span) or abs(approved_span-span)>1e-7):
        raise ValueError('Resolved guide spacing differs from the explicitly approved pocket span')
    shell_span = pocket['reference_previous_span_m']
    if span>shell_span:
        raise ValueError('Requested guide spacing exceeds the retained ceiling/back-wall span')
    wall = spec['wall_thickness_m']
    top = thickness(spec)
    rl_config = json.loads((HERE/'../lego_contact/task/config.json').read_text())
    stud_height = rl_config['visual_geometry_priors']['stud_height_m']
    if not all(isinstance(v,(float,int)) and not isinstance(v,bool) and math.isfinite(v) and v>0
               for v in (span,reach,wall,top)):
        raise ValueError('Cap dimensions must be positive finite metres')
    reports = []
    def box(path, lower, upper):
        points = [(lower[0],lower[1],lower[2]),(upper[0],lower[1],lower[2]),
                  (upper[0],upper[1],lower[2]),(lower[0],upper[1],lower[2]),
                  (lower[0],lower[1],upper[2]),(upper[0],lower[1],upper[2]),
                  (upper[0],upper[1],upper[2]),(lower[0],upper[1],upper[2])]
        faces = [0,2,1,0,3,2,4,5,6,4,6,7,0,1,5,0,5,4,
                 1,2,6,1,6,5,2,3,7,2,7,6,3,0,4,3,4,7]
        value = mesh(stage,path,points,[3]*12,faces,(.025,.026,.028))
        UsdPhysics.CollisionAPI.Apply(value.GetPrim()).CreateCollisionEnabledAttr(True)
        UsdPhysics.MeshCollisionAPI.Apply(value.GetPrim()).CreateApproximationAttr('convexHull')
        return value
    def hull(points):
        points = sorted(set(points))
        def cross(o,a,b):
            return (a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0])
        low,high = [],[]
        for p in points:
            while len(low)>=2 and cross(low[-2],low[-1],p)<=1e-15: low.pop()
            low.append(p)
        for p in reversed(points):
            while len(high)>=2 and cross(high[-2],high[-1],p)<=1e-15: high.pop()
            high.append(p)
        return low[:-1]+high[:-1]
    def profiled_wall(path,profile,length,point):
        n = len(profile)
        points = [point(s,d,z) for s in (0.,length) for d,z in profile]
        faces = []
        for k in range(1,n-1):
            faces.extend((0,k+1,k,n,n+k,n+k+1))
        for k in range(n):
            j=(k+1)%n
            faces.extend((k,j,n+j,k,n+j,n+k))
        def triple(a,b,c):
            return sum(a[i]*(b[(i+1)%3]*c[(i+2)%3]-b[(i+2)%3]*c[(i+1)%3]) for i in range(3))
        volume = sum(triple(points[a],points[b],points[c]) for a,b,c in zip(faces[::3],faces[1::3],faces[2::3]))
        if volume<0:
            for k in range(0,len(faces),3): faces[k+1],faces[k+2]=faces[k+2],faces[k+1]
        value = mesh(stage,path,points,[3]*(len(faces)//3),faces,(.025,.026,.028))
        UsdPhysics.CollisionAPI.Apply(value.GetPrim()).CreateCollisionEnabledAttr(True)
        UsdPhysics.MeshCollisionAPI.Apply(value.GetPrim()).CreateApproximationAttr('convexHull')
        return value
    for name,sign in (('fr3_leftfinger',1),('fr3_rightfinger',-1)):
        path = ROBOT+'/'+name+'/UpdatedTipSTLVisual'
        tip = stage.GetPrimAtPath(path)
        source = UsdGeom.Mesh(stage.GetPrimAtPath(path+'/Mesh'))
        points = [list(p) for p in source.GetPointsAttr().Get()]
        for key,value in (('insideSpanM',span),('insideReachM',reach),
                          ('wallThicknessM',wall),('topThicknessM',top)):
            tip.CreateAttribute('variant:'+key,Sdf.ValueTypeNames.Double).Set(value)
        tip.CreateAttribute('variant:geometryStatus', Sdf.ValueTypeNames.String).Set(
            'Operator-dimensioned cap; original CAD mounting spine unchanged; attachment fit/material/inertia unvalidated')
        UsdPhysics.CollisionAPI.Apply(source.GetPrim()).CreateCollisionEnabledAttr(True)
        UsdPhysics.MeshCollisionAPI.Apply(source.GetPrim()).CreateApproximationAttr('convexHull')
        inverse = UsdGeom.Xformable(stage.GetPrimAtPath(ROBOT+'/'+name)).ComputeLocalToWorldTransform(0).GetInverse()
        xf = UsdGeom.Xformable(source).ComputeLocalToWorldTransform(0)*inverse
        points_finger = [list(xf.Transform(Gf.Vec3d(*p))) for p in points]
        bounds = [[min(p[a] for p in points_finger) for a in range(3)],
                  [max(p[a] for p in points_finger) for a in range(3)]]
        # The native terminal box combines the removed rubber pad with its
        # metal support. Crop only its pad portion at the existing CAD-defined
        # metal backplane, retaining the outer support and the other 3 boxes.
        stock = UsdGeom.Cube(stage.GetPrimAtPath(ROBOT+'/'+name+'/collisions/mesh_3'))
        half = stock.GetSizeAttr().Get()/2
        stock_xf = stock.ComputeLocalToWorldTransform(0)*inverse
        corners = [stock_xf.Transform(Gf.Vec3d(x,y,z)) for x in (-half,half)
                   for y in (-half,half) for z in (-half,half)]
        old_stock = [[min(v[a] for v in corners) for a in range(3)],
                     [max(v[a] for v in corners) for a in range(3)]]
        if abs(old_stock[0][2]-.036)>1e-5 or abs(old_stock[1][2]-.0545)>1e-5:
            raise ValueError('Native terminal pad/support collision box changed')
        retained = copy.deepcopy(old_stock)
        backplane = spec['source_pad_backplane_abs_finger_y_m']
        retained[0 if sign>0 else 1][1] = sign*backplane
        size = [retained[1][a]-retained[0][a] for a in range(3)]
        center = [(retained[1][a]+retained[0][a])/2 for a in range(3)]
        target = Gf.Matrix4d(1.)
        target.SetScale(Gf.Vec3d(*size));target.SetTranslateOnly(Gf.Vec3d(*center))
        finger_world = inverse.GetInverse()
        parent_world = UsdGeom.Xformable(stock.GetPrim().GetParent()).ComputeLocalToWorldTransform(0)
        stock.CreateSizeAttr(1.)
        stock.MakeMatrixXform().Set(target*finger_world*parent_world.GetInverse())
        # Use the old body's side silhouette, excluding its mounting boss.
        # The original supplied CAD's backplane is X=-9.5 mm. Keep its height
        # and distal taper; normalize only wall-normal thickness to 3.2 mm.
        body_backplane = spec['source_body_backplane_cad_x_mm']
        raw_profile = hull([(float(p[0]),float(p[2])) for p in points if p[0]>=body_backplane-1e-5])
        profile = []
        for depth,z in raw_profile:
            profile.append((-depth/abs(body_backplane)*wall,
                            float(xf.Transform(Gf.Vec3d(0,0,z))[2])))
        distal = bounds[1][2]
        ceiling = min(z for d,z in profile)
        profile_height = distal-ceiling
        ceiling_alignment = calculate(spec,ceiling,distal,stud_height)
        height = distal-ceiling_alignment['underside_z_finger_m']
        front = bounds[0][1] if sign>0 else bounds[1][1]
        back, end = front+sign*wall, front-sign*reach
        root = ROBOT+'/'+name+'/WideCap'
        profiled_wall(root+'/BackWall',profile,shell_span+2*wall,
                      lambda s,d,z:(-shell_span/2-wall+s,front+sign*d,z))
        profiled_wall(root+'/EndNegative',profile,reach,
                      lambda s,d,z:(-span/2-d,front-sign*s,z))
        profiled_wall(root+'/EndPositive',profile,reach,
                      lambda s,d,z:(span/2+d,front-sign*s,z))
        roof0,roof1 = min(back,end),max(back,end)
        box(root+'/Top',[-shell_span/2-wall,roof0,ceiling_alignment['top_surface_z_finger_m']],
            [shell_span/2+wall,roof1,ceiling_alignment['underside_z_finger_m']])
        if list(map(list,source.GetPointsAttr().Get())) != points:
            raise ValueError('Original CAD mounting geometry changed')
        reports.append({'finger':name, 'bounds_finger_m':bounds,
                        'original_cad_mount_preserved':True,'clamping_plane_y_m':front,
                        'return_end_y_m':end,
                        'ceiling_underside_z_m':ceiling_alignment['underside_z_finger_m'],
                        'ceiling_top_surface_z_m':ceiling_alignment['top_surface_z_finger_m'],
                        'ceiling_alignment':ceiling_alignment,
                        'distal_z_m':distal,'cap_root':root,
                        'wall_profile_depth_z_m':profile,
                        'wall_profile_source':'Original CAD body side silhouette; original height/taper, thickness normalized to operator 3.2 mm; mounting boss excluded from wall silhouette and retained separately',
                        'stock_pad_collision_removed':True,
                        'native_terminal_collision_before_finger_m':old_stock,
                        'native_terminal_metal_support_retained_finger_m':retained,
                        'collision':'Four closed convex cap components plus unchanged CAD spine; stock supports retained'})
    left, right = [r['bounds_finger_m'] for r in reports]
    closed_gap = left[0][1]-right[1][1]
    tip_gap_at_zero = closed_gap-2*reach
    minimum_width = max(0.,spec['minimum_opposing_clearance_m']-tip_gap_at_zero)
    for i in (1,2):
        joint = UsdPhysics.PrismaticJoint(stage.GetPrimAtPath(ROBOT+f'/fr3_hand/fr3_finger_joint{i}'))
        if minimum_width/2 >= joint.GetUpperLimitAttr().Get():
            raise ValueError('Cap returns leave no valid native gripper stroke')
        joint.CreateLowerLimitAttr(minimum_width/2)
        joint.GetPrim().CreateAttribute('state:linear:physics:position',Sdf.ValueTypeNames.Float).Set(spec['native_open_width_m']/2)
        joint.GetPrim().CreateAttribute('state:linear:physics:velocity',Sdf.ValueTypeNames.Float).Set(0.)
    # Start the USD itself open as well as setting its joint state. The caps
    # intentionally cannot occupy the native hand's fully-closed zero pose.
    for name,sign in (('fr3_leftfinger',1),('fr3_rightfinger',-1)):
        finger = stage.GetPrimAtPath(ROBOT+'/'+name)
        xf = UsdGeom.Xformable(finger).ComputeLocalToWorldTransform(0)
        offset = xf.TransformDir(Gf.Vec3d(0,sign*spec['native_open_width_m']/2,0))
        position = finger.GetAttribute('xformOp:translate')
        position.Set(position.Get()+offset)
    _, layer_width, layer_height = spec['whole_layer_size_m']
    open_width = spec['native_open_width_m']
    close_width = spec['nominal_close_width_m']
    open_gap, close_gap = open_width+closed_gap, close_width+closed_gap
    contact_width = layer_width-closed_gap
    if not (span >= layer_span and span > spec['whole_layer_size_m'][0] and height > stud_height and
            open_gap > layer_width and close_gap <= layer_width and
            minimum_width <= close_width <= contact_width < open_width <= .08):
        raise ValueError('Whole-layer envelope or opening/closing feasibility check failed')
    return {'inside_span_m':span,'inside_reach_m':reach,'inside_height_m':height,
            'pocket_fit':{'assembled_span_used_m':layer_span,
                          'assembled_span_measured':measured_span is not None,
                          'measurement_evidence':pocket.get('measurement_evidence'),
                          'approved_clear_span_m':approved_span,
                          'span_approval_evidence':pocket.get('span_approval_evidence'),
                          'total_guide_clearance_m':guide_clearance,
                          'guide_clearance_each_side_m':guide_clearance/2,
                          'retained_shell_inner_reference_span_m':shell_span,
                          'only_end_returns_move':True,
                          'status':('measurement-based geometry' if measured_span is not None else
                                    f'operator-approved {approved_span*1000:g} mm span; assembled width not independently measured' if approved_span is not None else
                                    'nominal preview; assembled layer measurement required'),
                          'passive_sides_clamp':False,'brick_geometry_scaled':False,
                          'underside':'open; no bottom plate or ledge added',
                          'receiving_layer_vertical_clearance_verified':False,
                          'receiving_exposed_stud_clearance_required_m':stud_height,
                          'receiving_layer_note':'Existing ceiling and vertical profiles retained. Fingertip-to-brick-bottom and receiving-stud clearance still need a verified datum.'},
            'wall_profile_height_m':profile_height,
            'inside_height_status':'Free cavity under the thickened fixed-top ceiling; wall profiles unchanged',
            'wall_thickness_m':wall,'top_thickness_m':top,'outer_span_m':shell_span+2*wall,
            'ceiling_alignment':ceiling_alignment,
            'preview_native_open_width_m':open_width,
            'minimum_native_width_m':minimum_width,
            'instances':reports, 'physical_contact_or_lift_validated':False,
            'mass_and_materials_identified':False,
            'geometric_fit':{'whole_layer_size_m':spec['whole_layer_size_m'],
                'span_clearance_total_m':span-layer_span,
                'clamping_gap_at_zero_native_width_m':closed_gap,
                'return_tip_gap_at_zero_native_width_m':tip_gap_at_zero,
                'pad_gap_at_nominal_open_m':open_gap,
                'native_open_width_m':open_width,
                'pad_gap_at_nominal_close_target_m':close_gap,
                'return_tip_gap_at_nominal_open_m':open_gap-2*reach,
                'return_tip_gap_at_nominal_close_target_m':close_gap-2*reach,
                'native_width_at_rigid_layer_contact_m':contact_width,
                'opposite_cap_collision_free_over_native_width_m':[minimum_width,.08],
                'minimum_opposing_clearance_m':spec['minimum_opposing_clearance_m'],
                'nominal_stud_height_m':stud_height,
                'side_wall_body_engagement_when_roof_on_studs_m':min(layer_height,height-stud_height),
                'wall_bottom_above_nominal_flat_layer_floor_when_roof_on_studs_m':layer_height+stud_height-height,
                'meaning':'100 mm clear between end walls. 20 mm inward returns and top on each jaw. Simulation lower joint limits prevent cap collision; physical controller settings are unchanged. Geometric fit does not demonstrate frictional lifting.'}}


def apply(stage, environment, fingers, output):
    profiles = json.loads((HERE/'profiles.json').read_text())
    scene = profiles['environments'][environment]
    finger_spec = profiles['fingers'][fingers]
    report = {'environment':environment, 'fingers':fingers, 'explicit_selection':True,
              'hardware_clients':0, 'plate':None, 'fingertips':None,
              'source_sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest()
                               for p in (HERE/'profiles.json',HERE/'geometry.py',HERE/'assets/original_fingertip.stl')}}
    if scene['add_plate']:
        report['plate'] = add_plate(stage, profiles['plate'])
    if fingers == 'wide':
        report['fingertips'] = widen_fingers(stage, finger_spec)
    (output/'environment.json').write_text(json.dumps(report,indent=2)+'\n')
    return report
