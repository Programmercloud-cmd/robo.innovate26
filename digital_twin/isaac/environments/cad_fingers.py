"""Supplied wide CAD, normalized into the already registered slim CAD frame."""
import hashlib
import json
from pathlib import Path
import struct

import numpy as np

HERE = Path(__file__).resolve().parent
ROBOT = '/World/envs/env_0/Robot'


def ray_distance(triangles, origin, direction):
    """First surface hit, used only for independent CAD cavity measurements."""
    origin=np.asarray(origin);direction=np.asarray(direction)
    a=triangles[:,0];e1=triangles[:,1]-a;e2=triangles[:,2]-a
    h=np.cross(direction,e2);det=np.einsum('ij,ij->i',e1,h)
    valid=np.abs(det)>1e-10;inv=np.zeros_like(det);inv[valid]=1/det[valid]
    s=origin-a;u=inv*np.einsum('ij,ij->i',s,h);q=np.cross(s,e1)
    v=inv*(q@direction);distance=inv*np.einsum('ij,ij->i',e2,q)
    valid &= (u>=-1e-7)&(v>=-1e-7)&(u+v<=1+1e-7)&(distance>1e-6)
    if not valid.any():raise ValueError('CAD cavity measurement ray missed its surface')
    return float(distance[valid].min())


def load_cad(spec):
    path=HERE/spec['cad_asset'];data=path.read_bytes()
    if hashlib.sha256(data).hexdigest()!=spec['cad_sha256']:raise ValueError('Wide CAD hash mismatch')
    count=struct.unpack_from('<I',data,80)[0]
    if len(data)!=84+50*count:raise ValueError('Expected binary wide STL')
    raw=np.array([[struct.unpack_from('<fff',data,96+50*i+12*j) for j in range(3)] for i in range(count)])
    points=raw*spec['cad_to_original_scale']+np.array(spec['cad_to_original_translation_mm'])
    if not np.isfinite(points).all():raise ValueError('Nonfinite CAD vertices')
    lo=points.min(axis=(0,1));hi=points.max(axis=(0,1))
    # The known cavity is open in +CAD X and -CAD Z. Probe below the screw
    # opening, inside the pocket; do not infer the cavity from a convex hull.
    center=np.array([10.,-15.5,11.5])
    span=ray_distance(points,center,[0,1,0])+ray_distance(points,center,[0,-1,0])
    back=center[0]-ray_distance(points,center,[-1,0,0])
    ceiling=center[2]+ray_distance(points,center,[0,0,1])
    distal=points.reshape(-1,3);distal=distal[np.abs(distal[:,2]-lo[2])<1e-3]
    side=(float(distal[:,1].max()-distal[:,1].min())-span)/2
    dimensions=dict(inside_span_m=span*.001,inside_reach_m=(hi[0]-back)*.001,
        inside_height_m=(ceiling-lo[2])*.001,top_thickness_m=(hi[2]-ceiling)*.001,
        wall_thickness_m=side*.001,outer_span_m=(hi[1]-lo[1])*.001,
        roof_underside_cad_z_mm=ceiling,roof_top_cad_z_mm=float(hi[2]),
        bounds_original_cad_mm=[lo.tolist(),hi.tolist()],
        nominal_wall_thickness_status='5 mm distal return wall; proximal taper is thicker, as supplied')
    if (abs(span-95.8)>1e-3 or abs(hi[0]-back-20)>1e-3 or
            abs(hi[2]-lo[2]-25.55)>1e-3 or abs(lo[0]+11)>1e-3):
        raise ValueError('Wide STL scale/mount/pocket dimensions disagree with the known CAD references')
    return points.reshape(-1,3),dimensions


def ceiling_extension(spec,points,dimensions):
    """Add material below the original roof; retain every supplied CAD vertex."""
    settings=spec.get('ceiling',{})
    if settings.get('mode')!='target_cavity_height':return None,dimensions
    target=settings['target_cavity_height_m']
    old_height=dimensions['inside_height_m']
    if not isinstance(target,(int,float)) or isinstance(target,bool) or not np.isfinite(target) or not 0<target<old_height:
        raise ValueError('Ceiling thickening requires a positive cavity height below the original height')
    bottom=float(points[:,2].min());underside=bottom+target*1000
    original_underside=dimensions['roof_underside_cad_z_mm']
    span=dimensions['inside_span_m']*1000;reach=dimensions['inside_reach_m']*1000
    # A small overlap lies inside the original roof, avoiding a disconnected
    # coplanar collision component. The external top surface is untouched.
    upper=original_underside+min(.1,dimensions['top_thickness_m']*1000/2)
    lower=[0.,-15.5-span/2,underside];high=[reach,-15.5+span/2,upper]
    vertices=np.array([[lower[0],lower[1],lower[2]],[high[0],lower[1],lower[2]],
        [high[0],high[1],lower[2]],[lower[0],high[1],lower[2]],
        [lower[0],lower[1],high[2]],[high[0],lower[1],high[2]],
        [high[0],high[1],high[2]],[lower[0],high[1],high[2]]])
    faces=np.array([[0,2,1],[0,3,2],[4,5,6],[4,6,7],[0,1,5],[0,5,4],
                    [1,2,6],[1,6,5],[2,3,7],[2,7,6],[3,0,4],[3,4,7]])
    result=dict(dimensions,inside_height_m=target,
        roof_underside_cad_z_mm=underside,
        top_thickness_m=(dimensions['roof_top_cad_z_mm']-underside)*.001,
        ceiling_adjustment=dict(mode='downward_thickening_fixed_top',
            previous_inside_height_m=old_height,target_inside_height_m=target,
            added_downward_thickness_m=old_height-target,
            original_roof_top_cad_z_mm=dimensions['roof_top_cad_z_mm'],
            source_cad_vertices_and_mount_unchanged=True,
            extension_bounds_original_cad_mm=[lower,high]))
    return (vertices,faces),result


def apply_cad(stage,spec):
    from pxr import Gf,Sdf,UsdGeom,UsdPhysics
    points,source_dimensions=load_cad(spec)
    extension,dimensions=ceiling_extension(spec,points,source_dimensions)
    instances=[];projected=[];fronts=[]
    for name,sign in (('fr3_leftfinger',1),('fr3_rightfinger',-1)):
        finger=stage.GetPrimAtPath(ROBOT+'/'+name)
        inverse=UsdGeom.Xformable(finger).ComputeLocalToWorldTransform(0).GetInverse()
        tip=stage.GetPrimAtPath(ROBOT+'/'+name+'/UpdatedTipSTLVisual')
        mesh=UsdGeom.Mesh(stage.GetPrimAtPath(str(tip.GetPath())+'/Mesh'))
        # Preserve the original mount's proper rotations, scale and translation.
        xf=mesh.ComputeLocalToWorldTransform(0)*inverse
        before=UsdGeom.Xformable(tip).GetLocalTransformation()
        mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in points])
        mesh.CreateFaceVertexCountsAttr([3]*(len(points)//3))
        mesh.CreateFaceVertexIndicesAttr(list(range(len(points))))
        mesh.CreateExtentAttr([Gf.Vec3f(*points.min(0)),Gf.Vec3f(*points.max(0))])
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim()).CreateCollisionEnabledAttr(True)
        # A single convex hull would fill the pocket. Decomposition retains the
        # CAD concavity; its physical contact fidelity has not been measured.
        UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr('convexDecomposition')
        tip.CreateAttribute('variant:sourceSTLSha256',Sdf.ValueTypeNames.String).Set(spec['cad_sha256'])
        tip.CreateAttribute('variant:geometrySource',Sdf.ValueTypeNames.String).Set('supplied_stl')
        if extension is not None:
            vertices,faces=extension
            addition=UsdGeom.Mesh.Define(stage,str(tip.GetPath())+'/CeilingExtension')
            addition.CreatePointsAttr([Gf.Vec3f(*v) for v in vertices])
            addition.CreateFaceVertexCountsAttr([3]*len(faces))
            addition.CreateFaceVertexIndicesAttr(faces.reshape(-1).tolist())
            addition.CreateSubdivisionSchemeAttr('none')
            addition.CreateDoubleSidedAttr(True)
            addition.CreateDisplayColorAttr([Gf.Vec3f(.025,.026,.028)])
            UsdPhysics.CollisionAPI.Apply(addition.GetPrim()).CreateCollisionEnabledAttr(True)
            UsdPhysics.MeshCollisionAPI.Apply(addition.GetPrim()).CreateApproximationAttr('convexHull')
        if UsdGeom.Xformable(tip).GetLocalTransformation()!=before:raise ValueError('CAD mount transform changed')
        # Retain the native metal support; remove its obsolete inward pad part.
        stock=UsdGeom.Cube(stage.GetPrimAtPath(ROBOT+'/'+name+'/collisions/mesh_3'))
        half=stock.GetSizeAttr().Get()/2;stock_xf=stock.ComputeLocalToWorldTransform(0)*inverse
        corners=[stock_xf.Transform(Gf.Vec3d(x,y,z)) for x in (-half,half) for y in (-half,half) for z in (-half,half)]
        bounds=[[min(v[a] for v in corners) for a in range(3)],[max(v[a] for v in corners) for a in range(3)]]
        if abs(bounds[0][2]-.036)>1e-5 or abs(bounds[1][2]-.0545)>1e-5:raise ValueError('Native support geometry changed')
        bounds[0 if sign>0 else 1][1]=sign*spec['source_pad_backplane_abs_finger_y_m']
        transform=Gf.Matrix4d(1.);transform.SetScale(Gf.Vec3d(*[bounds[1][a]-bounds[0][a] for a in range(3)]))
        transform.SetTranslateOnly(Gf.Vec3d(*[(bounds[1][a]+bounds[0][a])/2 for a in range(3)]))
        parent_world=UsdGeom.Xformable(stock.GetPrim().GetParent()).ComputeLocalToWorldTransform(0)
        stock.CreateSizeAttr(1.);stock.MakeMatrixXform().Set(transform*inverse.GetInverse()*parent_world.GetInverse())
        local=[xf.Transform(Gf.Vec3d(*p)) for p in points]
        finger_bounds=[[min(p[a] for p in local) for a in range(3)],[max(p[a] for p in local) for a in range(3)]]
        projected.append([p[1] for p in local]);fronts.append(float(xf.Transform(Gf.Vec3d(0,-15.5,11.5))[1]))
        instances.append(dict(finger=name,mesh_path=str(mesh.GetPath()),bounds_finger_m=finger_bounds,
            actual_matrix_original_cad_mm_to_finger_m=list(map(list,xf)),
            mounting_transform_unchanged=True,collision_approximation='convexDecomposition'))
    zero_gap=min(projected[0])-max(projected[1]);minimum=max(0.,spec['minimum_opposing_clearance_m']-zero_gap)
    for i in (1,2):
        joint=UsdPhysics.PrismaticJoint(stage.GetPrimAtPath(ROBOT+f'/fr3_hand/fr3_finger_joint{i}'))
        if minimum/2>=joint.GetUpperLimitAttr().Get():raise ValueError('Wide CAD leaves no simulated stroke')
        joint.CreateLowerLimitAttr(minimum/2)
        joint.GetPrim().CreateAttribute('state:linear:physics:position',Sdf.ValueTypeNames.Float).Set(spec['native_open_width_m']/2)
        joint.GetPrim().CreateAttribute('state:linear:physics:velocity',Sdf.ValueTypeNames.Float).Set(0.)
    for name,sign in (('fr3_leftfinger',1),('fr3_rightfinger',-1)):
        finger=stage.GetPrimAtPath(ROBOT+'/'+name);world=UsdGeom.Xformable(finger).ComputeLocalToWorldTransform(0)
        position=finger.GetAttribute('xformOp:translate')
        position.Set(position.Get()+world.TransformDir(Gf.Vec3d(0,sign*spec['native_open_width_m']/2,0)))
    if minimum>spec['nominal_close_width_m']:raise ValueError('Wide CAD incompatible with nominal closing width')
    return dict(dimensions,geometry_source='supplied_stl',cad_asset=spec['cad_asset'],cad_sha256=spec['cad_sha256'],
        cad_to_original_scale=spec['cad_to_original_scale'],cad_to_original_translation_mm=spec['cad_to_original_translation_mm'],
        registration_basis=spec['cad_registration_basis'],source_cad_dimensions=source_dimensions,
        triangles_per_finger=len(points)//3+(12 if extension is not None else 0),
        preview_native_open_width_m=spec['native_open_width_m'],minimum_native_width_m=minimum,
        instances=instances,physical_contact_or_lift_validated=False,mass_and_materials_identified=False,
        geometric_fit=dict(return_tip_gap_at_zero_native_width_m=zero_gap,
            clamping_gap_at_zero_native_width_m=fronts[0]-fronts[1],
            minimum_opposing_clearance_m=spec['minimum_opposing_clearance_m'],
            whole_layer_size_m=spec['whole_layer_size_m'],
            wall_bottom_above_nominal_flat_layer_floor_when_roof_on_studs_m=.0096+.0017-dimensions['inside_height_m']))
