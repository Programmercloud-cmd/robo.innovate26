"""Stock collider audit and explicit surface-compatibility checks, SI meters."""
import json
import math
from pathlib import Path

ROBOT='/World/envs/env_0/Robot'
BODIES={'left':ROBOT+'/fr3_leftfinger','right':ROBOT+'/fr3_rightfinger',
        'coupon':'/World/envs/env_0/contact_coupon'}


def metadata_scalar(value):
    if isinstance(value,float) and not math.isfinite(value):
        return {'nonfinite_scalar':'nan' if math.isnan(value) else 'inf' if value>0 else '-inf'}
    return value


def distance_to_triangles(point, vertices, triangles):
    """Minimum unsigned point-to-triangle distance; faces and clamped edges."""
    import numpy as np
    t=np.asarray(vertices,float)[np.asarray(triangles,int)]
    a,b,c=t[:,0],t[:,1],t[:,2]
    ab,ac=b-a,c-a; n=np.cross(ab,ac); n2=np.sum(n*n,axis=1)
    ap=np.asarray(point)-a
    projected=np.asarray(point)-n*(np.sum(ap*n,axis=1)/np.maximum(n2,1e-30))[:,None]
    v=projected-a
    d00=np.sum(ab*ab,axis=1);d01=np.sum(ab*ac,axis=1);d11=np.sum(ac*ac,axis=1)
    d20=np.sum(v*ab,axis=1);d21=np.sum(v*ac,axis=1)
    den=d00*d11-d01*d01
    u=(d11*d20-d01*d21)/np.maximum(den,1e-30)
    w=(d00*d21-d01*d20)/np.maximum(den,1e-30)
    interior=(u>=0)&(w>=0)&(u+w<=1)&(n2>1e-30)
    best=np.min(np.linalg.norm(projected[interior]-point,axis=1)) if interior.any() else float('inf')
    for p,q in ((a,b),(b,c),(c,a)):
        edge=q-p; fraction=np.clip(np.sum((point-p)*edge,axis=1)/np.maximum(np.sum(edge*edge,axis=1),1e-30),0,1)
        best=min(best,float(np.linalg.norm(p+fraction[:,None]*edge-point,axis=1).min()))
    return float(best)


def audit(stage, output):
    import numpy as np
    from scipy.spatial import ConvexHull
    from pxr import Gf,Usd,UsdGeom,UsdPhysics,UsdShade,PhysxSchema
    result={'surface_provenance':'Composed authored collision surface / mathematical convex hull, not an exported PhysX cooked mesh',
            'fingers':{}}
    for label in ('left','right'):
        root=BODIES[label]; finger=stage.GetPrimAtPath(root)
        inverse=UsdGeom.Xformable(finger).ComputeLocalToWorldTransform(0).GetInverse()
        rows=[]
        result['fingers'][label]={'body_path':root,'colliders':rows}
        for prim in Usd.PrimRange(finger,Usd.TraverseInstanceProxies()):
            if not prim.HasAPI(UsdPhysics.CollisionAPI):continue
            if not UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get():continue
            row={'path':str(prim.GetPath()),'type':prim.GetTypeName(),'enabled':True}
            rows.append(row)
            Path(output).write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
            if not prim.IsA(UsdGeom.Mesh):raise RuntimeError('Unsupported stock collision type: '+str(prim.GetPath()))
            mesh=UsdGeom.Mesh(prim)
            xf=UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)*inverse
            points=np.asarray([list(xf.Transform(Gf.Vec3d(p))) for p in mesh.GetPointsAttr().Get()],float)
            indices=list(mesh.GetFaceVertexIndicesAttr().Get());triangles=[];start=0
            for count in mesh.GetFaceVertexCountsAttr().Get():
                face=indices[start:start+count];start+=count
                triangles.extend([face[0],face[k],face[k+1]] for k in range(1,count-1))
            hull=ConvexHull(points)
            approximation=UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get()
            # Mesh input must itself have hull support planes, not an arbitrary concave input.
            volume=abs(sum(np.dot(points[i],np.cross(points[j],points[k])) for i,j,k in triangles)/6)
            reference=triangles if approximation=='none' else hull.simplices.tolist()
            material,_=UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial('physics')
            props=PhysxSchema.PhysxCollisionAPI(prim)
            material_attrs={a.GetName():metadata_scalar(a.Get()) for a in (material.GetPrim().GetAttributes() if material else [])
                if a.GetName().startswith(('physics:','physxMaterial:')) and isinstance(a.Get(),(float,int,bool,str))}
            row.update({'path':str(prim.GetPath()),'is_pad':str(prim.GetPath())==root+'/collisions/mesh_3',
                'enabled':True,'is_instance_proxy':prim.IsInstanceProxy(),'approximation':approximation,
                'points_finger_m':points.tolist(),'triangle_indices':reference,
                'authored_triangle_indices':triangles,'reference_surface_kind':'mathematical_convex_hull' if approximation=='convexHull' else 'convex_authored_mesh',
                'bounds_finger_m':[points.min(axis=0).tolist(),points.max(axis=0).tolist()],
                'contact_offset_m':metadata_scalar(props.GetContactOffsetAttr().Get()),'rest_offset_m':metadata_scalar(props.GetRestOffsetAttr().Get()),
                'physics_material_path':str(material.GetPath()) if material else None,'physics_material_attributes':material_attrs,
                'material_binding_status':'resolved' if material else 'unbound/default unknown'})
            Path(output).write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
            if approximation not in ('convexHull','none') or (approximation=='none' and abs(volume-hull.volume)>max(1e-12,hull.volume*1e-4)):
                raise RuntimeError('Stock cooked surface cannot be safely represented: '+str(prim.GetPath()))
        pads=[row for row in rows if row['is_pad']]
        if len(pads)!=1 or len(rows)<2 or not (pads[0]['physics_material_path'] or '').endswith('/PhysicsMaterials/FingerPad'):
            raise RuntimeError('Stock pad/white-finger collision separation is ambiguous')
        bound=pads[0]['bounds_finger_m']
        inner=bound[0][1] if label=='left' else bound[1][1]
        result['fingers'][label]={'body_path':root,'pad_path':pads[0]['path'],
            'inner_plane_y_m':inner,'bounds_finger_m':bound,'colliders':rows}
    Path(output).write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    return result


def own_point(point_finger, separation, finger, protocol):
    pad=[];white=[]
    for collider in finger['colliders']:
        distance=distance_to_triangles(point_finger,collider['points_finger_m'],collider['triangle_indices'])
        (pad if collider['is_pad'] else white).append(distance)
    tolerance=abs(float(separation))+protocol['point_gate']['surface_slack_m']
    dp,dw=min(pad),min(white)
    return {'distance_pad_m':dp,'distance_white_m':dw,'tolerance_m':tolerance,
        'unique_pad':bool(dp<=tolerance and dw>tolerance+protocol['point_gate']['white_clearance_extra_m'])}


def self_test():
    assert abs(distance_to_triangles([.2,.2,1],[[0,0,0],[1,0,0],[0,1,0]],[[0,1,2]])-1)<1e-12
    assert abs(distance_to_triangles([2,0,0],[[0,0,0],[1,0,0],[0,1,0]],[[0,1,2]])-1)<1e-12
    print('Triangle face/edge distance contracts: PASS')


if __name__=='__main__':self_test()
