"""Native finger visual audit before registering the user-supplied tip mesh."""
import json


def audit_native_fingers(stage, output):
    from pxr import Gf, Usd, UsdGeom
    records = []
    for name in ('fr3_leftfinger', 'fr3_rightfinger'):
        finger = stage.GetPrimAtPath('/World/envs/env_0/Robot/'+name)
        inverse = UsdGeom.Xformable(finger).ComputeLocalToWorldTransform(0).GetInverse()
        for prim in Usd.PrimRange(finger):
            if prim.GetTypeName() != 'Mesh':
                continue
            xf = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)*inverse
            points = [list(xf.Transform(Gf.Vec3d(p))) for p in UsdGeom.Mesh(prim).GetPointsAttr().Get()]
            records.append({'finger':name,'path':str(prim.GetPath()),'bounds_finger_m':
                [[min(p[i] for p in points) for i in range(3)], [max(p[i] for p in points) for i in range(3)]],
                'point_count':len(points),'points_finger_m':points})
    (output/'native_finger_visual_audit.json').write_text(json.dumps(records))
    return {'status':'STL registration pending native-pad geometry audit','mesh_records':len(records)}
