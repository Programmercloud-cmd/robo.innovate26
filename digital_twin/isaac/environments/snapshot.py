"""Save the selected variant at its actual PhysX preview pose, without mutating it."""
import math


def save(stage, path, link_names, poses, positions, arm, fingers, frame, hand_stale, hand_profile_stop=None):
    from pxr import Gf, Sdf, Usd, UsdGeom
    from .geometry import ROBOT
    copy = Usd.Stage.Open(stage.Flatten())
    for name, p in zip(link_names, poses):
        prim = copy.GetPrimAtPath(ROBOT+'/'+name)
        if not prim:
            continue
        matrix = Gf.Matrix4d(1.)
        matrix.SetRotate(Gf.Quatd(float(p[6]),Gf.Vec3d(*map(float,p[3:6]))))
        matrix.SetTranslateOnly(Gf.Vec3d(*map(float,p[:3])))
        parent = UsdGeom.Xformable(prim.GetParent()).ComputeLocalToWorldTransform(0)
        UsdGeom.Xformable(prim).MakeMatrixXform().Set(matrix*parent.GetInverse())
    for i, index in enumerate(arm,1):
        prim = copy.GetPrimAtPath(ROBOT+f'/fr3_link{i-1}/fr3_joint{i}')
        prim.CreateAttribute('state:angular:physics:position',Sdf.ValueTypeNames.Float).Set(math.degrees(float(positions[0,index])))
        prim.CreateAttribute('state:angular:physics:velocity',Sdf.ValueTypeNames.Float).Set(0.)
    for i, index in enumerate(fingers,1):
        prim = copy.GetPrimAtPath(ROBOT+f'/fr3_hand/fr3_finger_joint{i}')
        prim.CreateAttribute('state:linear:physics:position',Sdf.ValueTypeNames.Float).Set(float(positions[0,index]))
        prim.CreateAttribute('state:linear:physics:velocity',Sdf.ValueTypeNames.Float).Set(0.)
    copy.GetRootLayer().customLayerData = {
        'variantSnapshotSource':'Existing twin measured arm recording; no hardware client',
        'sourceSequence':str(frame['seq']), 'sourceUnixNs':str(frame['unix_ns']),
        'handTelemetryUnavailable':bool(hand_stale),
        'selectedFingerProfileIncompatibleWithPhysicalWidth':bool(hand_profile_stop),
        'handPoseMeaning':'Simulated retained preview opening' if hand_stale or hand_profile_stop else 'Measured hand width',
        'contactOrLiftValidated':False}
    copy.Export(str(path))
