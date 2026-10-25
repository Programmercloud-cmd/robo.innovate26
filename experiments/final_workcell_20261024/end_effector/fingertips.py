"""Native finger visual audit before registering the user-supplied tip mesh."""
import json


def audit_native_fingers(stage, output):
    from pxr import Gf, Usd, UsdGeom, UsdShade
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
                'point_count':len(points),'points_finger_m':points,
                'face_counts':list(UsdGeom.Mesh(prim).GetFaceVertexCountsAttr().Get()),
                'face_indices':list(UsdGeom.Mesh(prim).GetFaceVertexIndicesAttr().Get()),
                'source_layers':[{'path':str(v.layer.realPath),'prim':str(v.path)} for v in prim.GetPrimStack()],
                'material_subsets':[{'path':str(q.GetPath()),'face_indices':list(UsdGeom.Subset(q).GetIndicesAttr().Get()),
                    'material':str(UsdShade.MaterialBindingAPI(q).ComputeBoundMaterial()[0].GetPath())}
                    for q in prim.GetChildren() if q.GetTypeName()=='GeomSubset']})
    (output/'native_finger_visual_audit.json').write_text(json.dumps(records))
    return {'status':'STL registration pending native-pad geometry audit','mesh_records':len(records)}


def add_updated_tip_visuals(stage, cfg, output):
    """Register flat mounting face and distal tip; retain stock collision for now."""
    import hashlib
    import struct
    from pathlib import Path
    from pxr import Gf, Sdf, UsdGeom, UsdShade

    spec = cfg['final_layout'].get('fingertip_visual')
    if spec is None:
        return None
    path = Path(__file__).parent/spec['asset']
    data = path.read_bytes()
    if hashlib.sha256(data).hexdigest() != spec['sha256']:
        raise RuntimeError('Unexpected supplied fingertip mesh')
    count = struct.unpack_from('<I',data,80)[0]
    if len(data) != 84+50*count:
        raise ValueError('Expected binary STL')
    points = []
    for i in range(count):
        points.extend([Gf.Vec3f(*p) for p in struct.iter_unpack('<fff',data[84+50*i+12:84+50*i+48])])
    root = '/World/envs/env_0/Robot'
    mat = UsdShade.Material.Define(stage, root+'/UpdatedTipVisualMaterial')
    shader = UsdShade.Shader.Define(stage,str(mat.GetPath())+'/Surface')
    shader.CreateIdAttr('UsdPreviewSurface')
    shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(.025,.026,.028))
    shader.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.65)
    shader.CreateInput('metallic',Sdf.ValueTypeNames.Float).Set(0.)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface')
    reports = []
    for name, angle, tx, ty in (('fr3_leftfinger',90.,-.0155,.01090669),
                                ('fr3_rightfinger',-90.,.0155,-.01090669)):
        parent = root+'/'+name
        original = UsdGeom.Mesh(stage.GetPrimAtPath(parent+'/visuals'))
        subset = UsdGeom.Subset(stage.GetPrimAtPath(parent+'/visuals/material_1'))
        pad_faces = list(subset.GetIndicesAttr().Get())
        if len(pad_faces) != 262 or len(original.GetFaceVertexCountsAttr().Get()) != 624:
            raise RuntimeError('Native combined finger/pad geometry changed; refusing broad visual removal')
        # Mesh holes suppress only the old pad triangles; white finger is kept.
        original.CreateHoleIndicesAttr(pad_faces)
        tip = UsdGeom.Xform.Define(stage,parent+'/UpdatedTipSTLVisual')
        transform = UsdGeom.Xformable(tip)
        transform.AddTranslateOp().Set(Gf.Vec3d(tx,ty,spec['distal_tip_finger_z_m']-.0276))
        transform.AddRotateZOp().Set(angle)
        transform.AddScaleOp().Set(Gf.Vec3f(*([spec['assumed_stl_unit_m']]*3)))
        mesh = UsdGeom.Mesh.Define(stage,str(tip.GetPath())+'/Mesh')
        mesh.CreatePointsAttr(points)
        mesh.CreateFaceVertexCountsAttr([3]*count)
        mesh.CreateFaceVertexIndicesAttr(list(range(3*count)))
        mesh.CreateSubdivisionSchemeAttr('none');mesh.CreateDoubleSidedAttr(True)
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
        tip.GetPrim().CreateAttribute('registrationStatus',Sdf.ValueTypeNames.String).Set(spec['status'])
        finger_inverse = UsdGeom.Xformable(stage.GetPrimAtPath(parent)).ComputeLocalToWorldTransform(0).GetInverse()
        xf = UsdGeom.Xformable(mesh).ComputeLocalToWorldTransform(0)*finger_inverse
        registered = [list(xf.Transform(Gf.Vec3d(v))) for v in points]
        bounds = [[min(v[i] for v in registered) for i in range(3)],
                  [max(v[i] for v in registered) for i in range(3)]]
        if abs(bounds[1][2]-spec['distal_tip_finger_z_m']) > 1e-6:
            raise RuntimeError('Registered tip distal alignment failed')
        reports.append({'bounds_finger_m':bounds,'finger':name,'prim':str(tip.GetPath()),'removed_stock_pad_faces':len(pad_faces),
            'retained_white_finger_faces':624-len(pad_faces),'rotation_z_deg':angle,
            'translation_finger_m':[tx,ty,spec['distal_tip_finger_z_m']-.0276],
            'scale':spec['assumed_stl_unit_m'],'triangles':count})
    report = {'status':spec['status'],'sha256':spec['sha256'],'instances':reports,
        'collision_status':'Stock pad collision and friction unchanged; new STL contact geometry not yet validated',
        'registration':'CAD X0 broad mounting face to stock pad backplane; CAD Y width centered; distal tips aligned',
        'proximal_extension_vs_stock_m':.03609037-(.00205+spec['distal_tip_finger_z_m']-.0276),
        'material_status':'Illustrative dark pad finish; physical material/color unknown'}
    (output/'updated_fingertips.json').write_text(json.dumps(report,indent=2)+'\n')
    return report
