"""Hand-attached C920 visual proxy. No optical, collision or rigid-body model."""
import math


def add_c920(stage, spec, hand_path):
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade, UsdPhysics

    root = hand_path+'/C920ApproximateVisualMount'
    assembly = UsdGeom.Xform.Define(stage, root)
    assembly.GetPrim().CreateAttribute('calibrationStatus', Sdf.ValueTypeNames.String).Set(spec['status'])
    assembly.GetPrim().CreateAttribute('opticalTransformAvailable', Sdf.ValueTypeNames.Bool).Set(False)
    assembly.GetPrim().CreateAttribute('visualOnly', Sdf.ValueTypeNames.Bool).Set(True)
    mats = {}
    for name, color, rough in [('plastic', (.018,.020,.023), .43),
                               ('face', (.035,.039,.044), .30),
                               ('speaker', (.008,.009,.010), .75),
                               ('glass', (.009,.020,.028), .10),
                               ('tape', (.025,.025,.027), .58),
                               ('trim', (.18,.19,.20), .32)]:
        material = UsdShade.Material.Define(stage, root+'/Materials/'+name)
        shader = UsdShade.Shader.Define(stage, str(material.GetPath())+'/Surface')
        shader.CreateIdAttr('UsdPreviewSurface')
        shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
        shader.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(rough)
        shader.CreateInput('metallic', Sdf.ValueTypeNames.Float).Set(0.)
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
        mats[name] = material

    def bind(prim, name):
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(mats[name])

    def box(path, pos, size, material):
        cube = UsdGeom.Cube.Define(stage, path)
        cube.CreateSizeAttr(1.)
        xf = UsdGeom.Xformable(cube)
        xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
        xf.AddScaleOp().Set(Gf.Vec3d(*size))
        bind(cube.GetPrim(), material)

    # Laminated leaves share a plane: a fully folded stand, not an open tripod.
    depth, width, length = spec['stand_size_depth_width_length_m']
    x = spec['stand_center_hand_x_m']
    box(root+'/FoldedStand/InnerLeaf', (x,0,length/2), (depth,width,length), 'plastic')
    box(root+'/FoldedStand/OuterLeaf', (x-depth*.85,0,length/2),
        (depth*.65,width*.94,length*.94), 'plastic')
    for i,z in enumerate(spec['tape_centers_hand_z_m']):
        box(root+f'/Tape/Band{i}', (x-depth*1.25,0,z), (0.0018,width+.012,.009), 'tape')
        for sign in (-1,1):
            box(root+f'/Tape/Return{i}_{sign+1}', (x+.008,sign*(width/2+.005),z),
                (.019,.0018,.009), 'tape')

    hinge = spec['hinge_position_hand_m']
    cylinder = UsdGeom.Cylinder.Define(stage, root+'/Hinge')
    cylinder.CreateAxisAttr('Y'); cylinder.CreateRadiusAttr(.0045); cylinder.CreateHeightAttr(width*.85)
    UsdGeom.Xformable(cylinder).AddTranslateOp().Set(Gf.Vec3d(*hinge))
    bind(cylinder.GetPrim(), 'plastic')
    body = UsdGeom.Xform.Define(stage, root+'/TiltedBody')
    xf = UsdGeom.Xformable(body)
    xf.AddTranslateOp().Set(Gf.Vec3d(*hinge))
    xf.AddRotateYOp().Set(spec['illustrative_forward_hinge_deg'])
    center = spec['body_center_from_hinge_local_m']
    body_geometry = UsdGeom.Xform.Define(stage, str(body.GetPath())+'/Housing')
    UsdGeom.Xformable(body_geometry).AddTranslateOp().Set(Gf.Vec3d(*center))
    bpath = str(body_geometry.GetPath())
    depth,width,height = spec['body_size_depth_width_height_m']
    # Rounded rectangular extrusion in Y/Z; outward camera fascia is local -X.
    radius = .006
    outline = []
    for cy,cz,start in [(width/2-radius,height/2-radius,0),
                        (-width/2+radius,height/2-radius,90),
                        (-width/2+radius,-height/2+radius,180),
                        (width/2-radius,-height/2+radius,270)]:
        for j in range(5):
            angle = math.radians(start+j*22.5)
            outline.append((cy+radius*math.cos(angle),cz+radius*math.sin(angle)))
    n = len(outline)
    points = [Gf.Vec3f(side*depth/2,y,z) for side in (-1,1) for y,z in outline]
    faces = [list(reversed(range(n))),list(range(n,2*n))]
    faces += [[i,(i+1)%n,(i+1)%n+n,i+n] for i in range(n)]
    mesh = UsdGeom.Mesh.Define(stage,bpath+'/Shell')
    mesh.CreatePointsAttr(points); mesh.CreateFaceVertexCountsAttr([len(f) for f in faces])
    mesh.CreateFaceVertexIndicesAttr([v for f in faces for v in f]); mesh.CreateSubdivisionSchemeAttr('none')
    mesh.CreateDoubleSidedAttr(True); bind(mesh.GetPrim(),'plastic')
    box(bpath+'/Fascia', (-depth/2-.0004,0,0),(.001,width*.84,height*.78),'face')
    for sign in (-1,1):
        box(bpath+f'/Speaker{sign+1}',(-depth/2-.001,sign*width*.335,0),(.0015,.020,.018),'speaker')
    for name,radius,front,material in [('LensRing',.011,-depth/2-.0015,'trim'),
                                     ('LensGlass',.0087,-depth/2-.003,'glass')]:
        lens = UsdGeom.Cylinder.Define(stage,bpath+'/'+name)
        lens.CreateAxisAttr('X'); lens.CreateRadiusAttr(radius); lens.CreateHeightAttr(.0018)
        UsdGeom.Xformable(lens).AddTranslateOp().Set(Gf.Vec3d(front,0,0)); bind(lens.GetPrim(),material)
    descendants = list(Usd.PrimRange(assembly.GetPrim()))
    visual_only = all(not p.HasAPI(UsdPhysics.RigidBodyAPI) and not p.HasAPI(UsdPhysics.CollisionAPI)
                      and not p.HasAPI(UsdPhysics.MassAPI) and p.GetTypeName() != 'Camera'
                      for p in descendants)
    if not visual_only:
        raise RuntimeError('C920 must remain visual-only')
    pitch = math.radians(spec['illustrative_forward_hinge_deg'])
    return {'prim':root,'type':'Logitech C920 approximate visual proxy','hand_side':'-X; D435i remains +X',
        'lateral_center_hand_y_m':0.,'stand_armward_end_hand_z_m':0.,
        'wrist_face_contact':'nominal hand-origin wrist-face plane only; no measured surface contact',
        'folded_stand':True,'body_stand_angle_deg':spec['illustrative_forward_hinge_deg'],
        'decorative_fascia_normal_hand':[-math.cos(pitch),0.,math.sin(pitch)],
        'angle_is_mechanical_limit_measurement':False,'optical_transform':None,
        'optical_camera_activated':False,'collision_or_mass_added':False,'visual_only_audit_passed':visual_only,'configuration':spec}
