"""Small final-layout adapter around the existing FR3/BrickSim scene factory."""

import math


def make_env_cfg(cfg, capture=False):
    from challenge2.env_cfg import make_env_cfg as existing_factory
    from isaaclab.sim import PreviewSurfaceCfg

    env = existing_factory(cfg, capture=capture)
    layout = cfg["final_layout"]
    table, robot = layout["table"], layout["robot"]
    env.scene.table.spawn.size = tuple(table["size_m"])
    env.scene.table.spawn.visual_material = PreviewSurfaceCfg(diffuse_color=(.86, .86, .83))
    env.scene.table.init_state.pos = tuple(table["center_table_m"])
    env.scene.robot.init_state.pos = tuple(robot["base_table_m"])
    angle = math.radians(robot["yaw_table_deg"])
    env.scene.robot.init_state.rot = (math.cos(angle/2), 0., 0., math.sin(angle/2))
    env.scene.plate_source.spawn.color = (235, 235, 230)
    env.scene.plate_mixed.spawn.dimensions = (layout["target"]["studs"],)*2+(1,)
    env.scene.plate_mixed.spawn.color = (150, 150, 150)
    env.scene.plate_mixed.init_state.pos = tuple(layout["target"]["center_table_m"])
    # The photographed final layout uses one target plate and a round bowl.
    for name in list(vars(env.scene)):
        if name.startswith("scrap_"):
            delattr(env.scene, name)
    return env


def decorate_stage(stage, cfg):
    from pxr import UsdGeom, UsdShade, UsdPhysics, Sdf, Gf

    layout = cfg["final_layout"]
    def material(name, color):
        value = UsdShade.Material.Define(stage, f"/World/FinalMaterials/{name}")
        shader = UsdShade.Shader.Define(stage, f"{value.GetPath()}/PreviewSurface")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(.65)
        value.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        return value
    black, yellow, paper, bowl_mat = [material(name, color) for name, color in (
        ("black_tape", (.025, .025, .025)), ("yellow_tape", (.82, .96, .015)),
        ("paper_placeholder", (.96, .96, .93)), ("cream_bowl", (.87, .84, .74)))]
    def box(name, center, size, mat):
        prim = UsdGeom.Cube.Define(stage, f"/World/FinalLayout/{name}")
        prim.CreateSizeAttr(1.)
        transform = UsdGeom.Xformable(prim.GetPrim())
        transform.AddTranslateOp().Set(Gf.Vec3d(*center))
        transform.AddScaleOp().Set(Gf.Vec3d(*size))
        UsdShade.MaterialBindingAPI.Apply(prim.GetPrim()).Bind(mat)
        return prim
    for prefix, center, studs in (("source", (0., 0.), layout["source"]["studs"]),
                                  ("target", layout["target"]["center_table_m"][:2], layout["target"]["studs"])):
        width = studs*.008
        plate = layout[prefix]
        tape_width = plate["black_tape_width_m"]
        # Tape intrudes exactly two8mm stud rows on each photographed Y edge.
        tape_center_offset = width/2-plate["black_tape_covered_rows_each_y_edge"]*.008+tape_width/2
        for sign in (-1, 1):
            box(f"{prefix}_black_border_{sign+1}", (center[0], center[1]+sign*tape_center_offset, .0052),
                (width+.07, tape_width, .0003), black)
    target = layout["target"]
    x, y, _ = target["center_table_m"]
    studs, tape = target["studs"], target["tape_width_m"]
    cut_x, cut_y = target["zone_cut_studs"]
    local_x, local_y = (cut_x-studs/2)*.008, (cut_y-studs/2)*.008
    box("target_T_crossbar", (x, y+local_y, .0052), (studs*.008, tape, .0003), yellow)
    stem_length = cut_y*.008
    box("target_T_stem_toward_robot", (x+local_x, y-studs*.008/2+stem_length/2, .0052),
        (tape, stem_length, .0003), yellow)
    board = layout["marker_board"]
    box("marker_paper_placeholder", board["center_table_m"], (*board["size_m"], .0002), paper)

    # Analytic open cup: inner/outer side walls, bottom and rim; NO top disk.
    bowl = layout["reject_bowl"]
    n, wall, height = bowl["segments"], bowl["wall_m"], bowl["height_m"]
    cx, cy, cz = bowl["center_table_m"]
    rings = [(bowl["bottom_radius_m"], 0.), (bowl["outer_radius_m"], height),
             (bowl["bottom_radius_m"]-wall, wall), (bowl["outer_radius_m"]-wall, height)]
    points = [Gf.Vec3f(cx+r*math.cos(2*math.pi*i/n), cy+r*math.sin(2*math.pi*i/n), cz+z)
              for r, z in rings for i in range(n)]
    faces = []
    for i in range(n):
        j = (i+1) % n
        faces.extend(([i, j, n+j, n+i], [2*n+j, 2*n+i, 3*n+i, 3*n+j],
                      [n+i, n+j, 3*n+j, 3*n+i], [j, i, 2*n+i, 2*n+j]))
    faces.extend((list(reversed(range(n))), list(range(2*n, 3*n))))
    mesh = UsdGeom.Mesh.Define(stage, "/World/FinalLayout/RejectBowl")
    mesh.CreatePointsAttr(points)
    mesh.CreateFaceVertexCountsAttr([len(face) for face in faces])
    mesh.CreateFaceVertexIndicesAttr([index for face in faces for index in face])
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateDoubleSidedAttr(True)
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(bowl_mat)
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr().Set("none")
    wrist = layout["wrist_camera"]
    placeholder = UsdGeom.Xform.Define(stage, wrist["parent_prim"]+"/WristD435iCalibrationRequired")
    placeholder.GetPrim().CreateAttribute("calibrationStatus", Sdf.ValueTypeNames.String).Set(wrist["status"])
    placeholder.GetPrim().CreateAttribute("opticalTransformAvailable", Sdf.ValueTypeNames.Bool).Set(False)
    diagnostic = stage.GetPrimAtPath("/World/Overview")
    diagnostic.CreateAttribute("sensorPurpose", Sdf.ValueTypeNames.String).Set("Virtual diagnostic inspection only; not the physical wrist camera")
    return {"source_studs": layout["source"]["studs"], "target_studs": target["studs"],
        "target_zones": target["zones"], "routing_implemented": False, "target_empty": True,
        "reject_bowl_open_top": True, "bowl_collision_validation": "unvalidated static approximation",
        "wrist_camera_config_count": 1, "wrist_optical_transform_available": False,
        "external_physical_cameras_modeled": 0, "diagnostic_cameras_are_virtual": True,
        "layout_estimates": layout}
