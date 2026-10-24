"""Named camera views of one frozen, natively connected full360 FR3 workcell.

Run through the default Pueue queue. Physics advances only during the explicit
idle settling interval; every selected RGB view then shares the same state/time.
The panel is for display only. All simulator truth is separate from RGB files.
"""

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import random
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from challenge2.layout import Brick, HEIGHT, PITCH, connection_offset, overlaps


def load_geometry(path):
    spec = json.loads(Path(path).read_text())
    if spec["status"] != "confirmed_explicit_geometry":
        raise ValueError("Confirmed explicit full geometry is required; no provisional fallback")
    origins = spec["layer_origins_studs_xy"]
    if (spec["required_bricks"] != 360 or spec["layers"] != 10
            or spec["bricks_per_layer"] != [6, 6]
            or spec["brick_dimensions_studs"] != [4, 2, 3]
            or spec["pitch_studs_xy"] != [4, 4]
            or len(origins) != 10 or any(not isinstance(o, list) or len(o) != 2 for o in origins)
            or any(not isinstance(v, int) for o in origins for v in o)
            or spec["layer_yaws_quarter_turn"] != [k % 2 for k in range(10)]):
        raise ValueError("Full360 explicit alternating lattice specification is invalid")
    rng = random.Random(26)
    palette = ["dark_red", "brown", "tan", "reject_4211201", "reject_6135191"]
    border = spec["border_studs"]
    bricks = [Brick(f"brick_{k}_{i}_{j}", border+origins[k][0]+i*4,
                    border+origins[k][1]+j*4, k, k % 2, rng.choice(palette))
              for k in range(10) for i in range(6) for j in range(6)]
    centers = []
    for layer in range(10):
        group = [b for b in bricks if b.layer == layer]
        left, bottom = min(b.x for b in group), min(b.y for b in group)
        right = max(b.x+b.footprint[0] for b in group)
        top = max(b.y+b.footprint[1] for b in group)
        centers.append(((left+right)/2, (bottom+top)/2))
        if not (0 <= left < right <= spec["baseplate_studs"] and 0 <= bottom < top <= spec["baseplate_studs"]):
            raise ValueError("Full layer does not fit on the configured studded baseplate")
    if any(center != centers[0] for center in centers) or centers[0] != (spec["baseplate_studs"]/2,)*2:
        raise ValueError("Confirmed layers must share the centered baseplate footprint")
    for brick in bricks:
        if brick.layer and not any(a.layer == brick.layer-1 and overlaps(a, brick) for a in bricks):
            raise ValueError(f"Unsupported source brick {brick.name}")
    return spec, bricks


def normalize(vector):
    length = math.sqrt(sum(v*v for v in vector))
    if length < 1e-12:
        raise ValueError("Cannot orient camera from a zero vector")
    return tuple(v/length for v in vector)


def cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])


def matrix_quaternion(matrix):
    """Proper rotation matrix to normalized WXYZ, without importing Isaac."""
    trace = sum(matrix[i][i] for i in range(3))
    if trace > 0:
        scale = math.sqrt(trace+1)*2
        q = (scale/4, (matrix[2][1]-matrix[1][2])/scale,
             (matrix[0][2]-matrix[2][0])/scale, (matrix[1][0]-matrix[0][1])/scale)
    else:
        i = max(range(3), key=lambda index: matrix[index][index])
        j, k = (i+1) % 3, (i+2) % 3
        scale = math.sqrt(1+matrix[i][i]-matrix[j][j]-matrix[k][k])*2
        values = [0., 0., 0., 0.]
        values[0] = (matrix[k][j]-matrix[j][k])/scale
        values[i+1] = scale/4
        values[j+1] = (matrix[j][i]+matrix[i][j])/scale
        values[k+1] = (matrix[k][i]+matrix[i][k])/scale
        q = tuple(values)
    return normalize(q)


def camera_pose(center, view):
    eye = tuple(c+d for c, d in zip(center, view["eye_offset_m"]))
    target = tuple(c+d for c, d in zip(center, view["target_offset_m"]))
    forward = normalize(tuple(t-e for e, t in zip(eye, target)))
    up = normalize(tuple(view["up_world"]))
    fallback = abs(sum(a*b for a, b in zip(forward, up))) > .999
    if fallback:
        up = min(((0., 1., 0.), (1., 0., 0.), (0., 0., 1.)),
                 key=lambda axis: abs(sum(a*b for a, b in zip(forward, axis))))
    right = normalize(cross(forward, up))
    actual_up = cross(right, forward)
    # OpenGL/Usd.Camera looks along -Z, with +Y image-up.
    matrix = [[right[i], actual_up[i], -forward[i]] for i in range(3)]
    return {"eye_world_m": eye, "target_world_m": target,
            "quaternion_wxyz_opengl": matrix_quaternion(matrix), "up_fallback_used": fallback,
            "rotation_camera_to_world": matrix}


def selected_views(config, selector):
    views = config["views"]
    identifiers = [view["id"] for view in views]
    if len(identifiers) != len(set(identifiers)):
        raise ValueError("Camera view IDs must be unique")
    requested = identifiers if selector == "all" else selector.split(",")
    if not requested or len(requested) != len(set(requested)) or any(name not in identifiers for name in requested):
        raise ValueError("--views must be all or a comma-separated selection of configured IDs")
    return [next(view for view in views if view["id"] == name) for name in requested]


def make_panel(output, views, config):
    from PIL import Image, ImageDraw
    tile_width, tile_height = config["panel"]["thumbnail_size"]
    caption = 24
    columns = min(config["panel"]["columns"], len(views))
    rows = math.ceil(len(views)/columns)
    panel = Image.new("RGB", (columns*tile_width, rows*(tile_height+caption)+28), (245, 245, 245))
    draw = ImageDraw.Draw(panel)
    draw.text((8, 8), "Same frozen physics state; independent cameras; display panel only", fill=(0, 0, 0))
    for index, view in enumerate(views):
        x, y = index % columns*tile_width, 28+index//columns*(tile_height+caption)
        with Image.open(output/"images"/f"{view['id']}.png") as source:
            panel.paste(source.convert("RGB").resize((tile_width, tile_height)), (x, y))
        draw.text((x+8, y+tile_height+5), view["id"], fill=(0, 0, 0))
    panel.save(output/"panel.png")


def self_test():
    for eye, up in (((0, 0, .36), (0, 1, 0)), ((0, 0, .36), (0, 0, 1)),
                    ((.18, 0, .32), (0, 0, 1)), ((0, 0, -.36), (0, 1, 0))):
        pose = camera_pose((0., 0., 0.), {"eye_offset_m": eye, "target_offset_m": (0, 0, 0), "up_world": up})
        assert abs(sum(v*v for v in pose["quaternion_wxyz_opengl"])-1) < 1e-10
        rotation = pose["rotation_camera_to_world"]
        w, x, y, z = pose["quaternion_wxyz_opengl"]
        reconstructed = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                         [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                         [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
        assert max(abs(rotation[i][j]-reconstructed[i][j]) for i in range(3) for j in range(3)) < 1e-10
        toward_target = normalize(tuple(-v for v in eye))
        assert max(abs(-rotation[i][2]-toward_target[i]) for i in range(3)) < 1e-10
        for i in range(3):
            for j in range(3):
                assert abs(sum(rotation[k][i]*rotation[k][j] for k in range(3))-(1 if i == j else 0)) < 1e-10
    assert camera_pose((0, 0, 0), {"eye_offset_m": (0, 0, 1), "target_offset_m": (0, 0, 0),
                                "up_world": (0, 0, 1)})["up_fallback_used"]
    print("camera pose tiny contracts passed")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--geometry", type=Path)
    parser.add_argument("--cameras", type=Path)
    parser.add_argument("--views", default="all")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--self-test", action="store_true")
    if "--self-test" in sys.argv:
        parser.parse_args()
        self_test()
        return
    if os.environ.get("ROBO_PUEUE_JOB") != "1":
        parser.error("Render jobs must use the frozen default Pueue queue")
    from isaaclab.app import AppLauncher
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    if args.geometry is None or args.cameras is None or args.output is None:
        parser.error("--geometry, --cameras and --output are required")
    if not args.enable_cameras:
        parser.error("--enable_cameras is required")
    spec, bricks = load_geometry(args.geometry)
    config = json.loads(args.cameras.read_text())
    views = selected_views(config, args.views)
    output = args.output
    (output/"images").mkdir(parents=True, exist_ok=True)
    (output/"evaluation_truth").mkdir(exist_ok=True)
    app = AppLauncher(args).app
    try:
        render(app, args, spec, bricks, config, views)
    finally:
        app.close()


def render(app, args, spec, bricks, config, views):
    import numpy as np
    import torch
    from PIL import Image
    from pxr import UsdGeom, UsdLux, Gf
    from isaaclab.scene import InteractiveScene
    from isaaclab.sim import SimulationContext, get_current_stage
    from isaaclab.utils.math import matrix_from_quat
    from bricksim.core import create_connection, compute_connected_component, get_disassembled_connections
    from challenge2.env_cfg import make_env_cfg

    output = args.output
    width, height = config["image_size"]
    cfg = json.loads((ROOT/"configs/workcell.json").read_text())
    cfg.update(source_cells=6, source_layers=10, baseplate_studs=spec["baseplate_studs"], source_xy=spec["source_xy_m"])
    env_cfg = make_env_cfg(cfg, capture=True)
    configured_positions, configured_quaternions = [], []
    for brick in bricks:
        asset = getattr(env_cfg.scene, brick.name)
        asset.init_state.pos, asset.init_state.rot = brick.pose(cfg["source_xy"], spec["baseplate_studs"])
        asset.spawn.color = tuple(cfg["colors"][brick.color])
        configured_positions.append(asset.init_state.pos)
        configured_quaternions.append(asset.init_state.rot)
    camera_cfg = env_cfg.scene.overview
    camera_cfg.width, camera_cfg.height = width, height
    camera_cfg.data_types = ["rgb", "distance_to_image_plane"]
    camera_cfg.update_latest_camera_pose = True
    camera_cfg.spawn.focal_length = config["focal_length_mm"]
    camera_cfg.spawn.horizontal_aperture = config["horizontal_aperture_mm"]
    camera_cfg.spawn.clipping_range = tuple(config["clipping_range_m"])
    sim = SimulationContext(env_cfg.sim)
    sim.get_physics_context().set_physx_update_transformations_settings(update_to_usd=True)
    scene = InteractiveScene(env_cfg.scene)
    stage = get_current_stage()
    sim.reset()
    scene.reset()
    # Installed omni.rtx.settings.core exposes this process-local auto-exposure
    # switch. Do not modify shared Kit files or global workstation settings.
    import carb.settings
    render_settings = carb.settings.get_settings()
    render_settings.set_bool("/rtx/post/histogram/enabled", False)
    render_setting_paths = ("/rtx/post/histogram/enabled", "/rtx/post/tonemap/op",
        "/rtx/post/tonemap/filmIso", "/rtx/post/tonemap/exposureTime",
        "/rtx/post/tonemap/fNumber", "/rtx/post/tonemap/cm2Factor")
    resolved_settings = {path: render_settings.get(path) for path in render_setting_paths}
    robot = scene["robot"]
    robot.write_joint_state_to_sim(robot.data.default_joint_pos, robot.data.default_joint_vel)
    robot.set_joint_position_target(robot.data.default_joint_pos)
    expected_connections = 0
    for brick in bricks:
        lower = [a for a in bricks if a.layer == brick.layer-1 and overlaps(a, brick)] if brick.layer else [None]
        for support in lower:
            offset, yaw = connection_offset(support, brick, spec["baseplate_studs"])
            create_connection(f"/World/envs/env_0/{support.name if support else 'plate_source'}", 1,
                              f"/World/envs/env_0/{brick.name}", 0, offset, yaw)
            expected_connections += 1
    source_path = "/World/envs/env_0/plate_source"
    parts_initial, connections_initial = compute_connected_component(source_path)
    if len(parts_initial) != 361 or len(connections_initial) != expected_connections:
        raise RuntimeError("Full360 source graph is incomplete before settling")
    for _ in range(config["settle_steps"]):
        scene.write_data_to_sim()
        sim.step()
        scene.update(sim.get_physics_dt())
    if get_disassembled_connections(clear=True):
        raise RuntimeError("Native connections broke during idle settling; refusing reduced benchmark")
    parts, connections = compute_connected_component(source_path)
    if set(parts) != set(parts_initial) or set(connections) != set(connections_initial):
        raise RuntimeError("Full native source graph changed during settling")

    roots = torch.stack([scene[brick.name].data.root_state_w[0] for brick in bricks]).cpu().numpy()
    position_drift = np.linalg.norm(roots[:, :3]-np.asarray(configured_positions), axis=1)
    quat_dot = np.clip(np.abs((roots[:, 3:7]*np.asarray(configured_quaternions)).sum(axis=1)), 0, 1)
    settle_audit = {"steps": config["settle_steps"], "max_position_drift_m": float(position_drift.max()),
        "max_orientation_drift_rad": float((2*np.arccos(quat_dot)).max()),
        "max_linear_speed_m_s": float(np.linalg.norm(roots[:, 7:10], axis=1).max()),
        "max_angular_speed_rad_s": float(np.linalg.norm(roots[:, 10:13], axis=1).max()),
        "note": "Idle settling audit; finite steps do not establish mechanical equilibrium or calibrated clutch forces"}
    center = np.mean([scene[b.name].data.root_pos_w[0].cpu().numpy() for b in bricks if b.layer == 9], axis=0)
    center[2] += HEIGHT+.0017
    lighting = config["lighting"]
    dome = UsdLux.DomeLight.Get(stage, "/World/Light")
    dome.GetIntensityAttr().Set(lighting["dome_intensity"])
    key = UsdLux.SphereLight.Define(stage, "/World/ExperimentKey")
    key.CreateRadiusAttr(lighting["key_radius_m"])
    key.GetIntensityAttr().Set(lighting["key_intensity"])
    UsdGeom.Xformable(key.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(*(center+np.asarray(lighting["key_offset_m"]))))
    camera = scene["overview"]

    def physics_state():
        return torch.cat([robot.data.joint_pos.flatten(), robot.data.joint_vel.flatten(),
            robot.data.body_state_w.flatten(), scene["plate_source"].data.root_state_w.flatten(),
            *(scene[brick.name].data.root_state_w.flatten() for brick in bricks)]).clone()

    frozen = physics_state()
    frozen_time = float(sim.current_time)
    np.save(output/"evaluation_truth/frozen_physics_state.npy", frozen.cpu().numpy())
    truth_images, camera_records = [], []
    for view in views:
        pose = camera_pose(center.tolist(), view)
        camera.set_world_poses(positions=torch.tensor([pose["eye_world_m"]], dtype=torch.float32),
            orientations=torch.tensor([pose["quaternion_wxyz_opengl"]], dtype=torch.float32), convention="opengl")
        camera.reset()
        for _ in range(config["render_warmup_frames"]):
            sim.render()
        camera.update(0., force_recompute=True)
        if float(sim.current_time) != frozen_time or not torch.equal(physics_state(), frozen):
            raise RuntimeError("Physics state/time changed during render-only camera capture")
        rgb = camera.data.output["rgb"][0, :, :, :3].cpu().numpy()
        depth = camera.data.output["distance_to_image_plane"][0, :, :, 0].cpu().numpy()
        if rgb.shape != (height, width, 3) or rgb.dtype != np.uint8:
            raise RuntimeError("Unexpected RGB shape/type; refusing silent conversion")
        if rgb.max() == rgb.min():
            raise RuntimeError("Uniform RGB frame; refusing to call it a useful scene render")
        if any(render_settings.get(path) != value for path, value in resolved_settings.items()):
            raise RuntimeError("Resolved exposure/tone-mapping settings changed between camera views")
        actual_eye = camera.data.pos_w[0].cpu().numpy()
        actual_quaternion = camera.data.quat_w_opengl[0].cpu().numpy()
        if np.max(np.abs(actual_eye-np.asarray(pose["eye_world_m"]))) > 1e-6 or 1-abs(float(actual_quaternion@np.asarray(pose["quaternion_wxyz_opengl"]))) > 1e-5:
            raise RuntimeError("Actual camera pose does not match the named preset")
        Image.fromarray(rgb).save(output/"images"/f"{view['id']}.png")
        np.save(output/"evaluation_truth"/f"{view['id']}_depth.npy", depth)
        camera_to_world = UsdGeom.Xformable(stage.GetPrimAtPath("/World/Overview")).ComputeLocalToWorldTransform(0)
        world_to_camera = camera_to_world.GetInverse()
        intrinsics = camera.data.intrinsic_matrices[0].cpu().numpy()

        def project(points):
            values = []
            for point in points:
                q = world_to_camera.Transform(Gf.Vec3d(*[float(v) for v in point]))
                z = -float(q[2])
                values.append([float(intrinsics[0, 0]*q[0]/z+intrinsics[0, 2]),
                               float(-intrinsics[1, 1]*q[1]/z+intrinsics[1, 2]), z])
            return values

        instances = []
        for brick in bricks:
            data = scene[brick.name].data
            position = data.root_pos_w[0].cpu().numpy()
            rotation = matrix_from_quat(data.root_quat_w[0:1])[0].cpu().numpy()
            corners = np.array([[-.016, -.008, HEIGHT], [.016, -.008, HEIGHT],
                                [.016, .008, HEIGHT], [-.016, .008, HEIGHT]])
            studs = np.array([[(i-1.5)*PITCH, (j-.5)*PITCH, HEIGHT+.0017] for i in range(4) for j in range(2)])
            polygon, stud_pixels = project(corners@rotation.T+position), project(studs@rotation.T+position)
            visible = []
            for u, v, z in stud_pixels:
                ix, iy = int(round(u)), int(round(v))
                visible.append(bool(z > 0 and 0 <= ix < width and 0 <= iy < height
                    and np.isfinite(depth[iy, ix]) and abs(float(depth[iy, ix])-z) < .002))
            fraction = sum(visible)/8
            inside = all(z > 0 and 0 <= u < width and 0 <= v < height for u, v, z in polygon)
            role = "not_visible" if fraction == 0 else "exposed_top" if brick.layer == 9 and inside and fraction >= .75 else "visible_lower" if brick.layer < 9 else "ambiguous"
            instances.append({"id": brick.name, "layer": brick.layer, "color": brick.color,
                "polygon": [point[:2] for point in polygon], "role": role,
                "orientation_deg": math.degrees(math.atan2(polygon[1][1]-polygon[0][1], polygon[1][0]-polygon[0][0])) % 180,
                "visible_stud_fraction_8_samples": fraction,
                "studs": [{"x": p[0], "y": p[1], "visible": vis} for p, vis in zip(stud_pixels, visible)]})
        camera_record = {"id": view["id"], "requested": view, **pose, "intrinsics": intrinsics.tolist(),
            "actual_eye_world_m": actual_eye.tolist(), "actual_quaternion_wxyz_opengl": actual_quaternion.tolist(),
            "resolved_render_settings": resolved_settings, "rgb_standard_deviation": float(rgb.std()),
            "camera_to_world_opengl": [[float(camera_to_world[i][j]) for j in range(4)] for i in range(4)],
            "physics_time_s": frozen_time, "camera_matching": "approximate; no measured physical intrinsics/extrinsics"}
        camera_records.append(camera_record)
        truth_images.append({"file": f"{view['id']}.png", "width": width, "height": height,
            "instances": instances, "intrinsics": intrinsics.tolist(), "physics_time_s": frozen_time,
            "correlation_group": "one_full360_frozen_sim_scene"})
        print("RENDER_SAVED", view["id"], "physics_time", frozen_time, flush=True)

    if get_disassembled_connections(clear=True):
        raise RuntimeError("Native connections broke during camera capture")
    parts_after, connections_after = compute_connected_component(source_path)
    if set(parts_after) != set(parts) or set(connections_after) != set(connections):
        raise RuntimeError("Full360 graph changed during camera capture")
    if float(sim.current_time) != frozen_time or not torch.equal(physics_state(), frozen):
        raise RuntimeError("Final physics state/time differs from the shared capture snapshot")
    truth = output/"evaluation_truth"
    (truth/"annotations.json").write_text(json.dumps({"images": truth_images,
        "visibility_method": "Eight depth-tested stud samples, not pixel-exact instance masks or gripper accessibility"}, indent=2))
    (truth/"cameras.json").write_text(json.dumps(camera_records, indent=2))
    (truth/"native_graph.json").write_text(json.dumps({"parts": parts_after, "connections": connections_after}, indent=2))
    stage.GetRootLayer().Export(str(truth/"full360_workcell.usda"))
    make_panel(output, views, config)
    manifest = {"geometry": spec, "selected_views": [v["id"] for v in views], "camera_config": config,
        "brick_count": 360, "native_connected_parts": len(parts_after), "native_connections": len(connections_after),
        "settling": settle_audit, "physics_time_s": frozen_time, "all_capture_states_exactly_equal": True,
        "resolved_render_settings": resolved_settings,
        "capture_semantics": "Sequential camera relocation/render-only at one physics snapshot; no detector fusion",
        "rgb_directory": "images", "truth_directory": "evaluation_truth", "panel_display_only": True,
        "frozen_state_sha256": hashlib.sha256(frozen.cpu().numpy().tobytes()).hexdigest(),
        "gpu": torch.cuda.get_device_name(0) if torch.cuda.is_available() else None,
        "camera_matching": "Approximate nominal pinhole; not calibrated D435i equivalence"}
    (output/"manifest.json").write_text(json.dumps(manifest, indent=2))
    sim.stop()
    (output/"run_complete.json").write_text(json.dumps({"completed": True, "views": len(views), "bricks": 360}))


if __name__ == "__main__":
    main()
