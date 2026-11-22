"""One-way measured-state mirroring into Isaac/PhysX. No hardware client exists here.

The TCP socket accepts only NDJSON measurements; it sends no replies or commands.
The simulated articulation is explicitly positioned from measured joints. This is
state playback, not an emulation of Franka's closed-source impedance controller.
"""
import argparse
import hashlib
import json
import math
import queue
import signal
import socket
import threading
import time
from pathlib import Path

ROOT = "/World/envs/env_0/Robot"


def validate_frame(frame):
    if frame.get("schema") != "fr3.telemetry.v1":
        raise ValueError("unsupported telemetry schema")
    for field in ("q", "dq"):
        if len(frame.get(field, [])) != 7 or not all(math.isfinite(float(v)) for v in frame[field]):
            raise ValueError("finite seven-joint " + field + " required")
    if not isinstance(frame.get("seq"), int) or not isinstance(frame.get("unix_ns"), int):
        raise ValueError("integer seq and unix_ns required")
    if frame.get("width_m") is not None and not 0 <= frame["width_m"] <= .09:
        raise ValueError("invalid measured gripper width")
    return frame


class Receiver:
    def __init__(self, port, output):
        self.latest = queue.Queue(maxsize=1)
        self.output = output
        self.stats = {"accepted": 0, "rejected": 0, "connections": 0}
        self.socket = socket.socket()
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind(("127.0.0.1", port))
        self.socket.listen(1)
        self.socket.settimeout(.5)
        self.stop = False
        threading.Thread(target=self.run, daemon=True).start()

    def run(self):
        with (self.output / "telemetry.ndjson").open("a", buffering=1) as log:
            while not self.stop:
                try:
                    client, _ = self.socket.accept()
                except socket.timeout:
                    continue
                self.stats["connections"] += 1
                client.settimeout(1.)
                buffer = b""
                last_seq = -1
                try:
                    with client:
                        while not self.stop:
                            try:
                                data = client.recv(65536)
                            except socket.timeout:
                                continue
                            if not data:
                                break
                            buffer += data
                            if len(buffer) > 1048576:
                                raise ValueError("telemetry line too large")
                            while b"\n" in buffer:
                                line, buffer = buffer.split(b"\n", 1)
                                try:
                                    frame = validate_frame(json.loads(line))
                                    if frame["seq"] <= last_seq:
                                        raise ValueError("sequence did not advance")
                                    last_seq = frame["seq"]
                                    received = time.monotonic()
                                    log.write(json.dumps(frame, separators=(",", ":")) + "\n")
                                    self.stats["accepted"] += 1
                                    if self.latest.full():
                                        self.latest.get_nowait()
                                    self.latest.put_nowait((received, frame))
                                except (ValueError, TypeError, KeyError):
                                    self.stats["rejected"] += 1
                except (OSError, ValueError):
                    self.stats["rejected"] += 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--snapshot", type=Path, default=Path("/home/ubuntu/workspace/robo26-scene-smoke/20261026T171524Z-brev-scene-smoke"))
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--duration", type=float, default=0., help="0 means run until interrupted")
    parser.add_argument("--registration", type=Path)
    parser.add_argument("--brick-observations", type=Path, help="Independent perception-only JSON; never motion inventory")
    parser.add_argument("--replay", type=Path, help="recorded telemetry; never hardware commands")
    parser.add_argument("--initial-telemetry", type=Path,
                        help="Retain a measured snapshot across a viewer restart, preserving its original age")
    parser.add_argument("--urdf", type=Path, default=Path(__file__).resolve().parents[1] / "robot_model.urdf")
    parser.add_argument("--export-viewer", action="store_true")
    parser.add_argument("--environment", choices=("factory", "board"), default="factory",
                        help="Explicit scene selection; independent of the fingertip selection")
    parser.add_argument("--fingers", choices=("original", "wide"), default="original")
    parser.add_argument("--follow-twin", type=Path,
                        help="Read an existing twin active_session.json recording; no SDK connection")
    args = parser.parse_args()
    if args.follow_twin and (args.replay or args.initial_telemetry):
        parser.error("--follow-twin cannot be combined with replay or initial telemetry")
    args.output.mkdir(parents=True, exist_ok=False)
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True, "enable_crashreporter": False},
                        experience=str(Path(__file__).with_name("physics_only.kit")))
    import sys
    import numpy as np
    import omni.usd
    import omni.physx
    import omni.physics.tensors as tensors
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, PhysxSchema
    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageMetersPerUnit(stage, 1.)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(stage, "/World")
    UsdPhysics.Scene.Define(stage, "/World/Physics").CreateGravityMagnitudeAttr(0.)
    robot = stage.DefinePrim(ROOT, "Xform")
    robot.GetReferences().AddReference(str(args.snapshot / "assets/bricksim/robots/fr3/robot.usda"))
    sys.path.insert(0, str(args.snapshot))
    cfg = json.loads((args.snapshot / "experiments/final_workcell_20261024/scene_config.json").read_text())
    from experiments.final_workcell_20261024.scene import correct_wrist_mount_side
    from experiments.final_workcell_20261024.end_effector.fingertips import add_updated_tip_visuals
    from experiments.final_workcell_20261024.c920_mount.model import add_c920
    mount = correct_wrist_mount_side(stage, cfg)
    tips = add_updated_tip_visuals(stage, cfg, args.output)
    c920 = add_c920(stage, cfg["final_layout"]["additional_wrist_camera"], ROOT + "/fr3_hand")
    environment = None
    if args.environment != "factory" or args.fingers != "original" or args.follow_twin:
        from environments.geometry import apply
        environment = apply(stage, args.environment, args.fingers, args.output)
    # Mirroring measured state must not add synthetic motion through the drives.
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateDisableGravityAttr(True)
        for axis in ("angular", "linear"):
            drive = UsdPhysics.DriveAPI.Get(prim, axis)
            if drive:
                drive.CreateStiffnessAttr(0.)
                drive.CreateDampingAttr(0.)
    if args.registration:
        from scene_geometry import author_registered_scene
        registration = author_registered_scene(stage, json.loads(args.registration.read_text()))
    else:
        registration = {"status": "robot-base frame only; no invented tabletop registration"}
    brick_display = None
    if args.brick_observations:
        from brick_display import BrickDisplay
        brick_display = BrickDisplay(stage, args.brick_observations)
    for _ in range(3):
        app.update()
    physics = omni.physx.get_physx_interface()
    simulation = omni.physx.get_physx_simulation_interface()
    simulation.attach_stage(omni.usd.get_context().get_stage_id())
    physics.force_load_physics_from_usd()
    view = tensors.create_simulation_view("numpy")
    view.set_subspace_roots("/")
    articulation = view.create_articulation_view(ROOT)
    if articulation.count != 1:
        raise RuntimeError("expected exactly one PhysX FR3 articulation")
    meta = articulation.shared_metatype
    names = list(meta.dof_names)
    arm = [names.index("fr3_joint" + str(i)) for i in range(1, 8)]
    fingers = [names.index("fr3_finger_joint" + str(i)) for i in (1, 2)]
    if args.fingers == "wide":
        # Explicit simulated preview opening while hand measurements are absent.
        # This is never reported as a measured physical gripper width.
        positions = articulation.get_dof_positions().copy()
        positions[0, fingers] = environment["fingertips"]["preview_native_open_width_m"] / 2
        articulation.set_dof_positions(positions, np.array([0], dtype=np.uint32))
    link_names = list(meta.link_names)
    from kinematics import Model, pose_xyzw, difference
    model = Model(args.urdf)
    chain = []
    for i in range(1, 8):
        j = UsdPhysics.Joint(stage.GetPrimAtPath(ROOT + f"/fr3_link{i-1}/fr3_joint{i}"))
        chain.append({"name": f"fr3_joint{i}", "p0": list(j.GetLocalPos0Attr().Get()),
                      "p1": list(j.GetLocalPos1Attr().Get()),
                      "r0_wxyz": [j.GetLocalRot0Attr().Get().GetReal(), *j.GetLocalRot0Attr().Get().GetImaginary()],
                      "r1_wxyz": [j.GetLocalRot1Attr().Get().GetReal(), *j.GetLocalRot1Attr().Get().GetImaginary()]})
    manifest = {"mode": "measured_state_mirror", "hardware_commands": 0,
                "rendering": "disabled; Isaac5.1/driver595 RTX startup incompatible",
                "dof_names": names, "link_names": link_names, "chain": chain,
                "wrist_mount": mount, "tips": tips, "c920": c920,
                "registration": registration, "controller_equivalence": "not claimed; measured q and dq are imposed on PhysX",
                "width_semantics": "Franka reported width = sum of native finger joint coordinates; modified pad gap uncalibrated"}
    manifest["environment"] = environment or {"environment":"factory", "fingers":"original", "explicit_selection":True}
    if environment and environment.get("fingertips"):
        manifest["tips"] = {"base_tip_registration": tips,
                            "selected_variant": environment["fingertips"]}
    source_paths = [args.urdf, *Path(__file__).parent.glob("*.py"), Path(__file__).with_name("physics_only.kit"),
                    *Path(__file__).parent.glob("*.json"),
                    args.snapshot / "experiments/final_workcell_20261024/scene_config.json"]
    source_paths += [Path(layer.realPath) for layer in stage.GetUsedLayers() if layer.realPath]
    if args.registration:
        source_paths.append(args.registration)
    manifest["source_sha256"] = {str(path): hashlib.sha256(path.read_bytes()).hexdigest()
                                 for path in source_paths if path.is_file()}
    manifest["dynamics_limits"] = "State mirror only; inherited inertias/contact geometry and camera housing approximations are not physical identification or loaded-contact validation. No current camera optical calibration applied."
    calibration_status = Path(__file__).with_name("camera_calibration_status.json")
    if calibration_status.is_file():
        manifest["camera_calibration_status"] = json.loads(calibration_status.read_text())
    if args.export_viewer:
        from export_viewer import export
        manifest["webgl_visual_monitor"] = export(stage, link_names, ROOT, args.output)
    (args.output / "manifest.json").write_text(json.dumps(manifest, indent=2))
    if args.follow_twin:
        from environments.recording import RecordingFollower
        receiver = RecordingFollower(args.follow_twin, validate_frame)
    else:
        receiver = Receiver(args.port, args.output)
    stopping = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stopping.set())
    signal.signal(signal.SIGINT, lambda *_: stopping.set())
    replay = iter(args.replay.read_text().splitlines()) if args.replay else None
    print("FR3_TWIN_READY " + ("recording follower" if args.follow_twin else "127.0.0.1:" + str(args.port)), flush=True)
    start = time.monotonic()
    last = None
    if args.initial_telemetry:
        retained=validate_frame(json.loads(args.initial_telemetry.read_text()))
        age=max(0.,(time.time_ns()-retained['unix_ns'])/1e9)
        last=(time.monotonic()-age,retained)
        if retained.get('width_m') is not None:
            positions=articulation.get_dof_positions().copy()
            positions[0,fingers]=retained['width_m']/2
            articulation.set_dof_positions(positions,np.array([0],dtype=np.uint32))
    count = 0
    max_joint_error = 0.
    fk_errors = []
    measured_tcp_errors = []
    last_seq = None
    try:
        while not stopping.is_set() and (not args.duration or time.monotonic() - start < args.duration):
            tick = time.monotonic()
            if replay:
                try:
                    received, frame = tick, validate_frame(json.loads(next(replay)))
                except StopIteration:
                    break
            else:
                try:
                    received, frame = receiver.latest.get_nowait()
                except queue.Empty:
                    received, frame = last if last else (tick, None)
            if frame is not None:
                positions = articulation.get_dof_positions().copy()
                velocities = np.zeros_like(positions)
                positions[0, arm] = frame["q"]
                velocities[0, arm] = frame["dq"] if tick - received < .5 else 0.
                hand_age=frame.get('hand_age_s')
                hand_age=None if hand_age is None else hand_age+max(0.,tick-received)
                hand_stale=(frame.get('hand_stale',True) or hand_age is None or hand_age>=.5
                            or frame.get('width_m') is None)
                hand_profile_stop = None
                if not hand_stale:
                    if args.fingers == "wide" and not environment['fingertips']['minimum_native_width_m'] <= frame['width_m'] <= .08:
                        hand_profile_stop = 'Selected virtual cap cannot follow this physical hand width safely; retaining simulated opening. Select the matching finger profile manually.'
                    else:
                        positions[0, fingers] = frame["width_m"] / 2
                indices = np.array([0], dtype=np.uint32)
                articulation.set_dof_positions(positions, indices)
                articulation.set_dof_velocities(velocities, indices)
                readback = articulation.get_dof_positions()[0, arm].tolist()
                error = max(abs(a-b) for a, b in zip(readback, frame["q"]))
                max_joint_error = max(max_joint_error, error)
                physics.update_transformations(False, True, True, False)
                links = articulation.get_link_transforms()[0].tolist()
                flange = pose_xyzw(links[link_names.index("fr3_link8")])
                fk_error = difference(flange, model.flange(frame["q"]))
                tcp_error = None
                if frame.get("F_T_EE") is not None and frame.get("O_T_EE") is not None:
                    tcp_error = difference(flange@np.asarray(frame["F_T_EE"]).reshape(4, 4, order="F"),
                                           np.asarray(frame["O_T_EE"]).reshape(4, 4, order="F"))
                if frame["seq"] != last_seq:
                    fk_errors.append(fk_error)
                    if tcp_error:
                        measured_tcp_errors.append(tcp_error)
                    with (args.output / "applied.ndjson").open("a") as applied:
                        applied.write(json.dumps({"seq": frame["seq"], "q": readback,
                            "urdf_flange_error": fk_error, "measured_tcp_error": tcp_error})+"\n")
                    last_seq = frame["seq"]
                state = {"seq": frame["seq"], "unix_ns": frame["unix_ns"],
                         "published_unix_ms": time.time_ns()/1e6,
                         "source": frame.get("source", "recorded_replay" if args.replay else "live_telemetry"),
                         "q_d_measured": frame.get("q_d"), "width_m": None if hand_stale else frame.get("width_m"),
                         "measured_tcp_O_T_EE": frame.get("O_T_EE"),
                         "hand_stale":hand_stale,"hand_age_s":hand_age,
                         "width_unix_ns":frame.get('width_unix_ns'),"hand_source":frame.get('hand_source','unavailable'),
                         "robot_mode": frame.get("robot_mode"), "current_errors": frame.get("current_errors"),
                         "sample_age_s": tick - received, "stale": tick - received >= .5,
                         "q_measured": frame["q"], "q_isaac": readback,
                         "max_joint_error_rad": error,
                         "links_world_xyzw": links, "urdf_flange_error": fk_error,
                         "measured_tcp_error": tcp_error,
                         "applied_frames": count + 1, "receiver": receiver.stats}
                if brick_display:
                    state['brick_display'] = brick_display.status
                if args.fingers == "wide" and (hand_stale or hand_profile_stop):
                    state['simulated_hand_width_m'] = float(positions[0, fingers].sum())
                    state['simulated_hand_note'] = hand_profile_stop or 'Retained simulated opening; physical hand telemetry unavailable'
                temp = args.output / "latest.tmp"
                temp.write_text(json.dumps(state))
                temp.replace(args.output / "latest.json")
                last = (received, frame)
                count += 1
            if brick_display:
                brick_display.update(frame.get('O_T_EE') if frame else None)
            if environment and count == 1:
                from environments.snapshot import save
                save(stage, args.output / "digital_twin.usdc", link_names, links,
                     positions, arm, fingers, frame, hand_stale, hand_profile_stop)
            app.update()
            remaining = 1/60 - (time.monotonic() - tick)
            if remaining > 0:
                time.sleep(remaining)
    finally:
        receiver.stop = True
        if environment and count:
            from environments.snapshot import save
            save(stage, args.output / "digital_twin.usdc", link_names, links,
                 positions, arm, fingers, frame, hand_stale, hand_profile_stop)
        else:
            stage.Export(str(args.output / "digital_twin.usdc"))
        stage.Export(str(args.output / "digital_twin.usda"))
        (args.output / "result.json").write_text(json.dumps({"applied_frames": count,
            "max_joint_error_rad": max_joint_error, "receiver": receiver.stats,
            "distinct_samples_applied": len(fk_errors),
            "max_urdf_flange_error": {key: max((e[key] for e in fk_errors), default=None)
                                     for key in ("position_m", "rotation_rad")},
            "max_measured_tcp_error": {key: max((e[key] for e in measured_tcp_errors), default=None)
                                       for key in ("position_m", "rotation_rad")},
            "hardware_commands": 0, "render_validated": False}, indent=2))
        simulation.detach_stage()
        app.close(wait_for_replicator=False)


if __name__ == "__main__":
    main()
