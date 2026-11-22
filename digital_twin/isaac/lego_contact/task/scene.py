"""Author a separate, explicitly unverified visual plate/stack USD candidate.

The existing workcell is an optional read-only sublayer. No collision, rigid
body, joint, tool command or engagement model is created by this authoring tool.
The public CLI supervises Isaac's fast-shutdown worker and checks its audit.
"""
import argparse
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys

try:
    from .stack_layout import LayoutError, compile_world_poses, generate_layout
except ImportError:
    from stack_layout import LayoutError, compile_world_poses, generate_layout


ROOT = "/World/LegoSeatingTaskCandidate"
FALLBACK_PRIORS = {
    "stud_diameter_m": .0048, "stud_height_m": .0017,
    "source": "https://github.com/intelligent-control-lab/BrickSim/blob/cbd5de3238e6ac12f44ef699e1507a9f16bdafc3/native/modules/bricksim/core/specs.cppm",
    "status": "nominal CAD visual priors only; not physical measurements or contact coefficients"
}


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def build_spec(config, registration):
    """Pure geometry/provenance preparation; never invent a physical transform."""
    layout = generate_layout(config)
    actual = config["baseplate"].get("T_O_plate")
    transform = actual if actual is not None else registration.get("T_O_P_display_proxy")
    if transform is None:
        raise LayoutError("no supplied physical transform or explicitly marked display proxy")
    display_config = copy.deepcopy(config)
    display_config["baseplate"]["T_O_plate"] = transform
    world = compile_world_poses(display_config, layout, require_verified=False)
    priors = copy.deepcopy(config.get("visual_geometry_priors", FALLBACK_PRIORS))
    for key in ("stud_diameter_m", "stud_height_m"):
        value = priors.get(key)
        if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or value <= 0:
            raise LayoutError("positive nominal visual geometry prior required: " + key)
    body = config["stack"].get("brick_body_size_m")
    if not isinstance(body, list) or len(body) != 3 or any(
            isinstance(v, bool) or not isinstance(v, (float, int)) or not math.isfinite(v) or v <= 0 for v in body):
        raise LayoutError("explicit operator-supplied brick body dimensions required")
    return {"layout": layout, "world": world, "T_O_P_display": transform,
            "transform_status": "supplied transform, verification independent" if actual is not None else "photo paper-plane display proxy; physical seating Z unknown",
            "visual_priors": priors, "brick_body_size_m": body,
            "physical_thickness_m": config["baseplate"].get("thickness_m"),
            "physical_support": config["baseplate"].get("mounting_fixation"),
            "physical_registration_verified": False, "contact_enabled": False,
            "contact_disabled_reason": "plate thickness, seating height, fixation/compliance and engagement model remain unvalidated; a visual plane/stud mesh is not support/contact physics"}


def author(stage, spec):
    from pxr import Gf, Sdf, UsdGeom
    if stage.GetPrimAtPath(ROOT):
        raise LayoutError("base stage already contains candidate root; choose a clean base")
    UsdGeom.Xform.Define(stage, "/World")
    frame = UsdGeom.Xform.Define(stage, ROOT)
    matrix = spec["T_O_P_display"]
    frame.AddTransformOp().Set(Gf.Matrix4d(*[matrix[j][i] for i in range(4) for j in range(4)]))
    prim = frame.GetPrim()
    for name, value in (("contactEnabled", False), ("supportEnabled", False), ("groundCollisionEnabled", False),
                        ("physicalRegistrationVerified", False), ("trainingReady", False), ("toolCommandGenerated", False)):
        prim.CreateAttribute("seating:"+name, Sdf.ValueTypeNames.Bool).Set(value)
    for name, value in (("transformStatus", spec["transform_status"]), ("contactDisabledReason", spec["contact_disabled_reason"]),
                        ("visualGeometryPriors", json.dumps(spec["visual_priors"])),
                        ("unknownPhysicalInputs", json.dumps({"plate_thickness_m": spec["physical_thickness_m"],
                                                               "mounting_support": spec["physical_support"]})),
                        ("toolConstraint", "world-vertical EE only; this stage contains no robot tool commands")):
        prim.CreateAttribute("seating:"+name, Sdf.ValueTypeNames.String).Set(value)
    plate = UsdGeom.Xform.Define(stage, ROOT+"/Plate")
    pitch = spec["layout"]["stud_pitch_m"]
    cols, rows = spec["layout"]["grid_shape"]
    surface = UsdGeom.Mesh.Define(stage, str(plate.GetPath())+"/VisualSurface")
    surface.CreatePointsAttr([Gf.Vec3f(-pitch/2,-pitch/2,0), Gf.Vec3f((cols-.5)*pitch,-pitch/2,0),
                             Gf.Vec3f((cols-.5)*pitch,(rows-.5)*pitch,0), Gf.Vec3f(-pitch/2,(rows-.5)*pitch,0)])
    surface.CreateFaceVertexCountsAttr([4]); surface.CreateFaceVertexIndicesAttr([0,1,2,3])
    surface.CreateDoubleSidedAttr(True); surface.CreateSubdivisionSchemeAttr("none")
    surface.CreateDisplayColorAttr([Gf.Vec3f(.95,.95,.95)])
    surface.GetPrim().CreateAttribute("seating:extentMeaning", Sdf.ValueTypeNames.String).Set(
        "nominal grid-cell visual extent only; no authored thickness or measured physical edge")
    radius = spec["visual_priors"]["stud_diameter_m"]/2
    stud_height = spec["visual_priors"]["stud_height_m"]
    def stud(path, center, color):
        cylinder = UsdGeom.Cylinder.Define(stage, path)
        cylinder.CreateRadiusAttr(radius); cylinder.CreateHeightAttr(stud_height)
        cylinder.CreateAxisAttr("Z")
        cylinder.AddTranslateOp().Set(Gf.Vec3d(*center))
        cylinder.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        return cylinder
    UsdGeom.Xform.Define(stage, str(plate.GetPath())+"/Studs")
    for i in range(cols):
        for j in range(rows):
            p = stud(str(plate.GetPath())+f"/Studs/p_{i}_{j}", (i*pitch,j*pitch,stud_height/2), (.95,.95,.95))
            p.GetPrim().CreateAttribute("seating:gridIndex", Sdf.ValueTypeNames.Int2).Set(Gf.Vec2i(i,j))
    UsdGeom.Xform.Define(stage, ROOT+"/Stack")
    length, width, height = spec["brick_body_size_m"]
    for brick in spec["layout"]["bricks"]:
        path = ROOT+"/Stack/"+brick["id"]
        frame = UsdGeom.Xform.Define(stage, path)
        frame.AddTranslateOp().Set(Gf.Vec3d(*brick["bottom_center_P_m"]))
        frame.AddRotateZOp().Set(math.degrees(brick["yaw_rad"]))
        frame.GetPrim().CreateAttribute("seating:brickIndex", Sdf.ValueTypeNames.Int3).Set(Gf.Vec3i(*brick["index"]))
        frame.GetPrim().CreateAttribute("seating:potentialStudCorrespondences", Sdf.ValueTypeNames.String).Set(json.dumps(brick["support"]))
        targets = [ROOT+"/Plate" if owner == "baseplate" else ROOT+"/Stack/"+owner for owner in brick["support"]["contributors"]]
        frame.GetPrim().CreateRelationship("seating:potentialSupports").SetTargets(targets)
        body = UsdGeom.Cube.Define(stage, path+"/Body")
        body.CreateSizeAttr(1.); body.AddTranslateOp().Set(Gf.Vec3d(0,0,height/2))
        body.AddScaleOp().Set(Gf.Vec3d(length,width,height))
        body.CreateDisplayColorAttr([Gf.Vec3f(.55,.55,.58)])
        body.GetPrim().CreateAttribute("seating:colorMeaning", Sdf.ValueTypeNames.String).Set("neutral preview style; no requested LEGO color asserted")
        UsdGeom.Xform.Define(stage, path+"/Studs")
        for v in range(2):
            for u in range(4):
                stud(path+f"/Studs/s_{u}_{v}", ((u-1.5)*pitch,(v-.5)*pitch,height+stud_height/2), (.55,.55,.58))


def audit(stage, spec):
    """Inspect actual parsed USD, not an authoring-side counter."""
    from pxr import Usd, UsdGeom, UsdPhysics
    root = stage.GetPrimAtPath(ROOT)
    if not root:
        raise LayoutError("parsed USD has no candidate root")
    prims = list(Usd.PrimRange(root))
    collisions = [str(p.GetPath()) for p in prims if p.HasAPI(UsdPhysics.CollisionAPI)]
    rigid = [str(p.GetPath()) for p in prims if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    joints = [str(p.GetPath()) for p in prims if p.IsA(UsdPhysics.Joint)]
    plate_studs = [p for p in prims if str(p.GetPath()).startswith(ROOT+"/Plate/Studs/") and p.IsA(UsdGeom.Cylinder)]
    brick_studs = [p for p in prims if str(p.GetPath()).startswith(ROOT+"/Stack/") and p.IsA(UsdGeom.Cylinder)]
    expected_plate = math.prod(spec["layout"]["grid_shape"])
    if collisions or rigid or joints or len(plate_studs) != expected_plate or len(brick_studs) != 8*len(spec["layout"]["bricks"]):
        raise LayoutError("parsed candidate collider/joint/count audit failed")
    cache = UsdGeom.XformCache()
    expected = {p["id"]: p for p in spec["world"]["poses"]}
    maximum_error = maximum_rotation_error = 0.
    for brick in spec["layout"]["bricks"]:
        p = stage.GetPrimAtPath(ROOT+"/Stack/"+brick["id"])
        if not p:
            raise LayoutError("missing brick prim: "+brick["id"])
        matrix = cache.GetLocalToWorldTransform(p)
        world = matrix.ExtractTranslation()
        target = expected[brick["id"]]
        maximum_error = max(maximum_error, max(abs(world[i]-target["brick_bottom_center_O_m"][i]) for i in range(3)))
        maximum_rotation_error = max(maximum_rotation_error, max(
            abs(matrix[j][i]-target["R_O_brick"][i][j]) for i in range(3) for j in range(3)))
        support = json.loads(p.GetAttribute("seating:potentialStudCorrespondences").Get())
        if support != brick["support"]:
            raise LayoutError("parsed potential support metadata mismatch")
        actual_targets = sorted(map(str, p.GetRelationship("seating:potentialSupports").GetTargets()))
        target_paths = sorted(ROOT+"/Plate" if owner == "baseplate" else ROOT+"/Stack/"+owner
                              for owner in support["contributors"])
        if actual_targets != target_paths:
            raise LayoutError("parsed nominal support relationships mismatch")
    if maximum_error > 1e-8 or maximum_rotation_error > 1e-8:
        raise LayoutError("world-frame placement disagrees with supplied transform (base /World must be identity)")
    return {"passed": True, "plate_visual_studs": len(plate_studs), "brick_count": len(expected),
            "brick_visual_studs": len(brick_studs), "candidate_collision_prims": collisions,
            "candidate_rigid_body_prims": rigid, "candidate_joint_prims": joints,
            "maximum_brick_world_position_error_m": maximum_error,
            "maximum_brick_world_rotation_element_error": maximum_rotation_error,
            "potential_support_metadata_matches_layout": True,
            "contact_enabled": False, "physical_registration_verified": False, "training_ready": False}


def worker(args):
    report = {"passed": False, "contact_enabled": False, "physical_registration_verified": False, "training_ready": False}
    app = None
    base_hash = digest(args.base_stage) if args.base_stage else None
    try:
        source_files = (args.config, args.registration, Path(__file__), Path(__file__).with_name("stack_layout.py"))
        source_hashes = {str(p): digest(p) for p in source_files}
        config = json.loads(args.config.read_text()); registration = json.loads(args.registration.read_text())
        spec = build_spec(config, registration)
        from isaacsim import SimulationApp
        kit = Path(__file__).resolve().parents[1]/"physics_only.kit"
        app = SimulationApp({"headless": True, "enable_crashreporter": False}, experience=str(kit))
        from pxr import Usd, UsdGeom
        stage_path = args.output/"scene_candidate.usdc"
        stage = Usd.Stage.CreateNew(str(stage_path))
        if args.base_stage:
            stage.GetRootLayer().subLayerPaths = [str(args.base_stage.resolve())]
        UsdGeom.SetStageMetersPerUnit(stage, 1.); UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        author(stage, spec)
        stage.GetRootLayer().Save()
        parsed = Usd.Stage.Open(str(stage_path))
        parsed.GetRootLayer().Reload()
        report.update(audit(parsed, spec))
        report.update({"scene_path": str(stage_path), "scene_sha256": digest(stage_path),
                       "display_transform_status": spec["transform_status"],
                       "T_O_P_display": spec["T_O_P_display"], "visual_geometry_priors": spec["visual_priors"],
                       "physical_plate_thickness_m": spec["physical_thickness_m"],
                       "physical_support": spec["physical_support"],
                       "contact_disabled_reason": spec["contact_disabled_reason"],
                       "support_summary": spec["layout"]["summary"],
                       "source_sha256": source_hashes,
                       "original_base_stage": str(args.base_stage) if args.base_stage else None,
                       "original_base_sha256_before": base_hash,
                       "original_base_sha256_after": digest(args.base_stage) if args.base_stage else None,
                       "hardware_clients": 0, "tool_commands": 0, "simulation_steps": 0})
        if args.base_stage and report["original_base_sha256_after"] != base_hash:
            raise LayoutError("original base stage hash changed")
        if {str(p): digest(p) for p in source_files} != source_hashes:
            raise LayoutError("source/config changed during authoring; candidate evidence is invalid")
        (args.output/"layout.json").write_text(json.dumps(spec["layout"], indent=2)+"\n")
    except Exception as error:
        report.update(passed=False, error=f"{type(error).__name__}: {error}")
    (args.output/"scene_audit.json").write_text(json.dumps(report, indent=2, allow_nan=False)+"\n")
    print(json.dumps(report), flush=True)
    if app is not None:
        app.close(wait_for_replicator=False)
    return 0 if report["passed"] else 2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=Path(__file__).with_name("config.json"))
    parser.add_argument("--registration", type=Path, default=Path(__file__).with_name("plate_registration_candidate.json"))
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--base-stage", type=Path)
    parser.add_argument("--_worker", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args._worker:
        return worker(args)
    args.output.mkdir(parents=True, exist_ok=False)
    command = [sys.executable, str(Path(__file__).resolve()), *sys.argv[1:], "--_worker"]
    with (args.output/"isaac-author.log").open("w") as log:
        child = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            status = child.wait(timeout=35)
        except subprocess.TimeoutExpired:
            os.killpg(child.pid, signal.SIGKILL); child.wait()
            print("Recoverable stop: bounded Isaac authoring timed out", file=sys.stderr)
            return 2
    result = args.output/"scene_audit.json"
    if status != 0 or not result.exists():
        print(f"Recoverable stop: authoring worker status={status}; inspect {args.output/'isaac-author.log'}", file=sys.stderr)
        return 2
    report = json.loads(result.read_text())
    print(json.dumps({key: report.get(key) for key in ("passed", "plate_visual_studs", "brick_count", "contact_enabled", "training_ready")}))
    return 0 if report.get("passed") else 2


if __name__ == "__main__":
    raise SystemExit(main())
