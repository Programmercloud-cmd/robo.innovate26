"""Recompute task prerequisites; prior synthetic checks cannot authorize PPO.

This is an evidence audit, not an assertion that an unimplemented model/env can
be enabled by editing a boolean. A new validated full task plant must be
implemented and integrated before this audit can issue training readiness.
"""
import argparse
import hashlib
import json
from pathlib import Path
try:
    from .stack_layout import generate_layout,compile_world_poses
    from .observations import CameraCalibration
    from .controller_replay import evaluate as replay_evaluate
except ImportError:
    from stack_layout import generate_layout,compile_world_poses
    from observations import CameraCalibration
    from controller_replay import evaluate as replay_evaluate


def assess(config,path):
    directory=path.resolve().parent
    checks=[]
    def add(name,passed,detail,evidence=None):
        checks.append({"requirement":name,"passed":bool(passed),"detail":detail,"evidence":evidence})
    layout=generate_layout(config)
    add("specified_stack_geometry",True,layout["summary"],"stack_layout.py")
    try:
        compile_world_poses(config,layout)
        registered=True
    except ValueError as error:
        registered=False
        registration_error=str(error)
    add("physical_plate_registration",registered,registration_error if not registered else "Verified grid and rigid physical seating datum supplied")
    plate=config["baseplate"]
    add("plate_fixture_thickness_compliance",False,
        {"missing_values":[key for key in ("thickness_m","mounting_fixation","support_compliance") if plate.get(key) is None],
         "missing_validation":"No held-out physical fixture/support validator is integrated; visible geometry or edited fields cannot establish contact behavior"})
    spec=config["observation"]["camera"]
    try:
        camera=CameraCalibration(**{**spec,"image_size":tuple(spec["image_size"])})
        camera.validate(require_mount=True)
        camera_valid=True
        reason="Camera packet is internally valid; independent physical optical validation is still required"
    except (ValueError,TypeError,RuntimeError) as error:
        camera_valid=False
        reason=str(error)
    add("logitech_packet_schema_and_profile",camera_valid,reason,"camera_evidence_audit.json")
    add("logitech_physical_optical_mount_validation",False,
        "Archived calibration was rejected; no independently verified current Logitech optical/mount/settings calibration artifact is available","camera_evidence_audit.json")
    add("measured_detector_visibility_latency",False,
        "No physical Logitech detector error/visibility/latency profile has been identified and held-out validated")
    add("grasp_transform_repeatability_slip",False,
        "No current single-brick visible transform or validated transform/slip bound; lower-stud detections alone are insufficient")
    contract_path=(directory/config["control"]["existing_contract"]).resolve()
    contract=json.loads(contract_path.read_text())
    add("world_vertical_actions",config["control"]["action_components"]==["x","y","z","yaw"] and not config["control"]["roll_pitch_actions"],
        "task/control.py rejects tilted nominal/task yaw axes and measured tilt; no implicit plate-normal tool rotation")
    add("commissioned_control_sensing_limits",False,
        {"missing_limits":[key for key,value in contract["control_contract"].items() if value is None],
         "missing_validation":"Force/wrench bias/noise/delay and loaded controller response are uncommissioned; target guards are not actuator validation"},str(contract_path))
    manifest_name=config["control"].get("controller_replay_manifest")
    physical_replay=False
    replay_detail="No recorded matched physical/simulation owner-stream replay manifest"
    if manifest_name:
        manifest_path=directory/manifest_name
        try:
            replay=replay_evaluate(json.loads(manifest_path.read_text()),manifest_path.parent)
            physical_replay=replay["physical_actuator_replay_pass"]
            replay_detail=replay
        except (ValueError,RuntimeError,KeyError,OSError,TypeError) as error:
            replay_detail=str(error)
    add("physical_joint_impedance_response_replay",physical_replay,replay_detail)
    add("task_contact_capture_jam_hysteresis",False,
        "Only axial contact is implemented; no validated XY/yaw capture, rate-independent hysteresis, partial engagement or jam model")
    add("all_support_patterns_physically_validated",False,
        {"required":config["contact"]["required_support_patterns"],"cantilever_bricks":layout["summary"]["cantilever_validation_bricks"],
         "reason":"Full-support pair evidence cannot validate 2-stud cantilevers or multi-brick bridges"})
    add("independent_seating_retention_success",False,
        "Independent outcome protocol is implemented, but no physical retained-engagement records or calibrated reward/classifier validation exist; low Z/force drop/weld are insufficient")
    add("full_task_timestep_solver_sensitivity",False,
        "Existing two-timestep synthetic axial tests do not validate the full registered multi-support task")
    add("matched_executable_rl_plant",False,
        "No identified FR3 servo/contact/observation plant is integrated; do not substitute ideal Cartesian control or the kinematic viewer")
    add("bounded_training_checkpoint_evaluation",False,
        "No policy job launched/checkpoint produced: essential model and observation gates fail. VM compute is available, not the blocker","vm_audit.json")
    artifacts={}
    for name in ("config.json","plate_registration_candidate.json","camera_evidence_audit.json","vm_audit.json","stack_layout.py","control.py","observations.py","controller_replay.py","stud_detector.py","contact_outcomes.py","owner_recorder.hpp","normalize_owner_recording.py"):
        file=directory/name
        if file.is_file():artifacts[name]=hashlib.sha256(file.read_bytes()).hexdigest()
    return {"schema":"lego_seating_task_evidence_audit_v1","training_ready":False,"hardware_deployment_enabled":False,
            "checks":checks,"open_requirements":[c["requirement"] for c in checks if not c["passed"]],
            "layout_summary":layout["summary"],"artifact_sha256":artifacts,
            "physical_data_missing":True,"trained_checkpoint":None,
            "warning":"Some checks deliberately remain unimplemented. This audit has no boolean-only promotion path; it must be extended with recomputed physical validators and a real matched plant before training."}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config",type=Path,default=Path(__file__).with_name("config.json"))
    parser.add_argument("--output",required=True,type=Path)
    args=parser.parse_args()
    report=assess(json.loads(args.config.read_text()),args.config)
    args.output.write_text(json.dumps(report,indent=2)+"\n")
    print(json.dumps({"training_ready":report["training_ready"],"open_requirements":report["open_requirements"]}))
    return 0 if report["training_ready"] else 2


if __name__=="__main__":
    raise SystemExit(main())
