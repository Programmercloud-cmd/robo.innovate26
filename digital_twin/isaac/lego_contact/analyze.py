"""Summarize the six bounded native gate diagnostics without transfer claims."""
import argparse
import hashlib
import json
from pathlib import Path

root=Path(__file__).resolve().parent
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument("--runs-root",type=Path,default=root/"evidence" if (root/"evidence").is_dir() else root)
parser.add_argument("--output",type=Path,default=root/"diagnostic_report.json")
args=parser.parse_args()
cases=["no_force","aligned_load","misaligned_load","too_high","yaw_error","assembly_disabled"]
results={}
for case in cases:
    run=args.runs_root/("clean-"+case)
    result=json.loads((run/"result.json").read_text())
    configuration=json.loads((run/"configuration.json").read_text())
    rows=[json.loads(line) for line in (run/"trace.ndjson").read_text().splitlines()]
    log=(args.runs_root/("clean-"+case+".log")).read_text()
    assert result["completed"] and len(rows)==600 and "[Fatal]" not in log and "[Error]" not in log
    assert result["assembly_events"] == (1 if case=="aligned_load" else 0), case
    assert result["final_connected"] == (case=="aligned_load"), case
    result["all_bricks_ungripped"]=True
    result["upper_gravity_disabled"]=case=="too_high"
    result["breakage_enabled"]=configuration["breakage"]["enabled"]
    result["assembly_enabled"]=configuration["assembly"]["enabled"]
    result["maximum_reported_compressive_force_N"]=max([-d["projected_force"] for row in rows for d in row["assembly_debug"]],default=0.)
    result["lower_final_displacement_from_first_sample_m"]=[rows[-1]["lower_pose_xyzw"][i]-rows[0]["lower_pose_xyzw"][i] for i in range(3)]
    result["runtime_mass_kg"]=json.loads((run/"runtime_mass_kg.json").read_text())
    events=[]
    for i,row in enumerate(rows):
        if row["assembled"]:
            before=rows[i-1]
            before_z=before["upper_pose_xyzw"][2]-before["lower_pose_xyzw"][2]
            after_z=row["upper_pose_xyzw"][2]-row["lower_pose_xyzw"][2]
            events.append({"step":row["step"],"t_s":row["t_s"],"relative_z_before_m":before_z,"relative_z_after_m":after_z,"relative_z_jump_m":after_z-before_z,"upper_vz_before_m_s":before["upper_velocity"][2],"upper_vz_after_m_s":row["upper_velocity"][2],"assembly_debug":row["assembly_debug"]})
    result["attachment_transitions"]=events
    result["artifacts"]={name:{"path":str(run/name),"sha256":hashlib.sha256((run/name).read_bytes()).hexdigest()} for name in ["configuration.json","trace.ndjson","result.json","runtime_mass_kg.json","scene.usdc"]}
    results[case]=result
native_revision="cbd5de3238e6ac12f44ef699e1507a9f16bdafc3"
refs={"two_solid_box_colliders_and_visual_shell_tubes":"native/modules/bricksim/usd/author.cppm#L196-L258",
      "force_and_geometric_assembly_gate":"native/modules/bricksim/physx/assembly.cppm#L110-L127",
      "contact_impulse_divided_by_dt":"native/modules/bricksim/physx/physics_graph.cppm#L598-L644",
      "connected_component_contact_filter":"native/modules/bricksim/physx/physics_graph.cppm#L382-L390",
      "custom_six_dof_weld":"native/modules/bricksim/physx/weld_constraint.cppm#L108-L157",
      "native_assembly_usd_writeback":"native/modules/bricksim/omni/usd_physics_bridge.cppm#L343-L370",
      "usd_graph_component_alignment":"native/modules/bricksim/usd/usd_graph.cppm#L330-L362",
      "usd_graph_pose_write":"native/modules/bricksim/usd/usd_graph.cppm#L590-L615"}
report={"training_readiness":False,"readiness_reason":"Native assembly is a force/geometric gate followed by a pose snap and rigid weld; no elastic stud/tube interference is simulated or calibrated.",
        "scope":"One nominal upstream 2x4 pair, ungripped, lower dynamic on generic static support. Upper gravity disabled only in separated too_high control. No robot, real-frame placement, real fixture, controller, training or hardware IO.",
        "diagnostic_source_sha256":{name:hashlib.sha256((root/name).read_bytes()).hexdigest() for name in ["contact_probe.py","physics_only.kit","launch.sh","analyze.py"]},
        "native_revision":native_revision,"source_reference_base":"https://github.com/intelligent-control-lab/BrickSim/blob/"+native_revision+"/",
        "source_references":refs,"observed_cases":results,
        "important_defaults":{"breakage_enabled":False,"assembly_required_force_N":1.,"assembly_position_tolerance_m":.002,"assembly_max_penetration_m":.005,"clutch_preload_N":3.5,"clutch_friction_coefficient":.2,"provenance":"installed native defaults, unchanged and unmeasured; preload/friction do not establish tested pull-off while breakage is disabled"},
        "contact_representation":{"actual_colliders_per_brick":2,"collider_type":"solid body cuboid plus full-footprint stud-height cuboid","stud_shell_tube_geometry":"visual only; no contact or deformation","material_limit":"Friction/restitution attributes are declared on colliders but no PhysicsMaterial bindings are present. Effective runtime material coefficients were not identified by these probes."},
        "force_input_provenance":"2N directly applied to free upper brick during100–349ms as an algorithm threshold probe; not measured seating force, gripper contact or proposed robot command.",
        "energy_and_teleport_limit":"Approx1.7mm pose snap observed at attachment. No continuous physical insertion path or force-displacement work validated; energy injection is not quantified or ruled out by this trace.",
        "beneficial_fix":"Isolated authoring removes prototype float quaternion and creates double quaternion matching native graph writeback. Preliminary runs/logs remain archived; reported clean runs have no error/fatal entries.",
        "minimum_practical_model_next":"Keep native model explicitly as a logic baseline only. Later add measured shell/stud/tube collision geometry and identified compliant engagement/retention law, matched material bindings and supported-lower-layer fixture; disable unmeasured automatic snap for any force-displacement comparison. Gate training on physical identification/acceptance contract.",
        "real_parameter_identification_performed":False,"hardware_clients":0,"training_launched":False}
args.output.write_text(json.dumps(report,indent=2)+"\n")
print(json.dumps({"training_readiness":False,"cases":{case:{"assembled":r["assembly_events"],"final_z_m":r["final_relative_z_m"]} for case,r in results.items()},"aligned_attachment":results["aligned_load"]["attachment_transitions"]},indent=2))
