"""Compare recorded owner-stream response with matched dynamic simulation.

No SDK connection, reader, controller, time-shifting or resampling is performed.
Samples are captured before command application in both domains: sent_q is the
command returned for that sample's next control interval. Tool/wrench conditions
must match. A matching trace establishes response replay, not contact readiness.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path


INTERFACE = "joint_positions_joint_impedance"
CONDITIONS = ("sdk_version", "tool_profile_id", "payload_profile_id",
              "impedance_profile_id", "limiter_profile_id", "filter_profile_id",
              "wrench_profile_id", "fixture_grasp_id", "control_period_s", "sample_phase")
TOLERANCES = ("maximum_sample_gap_s", "time_alignment_s", "command_match_rad",
              "joint_rmse_rad", "joint_max_error_rad", "joint_velocity_rmse_rad_s",
              "force_rmse_N", "torque_rmse_Nm", "minimum_command_excursion_rad")
ARRAYS = {"requested_q": 7, "sent_q": 7, "desired_q": 7, "measured_q": 7,
          "measured_dq": 7, "wrench_K": 6, "F_T_EE": 16, "EE_T_K": 16}


class ReplayStop(ValueError):
    """Recoverable data/interface stop; no readiness or inventory update."""


def number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ReplayStop(f"finite numeric {name} required")
    return float(value)


def validate_rows(rows, maximum_gap):
    if len(rows) < 3:
        raise ReplayStop("at least three owner-stream samples required")
    output = []
    old = None
    for row in rows:
        clean = {key: [number(v, key) for v in row[key]] for key in ARRAYS}
        if any(len(clean[key]) != length for key, length in ARRAYS.items()):
            raise ReplayStop("joint/wrench/frame array dimensions invalid")
        clean["t_s"] = number(row["t_s"], "episode-relative t_s")
        for key in ("wrench_state_t_s","wrench_filter_latency_s","wrench_nominal_force_t_s"):
            clean[key]=number(row.get(key),key)
        if (clean["wrench_filter_latency_s"]<0
                or abs(clean["wrench_state_t_s"]-clean["t_s"])>1e-9
                or abs(clean["wrench_nominal_force_t_s"]-(clean["t_s"]-clean["wrench_filter_latency_s"]))>1e-9):
            raise ReplayStop("inconsistent wrench sample/filter timing; no hidden time shift")
        sequence = row["sequence"]
        dropped = row["dropped_samples_since_last"]
        if (isinstance(sequence, bool) or not isinstance(sequence, int) or sequence < 0
                or isinstance(dropped, bool) or not isinstance(dropped, int) or dropped != 0):
            raise ReplayStop("valid sequence and zero accounted dropped samples required")
        clean["sequence"] = sequence
        if clean["t_s"] < 0:
            raise ReplayStop("episode-relative time cannot be negative")
        if any(row.get(key) is not True for key in ("wrench_valid", "singularity_valid", "grasp_valid")) or row.get("robot_fault") is not False:
            raise ReplayStop("fault/uncertain grasp/invalid wrench or singularity sample")
        if old is not None:
            if sequence <= old["sequence"] or not 0 < clean["t_s"]-old["t_s"] <= maximum_gap:
                raise ReplayStop("nonprogressing samples or excessive gap; no silent resampling")
            if clean["F_T_EE"] != old["F_T_EE"] or clean["EE_T_K"] != old["EE_T_K"]:
                raise ReplayStop("tool/stiffness frame changed during trial")
        old = clean
        output.append(clean)
    return output


def rmse_axes(a, b, key):
    return [math.sqrt(sum((x[key][j]-y[key][j])**2 for x, y in zip(a, b))/len(a))
            for j in range(len(a[0][key]))]


def within_trace(rows, source, destination):
    return max(abs(row[source][j]-row[destination][j]) for row in rows for j in range(7))


def evaluate(manifest, root):
    report = {"schema": 1, "actuator_replay_pass": False, "physical_actuator_replay_pass": False,
              "training_ready": False, "hardware_deployment_enabled": False,
              "scope": "matched command-schedule response only; no independent contact, retention, sensor or latency identification", "missing": []}
    if manifest.get("schema") != 1 or manifest.get("controller_interface") != INTERFACE:
        raise ReplayStop("joint_positions_joint_impedance interface required")
    reference, simulation = manifest["reference"], manifest["simulation"]
    if reference.get("actuation_method") != "franka_joint_impedance" or simulation.get("actuation_method") != "dynamic_joint_impedance_surrogate":
        raise ReplayStop("q overwrite, ideal Cartesian actuation or mismatched control interface refused")
    if reference.get("provenance") not in ("physical", "synthetic") or simulation.get("provenance") != "simulation":
        raise ReplayStop("explicit physical/synthetic reference and simulation provenance required")
    for side, trial in (("reference", reference), ("simulation", simulation)):
        conditions = trial.get("conditions", {})
        report["missing"].extend(f"{side}.conditions.{key}" for key in CONDITIONS if conditions.get(key) in (None, ""))
    tolerances = manifest.get("tolerances", {})
    report["missing"].extend(f"tolerances.{key}" for key in TOLERANCES if tolerances.get(key) is None)
    if report["missing"]:
        report["status"] = "uncommissioned"
        return report
    if reference["conditions"] != simulation["conditions"]:
        raise ReplayStop("payload/impedance/limiter/filter/wrench/fixture conditions differ")
    if reference["conditions"]["sample_phase"] != "before_command_application":
        raise ReplayStop("sample/command chronology must be declared before_command_application")
    period = number(reference["conditions"]["control_period_s"], "control period")
    tol = {key: number(tolerances[key], key) for key in TOLERANCES}
    if period <= 0 or tol["maximum_sample_gap_s"] <= 0 or tol["minimum_command_excursion_rad"] <= 0 or any(value < 0 for value in tol.values()):
        raise ReplayStop("positive control period/sample gap and nonnegative tolerances required")
    paths = [root/trial["path"] for trial in (reference, simulation)]
    raw = [[json.loads(line) for line in path.read_text().splitlines() if line.strip()] for path in paths]
    real_rows, sim_rows = [validate_rows(rows, tol["maximum_sample_gap_s"]) for rows in raw]
    if len(real_rows) != len(sim_rows):
        raise ReplayStop("sample counts differ; cannot silently discard unmatched data")
    time_error = max(abs(a["t_s"]-b["t_s"]) for a, b in zip(real_rows, sim_rows))
    if time_error > tol["time_alignment_s"] or any(a["sequence"] != b["sequence"] for a, b in zip(real_rows, sim_rows)):
        raise ReplayStop("replay sampling/command schedule mismatch; do not shift to hide latency")
    if any(a["F_T_EE"] != b["F_T_EE"] or a["EE_T_K"] != b["EE_T_K"] for a, b in zip(real_rows, sim_rows)):
        raise ReplayStop("reference/simulation frame mismatch")
    if any(abs(a["wrench_filter_latency_s"]-b["wrench_filter_latency_s"])>tol["time_alignment_s"] for a,b in zip(real_rows,sim_rows)):
        raise ReplayStop("reference/simulation wrench filter latency mismatch")
    command_error = max(abs(a[key][j]-b[key][j]) for a, b in zip(real_rows, sim_rows)
                        for key in ("requested_q", "sent_q") for j in range(7))
    if command_error > tol["command_match_rad"]:
        raise ReplayStop("replay command inputs differ; response errors would not be comparable")
    excursion = [max(row["sent_q"][j] for row in real_rows)-min(row["sent_q"][j] for row in real_rows) for j in range(7)]
    if max(excursion) < tol["minimum_command_excursion_rad"]:
        raise ReplayStop("stationary command replay cannot validate moving actuator response")
    q_error = rmse_axes(real_rows, sim_rows, "measured_q")
    dq_error = rmse_axes(real_rows, sim_rows, "measured_dq")
    wrench_error = rmse_axes(real_rows, sim_rows, "wrench_K")
    maximum_q = max(abs(a["measured_q"][j]-b["measured_q"][j]) for a, b in zip(real_rows, sim_rows) for j in range(7))
    checks = {"joint_rmse": max(q_error) <= tol["joint_rmse_rad"],
              "joint_max_error": maximum_q <= tol["joint_max_error_rad"],
              "joint_velocity_rmse": max(dq_error) <= tol["joint_velocity_rmse_rad_s"],
              "force_rmse": max(wrench_error[:3]) <= tol["force_rmse_N"],
              "torque_rmse": max(wrench_error[3:]) <= tol["torque_rmse_Nm"]}
    report.update({"status": "passed_response_replay" if all(checks.values()) else "failed_response_replay",
                   "checks": checks, "actuator_replay_pass": all(checks.values()),
                   "physical_actuator_replay_pass": all(checks.values()) and reference["provenance"] == "physical",
                   "samples": len(real_rows), "maximum_time_alignment_error_s": time_error,
                   "command_excursion_rad": excursion,
                   "maximum_command_input_error_rad": command_error,
                   "joint_rmse_rad": q_error, "joint_max_error_rad": maximum_q,
                   "joint_velocity_rmse_rad_s": dq_error,
                   "force_rmse_N": wrench_error[:3], "torque_rmse_Nm": wrench_error[3:],
                   "diagnostics": {name: {"requested_to_sent_max_rad": within_trace(rows, "requested_q", "sent_q"),
                                           "sent_to_desired_max_rad": within_trace(rows, "sent_q", "desired_q"),
                                           "desired_to_measured_max_rad": within_trace(rows, "desired_q", "measured_q")}
                                   for name, rows in (("reference", real_rows), ("simulation", sim_rows))},
                   "artifact_sha256": {name: hashlib.sha256(path.read_bytes()).hexdigest() for name, path in zip(("reference", "simulation"), paths)},
                   "conditions": reference["conditions"], "tolerances": tol})
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manifest", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    try:
        report = evaluate(json.loads(args.manifest.read_text()), args.manifest.resolve().parent)
    except (ReplayStop, KeyError, TypeError, OSError, json.JSONDecodeError) as error:
        report = {"status": "recoverable_stop", "reason": str(error), "actuator_replay_pass": False,
                  "physical_actuator_replay_pass": False, "training_ready": False,
                  "hardware_deployment_enabled": False}
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False)+"\n")
    print(json.dumps({key: report[key] for key in ("status", "actuator_replay_pass", "physical_actuator_replay_pass")}))
    return 0 if report["actuator_replay_pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
