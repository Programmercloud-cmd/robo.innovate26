"""Evaluate paired force predictions on complete held-out contact trials.

NDJSON rows: t_s, depth_m, force_z_N, predicted_force_z_N, phase.
Depth is positive into engagement, Fz is force on the upper brick along M +Z.
Predictions must use the SAME observed kinematics: this evaluates force replay,
not closed-loop robot dynamics, retention or policy transfer. No hardware IO.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path


PHASES = ("insertion", "withdrawal", "dwell")
METRICS = ("force_rmse_N", "peak_error_N", "work_error_J")


def reference_hash(reference):
    # Identity is independent of recording start time (round only clock noise).
    start = reference[0][0]
    relative = [[round(row[0]-start, 9), *row[1:]] for row in reference]
    return hashlib.sha256(json.dumps(relative, separators=(",", ":")).encode()).hexdigest()


def finite(value, name):
    if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
        raise ValueError(f"{name}: finite numeric value required")
    return float(value)


def trial_metrics(rows):
    if len(rows) < 3:
        raise ValueError("at least three timestamped samples required")
    previous = None
    squared_error_integral = 0.0
    works = {phase: [0.0, 0.0] for phase in (*PHASES, "transition")}
    peaks = {phase: [0.0, 0.0] for phase in PHASES}
    seen = set()
    directional_motion = {"insertion": False, "withdrawal": False}
    reference = []
    for row in rows:
        t, depth, force, predicted = [finite(row[key], key) for key in
                                    ("t_s", "depth_m", "force_z_N", "predicted_force_z_N")]
        phase = row["phase"]
        if t < 0 or phase not in PHASES:
            raise ValueError("nonnegative time and insertion/withdrawal/dwell phase required")
        seen.add(phase)
        reference.append([t, depth, force, phase])
        peaks[phase][0] = max(peaks[phase][0], abs(force))
        peaks[phase][1] = max(peaks[phase][1], abs(predicted))
        if previous is not None:
            old_t, old_depth, old_force, old_predicted, old_phase = previous
            dt = t - old_t
            if dt <= 0:
                raise ValueError("timestamps must increase strictly; do not silently drop samples")
            squared_error_integral += dt * ((force-predicted)**2 + (old_force-old_predicted)**2) / 2
            work = works[phase if old_phase == phase else "transition"]
            # dz = -ddepth; negative work on upper brick means absorbed energy.
            work[0] -= (force + old_force) * (depth-old_depth) / 2
            work[1] -= (predicted + old_predicted) * (depth-old_depth) / 2
            if phase == old_phase == "insertion" and depth > old_depth:
                directional_motion["insertion"] = True
            if phase == old_phase == "withdrawal" and depth < old_depth:
                directional_motion["withdrawal"] = True
        previous = t, depth, force, predicted, phase
    duration = reference[-1][0] - reference[0][0]
    return {
        "samples": len(rows), "duration_s": duration,
        "force_rmse_N": math.sqrt(squared_error_integral / duration),
        "peak_error_N": max(abs(peaks[p][0]-peaks[p][1]) for p in seen),
        "work_error_J": max(abs(a-b) for a, b in works.values()),
        "peak_abs_force_N": {p: {"observed": a, "predicted": b} for p, (a, b) in peaks.items() if p in seen},
        "work_on_upper_J": {p: {"observed": a, "predicted": b} for p, (a, b) in works.items()},
        "both_branches_present": all(directional_motion.values()),
        "directional_motion": directional_motion,
        "reference_sha256": reference_hash(reference)
    }


def evaluate(manifest, root):
    if manifest.get("schema") != 1 or not manifest.get("model_id") or not manifest.get("conditions_id"):
        raise ValueError("schema=1, model_id and fixed fixture/grasp conditions_id required")
    trials = manifest.get("trials", [])
    if not trials:
        raise ValueError("no trials; physical measurements are required, not invented")
    tolerances = manifest.get("tolerances", {})
    commissioned = all(tolerances.get(key) is not None for key in METRICS)
    if commissioned:
        tolerances = {key: finite(tolerances[key], key) for key in METRICS}
        if any(value < 0 for value in tolerances.values()):
            raise ValueError("nonnegative, predeclared tolerances required")
    ids, references, reports = set(), {}, []
    for trial in trials:
        identity, split, provenance = trial["id"], trial["split"], trial["provenance"]
        if not isinstance(identity, str) or not identity or identity in ids:
            raise ValueError("each complete trial needs a unique nonempty id")
        if split not in ("fit", "validation") or provenance not in ("physical", "synthetic"):
            raise ValueError("explicit fit/validation split and physical/synthetic provenance required")
        if trial.get("conditions_id") != manifest["conditions_id"]:
            raise ValueError("fixture/grasp/controller conditions mismatch; evaluate separately")
        ids.add(identity)
        path = root / trial["path"]
        rows = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
        report = trial_metrics(rows)
        reference = report["reference_sha256"]
        if reference in references and references[reference] != split:
            raise ValueError("same reference trace reused across fit and validation")
        references[reference] = split
        reports.append({"id": identity, "split": split, "provenance": provenance,
                        "trace_sha256": hashlib.sha256(path.read_bytes()).hexdigest(), **report})
    held_out = [trial for trial in reports if trial["split"] == "validation"]
    adequate = bool(held_out) and all(t["both_branches_present"] for t in held_out)
    passed = commissioned and adequate and all(t[key] <= tolerances[key] for t in held_out for key in METRICS)
    return {
        "model_id": manifest["model_id"], "conditions_id": manifest["conditions_id"],
        "evaluation": "observed-kinematics force replay only",
        "tolerances_commissioned": commissioned, "tolerances": tolerances,
        "complete_held_out_branches": adequate, "force_replay_pass": passed,
        "held_out_data_all_physical": bool(held_out) and all(t["provenance"] == "physical" for t in held_out),
        "physical_force_replay_pass": passed and all(t["provenance"] == "physical" for t in held_out),
        "training_ready": False, "hardware_deployment_enabled": False,
        "not_validated": ["closed-loop servo response", "independent seating/retention", "slip/jam/capture outcomes", "sensor calibration", "parameter identifiability"],
        "trials": reports
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manifest", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    try:
        report = evaluate(json.loads(args.manifest.read_text()), args.manifest.resolve().parent)
    except (ValueError, KeyError, OSError, TypeError) as error:
        parser.exit(2, f"Recoverable stop: {error}\n")
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print(json.dumps({key: report[key] for key in ("force_replay_pass", "physical_force_replay_pass", "training_ready")}))
    return 0 if report["force_replay_pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
