"""Fit an effective axial force replay model; never enables training/hardware.

Fixed contact datum and depth knots must be identified externally. The linear
fit uses complete fit trials only; held-out samples never change coefficients.
Unidentifiable or nonpassive candidates are rejected, never silently clipped.
NumPy is needed only for the linear solve. No SciPy or hardware client is used.
"""
import argparse
import bisect
import hashlib
import json
from pathlib import Path

import numpy as np

from effective_axial import EffectiveAxialModel
from evaluate_traces import evaluate, finite, reference_hash


def basis(depth, knots):
    if depth <= 0:
        return [0.0] * (len(knots)-1)
    if depth > knots[-1]:
        raise ValueError("sample beyond declared model depth; do not extrapolate")
    index = min(bisect.bisect_right(knots, depth)-1, len(knots)-2)
    alpha = (depth-knots[index]) / (knots[index+1]-knots[index])
    full = [0.0] * len(knots)
    full[index], full[index+1] = 1-alpha, alpha
    return full[1:]  # contact force and damping fixed to zero at depth=0


def fit(manifest, root):
    knots = manifest.get("depth_knots_m")
    if not isinstance(knots, list) or len(knots) < 2:
        raise ValueError("externally identified depth_knots_m required")
    knots = [finite(x, "depth knot") for x in knots]
    model_template = {"depth_knots_m": knots, "elastic_force_N": [0.0]*len(knots),
                      "damping_N_s_m": [0.0]*len(knots)}
    EffectiveAxialModel(model_template)  # validate domain before fitting
    if manifest.get("schema") != 1 or not manifest.get("conditions_id") or not manifest.get("model_id"):
        raise ValueError("schema, model_id and fixed conditions_id required")
    condition_limit = finite(manifest.get("max_condition_number"), "max_condition_number")
    if condition_limit <= 1:
        raise ValueError("numerical condition limit must exceed 1")
    trials = manifest.get("trials", [])
    design, target, loaded, ids, fit_hashes, validation_hashes = [], [], [], set(), set(), set()
    for trial in trials:
        if trial["id"] in ids or not trial["id"]:
            raise ValueError("unique complete trial IDs required")
        ids.add(trial["id"])
        if trial.get("conditions_id") != manifest["conditions_id"]:
            raise ValueError("fixture/grasp/controller conditions mismatch")
        if trial["split"] not in ("fit", "validation") or trial["provenance"] not in ("physical", "synthetic"):
            raise ValueError("trial split/provenance missing")
        path = root / trial["path"]
        rows = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
        if not rows:
            raise ValueError("empty contact trial")
        reference = []
        old_t = -1.0
        for row in rows:
            t, depth, velocity, force = [finite(row[key], key) for key in
                                       ("t_s", "depth_m", "depth_rate_m_s", "force_z_N")]
            if t < 0 or t <= old_t:
                raise ValueError("strictly increasing nonnegative timestamps required")
            old_t = t
            b = basis(depth, knots)
            reference.append([t, depth, velocity, force, row["phase"]])
            if trial["split"] == "fit":
                design.append(b + [x*velocity for x in b])
                target.append(force)
        digest = reference_hash(reference)
        (fit_hashes if trial["split"] == "fit" else validation_hashes).add(digest)
        loaded.append((trial, rows, hashlib.sha256(path.read_bytes()).hexdigest()))
    if not design or not validation_hashes or fit_hashes & validation_hashes:
        raise ValueError("separate complete fit and validation trials required; duplicated traces rejected")
    matrix, values = np.asarray(design, dtype=float), np.asarray(target, dtype=float)
    coefficients, _, rank, singular = np.linalg.lstsq(matrix, values, rcond=None)
    condition = float(singular[0]/singular[-1]) if len(singular) and singular[-1] > 0 else float("inf")
    if rank != matrix.shape[1] or condition > condition_limit:
        raise ValueError("unidentifiable/ill-conditioned elastic and damping coefficients; vary depth/rate independently")
    n = len(knots)-1
    candidate = {**model_template, "elastic_force_N": [0.0] + coefficients[:n].tolist(),
                 "damping_N_s_m": [0.0] + coefficients[n:].tolist(),
                 "parameter_status": "synthetic_candidate" if all(trial["provenance"] == "synthetic" for trial, _, _ in loaded if trial["split"] == "fit") else "identified_candidate",
                 "real_ready": False, "training_ready": False, "hardware_deployment_enabled": False,
                 "conditions_id": manifest["conditions_id"], "model_id": manifest["model_id"],
                 "fit_provenance": sorted({trial["provenance"] for trial, _, _ in loaded if trial["split"] == "fit"}),
                 "identification": "effective axial response only; does not separate brick, grip, fixture or servo compliance",
                 "fit_rank": int(rank), "fit_condition_number": condition,
                 "fit_trial_sha256": {trial["id"]: digest for trial, _, digest in loaded if trial["split"] == "fit"}}
    model = EffectiveAxialModel(candidate)  # reject negative damping/nonpassive potential
    replay = []
    for trial, rows, _ in loaded:
        paired = [{**row, "predicted_force_z_N": model.evaluate(row["depth_m"], row["depth_rate_m_s"])["force_N"]} for row in rows]
        replay.append((trial, paired))
    return candidate, replay


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manifest", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    try:
        manifest = json.loads(args.manifest.read_text())
        candidate, replay = fit(manifest, args.manifest.resolve().parent)
        args.output_dir.mkdir(parents=True, exist_ok=False)
        (args.output_dir/"candidate.json").write_text(json.dumps(candidate, indent=2, allow_nan=False)+"\n")
        paired_manifest = {key: manifest[key] for key in ("schema", "model_id", "conditions_id", "tolerances")}
        paired_manifest["trials"] = []
        for index, (trial, rows) in enumerate(replay):
            name = f"trial-{index}.ndjson"
            (args.output_dir/name).write_text("\n".join(json.dumps(row, allow_nan=False) for row in rows)+"\n")
            paired_manifest["trials"].append({**trial, "path": name})
        report = evaluate(paired_manifest, args.output_dir)
        (args.output_dir/"replay_manifest.json").write_text(json.dumps(paired_manifest, indent=2)+"\n")
        (args.output_dir/"force_replay_report.json").write_text(json.dumps(report, indent=2, allow_nan=False)+"\n")
    except (ValueError, KeyError, OSError, TypeError) as error:
        parser.exit(2, f"Recoverable stop: {error}\n")
    print(json.dumps({"candidate_written": True, "force_replay_pass": report["force_replay_pass"], "training_ready": False}))
    return 0 if report["force_replay_pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
