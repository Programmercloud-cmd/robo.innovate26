"""Classify recorded contact outcomes from explicit independent measurements.

This is a protocol evaluator, not a detector, physics model or reward based on
simulator attachment flags. Sources/calibrations must be established externally.
Pure classification never reads a robot or advances inventory. Physical record
evaluation additionally verifies raw-record/calibration artifact hashes.
Partial seating is an observed insertion-geometry label; jam is an operational
progress-stall label, not a diagnosis of unseen microscopic contact states.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path


OUTCOMES = ("success", "partial_seating", "jam", "slip", "recoverable_stop")
PROVENANCES = ("physical", "simulation", "synthetic")
LIMITS = ("xy_error_m", "depth_error_m", "yaw_error_rad", "tilt_rad", "fixture_motion_m",
          "fixture_tilt_rad", "grasp_slip_m", "grasp_slip_rad", "max_force_N", "max_torque_Nm",
          "minimum_partial_depth_m", "retention_load_N", "retention_duration_s", "retention_displacement_m",
          "jam_force_N", "jam_window_s", "jam_depth_excursion_m", "jam_xy_excursion_m", "jam_yaw_excursion_rad",
          "max_load_sample_gap_s", "unload_force_N", "unload_torque_Nm", "max_unload_to_release_s",
          "max_retention_to_final_pose_s", "retention_transverse_force_N", "retention_test_torque_Nm")
PATTERNS = {8: "baseplate_8_studs", 4: "inner_bridge_4_studs_4_owners", 2: "outer_cantilever_2_studs_2_owners"}


def number(value, name, minimum=None):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError("missing/nonfinite measurement or criterion: "+name)
    value = float(value)
    if minimum is not None and value < minimum:
        raise ValueError("negative measurement/uncertainty: "+name)
    return value


def required_true(packet, *keys):
    if not isinstance(packet, dict) or any(packet.get(key) is not True for key in keys):
        raise ValueError("missing/uncertain evidence: "+",".join(keys))


def measured(packet, key):
    return number(packet.get(key), key)


def upper_bound(packet, key, uncertainty):
    return abs(measured(packet, key))+number(packet.get(uncertainty), uncertainty, 0)


def interval_status(packet, key, uncertainty, maximum):
    magnitude = abs(measured(packet,key)); error = number(packet.get(uncertainty),uncertainty,0)
    return "inside" if magnitude+error <= maximum else "outside" if magnitude-error > maximum else "uncertain"


def support_pattern(layout, brick_id):
    brick = next((brick for brick in layout["bricks"] if brick["id"] == brick_id), None)
    if brick is None:
        raise ValueError("brick index is not in configured layout")
    support = brick["support"]
    count = support["potential_match_count"]
    pattern = PATTERNS.get(count)
    if (pattern is None or (count == 8 and support["contributors"] != ["baseplate"])
            or (count in (2, 4) and len(support["contributors"]) != count)):
        raise ValueError("unsupported or inconsistent nominal support topology")
    return pattern


def classify_trial(trial, criteria, layout):
    """Return a measured outcome; any uncertainty is an explicit recoverable stop.

    All SI bounds are predeclared. Geometry uses independent brick-relative
    measurements with uncertainty margins. ``pose`` is the final observation;
    for success it must be after unloaded release and the retention test.
    ``loads.samples`` contains t_s, depth_m (external, positive inward), wrench
    [Fx,Fy,Fz,Tx,Ty,Tz] at M, and valid. Fz>0 resists inward insertion.
    """
    trial = trial if isinstance(trial,dict) else {}
    result = {"trial_id": trial.get("trial_id"), "provenance": trial.get("provenance"),
              "outcome": "recoverable_stop", "reasons": [], "inventory_advanced": False,
              "training_ready": False, "calibration_independently_revalidated": False}
    def finish(outcome, reason):
        result.update(outcome=outcome, reasons=[reason], stop_required=outcome != "success")
        return result
    try:
        required_true(criteria, "predeclared")
        bounds = {key: number(criteria.get(key), key, 0) for key in LIMITS}
        if (any(value <= 0 for value in bounds.values()) or bounds["jam_force_N"] >= bounds["max_force_N"]
                or bounds["unload_force_N"] >= bounds["max_force_N"] or bounds["unload_torque_Nm"] >= bounds["max_torque_Nm"]
                or bounds["retention_test_torque_Nm"] >= bounds["max_torque_Nm"]):
            raise ValueError("positive commissioned bounds with unload/jam limits below safety maxima required")
        if trial.get("provenance") not in PROVENANCES or trial.get("split") not in ("fit", "validation"):
            raise ValueError("explicit provenance and whole-trial split required")
        if not trial.get("conditions_id") or trial.get("conditions_id") != criteria.get("conditions_id"):
            raise ValueError("fixture/grasp/controller conditions mismatch")
        if trial["provenance"] == "physical" and criteria.get("provenance") != "physical_repeatability_and_load_limits":
            raise ValueError("synthetic/default criteria cannot classify a physical trial")
        # Aborted attempts or retained static tests need not have a withdrawal.
        # Missing rates remain missing; aggregate coverage cannot invent them.
        for key in ("insertion_rate_m_s", "withdrawal_rate_m_s"):
            if key not in trial:
                raise ValueError("explicit measured rate or null required: "+key)
            if trial[key] is not None:
                number(trial[key], key, 0)
        profile = criteria.get("measurement_profile_id")
        calibration = trial["calibration"]
        required_true(calibration, "valid", "external_brick_pose", "wrench_bias_corrected")
        if not profile or calibration.get("profile_id") != profile or calibration.get("conditions_id") != trial["conditions_id"]:
            raise ValueError("calibration profile/conditions mismatch")
        pattern = support_pattern(layout, trial.get("brick_id"))
        if trial.get("support_pattern") != pattern:
            raise ValueError("claimed support pattern does not match b_n_m_k geometry")
        result["support_pattern"] = pattern
        pose, fixture, grasp, loads = (trial[key] for key in ("pose", "fixture", "grasp", "loads"))
        for packet in (pose, fixture, grasp, loads):
            required_true(packet, "valid")
            if packet.get("measurement_profile_id") != profile:
                raise ValueError("measurement packet calibration/profile mismatch")
        required_true(pose, "independent")
        required_true(fixture, "independent")
        required_true(fixture, "support_layout_verified")
        required_true(grasp, "observable")
        contributors = next(b["support"]["contributors"] for b in layout["bricks"] if b["id"] == trial["brick_id"])
        if sorted(fixture.get("support_owners", [])) != sorted(contributors):
            raise ValueError("observed supporting owners do not match configured target pattern")
        if (pose.get("source") != "external_brick_relative_pose" or loads.get("frame") != "M_at_M"
                or loads.get("depth_source") != "external_brick_relative_pose"):
            raise ValueError("external relative brick pose and calibrated wrench at M required; TCP/attachment is insufficient")
        pose_time = number(pose.get("observed_at_s"), "pose timestamp", 0)
        pose_status = [interval_status(pose,key,key+"_uncertainty",bounds[key])
                       for key in ("xy_error_m", "depth_error_m", "yaw_error_rad", "tilt_rad")]
        geometrically_seated = all(status == "inside" for status in pose_status)
        geometry_uncertain = not geometrically_seated and "outside" not in pose_status
        depth_lower = measured(pose, "engagement_depth_m")-number(pose.get("engagement_depth_uncertainty_m"), "engagement uncertainty", 0)
        fixture_moved = (upper_bound(fixture, "motion_m", "motion_uncertainty_m") > bounds["fixture_motion_m"]
                         or upper_bound(fixture, "tilt_rad", "tilt_uncertainty_rad") > bounds["fixture_tilt_rad"])
        slip_status = [interval_status(grasp,"slip_m","slip_uncertainty_m",bounds["grasp_slip_m"]),
                       interval_status(grasp,"slip_rad","slip_uncertainty_rad",bounds["grasp_slip_rad"])]
        force_uncertainty = number(loads.get("force_uncertainty_N"),"force uncertainty",0)
        torque_uncertainty = number(loads.get("torque_uncertainty_Nm"),"torque uncertainty",0)
        depth_uncertainty = number(loads.get("depth_uncertainty_m"),"depth uncertainty",0)
        xy_uncertainty = number(loads.get("xy_uncertainty_m"),"XY uncertainty",0)
        yaw_uncertainty = number(loads.get("yaw_uncertainty_rad"),"yaw uncertainty",0)
        samples = loads.get("samples", [])
        if len(samples) < 2:
            raise ValueError("timestamped calibrated load and external depth trace required")
        last_time, normalized = -1., []
        result.update(peak_force_N=0.,peak_torque_Nm=0.,peak_force_upper_bound_N=0.,peak_torque_upper_bound_Nm=0.)
        for sample in samples:
            required_true(sample, "valid")
            t = number(sample.get("t_s"), "load timestamp", 0)
            if t <= last_time:
                raise ValueError("nonprogressing load trace")
            if normalized and t-last_time > bounds["max_load_sample_gap_s"]:
                raise ValueError("calibrated force/depth trace has an observation gap")
            last_time = t
            wrench = sample.get("wrench")
            if not isinstance(wrench, (tuple, list)) or len(wrench) != 6:
                raise ValueError("six-axis calibrated wrench required")
            wrench = [number(x, "wrench") for x in wrench]
            force = math.hypot(*wrench[:3]); torque = math.hypot(*wrench[3:])
            force_upper = number(force+force_uncertainty,"force uncertainty bound",0)
            torque_upper = number(torque+torque_uncertainty,"torque uncertainty bound",0)
            result["peak_force_N"] = max(result["peak_force_N"],force)
            result["peak_torque_Nm"] = max(result["peak_torque_Nm"],torque)
            result["peak_force_upper_bound_N"] = max(result["peak_force_upper_bound_N"],force_upper)
            result["peak_torque_upper_bound_Nm"] = max(result["peak_torque_upper_bound_Nm"],torque_upper)
            phase = sample.get("phase")
            if phase not in ("insertion","contact","withdrawal","unload","retention"):
                raise ValueError("explicit load/contact phase required")
            normalized.append((t, measured(sample, "depth_m"), wrench[2], measured(sample,"relative_x_m"),
                               measured(sample,"relative_y_m"), measured(sample,"relative_yaw_rad"),phase))
            if force_upper > bounds["max_force_N"] or torque_upper > bounds["max_torque_Nm"]:
                return finish("recoverable_stop", "measured force/torque exceeds safety limit")
        if pose_time < last_time:
            raise ValueError("final independent pose predates recorded loading")
        result["recorded_load_to_final_pose_duration_s"] = pose_time-normalized[0][0]
        for packet in (fixture,grasp):
            if not number(packet.get("observed_from_s"),"observation start",0) <= normalized[0][0] <= last_time <= number(packet.get("observed_until_s"),"observation end",0):
                raise ValueError("fixture/grasp observation does not cover load interval")
        if fixture_moved:
            return finish("recoverable_stop", "lower fixture moved outside commissioned bounds")
        if "outside" in slip_status:
            return finish("slip", "independently observed held-brick slip exceeds bound")
        if "uncertain" in slip_status or geometry_uncertain:
            raise ValueError("measurement uncertainty straddles grasp/seating acceptance bounds")
        if not geometrically_seated:
            for start in range(len(normalized)):
                end = next((end for end in range(start+1,len(normalized))
                            if normalized[end][0]-normalized[start][0] >= bounds["jam_window_s"]),None)
                if end is None:
                    break
                window = normalized[start:end+1]
                if (all(row[6] in ("insertion","contact") for row in window)
                        and min(row[2] for row in window)-force_uncertainty >= bounds["jam_force_N"]
                        and max(row[1] for row in window)-min(row[1] for row in window)+2*depth_uncertainty <= bounds["jam_depth_excursion_m"]
                        and math.hypot(max(row[3] for row in window)-min(row[3] for row in window),
                                       max(row[4] for row in window)-min(row[4] for row in window))+2*xy_uncertainty <= bounds["jam_xy_excursion_m"]
                        and max(row[5] for row in window)-min(row[5] for row in window)+2*yaw_uncertainty <= bounds["jam_yaw_excursion_rad"]):
                    return finish("jam", "operational stall: external depth/XY/yaw progress bounded under sustained compressive load")
            if depth_lower > bounds["minimum_partial_depth_m"]:
                return finish("partial_seating", "measured engagement depth is positive but final geometry is not seated")
            return finish("recoverable_stop", "no independently verified seating or partial engagement")
        retention = trial["retention"]
        required_true(retention, "valid", "independent", "unloaded_before_release", "gripper_released", "fixture_valid", "axis_calibrated")
        if (samples[-1]["phase"] != "unload" or math.hypot(*samples[-1]["wrench"][:3])+force_uncertainty > bounds["unload_force_N"]
                or math.hypot(*samples[-1]["wrench"][3:])+torque_uncertainty > bounds["unload_torque_Nm"]):
            raise ValueError("calibrated trace does not verify unloaded state before release")
        if (retention.get("method") != "external_relative_pose_under_measured_pull"
                or retention.get("measurement_profile_id") != profile or retention.get("frame") != "M_at_M"
                or retention.get("outward_axis_M") != [0,0,1]):
            raise ValueError("independent post-release retention measurement required; attachment/resting is insufficient")
        released, start, end = [number(retention.get(key), key, 0) for key in ("release_s", "start_s", "end_s")]
        # Signed measured tensile component, not unsigned compression/lateral magnitude.
        load_lower = measured(retention,"minimum_outward_force_N")-number(retention.get("load_uncertainty_N"),"retention load uncertainty",0)
        maximum_load = number(retention.get("maximum_measured_load_N"),"retention peak load",0)
        maximum_torque = number(retention.get("maximum_measured_torque_Nm"),"retention peak torque",0)
        maximum_load_upper = number(maximum_load+number(retention.get("load_uncertainty_N"),"retention load uncertainty",0),"retention load bound",0)
        maximum_torque_upper = number(maximum_torque+number(retention.get("torque_uncertainty_Nm"),"retention torque uncertainty",0),"retention torque bound",0)
        if maximum_load < measured(retention,"minimum_outward_force_N"):
            raise ValueError("retention minimum/maximum measured load inconsistent")
        result["peak_force_N"] = max(result["peak_force_N"],maximum_load)
        result["peak_torque_Nm"] = max(result["peak_torque_Nm"],maximum_torque)
        result["peak_force_upper_bound_N"] = max(result["peak_force_upper_bound_N"],maximum_load_upper)
        result["peak_torque_upper_bound_Nm"] = max(result["peak_torque_upper_bound_Nm"],maximum_torque_upper)
        if maximum_load_upper > bounds["max_force_N"] or maximum_torque_upper > bounds["max_torque_Nm"]:
            return finish("recoverable_stop","independent released-pull force/torque exceeds safety bound")
        if not last_time <= released <= start < end <= pose_time:
            raise ValueError("unloaded release, retention and final-pose timestamps are inconsistent")
        if released-last_time > bounds["max_unload_to_release_s"] or pose_time-end > bounds["max_retention_to_final_pose_s"]:
            raise ValueError("unload/release or retention/final-pose observation gap is too large")
        if number(grasp.get("observed_until_s"),"grasp observation end",0) < released:
            raise ValueError("grasp observation does not cover release")
        if end-start < bounds["retention_duration_s"] or load_lower < bounds["retention_load_N"]:
            raise ValueError("retention load/time is insufficient for declared test")
        if number(fixture.get("observed_until_s"),"fixture observation end",0) < pose_time:
            raise ValueError("fixture observation does not cover released retention and final pose")
        transverse_upper = number(retention.get("maximum_transverse_force_N"),"retention transverse force",0)+number(retention.get("load_uncertainty_N"),"retention load uncertainty",0)
        if transverse_upper > bounds["retention_transverse_force_N"] or maximum_torque_upper > bounds["retention_test_torque_Nm"]:
            raise ValueError("pull direction/torque does not satisfy independent axial retention protocol")
        retained_status = interval_status(retention,"max_relative_motion_m","motion_uncertainty_m",bounds["retention_displacement_m"])
        if retained_status == "uncertain":
            raise ValueError("retention displacement uncertainty straddles acceptance bound")
        if retained_status == "outside":
            return finish("partial_seating", "apparently seated geometry did not retain under independent released test")
        return finish("success", "independent seated geometry and released retained engagement satisfy declared criteria")
    except (KeyError, TypeError, ValueError) as error:
        return finish("recoverable_stop", str(error))


def _artifact(root, value):
    if root is None or not isinstance(value, dict) or not value.get("path") or not value.get("sha256"):
        raise ValueError("physical raw-record and calibration artifacts with SHA256 are required")
    path = root/value["path"]
    data = path.read_bytes()
    if not data:
        raise ValueError("empty physical source artifact")
    actual = hashlib.sha256(data).hexdigest()
    if actual != value["sha256"]:
        raise ValueError("physical source artifact hash mismatch")
    return actual


def evaluate(manifest, layout, root=None):
    """Aggregate complete trials; physical records require existing hashed sources.

    Hash existence is provenance, not independent revalidation of a calibration
    or detector. This evaluator therefore never alone authorizes RL training.
    """
    if manifest.get("schema") != "lego_contact_outcome_v1":
        raise ValueError("unsupported outcome protocol schema")
    criteria, results, ids, recordings = manifest.get("criteria", {}), [], set(), {}
    split_valid = True
    for trial in manifest.get("trials", []):
        identity = trial.get("trial_id")
        if not identity or identity in ids:
            raise ValueError("unique whole-trial identifiers required")
        ids.add(identity)
        result = classify_trial(trial, criteria, layout)
        result["physical_source_artifacts_verified"] = False
        try:
            recording_id = trial.get("recording_id")
            if not recording_id:
                raise ValueError("whole-trial recording_id required")
            key = recording_id
            if trial.get("provenance") == "physical":
                key = _artifact(root, trial.get("recording_artifact"))
                _artifact(root, trial.get("calibration_artifact"))
                result["physical_source_artifacts_verified"] = True
            if key in recordings:
                split_valid = False
                raise ValueError("same complete recording reused; fit/validation and repeat counts must be independent")
            recordings[key] = trial.get("split")
        except (ValueError, OSError, TypeError) as error:
            result.update(outcome="recoverable_stop", reasons=[str(error)], stop_required=True)
        results.append(result)
    counts = {source: {name: sum(r["provenance"] == source and r["outcome"] == name for r in results)
                       for name in OUTCOMES} for source in PROVENANCES}
    trials = manifest.get("trials", [])
    rates = {key: sorted({float(t[key]) for t in trials if isinstance(t.get(key), (int,float)) and not isinstance(t.get(key),bool) and math.isfinite(t[key]) and t[key]>0})
             for key in ("insertion_rate_m_s", "withdrawal_rate_m_s")}
    rate_coverage = {source: {key: sorted({float(t[key]) for t,r in zip(trials,results)
                                         if r["provenance"] == source and r["outcome"] != "recoverable_stop"
                                         and isinstance(t.get(key),(int,float)) and not isinstance(t.get(key),bool)
                                         and math.isfinite(t[key]) and t[key] > 0})
                              for key in ("insertion_rate_m_s","withdrawal_rate_m_s")} for source in PROVENANCES}
    validated_patterns = sorted({r.get("support_pattern") for r, trial in zip(results,trials)
                                 if trial.get("split") == "validation" and r["provenance"] == "physical" and r["outcome"] == "success"})
    return {"schema": "lego_contact_outcome_report_v1", "results": results, "counts_by_provenance": counts,
            "rates_m_s": rates, "multiple_rates_present": all(len(values)>=2 for values in rates.values()),
            "eligible_rates_by_provenance_m_s": rate_coverage,
            "physical_multiple_rates_present": all(len(values)>=2 for values in rate_coverage["physical"].values()),
            "whole_trial_fit_and_validation_present": {t.get("split") for t in trials} >= {"fit","validation"},
            "recording_split_and_repeat_identity_valid": split_valid,
            "physical_retained_success_patterns": validated_patterns,
            "all_support_patterns_have_physical_retained_success": set(validated_patterns) == set(PATTERNS.values()),
            "training_ready": False, "hardware_deployment_enabled": False,
            "scope": "classification of supplied independent outcome records; not contact-model prediction, sensor calibration or policy validation",
            "missing_physical_trials": not any(t.get("provenance") == "physical" for t in trials)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manifest", type=Path)
    parser.add_argument("--layout", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    try:
        report = evaluate(json.loads(args.manifest.read_text()), json.loads(args.layout.read_text()), args.manifest.resolve().parent)
    except (ValueError, KeyError, TypeError, OSError) as error:
        parser.exit(2, f"Recoverable stop: {error}\n")
    args.output.write_text(json.dumps(report,indent=2,allow_nan=False)+"\n")
    print(json.dumps(report["counts_by_provenance"]))
    return 0 if report["results"] and all(r["outcome"] != "recoverable_stop" for r in report["results"]) else 2


if __name__ == "__main__":
    raise SystemExit(main())
