"""Synthetic data-contract tests; no physical actuator validation."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
try:
    from .controller_replay import CONDITIONS, TOLERANCES, ReplayStop, evaluate
except ImportError:
    from controller_replay import CONDITIONS, TOLERANCES, ReplayStop, evaluate


IDENTITY = [1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1.]


def fixture(root):
    rows = []
    for index in range(5):
        command = [index*.001]+[0.]*6
        measured = [index*.0009]+[0.]*6
        rows.append({"t_s": index*.01, "sequence": index, "requested_q": command,
                     "sent_q": command, "desired_q": command, "measured_q": measured,
                     "measured_dq": [.09]+[0.]*6, "wrench_K": [0.]*6,
                     "wrench_state_t_s":index*.01,"wrench_filter_latency_s":.002,"wrench_nominal_force_t_s":index*.01-.002,
                     "F_T_EE": IDENTITY, "EE_T_K": IDENTITY,
                     "wrench_valid": True, "singularity_valid": True, "grasp_valid": True,
                     "robot_fault": False, "dropped_samples_since_last": 0})
    for name in ("reference", "simulation"):
        write(root/name, rows)
    conditions = {key: "synthetic-conditions" for key in CONDITIONS}
    conditions.update({"control_period_s": .001, "sample_phase": "before_command_application"})
    tolerances = {key: .001 for key in TOLERANCES}
    tolerances.update({"maximum_sample_gap_s": .02, "minimum_command_excursion_rad": .001})
    return rows, {"schema": 1, "controller_interface": "joint_positions_joint_impedance",
                  "reference": {"path": "reference", "provenance": "synthetic", "actuation_method": "franka_joint_impedance", "conditions": conditions},
                  "simulation": {"path": "simulation", "provenance": "simulation", "actuation_method": "dynamic_joint_impedance_surrogate", "conditions": conditions},
                  "tolerances": tolerances}


def write(path, rows):
    path.write_text("\n".join(map(json.dumps, rows)))


class ControllerReplayTests(unittest.TestCase):
    def test_matching_response_has_diagnostics_but_no_physical_readiness(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            _, manifest = fixture(root)
            report = evaluate(manifest, root)
            self.assertTrue(report["actuator_replay_pass"])
            self.assertFalse(report["physical_actuator_replay_pass"])
            self.assertFalse(report["training_ready"])
            self.assertAlmostEqual(report["diagnostics"]["reference"]["desired_to_measured_max_rad"], .0004)

    def test_moving_response_error_fails(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            rows, manifest = fixture(root)
            rows[-1]["measured_q"][0] += .02
            write(root/"simulation", rows)
            report = evaluate(manifest, root)
            self.assertFalse(report["actuator_replay_pass"])
            self.assertFalse(report["checks"]["joint_max_error"])

    def test_mismatched_input_ideal_actuator_and_uncommissioned_values(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            rows, original = fixture(root)
            manifest = copy.deepcopy(original)
            manifest["simulation"]["actuation_method"] = "measured_q_overwrite"
            with self.assertRaises(ReplayStop): evaluate(manifest, root)
            rows[1]["sent_q"][0] += .02
            write(root/"simulation", rows)
            with self.assertRaisesRegex(ReplayStop, "inputs differ"): evaluate(original, root)
            original["tolerances"]["force_rmse_N"] = None
            self.assertEqual(evaluate(original, root)["status"], "uncommissioned")

    def test_time_shift_drop_and_false_validity_stop(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            original_rows, manifest = fixture(root)
            for change in (lambda row: row.update(t_s=row["t_s"]+.02),
                           lambda row: row.update(dropped_samples_since_last=1),
                           lambda row: row.update(wrench_valid=False)):
                rows = copy.deepcopy(original_rows)
                change(rows[1])
                write(root/"simulation", rows)
                with self.assertRaises(ReplayStop): evaluate(manifest, root)

    def test_stationary_replay_is_not_motion_evidence(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            rows, manifest = fixture(root)
            for row in rows:
                row["sent_q"] = row["requested_q"] = [0.]*7
            for name in ("reference", "simulation"):
                write(root/name, rows)
            with self.assertRaisesRegex(ReplayStop, "stationary"): evaluate(manifest, root)

    def test_wrench_latency_mismatch_or_missing_metadata_stops(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            original_rows,manifest=fixture(root)
            rows=copy.deepcopy(original_rows)
            for row in rows:
                row["wrench_filter_latency_s"]+=.01
                row["wrench_nominal_force_t_s"]-=.01
            write(root/"simulation",rows)
            with self.assertRaisesRegex(ReplayStop,"latency mismatch"):evaluate(manifest,root)
            rows=copy.deepcopy(original_rows)
            del rows[1]["wrench_filter_latency_s"]
            write(root/"simulation",rows)
            with self.assertRaises(ReplayStop):evaluate(manifest,root)


if __name__ == "__main__":
    unittest.main()
