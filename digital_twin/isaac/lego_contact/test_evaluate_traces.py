"""Synthetic regression checks for trace accounting and data leakage guards."""
import json
import tempfile
import unittest
from pathlib import Path
from evaluate_traces import evaluate, trial_metrics


def coupon(predicted_scale=1.0):
    return [{"t_s": t, "depth_m": d, "force_z_N": f,
             "predicted_force_z_N": f*predicted_scale, "phase": p}
            for t, d, f, p in [(0, 0, 0, "insertion"), (1, .001, 2, "insertion"),
                                (2, .002, 0, "dwell"), (3, .001, -1, "withdrawal"),
                                (4, 0, 0, "withdrawal")]]


class EvaluationTests(unittest.TestCase):
    def test_work_sign_and_exact_prediction(self):
        result = trial_metrics(coupon())
        self.assertEqual(result["force_rmse_N"], 0)
        self.assertLess(result["work_on_upper_J"]["insertion"]["observed"], 0)
        self.assertLess(result["work_on_upper_J"]["withdrawal"]["observed"], 0)
        self.assertEqual(result["work_error_J"], 0)

    def test_force_error_is_not_hidden_by_net_work(self):
        result = trial_metrics(coupon(2))
        self.assertGreater(result["force_rmse_N"], 0)
        self.assertEqual(result["peak_error_N"], 2)
        self.assertGreater(result["work_error_J"], 0)

    def test_bad_clock_and_nonfinite_rejected(self):
        for key, value in [("t_s", 0), ("force_z_N", float("nan"))]:
            rows = coupon()
            rows[1][key] = value
            with self.assertRaises(ValueError):
                trial_metrics(rows)

    def test_stationary_labels_are_not_insertion_or_withdrawal(self):
        rows = coupon()
        for row in rows:
            row["depth_m"] = 0
        self.assertFalse(trial_metrics(rows)["both_branches_present"])

    def test_identity_ignores_absolute_start_time(self):
        rows = coupon()
        before = trial_metrics(rows)["reference_sha256"]
        for row in rows:
            row["t_s"] += 100
        self.assertEqual(before, trial_metrics(rows)["reference_sha256"])

    def test_split_and_readiness(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root/"one.ndjson").write_text("\n".join(map(json.dumps, coupon())))
            trial = {"id": "one", "split": "validation", "provenance": "synthetic", "conditions_id": "fixture", "path": "one.ndjson"}
            manifest = {"schema": 1, "model_id": "synthetic", "conditions_id": "fixture",
                        "tolerances": {"force_rmse_N": 0, "peak_error_N": 0, "work_error_J": 0}, "trials": [trial]}
            result = evaluate(manifest, root)
            self.assertTrue(result["force_replay_pass"])
            self.assertFalse(result["physical_force_replay_pass"])
            self.assertFalse(result["training_ready"])
            manifest["trials"].append({**trial, "id": "two", "split": "fit"})
            with self.assertRaisesRegex(ValueError, "reused"):
                evaluate(manifest, root)
            manifest["trials"] = [trial]
            manifest["tolerances"]["force_rmse_N"] = None
            self.assertFalse(evaluate(manifest, root)["force_replay_pass"])


if __name__ == "__main__":
    unittest.main()
