"""Synthetic parameter recovery and held-out isolation; no physical evidence."""
import copy
import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from effective_axial import EffectiveAxialModel
from fit_axial import fit


class FitTests(unittest.TestCase):
    def test_recovery_held_out_isolation_and_rank_rejection(self):
        coefficients = {"depth_knots_m": [0, .001, .002],
                        "elastic_force_N": [0, 1, 2], "damping_N_s_m": [0, 3, 4]}
        model = EffectiveAxialModel(coefficients)
        rows = []
        for index, depth in enumerate([0, .0005, .001, .0015, .002, .0015, .001, .0005, 0]):
            speed = .001 if index < 4 else (0 if index == 4 else -.001)
            rows.append({"t_s": index*.5, "depth_m": depth, "depth_rate_m_s": speed,
                         "force_z_N": model.evaluate(depth, speed)["force_N"],
                         "phase": "insertion" if index < 4 else ("dwell" if index == 4 else "withdrawal")})
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            validation = copy.deepcopy(rows)
            validation[1]["force_z_N"] += .01
            def write(name, data):
                (root/name).write_text("\n".join(map(json.dumps, data)))
            write("fit.ndjson", rows)
            write("validation.ndjson", validation)
            manifest = {"schema": 1, "model_id": "synthetic-only", "conditions_id": "synthetic-fixture",
                        "depth_knots_m": coefficients["depth_knots_m"], "max_condition_number": 1e8,
                        "trials": [{"id": name, "path": name+".ndjson", "split": name,
                                    "provenance": "synthetic", "conditions_id": "synthetic-fixture"}
                                   for name in ("fit", "validation")]}
            candidate, _ = fit(manifest, root)
            np.testing.assert_allclose(candidate["elastic_force_N"], coefficients["elastic_force_N"], atol=1e-10)
            np.testing.assert_allclose(candidate["damping_N_s_m"], coefficients["damping_N_s_m"], atol=1e-9)
            self.assertFalse(candidate["training_ready"])
            validation[2]["force_z_N"] += 10
            write("validation.ndjson", validation)
            changed, _ = fit(manifest, root)
            self.assertEqual(changed["elastic_force_N"], candidate["elastic_force_N"])
            self.assertEqual(changed["damping_N_s_m"], candidate["damping_N_s_m"])
            # Without variation in rate, elastic and damping are confounded.
            for row in rows:
                row["depth_rate_m_s"] = .001
            write("fit.ndjson", rows)
            with self.assertRaisesRegex(ValueError, "unidentifiable"):
                fit(manifest, root)


if __name__ == "__main__":
    unittest.main()
