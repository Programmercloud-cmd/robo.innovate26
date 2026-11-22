"""Task speed bypass must preserve opening evidence and truthful speed logs."""
import copy
from contextlib import redirect_stdout
import io
import json
from pathlib import Path
import tempfile
import unittest

from run_cycle import Factory,HERE
from runtime import Stop


class ReleaseSpeedPolicyTests(unittest.TestCase):
    def test_task_warns_without_falsely_verifying_speed_and_normal_stops(self):
        # Synthetic complete-opening evidence, deliberately outside speed range.
        metrics=dict(samples=100,release_start_t=2.11,opening_complete_t=2.60,
            maximum_sample_gap_s=.001,full_opening_speed_verified=False,
            min_vy_m_s=.014,mean_vy_m_s=.019,max_vy_m_s=.021,bad_speed_samples=1)
        original=copy.deepcopy(metrics)
        with tempfile.TemporaryDirectory() as temp:
            f=Factory.__new__(Factory);f.out=Path(temp);f.task_execution=True
            with redirect_stdout(io.StringIO()) as output:f.verify_release_metrics(metrics)
            self.assertIn('speed gate disabled',output.getvalue())
            self.assertEqual(metrics,original)
            self.assertFalse(json.loads((f.out/'warnings.ndjson').read_text())['release_speed_verified'])
            f.task_execution=False
            with self.assertRaisesRegex(Stop,'Measured gripper'):f.verify_release_metrics(metrics)

    def test_speed_bypass_does_not_accept_incomplete_or_stale_opening(self):
        f=Factory.__new__(Factory);f.task_execution=True
        for metrics in (None,dict(samples=105,release_start_t=2.111,opening_complete_t=-1,
                maximum_sample_gap_s=.001,full_opening_speed_verified=False),
                dict(samples=100,release_start_t=2.11,opening_complete_t=2.6,
                maximum_sample_gap_s=.025,full_opening_speed_verified=True)):
            with self.assertRaisesRegex(Stop,'complete fresh release telemetry'):
                f.verify_release_metrics(metrics)

    def test_new_normal_range_includes_recorded_15_point_8_mm_s_dip(self):
        cfg=json.loads((HERE/'setups.json').read_text())['setups']['conveyor_moving']['moving_parameters']
        self.assertEqual(cfg['minimum_release_vy_m_s'],.015)
        self.assertEqual(cfg['maximum_release_vy_m_s'],.025)
        # Extrema from the actual failed 20261028T095950.761045Z opening trace.
        self.assertLessEqual(cfg['minimum_release_vy_m_s'],.015800907498316006)
        self.assertGreaterEqual(cfg['maximum_release_vy_m_s'],.020725386483330964)


if __name__=='__main__':unittest.main()
