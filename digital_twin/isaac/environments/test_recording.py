"""Focused freshness/ownership checks; no hardware or Isaac dependency."""
import json
from pathlib import Path
import tempfile
import time
import unittest

from digital_twin.isaac.environments.recording import RecordingFollower
from digital_twin.isaac.mirror import validate_frame


class RecordingTests(unittest.TestCase):
    def test_stale_samples_restarts_and_incomplete_writes(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            session = root/'active.json'
            session.write_text(json.dumps({'output':str(root)}))
            recording = root/'telemetry.ndjson'
            first = {'schema':'fr3.telemetry.v1','seq':9,'unix_ns':time.time_ns()-2_000_000_000,
                     'q':[0.]*7,'dq':[0.]*7,'width_m':None,'hand_stale':True}
            recording.write_text(json.dumps(first)+'\n')
            follower = RecordingFollower(session,validate_frame)
            try:
                received, frame = follower.latest.get(timeout=1)
                self.assertGreater(time.monotonic()-received,1.9)
                self.assertEqual(frame['unix_ns'],first['unix_ns'])
                time.sleep(.12)
                self.assertTrue(follower.latest.empty(), 'Unchanged sample must not become fresh')
                second = dict(first,seq=10,unix_ns=time.time_ns(),q=[.1]*7)
                text = json.dumps(second)
                recording.write_text(json.dumps(first)+'\n'+text[:20])
                time.sleep(.1)
                self.assertTrue(follower.latest.empty(), 'Partial write must not be consumed')
                recording.write_text(json.dumps(first)+'\n'+text+'\n')
                _, fresh = follower.latest.get(timeout=1)
                self.assertEqual(fresh['q'],[.1]*7)
                restarted = dict(second,seq=0,unix_ns=time.time_ns())
                recording.write_text(json.dumps(restarted)+'\n')
                _, frame = follower.latest.get(timeout=1)
                self.assertEqual(frame['seq'],0)
                self.assertEqual(follower.stats['hardware_clients'],0)
            finally:
                follower.stop = True
                follower.thread.join(timeout=1)


if __name__ == '__main__':
    unittest.main()
