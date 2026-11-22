"""Focused lifecycle/ownership/freshness regressions. No hardware access."""
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parent))
from hand_stream import HandStream,hand_fields


class HandLifecycle(unittest.TestCase):
    def wait_for(self,predicate,timeout=2):
        end=time.monotonic()+timeout
        while not predicate():
            if time.monotonic()>end:self.fail('lifecycle condition timed out')
            time.sleep(.01)

    def test_timestamp_and_owner_priority(self):
        now=1_000_000_000;background=dict(width=.07,unix_ns=now)
        self.assertFalse(hand_fields(background,now_ns=now)['hand_stale'])
        self.assertTrue(hand_fields(background,now_ns=now+500_000_000)['hand_stale'])
        self.assertTrue(hand_fields(background,is_owner=True,now_ns=now)['hand_stale'])
        owner=dict(width_m=.065,width_unix_ns=now-100)
        selected=hand_fields(background,owner_sample=owner,is_owner=True,now_ns=now)
        self.assertEqual(selected['width_m'],.065)
        self.assertEqual(selected['width_unix_ns'],now-100)
        self.assertEqual(selected['hand_source'],'foreground_sdk_owner')

    def test_one_connection_pause_and_foreground_handoff(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);script=p/'fake.py'
            script.write_text('import json,time\nwhile True:\n print(json.dumps(dict(unix_ns=time.time_ns(),state=dict(width=.07))),flush=True);time.sleep(.03)\n')
            stop=threading.Event();busy=threading.Event();pause=p/'pause'
            stream=HandStream([sys.executable,str(script)],p,pause,busy.is_set)
            worker=threading.Thread(target=stream.run,args=(stop,));worker.start()
            try:
                self.wait_for(lambda:stream.state=='streaming');pid=stream.pid
                time.sleep(.4);self.assertEqual(stream.pid,pid);self.assertEqual(stream.starts,1)
                pause.touch();self.wait_for(lambda:stream.state=='paused')
                self.assertTrue(stream.status()['hand_stale']);self.assertIsNone(stream.pid)
                time.sleep(.2);self.assertEqual(stream.starts,1)
                pause.unlink();self.wait_for(lambda:stream.state=='streaming');self.assertEqual(stream.starts,2)
                busy.set();self.wait_for(lambda:stream.state=='foreground_owner');self.assertTrue(stream.status()['hand_stale'])
                time.sleep(.2);self.assertEqual(stream.starts,2)
                busy.clear();self.wait_for(lambda:stream.state=='streaming');self.assertEqual(stream.starts,3)
            finally:stop.set();worker.join(timeout=2)
            self.assertFalse(worker.is_alive())

    def test_failures_are_bounded_and_stale(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);stop=threading.Event()
            stream=HandStream([sys.executable,'-c','raise SystemExit(1)'],p,p/'pause',lambda:False,
                              max_retries=2,retry_initial=.03,retry_max=.04)
            worker=threading.Thread(target=stream.run,args=(stop,));worker.start()
            try:
                self.wait_for(lambda:stream.state=='exhausted')
                self.assertEqual(stream.starts,3);self.assertTrue(stream.status()['hand_stale'])
                time.sleep(.15);self.assertEqual(stream.starts,3)
            finally:stop.set();worker.join(timeout=2)


if __name__=='__main__':unittest.main()
