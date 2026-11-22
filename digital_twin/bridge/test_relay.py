"""Process-level regression checks; fake reader and loopback sink, no robot/VM."""
import json
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import threading
import time
import unittest

BRIDGE = Path(__file__).resolve().parent


class ReaderRecovery(unittest.TestCase):
    def scenario(self, mode, *, duration=1.8, retries=2, idle=.6):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        root = Path(temporary.name)
        reader = root / 'reader'
        reader.write_text('''#!/usr/bin/python3
import json,sys,time
from pathlib import Path
root=Path(__file__).parent
counter=root/'attempts'
n=int(counter.read_text())+1 if counter.exists() else 1
counter.write_text(str(n))
mode=(root/'mode').read_text()
def sample():
 s=dict(q=[0.]*7,dq=[0.]*7,q_d=[0.]*7,O_T_EE=[0.]*16,F_T_EE=[0.]*16,
        O_T_EE_d=[0.]*16,tau_J=[0.]*7,tau_J_d=[0.]*7,robot_mode='Idle',current_errors=[],time=0.)
 stamp=time.time_ns()-int(2e9) if mode=='stale' else time.time_ns()
 print(json.dumps(dict(unix_ns=stamp,state=s)),flush=True)
if mode=='fail' or mode=='recover' and n==1:
 print('libfranka: UDP receive: Timeout',file=sys.stderr);sys.exit(1)
if mode=='stall' and n==1:time.sleep(10)
if mode=='flap':sample();sys.exit(1)
while True:sample();time.sleep(.02)
''')
        reader.chmod(0o755)
        (root / 'mode').write_text(mode)
        sink = socket.socket()
        sink.bind(('127.0.0.1', 0))
        sink.listen()
        sink.settimeout(.05)
        self.addCleanup(sink.close)
        packets = []
        done = threading.Event()

        def receive():
            while not done.is_set():
                try:
                    connection, _ = sink.accept()
                except socket.timeout:
                    continue
                with connection:
                    connection.settimeout(.05)
                    data = b''
                    while not done.is_set():
                        try:
                            chunk = connection.recv(65536)
                        except socket.timeout:
                            continue
                        if not chunk:
                            break
                        data += chunk
                        while b'\n' in data:
                            line, data = data.split(b'\n', 1)
                            packets.append(json.loads(line))
        thread = threading.Thread(target=receive)
        thread.start()
        # Seed prior history to check monotone sequence across a relay restart.
        output = root / 'output'
        output.mkdir()
        (output / 'telemetry.ndjson').write_text('{"seq":101106}\n')
        script = ('import sys;sys.path.insert(0,sys.argv[1]);import relay;'
                  'raise SystemExit(relay.main(sys.argv[4:],reader_path=sys.argv[2],'
                  'poll_hand=False,lock_path=sys.argv[3],use_owner_socket=False))')
        command = [sys.executable, '-c', script, str(BRIDGE), str(reader), str(root/'lock'),
                   '--port', str(sink.getsockname()[1]), '--output', str(output),
                   '--duration', str(duration), '--reader-retries', str(retries),
                   '--retry-initial-s', '.05', '--retry-max-s', '.1',
                   '--reader-healthy-s', '.3', '--reader-idle-timeout-s', str(idle)]
        try:
            result = subprocess.run(command, capture_output=True, text=True, timeout=8)
        finally:
            done.set()
            thread.join(timeout=2)
        events = [json.loads(line) for line in (output/'reader_events.ndjson').read_text().splitlines()]
        return result, json.loads((output/'final_status.json').read_text()), events, packets

    def test_udp_timeout_then_fresh_reader(self):
        result, status, events, packets = self.scenario('recover')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(status['reader_restarts'], 1)
        self.assertTrue(packets)
        self.assertEqual(packets[0]['seq'], 101107)
        self.assertEqual([p['seq'] for p in packets], list(range(101107, 101107+len(packets))))
        self.assertTrue(any(e['event']=='reader_healthy_retry_budget_reset' for e in events))

    def test_permanent_failure_has_bounded_capped_backoff(self):
        result, status, events, packets = self.scenario('fail')
        self.assertEqual(result.returncode, 1)
        self.assertEqual(status['reader_state'], 'exhausted')
        self.assertTrue(status['stale'])
        self.assertEqual(len([e for e in events if e['event']=='reader_started']), 3)
        self.assertEqual([e['delay_s'] for e in events if e['event']=='reader_retry_scheduled'], [.05,.1])
        self.assertEqual(packets, [])

    def test_flapping_samples_do_not_reset_budget(self):
        result, status, _, packets = self.scenario('flap')
        self.assertEqual(result.returncode, 1)
        self.assertEqual(status['reader_restarts'], 2)
        self.assertEqual(len(packets), 3)

    def test_hung_reader_is_reaped_and_replaced(self):
        result, status, events, packets = self.scenario('stall')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertTrue(any(e.get('reason')=='reader_idle_watchdog' for e in events))
        self.assertEqual(status['reader_restarts'], 1)
        self.assertTrue(packets)

    def test_old_samples_never_clear_stale_or_reach_sink(self):
        result, status, _, packets = self.scenario('stale')
        self.assertEqual(result.returncode, 1)
        self.assertTrue(status['stale'])
        self.assertEqual(packets, [])

    def test_duration_is_global_across_restarts(self):
        result, status, _, _ = self.scenario('fail', duration=.12, retries=20)
        self.assertEqual(result.returncode, 0)
        self.assertEqual(status['exit_reason'], 'duration_elapsed')
        self.assertLess(status['elapsed_s'], .5)


if __name__ == '__main__':
    unittest.main()
