"""Consume the existing twin's recording without obtaining an SDK connection.

The upstream sample timestamp, including its age, is retained. Reopening or
re-reading an unchanged file never freshens telemetry.
"""
import json
import queue
import threading
import time
from pathlib import Path


class RecordingFollower:
    def __init__(self, active_session, validate):
        self.active_session = Path(active_session)
        self.validate = validate
        self.latest = queue.Queue(maxsize=1)
        self.stats = {"accepted": 0, "rejected": 0, "connections": 0,
                      "source": "existing_twin_recording", "hardware_clients": 0}
        self.stop = False
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def read_last(self):
        session = json.loads(self.active_session.read_text())
        path = Path(session['output']) / 'telemetry.ndjson'
        with path.open('rb') as stream:
            stream.seek(0, 2)
            length = stream.tell()
            start = max(0, length - 131072)
            stream.seek(start)
            data = stream.read()
        # Skip a possible truncated first record and an incomplete last write.
        lines = data.split(b'\n')
        if start:
            lines = lines[1:]
        for line in reversed(lines[:-1]):
            if line.strip():
                return self.validate(json.loads(line))
        return None

    def run(self):
        previous = None
        while not self.stop:
            try:
                frame = self.read_last()
                if frame is not None:
                    key = (frame['unix_ns'], frame['seq'])
                    if key != previous:
                        age = max(0., (time.time_ns() - frame['unix_ns']) / 1e9)
                        received = time.monotonic() - age
                        try:
                            self.latest.put_nowait((received, frame))
                        except queue.Full:
                            try:
                                self.latest.get_nowait()
                            except queue.Empty:
                                pass  # The simulation consumed it concurrently.
                            self.latest.put_nowait((received, frame))
                        previous = key
                        self.stats['accepted'] += 1
            except (OSError, ValueError, TypeError, KeyError):
                self.stats['rejected'] += 1
            time.sleep(.01)
