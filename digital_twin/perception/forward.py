"""Forward perception files to the twin and publish a read-only motion guard.

Consumes the existing HTTP measured-state mirror, never the robot SDK. A single
SSH stdin stream transports independent perception data. Camera capture remains
owned exclusively by teaching/external_layout. No execution inventory is read
or written here.
"""
import argparse
import base64
import fcntl
import json
import os
import select
import shlex
import signal
import subprocess
import threading
import time
import urllib.request
from pathlib import Path
from observations import validate
from display_state import DisplayState, InventoryObserver


def atomic_json(path, data):
    temp = path.with_suffix('.tmp')
    temp.write_text(json.dumps(data))
    temp.replace(path)


def guard_loop(output, url, stopped):
    previous = None
    moving_until = 0.
    while not stopped.is_set():
        now = time.monotonic()
        result = dict(observed_unix_ns=time.time_ns(), stale=True, robot_moving=True)
        try:
            with urllib.request.urlopen(url, timeout=.3) as r:
                state = json.load(r)
            q = state['q_measured']
            age = (time.time_ns() - state['unix_ns']) / 1e9
            stale = state.get('stale', True) or not 0 <= age < .5
            moved = state.get('robot_mode') == 'Move'
            if previous:
                dt = now - previous[0]
                moved |= dt > 0 and max(abs(a-b) for a,b in zip(q, previous[1])) / dt > .02
            if moved or stale:
                moving_until = now + .5
            result.update(stale=stale, robot_moving=now < moving_until,
                          robot_sample_unix_ns=state['unix_ns'], q_measured=q,
                          source='existing_twin_http')
            previous = (now, q)
        except (OSError, ValueError, KeyError, TypeError):
            moving_until = now + .5
        atomic_json(output / 'robot_guard.json', result)
        stopped.wait(.1)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input', type=Path, required=True)
    p.add_argument('--remote', default='robot-innovate26-place-rl3')
    p.add_argument('--remote-script', required=True)
    p.add_argument('--remote-output', required=True)
    p.add_argument('--robot-url', default='http://127.0.0.1:18766/latest.json')
    p.add_argument('--layout', type=Path, help='Separate display-only slot transforms; never execution poses')
    args = p.parse_args()
    args.input.mkdir(parents=True, exist_ok=True)
    process_lock = (args.input/'forward.lock').open('a')
    fcntl.flock(process_lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    layout = json.loads(args.layout.read_text()) if args.layout else None
    root=Path(__file__).resolve().parents[2]
    external=json.loads((root/'teaching/external_layout/config.json').read_text())
    display=DisplayState(Path(__file__).with_name('display_state.json'),layout,
        json.loads((root/'teaching/positions.json').read_text()),
        json.loads((root/'teaching/factory_vision/config.json').read_text()),
        policy=external.get('display_policy',{})) if layout else None
    inventory=InventoryObserver(root/'teaching/factory_cycle/inventory.json')
    stopped = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stopped.set())
    signal.signal(signal.SIGINT, lambda *_: stopped.set())
    guard = threading.Thread(target=guard_loop, args=(args.input, args.robot_url, stopped), daemon=True)
    guard.start()
    process = None
    last_stamp = -1
    retry_after = 0.
    backoff = .5
    try:
        while not stopped.is_set():
            try:
                if process is None or process.poll() is not None:
                    if time.monotonic() < retry_after:
                        stopped.wait(.1)
                        continue
                    command = 'python3 -u ' + shlex.quote(args.remote_script) + ' --output ' + shlex.quote(args.remote_output)
                    process = subprocess.Popen(['ssh','-T','-o','BatchMode=yes','-o','ConnectTimeout=5',
                        '-o','ServerAliveInterval=5','-o','ServerAliveCountMax=2',args.remote,command], stdin=subprocess.PIPE)
                    os.set_blocking(process.stdin.fileno(), False)
                    last_stamp = -1
                with (args.input / 'publish.lock').open('a') as lock:
                    fcntl.flock(lock,fcntl.LOCK_SH)
                    packet = validate(json.loads((args.input / 'latest.json').read_text()))
                    envelope = {'observations': packet}
                    for key, file in [('overlay_jpeg','overlay.jpg'),('raw_jpeg','raw.jpg')]:
                        path = args.input / file
                        if path.exists():
                            envelope[key] = base64.b64encode(path.read_bytes()).decode()
                if layout:
                    packet['display_registration'] = layout['registration']
                    transforms = {s['slot_id']: s for s in layout['slots']+layout.get('conveyor_slots',[])}
                    for slot in packet['slots']+packet.get('conveyor_slots',[]):
                        for field in ('display_base_xyz_m','display_size_m'):
                            slot[field] = transforms[slot['slot_id']][field]
                if display:
                    packet=display.update(packet,inventory.read())
                    atomic_json(args.input/'source_exclusions.json',display.source_exclusions())
                    envelope['observations']=packet
                stamp = (packet['frame_unix_ns'],packet.get('display_revision',0))
                if stamp == last_stamp:
                    stopped.wait(.1)
                    continue
                data = memoryview((json.dumps(envelope,separators=(',',':'))+'\n').encode())
                deadline = time.monotonic() + 2
                while data:
                    if stopped.is_set() or time.monotonic() > deadline:
                        raise TimeoutError('Display transport stalled')
                    _, ready, _ = select.select([], [process.stdin.fileno()], [], .1)
                    if ready:
                        try:
                            n = os.write(process.stdin.fileno(), data)
                            data = data[n:]
                        except BlockingIOError:
                            continue
                last_stamp = stamp
                backoff = .5
                stopped.wait(.1)
            except (FileNotFoundError, json.JSONDecodeError):
                stopped.wait(.2)
            except (OSError, ValueError, KeyError, TypeError) as e:
                print(str(e), flush=True)
                if process:
                    process.terminate()
                    try: process.wait(timeout=1)
                    except subprocess.TimeoutExpired: process.kill(); process.wait()
                    process = None
                retry_after = time.monotonic() + backoff
                backoff = min(8., backoff*2)
                stopped.wait(.2)
    finally:
        stopped.set()
        guard.join(timeout=1)
        atomic_json(args.input / 'robot_guard.json', dict(observed_unix_ns=time.time_ns(), stale=True, robot_moving=True))
        if process:
            process.terminate()
            try: process.wait(timeout=1)
            except subprocess.TimeoutExpired: process.kill(); process.wait()


if __name__ == '__main__':
    main()
