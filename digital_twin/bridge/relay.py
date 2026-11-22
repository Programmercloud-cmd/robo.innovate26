#!/usr/bin/python3
"""One-way FR3 telemetry with bounded receive-only reader reconnection."""
import argparse
from datetime import datetime, timezone
import fcntl
import json
import math
import os
from pathlib import Path
import selectors
import signal
import socket
import subprocess
import threading
import time
from hand_stream import HandStream,hand_fields

ROOT = Path(__file__).resolve().parents[2]


def atomic_json(path, value):
    temporary = path.with_name('.' + path.name + '.tmp')
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')
    temporary.replace(path)


def next_sequence(path):
    """Continue the append-only log across relay process restarts."""
    if not path.exists() or not path.stat().st_size:
        return 0
    with path.open('rb') as stream:
        stream.seek(max(0, path.stat().st_size - 131072))
        for line in reversed(stream.read().splitlines()):
            try:
                seq = json.loads(line)['seq']
                if type(seq) is int and seq >= 0:
                    return seq + 1
            except (ValueError, KeyError, TypeError):
                continue
    raise ValueError('Cannot recover sequence from existing telemetry log')


def main(argv=None, *, reader_path=None, poll_hand=True, lock_path=None, use_owner_socket=True):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot-ip', default='172.16.0.2')
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=18765)
    parser.add_argument('--hz', type=float, default=50)
    parser.add_argument('--duration', type=float, default=0, help='Total relay lifetime; 0 means unlimited')
    parser.add_argument('--reader-retries', type=int, default=5, help='Restarts per unhealthy episode after the initial attempt')
    parser.add_argument('--retry-initial-s', type=float, default=.5)
    parser.add_argument('--retry-max-s', type=float, default=8.)
    parser.add_argument('--reader-healthy-s', type=float, default=5.)
    parser.add_argument('--reader-idle-timeout-s', type=float, default=3.)
    parser.add_argument('--output', type=Path, default=ROOT / 'digital_twin/runs/live')
    parser.add_argument('--no-hand-polling',action='store_true')
    parser.add_argument('--foreground-only',action='store_true',
                        help='Forward SDK-owner measurements only; never connect background arm or hand readers')
    parser.add_argument('--hand-pause-file',type=Path,default=Path('/tmp')/('fr3-sdk-'+str(os.getuid()))/'hand_poll.pause')
    args = parser.parse_args(argv)
    values = [args.hz, args.duration, args.retry_initial_s, args.retry_max_s,
              args.reader_healthy_s, args.reader_idle_timeout_s]
    if (not all(math.isfinite(v) for v in values) or not 1 <= args.hz <= 100
            or args.duration < 0 or not 0 <= args.reader_retries <= 20
            or not 0 < args.retry_initial_s <= args.retry_max_s <= 60
            or args.reader_healthy_s <= 0 or args.reader_idle_timeout_s <= .5):
        parser.error('Invalid sampling, retry or watchdog limits')
    args.output.mkdir(parents=True, exist_ok=True)
    # Prevent duplicate relays interleaving the reader, sequence and log state.
    lock = Path(lock_path or ROOT / 'digital_twin/bridge/.relay.lock').open('a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    sequence = next_sequence(args.output / 'telemetry.ndjson')
    sequence_start = sequence
    samples = sent = dropped = restarts = failures = 0
    began = time.monotonic()
    deadline = began + args.duration if args.duration else math.inf
    last_sample_ns = None
    last_packet_ns = 0
    last_origin = None
    last_owner_hand = {}
    reader_state = 'starting'
    last_returncode = None
    next_start = began
    healthy_since = None
    process = connection = None
    network_retry_at = 0.
    network_backoff = .1
    selector = selectors.DefaultSelector()
    owner_socket = demand_lock = hand_demand_lock = None
    owner_socket_path = None
    if use_owner_socket:
        ipc = Path('/tmp') / ('fr3-sdk-' + str(os.getuid()))
        ipc.mkdir(mode=0o700, exist_ok=True)
        info = ipc.lstat()
        if not ipc.is_dir() or info.st_uid != os.getuid() or info.st_mode & 0o077:
            raise RuntimeError('SDK IPC directory must belong to this user, mode 0700')
        demand_lock = (ipc / 'foreground.lock').open('a')
        hand_demand_lock = (ipc / 'hand_foreground.lock').open('a')
        owner_socket_path = ipc / 'state.sock'
        owner_socket_path.unlink(missing_ok=True)
        owner_socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        owner_socket.bind(str(owner_socket_path))
        owner_socket_path.chmod(0o600)
        owner_socket.setblocking(False)
        selector.register(owner_socket, selectors.EVENT_READ, 'owner')

    def foreground_busy():
        if demand_lock is None:
            return False
        try:
            fcntl.flock(demand_lock, fcntl.LOCK_SH | fcntl.LOCK_NB)
        except BlockingIOError:
            return True
        fcntl.flock(demand_lock, fcntl.LOCK_UN)
        return False

    def hand_foreground_busy():
        if foreground_busy():return True
        if hand_demand_lock is None:return False
        try:fcntl.flock(hand_demand_lock,fcntl.LOCK_SH|fcntl.LOCK_NB)
        except BlockingIOError:return True
        fcntl.flock(hand_demand_lock,fcntl.LOCK_UN)
        return False
    buffer = b''
    last_reader_data = began
    last_status = 0.
    reason = 'stopped'
    result_code = 0
    metadata = dict(started_utc=datetime.now(timezone.utc).isoformat(), source='libfranka.readOnce',
                    read_only=True, hardware_motion_commands_sent=False, robot_ip=args.robot_ip,
                    target=[args.host, args.port], hz=args.hz, sequence_start=sequence_start,
                    reader_retries=args.reader_retries, retry_initial_s=args.retry_initial_s,
                    retry_max_s=args.retry_max_s, healthy_reset_s=args.reader_healthy_s,
                    reader_idle_timeout_s=args.reader_idle_timeout_s,
                    note='No control, configuration, recovery or hand movement API. During foreground SDK ownership, forwards measured states from that owner instead of opening a competing connection. No substitute packets during outage.')
    atomic_json(args.output / 'manifest.json', metadata)

    def status():
        age = (time.time_ns() - last_sample_ns) / 1e9 if last_sample_ns is not None else None
        return dict(samples=samples, sequence_start=sequence_start, next_sequence=sequence,
                    sent=sent, dropped=dropped, elapsed_s=time.monotonic() - began,
                    reader_state=reader_state, reader_pid=process.pid if process else None,
                    reader_restarts=restarts, consecutive_reader_failures=failures,
                    reader_returncode=last_returncode, latest_sample_unix_ns=last_sample_ns,
                    sample_age_s=age, stale=age is None or age > .5, read_only=True,
                    telemetry_origin=last_origin,
                    **hand_reader.status(),
                    forwarded_hand=hand_fields(hand_reader.sample(),owner_sample=last_owner_hand,
                                              is_owner=last_origin=='foreground_sdk_owner'),
                    hardware_motion_commands_sent=False)

    def retire_reader():
        nonlocal process, last_returncode
        if process is None:
            return
        selector.unregister(process.stdout)
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=.5)
            except subprocess.TimeoutExpired:
                process.kill()
        last_returncode = process.wait(timeout=1)
        process.stdout.close()
        process = None

    hand_reader=HandStream([str(ROOT/'digital_twin/bridge/read_hand'),args.robot_ip],
        args.output,args.hand_pause_file,hand_foreground_busy,
        enabled=poll_hand and not args.no_hand_polling and not args.foreground_only)
    thread = threading.Thread(target=hand_reader.run,args=(stop,),daemon=True) if hand_reader.enabled else None
    if thread:
        thread.start()
    try:
        with (args.output / 'telemetry.ndjson').open('a', buffering=1) as log, \
                (args.output / 'reader_errors.log').open('a', buffering=1) as errors, \
                (args.output / 'reader_events.ndjson').open('a', buffering=1) as events:
            def event(kind, **details):
                entry = dict(unix_ns=time.time_ns(), event=kind, **details)
                events.write(json.dumps(entry) + '\n')
                print(json.dumps(entry), flush=True)

            while not stop.is_set() and time.monotonic() < deadline:
                now = time.monotonic()
                failure = None
                busy = foreground_busy()
                if busy:
                    if process is not None:
                        retire_reader()
                        event('reader_yielded_to_foreground_owner')
                    reader_state = 'foreground_owner'
                if not busy and args.foreground_only:
                    reader_state = 'waiting_for_foreground_owner'
                if not busy and not args.foreground_only and process is None and now >= next_start:
                    reader_state = 'starting'
                    buffer = b''
                    healthy_since = None
                    last_reader_data = now
                    try:
                        # Relay owns lifetime; replacement readers never reset --duration.
                        process = subprocess.Popen([str(reader_path or ROOT / 'digital_twin/bridge/read_state'),
                                                    args.robot_ip, str(args.hz), '0'],
                                                   stdout=subprocess.PIPE, stderr=errors)
                        selector.register(process.stdout, selectors.EVENT_READ, 'reader')
                        event('reader_started', pid=process.pid, restarts=restarts)
                    except OSError as error:
                        failure = 'reader_launch_failed: ' + str(error)
                ready = selector.select(timeout=min(.05, max(0., deadline - now)))
                for key, _ in ready:
                    is_owner = key.data == 'owner'
                    if is_owner:
                        lines = [owner_socket.recv(1048576).strip()]
                    else:
                        data = os.read(process.stdout.fileno(), 65536)
                        if not data:
                            last_returncode = process.poll()
                            if last_returncode == 75 or foreground_busy():
                                retire_reader()
                                reader_state = 'foreground_owner'
                                event('reader_yielded_to_foreground_owner')
                            else:
                                failure = 'reader_stdout_closed'
                            continue
                        buffer += data
                        if len(buffer) > 1048576:
                            failure = 'reader_line_too_large'
                            break
                        lines = []
                        while b'\n' in buffer:
                            line, buffer = buffer.split(b'\n', 1)
                            lines.append(line)
                    for line in lines:
                        try:
                            sample = json.loads(line)
                            state = sample['state']
                            q, dq = state['q'], state['dq']
                            if len(q) != 7 or len(dq) != 7 or not all(math.isfinite(x) for x in q + dq):
                                raise ValueError('Invalid joint state')
                            stamp = sample['unix_ns']
                            if type(stamp) is not int or stamp <= last_packet_ns or not 0 <= (time.time_ns() - stamp) / 1e9 <= .5:
                                raise ValueError('Stale, future or non-monotonic measurement timestamp')
                            packet = dict(schema='fr3.telemetry.v1', seq=sequence, unix_ns=stamp,
                                          source='live_hardware_read_only',
                                          telemetry_origin='foreground_sdk_owner' if is_owner else 'background_read_once',
                                          **{k: state[k] for k in ('q', 'dq', 'q_d', 'O_T_EE', 'F_T_EE',
                                             'O_T_EE_d', 'tau_J', 'tau_J_d', 'robot_mode', 'current_errors')},
                                          robot_time_ms=state['time'], robot_time_s=state['time'] / 1000.)
                            packet.update(hand_fields(hand_reader.sample(),owner_sample=sample,is_owner=is_owner))
                            if is_owner:
                                last_owner_hand={k:sample[k] for k in ('width_m','width_unix_ns') if k in sample}
                            encoded = json.dumps(packet, separators=(',', ':'), allow_nan=False) + '\n'
                        except (ValueError, KeyError, TypeError) as error:
                            if is_owner:
                                event('owner_sample_discarded', reason=str(error))
                                continue
                            failure = 'invalid_reader_sample: ' + str(error)
                            break
                        last_packet_ns = last_sample_ns = stamp
                        last_origin = packet['telemetry_origin']
                        if not is_owner:
                            last_reader_data = time.monotonic()
                            if healthy_since is None:
                                healthy_since = last_reader_data
                                event('fresh_reader_sample', seq=sequence)
                            if failures and last_reader_data - healthy_since >= args.reader_healthy_s:
                                event('reader_healthy_retry_budget_reset', previous_failures=failures)
                                failures = 0
                        reader_state = 'foreground_owner_streaming' if is_owner else 'streaming'
                        log.write(encoded)
                        sequence += 1
                        samples += 1
                        if connection is None and time.monotonic() >= network_retry_at:
                            try:
                                connection = socket.create_connection((args.host, args.port), timeout=.05)
                                connection.settimeout(.05)
                                network_backoff = .1
                            except OSError:
                                network_retry_at = time.monotonic() + network_backoff
                                network_backoff = min(2., network_backoff * 2)
                        if connection is None:
                            dropped += 1
                        else:
                            try:
                                connection.sendall(encoded.encode())
                                sent += 1
                            except OSError:
                                connection.close()
                                connection = None
                                network_retry_at = time.monotonic() + network_backoff
                                dropped += 1
                if process is not None:
                    if time.monotonic() - last_reader_data > .5:
                        healthy_since = None
                    if time.monotonic() - last_reader_data >= args.reader_idle_timeout_s:
                        failure = 'reader_idle_watchdog'
                if failure:
                    retire_reader()
                    failures += 1
                    event('reader_failed', reason=failure, returncode=last_returncode,
                          consecutive_failures=failures)
                    if failures > args.reader_retries:
                        reader_state = 'exhausted'
                        reason = 'reader_retry_budget_exhausted'
                        result_code = 1
                        break
                    delay = min(args.retry_max_s, args.retry_initial_s * 2 ** (failures - 1))
                    next_start = time.monotonic() + delay
                    restarts += 1
                    reader_state = 'backoff'
                    event('reader_retry_scheduled', delay_s=delay, restart=restarts)
                if time.monotonic() - last_status >= .1:
                    atomic_json(args.output / 'status.json', status())
                    last_status = time.monotonic()
            if time.monotonic() >= deadline:
                reason = 'duration_elapsed'
    finally:
        stop.set()
        retire_reader()
        if connection:
            connection.close()
        if owner_socket:
            owner_socket.close()
            owner_socket_path.unlink(missing_ok=True)
        if demand_lock:
            demand_lock.close()
        if hand_demand_lock:
            hand_demand_lock.close()
        selector.close()
        if thread:
            thread.join(timeout=3)
        if reader_state != 'exhausted':
            reader_state = 'stopped'
        final = dict(status(), exit_reason=reason)
        atomic_json(args.output / 'status.json', final)
        atomic_json(args.output / 'final_status.json', final)
        print(json.dumps(final), flush=True)
        lock.close()
    return result_code


if __name__ == '__main__':
    raise SystemExit(main())
