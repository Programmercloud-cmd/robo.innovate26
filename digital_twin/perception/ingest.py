"""Receive display files on stdin over SSH; never connect to hardware.

One newline-delimited envelope contains an observation packet and optional JPEGs.
Only fixed file names under --output are written. Disconnects retain original
camera timestamps; consumers independently expire every observation.
"""
import argparse
import base64
import json
import os
import select
import sys
import time
from pathlib import Path
from observations import validate


def atomic(path, data):
    tmp = path.with_suffix(path.suffix + '.tmp')
    tmp.write_bytes(data)
    tmp.replace(path)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    last_ns = (0,0)
    current = None
    buffer = b''
    def expire(force=False):
        if current and not current.get('stale') and (force or time.time_ns()-current['frame_unix_ns']>1_000_000_000):
            current['stale'] = True
            current['unavailable_reason'] = 'perception_transport_closed' if force else 'camera_frame_expired'
            for slot in current['slots']+current.get('conveyor_slots',[]):
                slot.update(status='uncertain', color_id=None, confidence=0., visible=False, reason=current['unavailable_reason'])
            for slot in current.get('display_slots',[])+current.get('display_conveyor_slots',[]):
                slot.update(observation_status='stale',confirmed_in_current_frame=False)
                if slot.get('provenance')=='current_camera':slot['provenance']='last_confirmed_camera_unverified'
            atomic(args.output / 'brick_observations.json', json.dumps(current).encode())
    while True:
        ready, _, _ = select.select([sys.stdin.fileno()],[],[],.2)
        expire()
        if not ready:
            continue
        data = os.read(sys.stdin.fileno(),65536)
        if not data:
            expire(force=True)
            break
        buffer += data
        if len(buffer)>2_000_000:
            raise ValueError('Oversize display envelope')
        while b'\n' in buffer:
            line,buffer=buffer.split(b'\n',1)
            current,last_ns=ingest_line(line,args.output,current,last_ns)


def ingest_line(line, output, current, last_ns):
    try:
        if len(line) > 2_000_000:
            raise ValueError('Oversize display envelope')
        envelope = json.loads(line)
        packet = validate(envelope['observations'])
        key=(packet['frame_unix_ns'],packet.get('display_revision',0))
        if key <= last_ns:
            raise ValueError('Neither camera timestamp nor display revision advanced')
        for field, filename in [('overlay_jpeg', 'brick_overlay.jpg'), ('raw_jpeg', 'brick_camera.jpg')]:
            if envelope.get(field):
                data = base64.b64decode(envelope[field], validate=True)
                if len(data) > 800_000 or not data.startswith(b'\xff\xd8'):
                    raise ValueError('Invalid JPEG')
                atomic(output / filename, data)
        packet['display_received_unix_ns'] = time.time_ns()
        atomic(output / 'brick_observations.json', json.dumps(packet).encode())
        last_ns = key
        current = packet
    except (ValueError, KeyError, TypeError) as e:
        print('Rejected display observation: ' + str(e), file=sys.stderr, flush=True)
    return current,last_ns

if __name__ == '__main__':
    main()
