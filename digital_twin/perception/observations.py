"""Display-only observation contract. No robot/camera clients or inventory writes."""
import math
import time

COLORS = {
    '300121': [0.72, 0.025, 0.035],
    '6117418': [0.31, 0.035, 0.055],
    '6487946': [0.59, 0.26, 0.08],
    '6135191': [0.77, 0.66, 0.46],
    '4211201': [0.18, 0.09, 0.045],
}


def validate(packet):
    if packet.get('schema') != 'fr3.external_layout.v1':
        raise ValueError('Unsupported external-layout schema')
    if packet.get('camera_serial') != '03FB123F':
        raise ValueError('Wrong external camera')
    if not isinstance(packet.get('frame_unix_ns'), int):
        raise ValueError('Missing camera frame timestamp')
    if packet['frame_unix_ns'] > time.time_ns() + 500_000_000:
        raise ValueError('Future camera timestamp')
    slots = packet.get('slots', [])
    if len(slots) != 144 or len({s['slot_id'] for s in slots}) != 144:
        raise ValueError('Expected 144 unique display slots')
    conveyor=packet.get('conveyor_slots',[])
    if 'conveyor_slots' in packet:
        expected={f'conveyor_r{r}_n{n}' for r in range(6) for n in range(4)}
        if len(conveyor)!=24 or {s['slot_id'] for s in conveyor}!=expected:
            raise ValueError('Expected 24 unique configured conveyor slots')
    for s in slots+conveyor:
        if s['status'] not in ('occupied', 'empty', 'uncertain'):
            raise ValueError('Invalid slot status')
        c = s.get('color_id')
        if s['status'] == 'occupied' and str(c) not in COLORS:
            raise ValueError('Occupied slot needs a known color')
        if s['status'] == 'occupied' and s.get('visible') is not True:
            raise ValueError('Occluded slots cannot assert occupancy')
        if s['status'] != 'occupied' and c is not None:
            raise ValueError('Uncertain/empty slots cannot carry an observed color')
        confidence = s.get('confidence')
        if not isinstance(confidence, (int, float)) or not math.isfinite(confidence) or not 0 <= confidence <= 1:
            raise ValueError('Invalid confidence')
        xyz = s.get('display_base_xyz_m')
        if xyz is not None and (len(xyz) != 3 or not all(math.isfinite(v) for v in xyz)):
            raise ValueError('Invalid display position')
        size = s.get('display_size_m')
        if size is not None and (len(size) != 3 or not all(math.isfinite(v) and 0 < v < .1 for v in size)):
            raise ValueError('Invalid display size')
    return packet


def fresh(packet, now_ns=None):
    now_ns = time.time_ns() if now_ns is None else now_ns
    age = (now_ns - packet['frame_unix_ns']) / 1e9
    return not packet.get('stale', True) and 0 <= age <= 1.0
