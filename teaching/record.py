#!/usr/bin/python3
"""Save a stationary, measured robot pose. Never sends motion commands."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
from pose_ids import normalize_pose_id, pose_key

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
CATALOG = HERE / 'positions.json'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('point', help='Source n_j (n=0..5, j=0..2), or numeric conveyor/reference ID')
    parser.add_argument('--label', default='')
    parser.add_argument('--note', default='')
    parser.add_argument('--above-grasp-mm', type=float, help='Override catalog default grasp offset')
    parser.add_argument('--vertical', action='store_true', help='Record operator-declared vertical alignment')
    parser.add_argument('--capture-file', type=Path, help='Import an existing read_pose capture instead of rereading the arm')
    parser.add_argument('--replace', action='store_true', help='Explicitly replace an existing point; old catalog is archived')
    args = parser.parse_args()
    try:
        identifier=normalize_pose_id(args.point);name=pose_key(identifier)
    except ValueError as error:parser.error(str(error))
    image_point=int(identifier.split('_')[0])+(1 if '_' in identifier else 0)
    if not CATALOG.exists():parser.error('Factory pose catalog is missing')
    catalog=json.loads(CATALOG.read_text())
    if catalog.get('task_id')!='factory_columns_of_four':parser.error('Not the isolated factory pose catalog')
    image=ROOT/catalog['reference_image']
    if hashlib.sha256(image.read_bytes()).hexdigest()!=catalog['reference_image_sha256']:
        parser.error('Factory reference image changed; update its configuration before recording')
    if name in catalog['points'] and not args.replace:
        parser.error(f'{name} already exists; use --replace only when intentionally reteaching it')
    if args.above_grasp_mm is not None and (not math.isfinite(args.above_grasp_mm) or args.above_grasp_mm < 0):
        parser.error('--above-grasp-mm must be finite and nonnegative')
    # Operator-established convention for all recorded image points.
    catalog.setdefault('default_grasp_offset_base_m', [0, 0, 0])
    if args.capture_file:
        capture = json.loads(args.capture_file.read_text())
        captured = datetime.fromtimestamp(args.capture_file.stat().st_mtime, timezone.utc)
    else:
        result = subprocess.run([str(HERE / 'read_pose'),'--allow-user-stopped'], capture_output=True, text=True, timeout=10)
        if result.returncode:
            print('STOP: pose was not recorded: '+(result.stderr.strip() or result.stdout.strip()), file=sys.stderr)
            raise SystemExit(result.returncode)
        capture = json.loads(result.stdout)
        captured = datetime.now(timezone.utc)
    state = capture['state']
    if state['current_errors'] or state['robot_mode'] not in ('Idle', 'Guiding', 'User stopped'):
        raise RuntimeError('Capture is not fault-free in a supported stationary recording mode')
    check = capture['stationary_check']
    if check['duration_ms'] < 500 or check['max_joint_speed_rad_s'] > .01 or check['max_joint_drift_rad'] > .001 or check['max_tcp_drift_m'] > .0005:
        raise RuntimeError('Capture failed the stationary check')
    pose = state['O_T_EE']
    if len(pose) != 16 or len(state['q']) != 7 or not all(math.isfinite(x) for x in pose + state['q']):
        raise RuntimeError('Invalid measured pose/joints')
    stamp = captured.strftime('%Y%m%dT%H%M%S.%fZ')
    history = HERE / 'history'
    history.mkdir(exist_ok=True)
    raw = history / f'{name}_{stamp}.json'
    # Exclusive creation preserves the original measurement as an immutable record.
    with raw.open('x') as output:
        json.dump(capture, output, indent=2, allow_nan=False)
        output.write('\n')
    point = dict(image_point=image_point, pose_id=identifier, label=args.label or identifier, recorded_utc=captured.isoformat(),
                 xyz_m=pose[12:15], O_T_EE=pose, q_rad=state['q'], elbow=state['elbow'],
                 tool_frames={key: state[key] for key in ('F_T_EE', 'F_T_NE', 'NE_T_EE', 'EE_T_K')},
                 tool_mass_kg=state['m_ee'], payload_mass_kg=state['m_load'],
                 operator_declared_vertical=args.vertical,
                 measured_tool_z_tilt_from_base_down_deg=math.degrees(math.acos(max(-1, min(1, -pose[10])))),
                 note=args.note, stationary_check=check, raw_capture=str(raw.relative_to(ROOT)),
                 motion_to_pose_validated=False)
    offset = ([0, 0, -args.above_grasp_mm / 1000] if args.above_grasp_mm is not None
              else catalog['default_grasp_offset_base_m'].copy())
    derived = pose.copy()
    for axis in range(3):
        derived[12 + axis] += offset[axis]
    direct=all(value==0 for value in offset)
    point['pose_kind']='direct_grasp' if direct else 'upper_reference'
    point['grasp_reference'] = dict(source='Directly taught grasp pose; no offset' if direct else 'operator-provided vertical offset; derived, not physically taught',
                                    taught_directly=direct,
                                    offset_from_recorded_pose_base_m=offset,
                                    xyz_m=derived[12:15], O_T_EE=derived, motion_validated=False)
    if 'gripper_state' in capture:
        point['gripper_state_at_recording']=capture['gripper_state']
    point['task_id']=catalog['task_id']
    point['role']=catalog['layout']['reference_roles'].get(identifier,'additional_taught_pose')
    catalog['points'][name] = point
    if CATALOG.exists():
        backup_stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
        with (history / f'positions_before_{backup_stamp}.json').open('x') as backup:
            backup.write(CATALOG.read_text())
    temporary = HERE / '.positions.json.tmp'
    temporary.write_text(json.dumps(catalog, indent=2, allow_nan=False) + '\n')
    temporary.replace(CATALOG)
    print(f'Saved {name}: XYZ (m) {point["xyz_m"]}')
    print(f'Measured tool-Z tilt from downward vertical: {point["measured_tool_z_tilt_from_base_down_deg"]:.3f} deg')
    if 'grasp_reference' in point:
        print(('Directly recorded grasp XYZ (m): ' if direct else 'Derived grasp XYZ (m), not executed: ')+str(point['grasp_reference']['xyz_m']))
    print(f'Catalog: {CATALOG}')


if __name__ == '__main__':
    main()
