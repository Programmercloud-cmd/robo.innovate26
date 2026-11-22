#!/usr/bin/python3
"""Prepare/check the conveyor profile; physical execution requires measured release timing."""
import argparse
from datetime import datetime,timezone
import json
from pathlib import Path
import subprocess
import sys

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent))
from prepare import prepare
from freeze import freeze
from move_xyz import capture_pose,resolve_pose
from arc_motion import run_arc


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--layer',type=int,default=5)
    parser.add_argument('--execute',action='store_true');args=parser.parse_args()
    config=json.loads((HERE/'config.json').read_text())
    if args.execute and not (config['execution_enabled'] and config['release_timing_physically_validated']):
        parser.error('Physical execution is disabled until the 180 ms release window is measured and validated; use without --execute to prepare/check only')
    output=HERE/'runs'/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ');output.mkdir(parents=True)
    raw=(HERE.parent/'positions.json').read_bytes();catalog=json.loads(raw)
    if not args.execute:
        plan=prepare(args.layer,.0652,output);frozen=freeze(output/'plan.json')
        result=subprocess.run([str(HERE/'execute'),str(frozen),str(output/'trace.csv'),'--check'],capture_output=True,text=True)
        (output/'model_check.log').write_text(result.stdout+result.stderr);print(result.stdout+result.stderr)
        if result.returncode:return result.returncode
        print('Prepared only; no hardware connection or command. Output:',output);return 0
    progress_path=HERE.parent/'factory_vision/progress.json';progress=json.loads(progress_path.read_text());active=progress.get('active_operation')
    if not active or active['phase']!='retention_passed':raise RuntimeError('A verified retained four-brick grasp is required')
    if int(active['pose'].split('_')[0])!=args.layer:raise RuntimeError('Held source layer does not match requested home')
    label,point,_=resolve_pose(catalog,active['pose'])
    hand=json.loads(subprocess.check_output([str(HERE.parent/'gripper_width'),'--state'],text=True))
    if not config['loaded_width_min_m']<=hand['width']<=config['loaded_width_max_m']:raise RuntimeError('Loaded width outside measured range')
    plan=prepare(args.layer,hand['width'],output)
    def move(target,stage):
        code=run_arc(label,target,capture_pose(),raw,point,True,dict(stage=stage,endpoint_position_tolerance_m=.0015))
        if code:raise RuntimeError('Motion failed: '+stage)
    # Home is over the layer's conveyor point, never over the input source.
    move(plan['home_O_T_EE'],'factory_vision_moving_conveyor_home_approach')
    move(plan['hover_O_T_EE'],'factory_vision_moving_conveyor_hover')
    capture=capture_pose();(output/'hover_capture.json').write_text(json.dumps(capture,indent=2)+'\n')
    plan=prepare(args.layer,hand['width'],output,output/'hover_capture.json');frozen=freeze(output/'plan.json')
    result=subprocess.run([str(HERE/'execute'),str(frozen),str(output/'check.csv'),'--check'],capture_output=True,text=True)
    (output/'model_check.log').write_text(result.stdout+result.stderr)
    if result.returncode:raise RuntimeError('Independent timed model check failed')
    command=[str(HERE/'execute'),str(frozen),str(output/'trace.csv'),'--execute']
    motion=json.loads((HERE.parent/'motion_config.json').read_text())
    if motion.get('performance_profile_hold'):command=['powerprofilesctl','launch','--profile','performance','--reason','Franka timed conveyor tracking','--appid','franka-conveyor']+command
    result=subprocess.run(command,capture_output=True,text=True);log=result.stdout+result.stderr;(output/'execution.log').write_text(log);print(log)
    if result.returncode or 'PASS timed conveyor stroke' not in log.splitlines():
        progress['active_operation'].update(phase='moving_conveyor_uncertain',run=str(output));progress_path.write_text(json.dumps(progress,indent=2)+'\n')
        raise RuntimeError('Timed placement did not verify; no automatic return, release or recovery')
    progress['active_operation'].update(phase='released_pending_conveyor_verification',run=str(output));progress_path.write_text(json.dumps(progress,indent=2)+'\n')
    move(plan['home_O_T_EE'],'factory_vision_moving_conveyor_return_home')
    (output/'outcome.json').write_text(json.dumps(dict(release_verified=True,returned_home=True,placement_inventory_advanced=False,reason='Conveyor/object verification remains separate'),indent=2)+'\n')
    return 0


if __name__=='__main__':raise SystemExit(main())
