#!/usr/bin/python3
"""Resident ordered row deliveries using the existing factory state machine."""
import argparse
from datetime import datetime,timezone
import json
import os
from pathlib import Path
import sys
import time
import cv2
import numpy as np

from run_cycle import Factory,HERE,TEACHING,Stop,save,capture_pose
from setups import SETUP_NAMES,resolve_setup


def parse_args(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument('--setup',required=True,choices=SETUP_NAMES,
                        help='Select per-invocation placement/home geometry; never changes source poses')
    parser.add_argument('columns',nargs='*')
    parser.add_argument('--all',dest='all_columns',action='store_true',
                        help='Expand to all 18 source columns, 0_0 through 5_2, in row-major order')
    parser.add_argument('--execute',action='store_true')
    parser.add_argument('--validate-only',action='store_true',
                        help='Validate selected paths using the saved teaching seed; no SDK, cameras or robot commands')
    parser.add_argument('--verify-placement',action='store_true',
                        help='Enable post-release camera verification and bounded inspection views (default: off)')
    parser.add_argument('--confirm-empty-gripper',action='store_true',
                        help='Operator confirms no bricks are held; archive pending operation and restart after live Idle/open-hand checks')
    parser.add_argument('--execute-task',action='store_true',
                        help='Execute taught poses without camera/occupancy/visual-verification or schedule gates; includes empty-gripper confirmation; motion/fault/contact checks retained')
    args=parser.parse_args(argv)
    if args.all_columns:
        if args.columns:parser.error('Use --all or explicit column IDs, not both')
        args.columns=[f'{row}_{col}' for row in range(6) for col in range(3)]
    allowed={f'{r}_{m}' for r in range(6) for m in range(3)}
    if not args.columns or any(s not in allowed for s in args.columns) or len(set(args.columns))!=len(args.columns):
        parser.error('Use distinct taught source IDs 0_0 through 5_2, or --all')
    return args


def main():
    args=parse_args()
    if args.all_columns:print('COLUMNS (--all):',' '.join(args.columns),flush=True)
    if args.execute_task:
        args.confirm_empty_gripper=True;args.verify_placement=False
        print('TASK_MODE: camera/occupancy/visual-verification, schedule and historical geometry-commissioning gates disabled for the selected supported setup; operator manages conveyor clearing; physical guards retained',flush=True)
    _,_,_,setup=resolve_setup(args.setup,json.loads((HERE/'settings.json').read_text()),
        json.loads((TEACHING/'factory_vision/config.json').read_text()),
        json.loads((TEACHING/'motion_config.json').read_text()))
    print('SETUP:',setup['description'],'offsets world XYZ (mm):',
          [round(v*1000,3) for v in setup['placement_home_offset_world_m']],
          'placement:',setup['placement_mode'],flush=True)
    print('PLACEMENT_ONLY measured Z correction (mm):',
          round(setup['placement_height_correction_m']*1000,3),'home heights unchanged',flush=True)
    if setup['moving_parameters']:
        moving=setup['moving_parameters']
        print('TRACKING: world +Y',moving['belt_velocity_base_m_s'][1],'m/s; travel',
              moving['forward_travel_m']*1000,'mm; measured release speed limits',
              [moving['minimum_release_vy_m_s'],moving['maximum_release_vy_m_s']],'m/s',flush=True)
        print('GRIPPER RELEASE SPEED CHECK:',
              'warning-only (--execute-task)' if args.execute_task else 'enforced',flush=True)
    print('WORKSPACE (m):',setup['workspace_bounds_m'],flush=True)
    cv2.setNumThreads(1);os.sched_setaffinity(0,{3,4,5,9,10,11})
    out=HERE/'runs'/('rows_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ'));out.mkdir()
    factories=[];transitions=[];results=[];owner=camera=None;current=None
    try:
        if args.validate_only:
            point=json.loads((TEACHING/'positions.json').read_text())['points']['point_5_0']
            snapshot=dict(F_T_EE=point['tool_frames']['F_T_EE'],q_d=point['q_rad'])
            print('VALIDATION_ONLY: saved teaching tool/joint seed; no live robot state or physical clearance inferred',flush=True)
        else:
            snapshot=capture_pose()['state']
            if snapshot['robot_mode']!='Idle' or snapshot['current_errors'] or max(map(abs,snapshot['dq']))>=.01:
                raise Stop('Ordered delivery requires stationary Idle robot with no errors')
        for label in args.columns:
            row=int(label.split('_')[0]);directory=out/label;directory.mkdir()
            f=Factory(directory,[label],home_layer=row,initial_state=snapshot,
                      placement_verification=args.verify_placement,
                      empty_gripper_confirmed=args.confirm_empty_gripper,
                      task_execution=args.execute_task,setup=args.setup)
            factories.append(f)
            save(directory/'display_geometry.json',dict(setup=f.planner.setup,source=label,placement_O_T_EE=f.place,home_O_T_EE=f.home,
                basis='Existing per-row Planner.home mapping and configured conveyor-only surface correction'))
            print('PREFLIGHT',label,'source',f.plans[label]['grasp'][12:15],'conveyor',f.place[12:15],flush=True)
        for first,second in zip(factories,factories[1:]):
            transitions.append(first.planner.path('row_home_transition',first.home,second.home,first.qhome))
        first=factories[0]
        start_pose=first.planner.model.fk(snapshot['q_d']).homogeneous.reshape(-1,order='F').tolist()
        initial_path=first.planner.path('initial_home_preflight',start_pose,first.home,snapshot['q_d'])
        certificates=[]
        for f in factories:
            paths=list(f.common.values())
            for item in f.plans.values():paths.extend(item['paths'].values())
            for item in f.inspection_recovery:paths.extend((item['path'],item['home']))
            certificates.extend(f.planner.preflight_path(path) for path in paths)
        certificates.extend(first.planner.preflight_path(path) for path in [initial_path]+transitions)
        save(out/'preflight.json',dict(setup=setup,columns=args.columns,completed=True,source_poses_unchanged=True,
            all_columns=args.all_columns,
            placement_mode=setup['placement_mode'],placement_verification_enabled=args.verify_placement,
            empty_gripper_confirmed=args.confirm_empty_gripper,
            task_execution=args.execute_task,
            validation_only=args.validate_only,initial_home_path=initial_path,
            path_preflight=certificates,
            cache_identities=[f.planner.identity for f in factories],
            homes=[f.home for f in factories],transitions=transitions))
        if not args.execute or args.validate_only:
            print('PREFLIGHT_COMPLETE',out,flush=True);return
        current=factories[0]
        try:current.initialize()
        finally:owner=current.owner;camera=current.camera
        shared=current.inventory
        for label in args.columns:shared['available'][label]=dict(status='User requested ordered delivery from freshly populated fixed source',run=str(out))
        save(HERE/'inventory.json',shared)
        # All SDK commands share this one owner; the cameras also stay resident.
        for f in factories:
            f.owner=owner;f.camera=camera;f.inventory=shared
            if f is not current:
                f.ids['common']={k:owner.load(p) for k,p in f.common.items()}
                f.ids['inspection_recovery']=[dict(path=owner.load(p['path']),home=owner.load(p['home'])) for p in f.inspection_recovery]
                for label,item in f.plans.items():f.ids[label]={k:owner.load(p) for k,p in item['paths'].items()}
        transition_ids=[owner.load(p) for p in transitions]
        for index,(label,f) in enumerate(zip(args.columns,factories)):
            current=f
            if index:
                owner.begin();f.motion('row_home_transition',transitions[index-1],transition_ids[index-1]);owner.finish()
            print('CYCLE_START',label,flush=True)
            result=f.cycle(label);results.append(result)
            save(out/'outcome.json',dict(completed=False,in_progress=True,columns=args.columns,cycles=results,
                correct_picks=sum(r['correct_picks'] for r in results),
                correct_placements=sum(r['correct_placements'] for r in results),
                placements_released=len(results),placement_verification_enabled=args.verify_placement,stops=0))
            print('CYCLE_COMPLETE',label,'seconds',result.get('total_duration_s'),flush=True)
        save(out/'outcome.json',dict(completed=True,columns=args.columns,cycles=results,
            correct_picks=sum(r['correct_picks'] for r in results),
            correct_placements=sum(r['correct_placements'] for r in results),
            placements_released=len(results),placement_verification_enabled=args.verify_placement,stops=0))
        print('BATCH_COMPLETE',out,flush=True)
    except Exception as error:
        recovery=None
        if current and owner and current.inventory.get('active'):
            active=current.inventory['active'];active['last_stop']=str(error)
            save(HERE/'inventory.json',current.inventory)
            if active['phase'] in ('release_started_placement_unverified','released_placement_unverified'):
                try:recovery=current.recover_released_home(active['pose'])
                except Exception as e:recovery=dict(returned_home=False,error=str(e))
        save(out/'outcome.json',dict(completed=False,error=str(error),columns=args.columns,cycles=results,
            correct_picks=sum(f.confirmed_picks for f in factories),correct_placements=sum(f.confirmed_placements for f in factories),
            stops=1,bounded_recovery=recovery,unverified_inventory_not_advanced=True))
        print('RECOVERABLE_STOP',error,'LOG',out,flush=True)
        raise SystemExit(1)
    finally:
        if owner:owner.close()
        if camera:camera.close()


if __name__=='__main__':main()
