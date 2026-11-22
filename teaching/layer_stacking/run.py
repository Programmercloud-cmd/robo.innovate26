"""Whole-layer planner, synthetic software rehearsal, or commissioned hardware."""
import argparse
import copy
from datetime import datetime,timezone
import json
import os
from pathlib import Path

import cv2

from .geometry import build_plan,ROOT
from .pipeline import LayerStacker,RecoverableStop,save

HERE=Path(__file__).resolve().parent


def main():
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument('--config',type=Path,default=HERE/'config.json')
    mode=parser.add_mutually_exclusive_group();mode.add_argument('--plan',action='store_true');mode.add_argument('--simulate',action='store_true');mode.add_argument('--execute',action='store_true')
    parser.add_argument('--sources',nargs='+',help='Whole footprints: external_r0_c0 through external_r5_c1')
    parser.add_argument('--out',type=Path)
    parser.add_argument('--confirm-empty-gripper',action='store_true')
    parser.add_argument('--reset-confirmed',action='store_true',help='Operator restored selected sources, cleared the receiving target and confirmed no bricks held after a previous physical job')
    args=parser.parse_args();config=json.loads(args.config.read_text());cv2.setNumThreads(1)
    if args.sources:config['job']['sources']=args.sources
    plan=build_plan(config)
    if args.plan or not (args.simulate or args.execute):
        out=args.out or HERE/'poses.simulation.json';save(out,plan)
        print('PLAN_ONLY',out,'simulation coordinates; brick-bottom poses, not real robot TCP targets');return
    out=args.out or HERE/'runs'/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ');out.parent.mkdir(parents=True,exist_ok=True)
    if args.execute:
        os.sched_setaffinity(0,{3,4,5,9,10,11})
        from .hardware import Hardware
        try:backend=Hardware(config,out,confirm_empty=args.confirm_empty_gripper,reset_confirmed=args.reset_confirmed)
        except RecoverableStop as error:parser.exit(2,'RECOVERABLE_STOP '+str(error)+'\n')
        print('HARDWARE_PROFILE: wide layer tool, 6 mm cavity, 70/60 mm open/close; stationary vertical seating; commissioning',config['hardware']['commissioning_id'],flush=True)
    else:
        from .fixture import Fixture
        backend=Fixture(config)
        print('SYNTHETIC_SOFTWARE_REHEARSAL: generated camera pixels and unvalidated wrench surrogate; not physical engagement')
    try:stacker=LayerStacker(config,plan,backend,out)
    except RecoverableStop as error:parser.exit(2,'RECOVERABLE_STOP '+str(error)+'\n')
    result=stacker.run()
    if args.simulate:save(out/'choreography.json',dict(domain='synthetic_software_rehearsal',plan=plan,events=stacker.events,trajectory=backend.trajectory))
    print(json.dumps(result));print('LOG',out)
    if not result['completed']:raise SystemExit(1)


if __name__=='__main__':main()
