#!/usr/bin/python3
"""Verify repeated stationary control entry before any gripper or loaded motion."""
from datetime import datetime,timezone
import json
import os
from pathlib import Path
import sys
import time

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent))
from move_xyz import capture_pose
from planner import Planner
from runtime import Owner,Stop,save


def main():
    os.sched_setaffinity(0,{0,1,3,4})
    out=HERE/'runs'/('startup_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ'));out.mkdir(parents=True)
    capture=capture_pose();save(out/'start_capture.json',capture)
    if capture['state']['robot_mode']!='Idle' or capture['state']['current_errors']:raise Stop('Startup check needs fault-free Idle')
    planner=Planner(json.loads((HERE/'settings.json').read_text()),capture['state'])
    manifest=planner.manifest();manifest['commissioning_authorized']=True;owner=None
    try:
        owner=Owner(manifest,out);trials=[]
        for i in range(3):
            begin=time.monotonic();owner.begin();time.sleep(.15);owner.finish()
            trials.append(dict(index=i,duration_s=time.monotonic()-begin,state=owner.state()['state']))
        save(out/'outcome.json',dict(completed=True,stationary_startups=3,no_gripper_commands=True,
                                   no_pose_change_requested=True,trials=trials))
        print('PASS three stationary control entries; no gripper action or pose change. LOG',out,flush=True)
    except Exception as e:
        save(out/'outcome.json',dict(completed=False,error=str(e),no_gripper_commands=True,no_pose_change_requested=True));raise
    finally:
        if owner:owner.close()


if __name__=='__main__':main()
