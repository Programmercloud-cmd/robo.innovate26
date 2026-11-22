#!/usr/bin/python3
"""Elevated empty-arm check of SDK stop preemption; no hand motion commands."""
from datetime import datetime,timezone
import hashlib
import json
import os
from pathlib import Path
import sys
import time

import numpy as np

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent))
from move_xyz import capture_pose
from planner import Planner
from runtime import Owner,Stop,save


def main():
    os.sched_setaffinity(0,{3,4,5,9,10,11})
    out=HERE/'runs'/('sdk_stop_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ'));out.mkdir(parents=True)
    capture=capture_pose();save(out/'start_capture.json',capture)
    planner=Planner(json.loads((HERE/'settings.json').read_text()),capture['state'])
    if capture['state']['O_T_EE'][14]-planner.frame('5_0')[14]<.09:raise Stop('Stop commissioning stays90mm+ above grasp height')
    start=planner.model.fk(capture['state']['q_d']).homogeneous.reshape(-1,order='F').tolist();target=start.copy();target[12]-=.02
    path=planner.path('elevated_stop_probe',start,target,capture['state']['q_d']);save(out/'plan.json',path)
    manifest=planner.manifest();manifest['commissioning_authorized']=True;owner=None;passed=False
    try:
        owner=Owner(manifest,out);id=owner.load(path);owner.begin();owner.command('go',id=id)
        owner.wait(lambda:owner.latest and owner.latest['phase']==id and owner.latest['phase_time_s']>=path['duration_s']*.45,path['duration_s']+1)
        before=owner.state();stamp=time.monotonic();after=owner.stop_and_wait();latency=time.monotonic()-stamp
        displacement=float(np.linalg.norm(np.array(after['state']['O_T_EE'][12:15])-before['state']['O_T_EE'][12:15]))
        if latency>.5 or displacement>.01 or after['state']['current_errors'] or after['state']['robot_mode']!='Idle':raise Stop('SDK stop exceeded bounded commissioning displacement/time or left a fault')
        owner.recover_idle()
        stopped_pose=planner.model.fk(after['state']['q_d']).homogeneous.reshape(-1,order='F').tolist()
        back=planner.path('elevated_stop_recovery',stopped_pose,start,after['state']['q_d']);back_id=owner.load(back)
        owner.begin();owner.go(back_id,back['duration_s']);owner.finish()
        report=dict(completed=True,no_gripper_commands=True,empty_elevated_motion=True,
            stop_response_s=latency,measured_displacement_after_request_m=displacement,
            same_sdk_owner_idle_recovery_and_return_verified=True,
            before=before['state'],after=after['state'],controller_sha256=hashlib.sha256((HERE/'controller').read_bytes()).hexdigest(),run=str(out))
        save(out/'outcome.json',report);save(HERE/'stop_verified.json',report);passed=True
        print(json.dumps(dict(completed=True,stop_response_s=latency,measured_stop_displacement_mm=displacement*1000,mode=after['state']['robot_mode'],errors=after['state']['current_errors'],run=str(out))),flush=True)
    except Exception as e:
        save(out/'outcome.json',dict(completed=False,error=str(e),no_gripper_commands=True));raise
    finally:
        if owner:owner.close()
        if not passed:os.chmod(HERE/'controller',0o644)


if __name__=='__main__':main()
