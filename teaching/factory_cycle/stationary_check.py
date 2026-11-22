#!/usr/bin/python3
"""Validate stationary vertical placement while empty,50mm above the lane."""
from datetime import datetime,timezone
import hashlib
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
    os.sched_setaffinity(0,{3,4,5,9,10,11})
    out=HERE/'runs'/('stationary_empty_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ'));out.mkdir()
    captured=capture_pose();planner=Planner(json.loads((HERE/'settings.json').read_text()),captured['state'])
    if captured['state']['O_T_EE'][14]-planner.frame('5_0')[14]<.09:raise Stop('Stationary commissioning requires elevated home/open start')
    # This empty-hand commissioning probe keeps its elevated test reference;
    # it is independent of the lower operational home/transfer plane.
    home=planner.placement(5);home[14]+=planner.moving['home_height_m']
    start=planner.model.fk(captured['state']['q_d']).homogeneous.reshape(-1,order='F').tolist()
    approach=planner.path('stationary_empty_home',start,home,captured['state']['q_d'])
    place=home.copy();place[14]-=.05
    descend=planner.slower(planner.path('stationary_empty_descent',home,place,approach['q1']),2.)
    ascend=planner.slower(planner.path('stationary_empty_ascent',place,home,descend['q1']),1.5)
    owner=None
    try:
        manifest=planner.manifest();manifest['commissioning_authorized']=True;owner=Owner(manifest,out)
        owner.wait(lambda:owner.latest and owner.latest['hand_stamp']>0,1)
        if abs(owner.state()['hand_width']-.07)>.001:raise Stop('Known empty/open hand required')
        ids=[owner.load(p) for p in (approach,descend,ascend)]
        owner.hand(1);owner.begin();owner.go(ids[0],approach['duration_s']);owner.go(ids[1],descend['duration_s'])
        began=time.monotonic();opened=owner.hand(2);latency=time.monotonic()-began
        if latency>.70:raise Stop('Opening exceeds synchronized completion allowance')
        owner.go(ids[2],ascend['duration_s']);owner.finish()
        report=dict(completed=True,placement_mode='stationary_vertical',elevated_empty_hand=True,
            planner_identity=planner.identity,controller_sha256=hashlib.sha256((HERE/'controller').read_bytes()).hexdigest(),
            opening_wall_latency_s=latency,opening_sdk_latency_s=opened['hand_latency_s'],
            returned_home_open=True,final_mode=owner.state()['state']['robot_mode'],final_errors=owner.state()['state']['current_errors'],run=str(out))
        save(out/'outcome.json',report);save(HERE/'stationary_verified.json',report);print(json.dumps(report),flush=True)
    finally:
        if owner:owner.close()


if __name__=='__main__':main()
