#!/usr/bin/python3
"""Elevated empty-hand check of resident recovery during an opening action."""
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
    os.sched_setaffinity(0,{3,4,5,9,10,11})
    out=HERE/'runs'/('release_recovery_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ'));out.mkdir()
    captured=capture_pose();planner=Planner(json.loads((HERE/'settings.json').read_text()),captured['state'])
    if captured['state']['O_T_EE'][14]-planner.frame('5_0')[14]<.09:raise Stop('Recovery check starts at least90mm above source')
    home=planner.placement(5);home[14]+=planner.moving['home_height_m']
    start=planner.model.fk(captured['state']['q_d']).homogeneous.reshape(-1,order='F').tolist()
    approach=planner.path('release_recovery_elevated_start',start,home,captured['state']['q_d'])
    tracking=planner.tracking(home,approach['q1'])
    manifest=planner.manifest();manifest['commissioning_authorized']=True;owner=None
    try:
        owner=Owner(manifest,out);pid=owner.process.pid
        owner.wait(lambda:owner.latest and owner.latest['hand_stamp']>0,1)
        if abs(owner.state()['hand_width']-.07)>.001:raise Stop('Recovery check requires the known empty/open hand')
        ids=[owner.load(p) for p in (approach,tracking)]
        owner.hand(1);owner.begin();owner.go(ids[0],approach['duration_s']);owner.command('go',id=ids[1])
        owner.wait(lambda:owner.latest and owner.latest['phase']==ids[1] and owner.latest['phase_time_s']>tracking['release_at']+.08,tracking['duration_s']+1)
        if owner.latest['hand_done']>=1000000:raise Stop('Opening finished before the intended recovery probe')
        owner.command('stop');began=time.monotonic();owner.recover_idle(require_opening_done=True)
        state=owner.state()['state'];pose=planner.model.fk(state['q_d']).homogeneous.reshape(-1,order='F').tolist()
        path=planner.path('resident_opening_stop_return_home',pose,home,state['q_d']);id=owner.load(path)
        owner.begin();owner.go(id,path['duration_s']);owner.finish();final=owner.state()
        if owner.process.pid!=pid or abs(final['hand_width']-.07)>.001:raise Stop('Persistent ownership/open-hand recovery did not verify')
        report=dict(completed=True,elevated_empty_hand=True,intentional_stop_during_opening=True,
            same_sdk_owner_pid=pid,no_sdk_reconnect=True,returned_home_open=True,
            recovery_duration_s=time.monotonic()-began,final_mode=final['state']['robot_mode'],
            final_errors=final['state']['current_errors'],inventory_untouched=True,run=str(out))
        save(out/'outcome.json',report);save(HERE/'release_recovery_verified.json',report);print(json.dumps(report),flush=True)
    finally:
        if owner:owner.close()


if __name__=='__main__':main()
