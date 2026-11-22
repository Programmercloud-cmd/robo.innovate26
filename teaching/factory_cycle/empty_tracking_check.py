#!/usr/bin/python3
"""One-time elevated empty-hand commissioning of the exact moving-release path."""
from datetime import datetime,timezone
import json
import hashlib
import os
from pathlib import Path
import sys

import numpy as np

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent))
from move_xyz import capture_pose
from planner import Planner
from runtime import Owner,Stop,save


def main():
    os.sched_setaffinity(0,{0,1,3,4})
    out=HERE/'runs'/('empty_tracking_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ'));out.mkdir(parents=True)
    capture=capture_pose();save(out/'start_capture.json',capture)
    planner=Planner(json.loads((HERE/'settings.json').read_text()),capture['state'])
    nominal=planner.frame('5_0');nominal[14]+=.05
    if capture['state']['O_T_EE'][14]-planner.frame('5_0')[14]<.045:
        raise Stop('Empty commissioning requires at least45mm measured clearance above grasp height')
    home=planner.placement(5);home[14]+=planner.moving['home_height_m']
    hover=planner.placement(5);hover[14]+=.05+planner.moving['tracking_entry_height_m']
    start=planner.model.fk(capture['state']['q_d']).homogeneous.reshape(-1,order='F').tolist()
    approach=planner.path('empty_elevated_approach',start,hover,capture['state']['q_d'])
    tracking=planner.tracking(hover,approach['q1'])
    retreat=planner.path('empty_elevated_return',tracking['target'],home,tracking['q1'])
    save(out/'plans.json',dict(approach=approach,tracking=tracking,retreat=retreat))
    manifest=planner.manifest();manifest['commissioning_authorized']=True;owner=None
    try:
        owner=Owner(manifest,out)
        owner.wait(lambda:owner.latest and owner.latest['hand_stamp']>0,1)
        width=owner.state()['hand_width']
        if min(abs(width-.07),abs(width-.06))>.001:raise Stop('Expected known empty70mm opening or empty60mm commissioning closure')
        ids=[owner.load(p) for p in (approach,tracking,retreat)]
        owner.hand(1);owner.begin();owner.go(ids[0],approach['duration_s']);released=owner.go(ids[1],tracking['duration_s'])
        owner.go(ids[2],retreat['duration_s']);owner.finish()
        metrics=next((e for e in reversed(owner.events) if e['type']=='release_metrics'),None)
        if not metrics or not metrics['full_opening_speed_verified']:raise Stop('Full callback-rate opening speed verification did not pass')
        samples=[s for s in owner.states if s['phase']==ids[1] and tracking['release_at']+.025<s['phase_time_s']<tracking['release_deadline']-.01]
        speed=[float((planner.model.jacobian(s['state']['q'])@np.array(s['state']['dq']))[1]) for s in samples]
        if not speed or any(not planner.moving['minimum_release_vy_m_s']<=v<=planner.moving['maximum_release_vy_m_s'] for v in speed):
            raise Stop('Measured moving-release speed is outside configured bounds')
        if not released['hand_ok'] or abs(released['hand_width']-.07)>.001 or released['hand_latency_s']>planner.moving['matched_release_time_s']:
            raise Stop('Opening missed the matched-speed window')
        report=dict(completed=True,elevated_empty_hand=True,loaded_cycle_demonstrated=False,run=str(out),
            planner_identity=planner.identity,config=planner.moving,
            controller_sha256=hashlib.sha256((HERE/'controller').read_bytes()).hexdigest(),
            measured_vy_m_s=dict(mean=float(np.mean(speed)),minimum=min(speed),maximum=max(speed)),
            full_opening_release_metrics=metrics,
            verified_opening_latency_s=released['hand_latency_s'],matching_window_s=planner.moving['matched_release_time_s'],
            commanded_stroke_m=tracking['certificate']['stroke_m'],returned_home=True,correct_picks=0,correct_placements=0)
        save(out/'outcome.json',report);save(HERE/'empty_tracking_verified.json',report);print(json.dumps(report),flush=True)
    except Exception as e:
        save(out/'outcome.json',dict(completed=False,error=str(e),correct_picks=0,correct_placements=0));raise
    finally:
        if owner:owner.close()


if __name__=='__main__':main()
