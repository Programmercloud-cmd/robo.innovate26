#!/usr/bin/python3
"""One-time empty-hand timing and local image-Jacobian commissioning."""
import argparse
from datetime import datetime,timezone
import json
import math
import os
from pathlib import Path
import sys
import time

import cv2
import numpy as np

HERE=Path(__file__).resolve().parent;TEACHING=HERE.parent
sys.path.insert(0,str(TEACHING));sys.path.insert(0,str(TEACHING/'factory_vision'))
from move_xyz import capture_pose,vertical_pose,resolve_pose
from run import with_delta,delta_pose,fingerprint
from vision import detect,Uncertain
from planner import Planner
from runtime import Owner,Camera,Stop,save


def observe(camera,config,out,name,after):
    errors=[]
    for attempt in range(3):
        f=camera.frame(after=after);image=f.pop('image');cv2.imwrite(str(out/f'{name}_{attempt}.jpg'),image)
        try:
            d,overlay,rect=detect(image,config['detector'])
            cv2.imwrite(str(out/f'{name}_{attempt}_overlay.jpg'),overlay)
            d.update(f);return d
        except Uncertain as e:errors.append(str(e));after=time.monotonic()
    raise Stop('Bounded row observation failed: '+str(errors))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute',action='store_true')
    parser.add_argument('--hand-timing-evidence',type=Path,help='Reuse measured opening trials from this commissioning session')
    args=parser.parse_args()
    if not args.execute:print('Preview only; --execute commissions local empty-hand movements.');return
    cv2.setNumThreads(1)
    os.sched_setaffinity(0,{0,1,3,4})
    out=HERE/'runs'/('commission_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ'));out.mkdir(parents=True)
    cfg=json.loads((TEACHING/'factory_vision/config.json').read_text())
    settings=json.loads((HERE/'settings.json').read_text());capture=capture_pose();save(out/'start_capture.json',capture)
    planner=Planner(settings,capture['state']);nominal=planner.frame('5_0');nominal[14]+=.05
    if np.linalg.norm(np.array(capture['state']['O_T_EE'][12:15])-nominal[12:15])>.003:
        raise Stop('Commissioning must start within a local3mm approach of confirmed empty5_0 plus50mm')
    manifest=planner.manifest();manifest['commissioning_authorized']=True
    camera=owner=None
    try:
        camera=Camera(cfg['camera']);owner=Owner(manifest,out)
        owner.wait(lambda:owner.latest and owner.latest['hand_stamp']>0 and
                   time.monotonic()-owner.latest['hand_stamp']<.2,1)
        first=owner.state()
        if abs(first['hand_width']-.07)>.001:raise Stop('Commissioning requires already open70mm hand')
        hand_trials=[]
        if args.hand_timing_evidence:
            prior=json.loads(args.hand_timing_evidence.read_text())
            if not prior['empty_hand'] or prior['closed_width_m']!=.06 or prior['open_width_m']!=.07 or prior['opening_speed_m_s']!=.08:
                raise Stop('Opening timing evidence does not match current hand settings')
            hand_trials=prior['latencies_s']
            if not hand_trials or any(not math.isfinite(x) or not 0<x<2 for x in hand_trials):raise Stop('Invalid opening timing evidence')
        else:
            for i in range(3):
                owner.hand(1);s=owner.hand(2);hand_trials.append(s['hand_latency_s'])
        save(out/'opening_latency.json',dict(empty_hand=True,closed_width_m=.06,open_width_m=.07,
             opening_speed_m_s=.08,latencies_s=hand_trials,maximum_s=max(hand_trials),loaded_validation_pending=True))
        owner.begin()
        current=owner.state()['state'];q,target=planner.local_target(current['q_d'],nominal,[0,0,0])
        owner.servo(q,target)
        first=observe(camera,cfg,out,'origin',time.monotonic());origin=owner.state()['state']['O_T_EE']
        positions=[np.zeros(3)];features=[first['feature']]
        # Small commissioning jogs are each locally certified. Retain the taught
        # reference; these movements do not update the pose catalog.
        for axis,amount in ((0,.0015),(1,.0015),(2,math.radians(.7))):
            for sign in (-1,1):
                desired=with_delta(nominal,[amount*sign if j==axis else 0 for j in range(3)])
                current=owner.state()['state'];q,target=planner.local_target(current['q_d'],nominal,
                    [amount*sign if j==axis else 0 for j in range(3)])
                owner.servo(q,target)
                obs=observe(camera,cfg,out,f'axis{axis}_{sign}',time.monotonic())
                actual=owner.state()['state']['O_T_EE'];positions.append(delta_pose(origin,actual));features.append(obs['feature'])
                current=owner.state()['state'];q=planner.solve(nominal,current['q_d']);owner.servo(q,nominal)
        owner.finish()
        x=np.array(positions);y=np.array(features);scale=np.array([.001,.001,math.pi/180])
        design=np.column_stack([np.ones(len(x)),x/scale]);coeff,_,rank,_=np.linalg.lstsq(design,y,rcond=None)
        J=coeff[1:].T/scale[None,:];weighted=np.array([1,1,500])
        condition=float(np.linalg.cond(J*scale[None,:]*weighted[:,None]));rms=float(np.sqrt(np.mean(((design@coeff-y)*weighted)**2)))
        if rank!=4 or condition>30 or rms>4:raise Stop('Fresh local camera calibration did not validate')
        result=dict(schema_version=1,created_utc=datetime.now(timezone.utc).isoformat(),
             source_signature=fingerprint(cfg,planner.catalog),reference_pose='5_0',observation_offset_m=.05,
             measured_reference_pose=origin,commanded_reference_pose=nominal,
             target_feature=(coeff[0]+J@delta_pose(origin,nominal)).tolist(),image_jacobian=J.tolist(),
             normalized_condition=condition,calibration_rms_weighted_px=rms,
             samples=[dict(actual_delta_local_m_rad=a.tolist(),feature=b.tolist()) for a,b in zip(x,y)],
             physical_grasp_verified=False,reference_basis='Operator confirmed5_0; fresh locally measured six-jog camera response',
             opening_latency=hand_trials,run=str(out))
        save(out/'calibration.json',result);save(HERE/'calibration.json',result)
        save(out/'outcome.json',dict(completed=True,camera_calibration=result,empty_hand_opening_s=hand_trials))
        print(json.dumps(dict(completed=True,run=str(out),rms=rms,condition=condition,opening_latency_s=hand_trials)),flush=True)
    except Exception as e:
        save(out/'outcome.json',dict(completed=False,error=str(e),inventory_advanced=False));print('STOP',e,'LOG',out,flush=True);raise
    finally:
        if owner:owner.close()
        if camera:camera.close()


if __name__=='__main__':main()
