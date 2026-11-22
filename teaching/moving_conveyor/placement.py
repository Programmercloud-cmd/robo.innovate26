"""Speed-matched tabletop placement of an independently verified held column."""
import json
from pathlib import Path
import subprocess
import math
import time

import cv2
import numpy as np

from .prepare import prepare
from .freeze import freeze

HERE=Path(__file__).resolve().parent


def save(path,value):
    path.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')


def transfer(session,label,grasp_pose,source_observation,*,resume_clearance=False):
    config=json.loads((HERE/'config.json').read_text())
    if not(config['execution_enabled'] and config['release_timing_physically_validated']):
        raise RuntimeError('Speed-matched placement is not commissioned; no stationary fallback')
    active=session.progress()['active_operation']
    expected='clearance_retention_uncertain' if resume_clearance else 'retention_passed'
    if not active or active['pose']!=label or active['phase']!=expected:
        raise RuntimeError('A separately verified retained column is required')
    out=session.directory/'conveyor_motion';out.mkdir(exist_ok=True)
    lift=grasp_pose.copy();lift[14]+=session.config['lift_clearance_m']
    raised=session.capture() if resume_clearance else session.move(label,lift,'factory_vision_moving_conveyor_extraction',direct='vertical')
    from vision import clearance_retention_evidence
    previous=json.loads((session.directory/'retention_lift_capture.json').read_text())['state']['O_T_EE']
    actual_clearance_lift=raised['state']['O_T_EE'][14]-previous[14]
    session.camera.first_frame=True;clearance_frame=session.camera.frame()
    cv2.imwrite(str(out/'after_clearance_lift.jpg'),clearance_frame)
    settled=cv2.imread(str(session.directory/('clearance_reference.jpg' if resume_clearance else 'after_lift.jpg')))
    if settled is None:raise RuntimeError('Missing recorded retained-row reference image')
    evidence=clearance_retention_evidence(settled,clearance_frame)
    evidence['additional_actual_lift_m']=actual_clearance_lift
    evidence['actual_height_above_grasp_m']=raised['state']['O_T_EE'][14]-grasp_pose[14]
    evidence['verified_O_T_EE']=raised['state']['O_T_EE']
    evidence['verified_unix_ns']=time.time_ns()
    save(out/'clearance_retention.json',evidence)
    if not evidence['all_four_retained'] or evidence['actual_height_above_grasp_m']<.035:
        session.progress('clearance_retention_uncertain',label)
        raise RuntimeError('Four-brick retention did not verify after the full clearance lift')
    session.log('full_clearance_four_brick_lift_verified',pose=label,**evidence)
    return finish_from_clearance(session,label,grasp_pose,source_observation,evidence)


def finish_from_clearance(session,label,grasp_pose,source_observation,clearance_evidence):
    """Continue after logged automatic proof or an explicit visual recovery.

    An uncertain pixel tracker never resumes on its own. Recovery supplies the
    actual inspected image/capture, four-brick conclusion and verification time.
    """
    config=json.loads((HERE/'config.json').read_text())
    if not(config['execution_enabled'] and config['release_timing_physically_validated']):
        raise RuntimeError('Speed-matched release is disabled')
    active=session.progress()['active_operation']
    if not active or active['pose']!=label or active['phase'] not in ('retention_passed','clearance_retention_uncertain'):
        raise RuntimeError('No matching retained-column operation')
    current=session.capture()['state']['O_T_EE']
    if not clearance_evidence.get('all_four_retained') or time.time_ns()-clearance_evidence['verified_unix_ns']>120_000_000_000:
        raise RuntimeError('Fresh explicit full-clearance verification required')
    if math.dist(current[12:15],clearance_evidence['verified_O_T_EE'][12:15])>.0015 or current[14]-grasp_pose[14]<.035:
        raise RuntimeError('Arm moved from the verified held-row clearance')
    out=session.directory/'conveyor_motion';out.mkdir(exist_ok=True)
    save(out/'accepted_clearance_evidence.json',clearance_evidence)
    session.log('clearance_proof_accepted',pose=label,evidence=clearance_evidence)
    session.progress('carrying_unverified',label)
    target=session.conveyor_target();hover=target.copy();hover[14]+=config['preplace_height_m']
    session.move(label,hover,'factory_vision_moving_conveyor_hover')
    capture=session.capture();height=capture['state']['O_T_EE'][14]-target[14]
    if not .0012<height<.00265:
        correction=target.copy();correction[14]+=.002
        if abs(correction[14]-capture['state']['O_T_EE'][14])>.002:
            raise RuntimeError('Measured conveyor hover needs more than a two mm correction')
        session.move(label,correction,'factory_vision_moving_conveyor_hover_correction',direct='vertical')
        capture=session.capture()
    save(out/'hover_capture.json',capture)
    hand=json.loads(subprocess.check_output([str(HERE.parent/'gripper_width'),'--state'],text=True))
    plan=prepare(int(label.split('_')[0]),hand['width'],out,out/'hover_capture.json')
    frozen=freeze(out/'plan.json')
    check=subprocess.run([str(HERE/'execute'),str(frozen),str(out/'check.csv'),'--check'],capture_output=True,text=True)
    (out/'model_check.log').write_text(check.stdout+check.stderr)
    if check.returncode:
        raise RuntimeError('Independent timed trajectory check failed')
    session.progress('speed_matched_placement_running',label)
    command=[str(HERE/'execute'),str(frozen),str(out/'trace.csv'),'--execute']
    if session.policy.get('performance_profile_hold'):
        command=['powerprofilesctl','launch','--profile','performance','--reason',
                 'Franka conveyor speed-matched placement','--appid','franka-conveyor']+command
    result=subprocess.run(command,capture_output=True,text=True)
    log=result.stdout+result.stderr;(out/'execution.log').write_text(log);print(log,flush=True)
    if result.returncode or 'PASS timed conveyor stroke' not in log.splitlines():
        session.progress('moving_conveyor_uncertain',label)
        raise RuntimeError('Moving placement stopped; no automatic release, retry or return')
    session.progress('released_pending_conveyor_verification',label)
    trace=np.genfromtxt(out/'trace.csv',delimiter=',',names=True)
    matched=trace[(trace['t_s']>plan['release_start_s']+.025)&(trace['t_s']<plan['release_deadline_s']-.01)]
    opened=trace[trace['released']>.5]
    gap=trace[trace['width_m']>=hand['width']+.0008]
    if not len(matched) or not len(opened) or not len(gap):
        raise RuntimeError('Missing measured speed/release evidence')
    inspection=target.copy();inspection[12:14]=[float(gap[0]['x_m']),float(gap[0]['y_m'])]
    inspection[14]+=session.config['observation_offset_m']
    metrics=dict(mean_vy_m_s=float(matched['vy_actual_m_s'].mean()),
        min_vy_m_s=float(matched['vy_actual_m_s'].min()),max_vy_m_s=float(matched['vy_actual_m_s'].max()),
        first_confirmed_open_time_s=float(opened[0]['t_s']),gap_measurement_time_s=float(gap[0]['t_s']),
        actual_forward_stroke_m=float(trace['y_m'].max()-trace['y_m'][0]),
        commanded_stroke_m=plan['stroke_m'],release_window_s=config['matched_release_time_s'],
        contact_gap_pose_estimate_xy_m=inspection[12:14],
        caveat='Hand-gap observation estimates where to inspect; it is not a measured contact-loss instant')
    save(out/'measured_motion.json',metrics)
    session.log('conveyor_speed_matched_release_verified',pose=label,**metrics)
    # Lift at the downstream endpoint before returning toward the input end.
    current=session.capture()['state']['O_T_EE'];raised=current.copy();raised[14]+=.05
    session.move(label,raised,'factory_vision_moving_conveyor_release_lift',direct='vertical')
    session.move(label,inspection,'factory_vision_moving_conveyor_inspection')
    session.camera.previous_observation=None
    error=None
    try:
        placed=session.observation('destination_verification')
        distances=[float(cv2.compareHist(np.array(a,np.float32),np.array(b,np.float32),cv2.HISTCMP_BHATTACHARYYA))
                   for a,b in zip(source_observation['appearance_hists'],placed['appearance_hists'])]
        if max(distances)>.4:
            raise RuntimeError('Destination appearance differs from the source: '+str(distances))
        session.log('placement_observed',pose=label,visible_bricks=4,studs_per_brick=placed['studs_per_brick'],
                    appearance_distances=distances,release_opening_verified=True,placement_mode='speed_matched')
        session.progress(pose=label,completed=True)
        progress=session.progress();progress['completed_columns'][label].update(
            placement_mode='speed_matched',measured_motion=metrics,conveyor_motion_run=str(out))
        save(session.directory.parent.parent/'progress.json',progress)
    except Exception as failure:
        error=failure
        save(out/'placement_verification_stop.json',dict(error=str(failure),inventory_advanced=False))
    session.move(label,plan['home_O_T_EE'],'factory_vision_moving_conveyor_return_home')
    if error:
        raise error
    return dict(transferred=True,speed_matched=True,release_verified=True,destination_four_brick_row_observed=True,
                returned_home_100mm_above_conveyor=True,measured_motion=metrics)
