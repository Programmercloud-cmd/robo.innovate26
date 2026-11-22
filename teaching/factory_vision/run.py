#!/usr/bin/python3
"""Measured wrist-camera alignment for the configured factory development row."""
import argparse
import copy
import fcntl
from datetime import datetime,timezone
import hashlib
import inspect
import json
import math
from pathlib import Path
import sys
import subprocess
import time
import numpy as np
import cv2

HERE=Path(__file__).resolve().parent
TEACHING=HERE.parent
sys.path.insert(0,str(TEACHING))
from move_xyz import capture_pose,resolve_pose,vertical_pose
from pose_ids import pose_key
from arc_motion import run_arc
from gripper_width import command_width
from vision import Camera,Uncertain,observe,detect,fit_line,validate_row,track_row,retention_evidence,conveyor_clear_evidence


def save(path,value):
    path.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')


def fingerprint(config,catalog):
    anchor=catalog['points'][pose_key(config['reference_pose'])]
    geometry_config=dict(camera={k:config['camera'][k] for k in ('device','width','height','focus')},
        detector=config['detector'],reference_pose=config['reference_pose'],observation_offset_m=config['observation_offset_m'])
    geometry_config['camera']['controls']=config['camera'].get('controls',{})
    content=dict(config=geometry_config,anchor={k:anchor[k] for k in ('O_T_EE','tool_frames')},
        camera_path=str(Path(config['camera']['device']).resolve()),
        detector_sha256=hashlib.sha256(''.join(inspect.getsource(f) for f in (detect,fit_line,validate_row,track_row)).encode()).hexdigest())
    return hashlib.sha256(json.dumps(content,sort_keys=True).encode()).hexdigest()


def yaw(matrix):return math.atan2(matrix[1],matrix[0])


def delta_pose(origin,target):
    rotation=np.array(origin).reshape(4,4,order='F')[:3,:3]
    delta=rotation.T@(np.array(target[12:15])-origin[12:15])
    angle=(yaw(target)-yaw(origin)+math.pi)%(2*math.pi)-math.pi
    return np.array([delta[0],delta[1],angle])


def with_delta(pose,delta):
    matrix=np.array(vertical_pose(pose)).reshape(4,4,order='F').copy()
    matrix[:3,3]+=matrix[:3,:3]@np.array([delta[0],delta[1],0])
    a=delta[2];rotation=np.array([[math.cos(a),-math.sin(a),0],[math.sin(a),math.cos(a),0],[0,0,1]])
    matrix[:3,:3]=rotation@matrix[:3,:3]
    return matrix.reshape(-1,order='F').tolist()


class Session:
    def __init__(self,config,execute,directory):
        self.config=config;self.execute=execute;self.directory=directory;self.events=[]
        self.restored_source=None
        self.raw=(TEACHING/'positions.json').read_bytes();self.catalog=json.loads(self.raw)
        self.signature=fingerprint(config,self.catalog)
        self.operation_lock=(HERE/'.session.lock').open('a')
        fcntl.flock(self.operation_lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        self.camera=Camera(config['camera'])
        self.sequence=0
        policy=json.loads((TEACHING/'motion_config.json').read_text())
        if policy['task_id']!=config['task_id'] or self.catalog['task_id']!=config['task_id']:raise Uncertain('Wrong task configuration')
        if policy['gripper']['close_target_m']!=config['close_width_m']:raise Uncertain('Configured closing width is not 60 mm')
        self.policy=policy

    def progress(self,phase=None,pose=None,completed=False):
        path=HERE/'progress.json'
        state=json.loads(path.read_text()) if path.exists() else dict(task_id=self.config['task_id'],active_operation=None,completed_columns={})
        if phase:
            previous=state.get('active_operation') or {}
            state['active_operation']=dict(phase=phase,pose=pose,run=str(self.directory),
                origin_run=previous.get('origin_run',previous.get('run',str(self.directory))),utc=datetime.now(timezone.utc).isoformat())
        if completed:
            state['completed_columns'][pose]=dict(run=str(self.directory),visible_bricks_after_release=4,
                destination_mode='fixed_conveyor',conveyor_target_O_T_EE=self.conveyor_target())
            state['active_operation']=None
            state.get('deliveries_needing_correction',{}).pop(pose,None)
        if phase or completed:save(path,state)
        return state

    def conveyor_target(self):
        target=self.config.get('conveyor_target_O_T_EE')
        if not isinstance(target,list) or len(target)!=16 or not all(math.isfinite(v) for v in target):
            raise Uncertain('A fixed conveyor target must be configured')
        return target.copy()

    def check_conveyor_clear(self,label):
        hand=json.loads(subprocess.check_output([str(TEACHING/'gripper_width'),'--state'],text=True))
        if abs(hand['width']-self.policy['gripper']['approach_opening_m'])>.001:
            raise Uncertain('Conveyor clearance inspection requires an already open, empty hand')
        target=self.conveyor_target();target[14]+=self.config['observation_offset_m']
        current=self.capture()['state']['O_T_EE']
        if math.dist(current[12:15],target[12:15])>.002:
            self.move(label,target,'factory_vision_conveyor_clearance_view')
        self.camera.previous_observation=None
        deadline=time.monotonic()+self.config['conveyor_clear_wait_s'];attempt=0
        while True:
            evidence=[];self.camera.first_frame=True
            for frame in range(3):
                image=self.camera.frame();cv2.imwrite(str(self.directory/f'conveyor_clear_{attempt}_{frame}.jpg'),image)
                evidence.append(conveyor_clear_evidence(image,self.config['conveyor_clear_roi_px']))
            if all(item['clear'] for item in evidence):
                save(self.directory/'conveyor_clearance.json',dict(clear=True,observations=evidence,utc=datetime.now(timezone.utc).isoformat()))
                self.log('fixed_conveyor_clear',observations=evidence);return
            self.log('waiting_for_conveyor_clear',observations=evidence)
            if time.monotonic()>=deadline:raise Uncertain('Fixed conveyor spot is occupied or uncertain; clear it before another delivery')
            time.sleep(2);attempt+=1

    def log(self,kind,**details):
        event=dict(utc=datetime.now(timezone.utc).isoformat(),kind=kind,**details)
        self.events.append(event)
        with (self.directory/'events.jsonl').open('a') as f:f.write(json.dumps(event,allow_nan=False)+'\n')
        visible={k:v for k,v in details.items() if k!='appearance_hists'}
        print(kind,json.dumps(visible),flush=True)

    def capture(self):
        if (TEACHING/'positions.json').read_bytes()!=self.raw:raise Uncertain('Pose catalog changed during this run')
        capture=capture_pose()
        if capture['state']['robot_mode']!='Idle' or capture['state']['current_errors']:raise Uncertain('Robot must be fault-free and Idle')
        return capture

    def observation(self,name):
        result=observe(self.camera,self.config['detector'],self.directory/name)
        self.log('observation',name=name,**result)
        return result

    def move(self,label,target,stage,direct=None,time_scale=1):
        if not self.execute:raise Uncertain('Motion requires --execute')
        identifier,point,_=resolve_pose(self.catalog,label)
        capture=self.capture()
        if direct=='vertical':
            current=capture['state']['O_T_EE']
            if math.dist(current[12:14],target[12:14])>.0015 or abs(current[14]-target[14])>.055:
                raise Uncertain('Vertical moves are restricted to 55 mm height travel and 1.5 mm lateral tracking correction')
        overrides=dict(stage=stage,time_scale=time_scale,endpoint_position_tolerance_m=.0015)
        if direct=='alignment':overrides['direct_alignment']=True
        if direct=='vertical':overrides['direct_vertical']=True
        self.log('motion_requested',label=label,stage=stage,target=target,overrides=overrides)
        result=run_arc(identifier,target,capture,self.raw,point,True,overrides)
        if result:raise Uncertain(f'Motion failed during {stage}; no automatic retry, recovery, descent or closure')
        fresh=self.capture();self.last_target=target.copy()
        self.log('motion_completed',stage=stage,actual_pose=fresh['state']['O_T_EE'])
        return fresh

    def approach(self,label,target_override=None):
        self.camera.previous_observation=None
        progress=self.progress()
        if progress['active_operation']:raise Uncertain('Previous operation may hold bricks or have an uncertain placement; inspect progress.json before opening')
        if str(label) in progress['completed_columns'] and str(label)!=self.restored_source:raise Uncertain('The reference/source column has already moved; reteach or restore it before this approach')
        identifier,point,_=resolve_pose(self.catalog,label)
        target=vertical_pose(target_override or point['O_T_EE']);target[14]+=self.config['observation_offset_m']
        if command_width('approach',self.execute):raise Uncertain('Gripper opening failed')
        return self.move(identifier,target,'factory_vision_observation_approach')

    def calibrate(self):
        label=self.config['reference_pose'];initial=self.approach(label)
        origin=initial['state']['O_T_EE'];first=self.observation('calibration_origin')
        nominal=vertical_pose(self.catalog['points'][pose_key(label)]['O_T_EE'])
        nominal[14]+=self.config['observation_offset_m']
        x=[np.zeros(3)];y=[np.array(first['feature'])]
        scale=np.array([.001,.001,math.pi/180]);servo=self.config['servo']
        for axis,step in [(0,servo['calibration_translation_m']),(1,servo['calibration_translation_m']),
                          (2,math.radians(servo['calibration_yaw_deg']))]:
            for sign in [-1,1]:
                delta=np.zeros(3);delta[axis]=step*sign
                # Return to the same reference before each bounded sample.
                if len(x)>1:self.move(label,nominal,'calibration_return',direct='alignment')
                target=with_delta(nominal,delta)
                measured=self.move(label,target,f'calibration_axis_{axis}_{sign}',direct='alignment')
                obs=self.observation(f'calibration_axis_{axis}_{sign}')
                actual=measured['state']['O_T_EE']
                if abs(actual[14]-origin[14])>.0008:raise Uncertain('Calibration height varied by more than 0.8 mm')
                x.append(delta_pose(origin,actual)/scale);y.append(np.array(obs['feature']))
        x=np.asarray(x);y=np.asarray(y)
        design=np.column_stack([np.ones(len(x)),x]);coefficients,_,rank,_=np.linalg.lstsq(design,y,rcond=None)
        if rank!=4:raise Uncertain('Calibration movements did not excite all three alignment directions')
        jacobian=coefficients[1:].T/scale[None,:]
        feature_scale=np.array([1,1,500]);normalized=jacobian*scale[None,:]*feature_scale[:,None]
        condition=float(np.linalg.cond(normalized));residuals=(design@coefficients-y)*feature_scale
        rms=float(np.sqrt(np.mean(residuals**2)))
        if condition>servo['maximum_jacobian_condition'] or rms>servo['maximum_calibration_residual_px']:
            raise Uncertain(f'Unreliable measured image Jacobian: condition={condition:.2f}, residual={rms:.2f}')
        # Reference is tied to the operator-taught grasp center, not the image center.
        reference=vertical_pose(self.catalog['points'][pose_key(label)]['O_T_EE']);reference[14]+=self.config['observation_offset_m']
        target_feature=coefficients[0]+jacobian@delta_pose(origin,reference)
        result=dict(schema_version=1,created_utc=datetime.now(timezone.utc).isoformat(),
            source_signature=self.signature,reference_pose=label,observation_offset_m=self.config['observation_offset_m'],
            measured_reference_pose=origin,commanded_reference_pose=reference,
            target_feature=target_feature.tolist(),image_jacobian=jacobian.tolist(),
            normalized_condition=condition,calibration_rms_weighted_px=rms,
            samples=[dict(local_delta_normalized=a.tolist(),feature=b.tolist()) for a,b in zip(x,y)],
            physical_grasp_verified=False,reference_basis=f'Configured reference {label} grasp center observed at raised height')
        save(HERE/self.config['calibration_file'],result);save(self.directory/'calibration.json',result)
        self.log('calibration_completed',condition=condition,residual=rms,target_feature=target_feature.tolist())
        return result

    def align(self,label,close,transfer=False,source_override=None,source_history=None):
        normalized,point,_=resolve_pose(self.catalog,label)
        if source_override is not None:
            point=copy.deepcopy(point);point['O_T_EE']=source_override.copy();point['xyz_m']=source_override[12:15]
            self.log('source_override_from_recorded_placement',pose=normalized,target_O_T_EE=source_override)
        row=self.config['development_row']
        if normalized not in tuple(f'{row}_{column}' for column in range(3)):
            raise Uncertain(f'Development is limited to populated source row {row}')
        progress=self.progress()
        if progress['active_operation']:raise Uncertain('Previous grasp/placement needs operator inspection; see progress.json before restarting')
        if normalized in progress['completed_columns'] and normalized!=self.restored_source:raise Uncertain('This source column has already been transferred')
        calibration=json.loads((HERE/self.config['calibration_file']).read_text())
        if calibration['source_signature']!=self.signature:raise Uncertain('Calibration is stale; recalibrate with current camera, geometry and settings')
        jacobian=np.array(calibration['image_jacobian']);goal=np.array(calibration['target_feature'])
        if transfer:
            if self.config.get('conveyor_placement_mode')!='speed_matched':
                raise Uncertain('Conveyor placement requires speed matching; stationary fallback is disabled')
            moving=json.loads((TEACHING/'moving_conveyor/config.json').read_text())
            if not(moving['execution_enabled'] and moving['release_timing_physically_validated']):
                raise Uncertain('Moving placement is not commissioned; no pickup is started')
            if self.config.get('conveyor_occupancy_policy')!='cleared_between_deliveries':
                raise Uncertain('Conveyor clearing/stacking policy must be resolved before another delivery')
            candidate=self.conveyor_target()
            code=run_arc(normalized,candidate,self.capture(),self.raw,point,False,
                dict(stage='factory_vision_nominal_transfer_preflight',endpoint_position_tolerance_m=.0015))
            if code:raise Uncertain('Nominal destination preflight failed; no approach, descent or closure')
            self.check_conveyor_clear(normalized)
        servo=self.config['servo'];initial=self.approach(normalized,source_override);origin=initial['state']['O_T_EE']
        if source_history is not None:self.camera.previous_observation=source_history
        converged=False
        for iteration in range(servo['maximum_iterations']+1):
            capture=self.capture();current=capture['state']['O_T_EE']
            expected_z=point['xyz_m'][2]+self.config['observation_offset_m']
            calibrated_z=calibration['measured_reference_pose'][14]+point['xyz_m'][2]-self.catalog['points'][pose_key(calibration['reference_pose'])]['xyz_m'][2]
            if abs(current[14]-calibrated_z)>.0008:raise Uncertain('Observation height differs from measured calibration height by over 0.8 mm')
            observation=self.observation(f'alignment_{iteration}')
            if time.monotonic()-observation['observed_monotonic']>2:raise Uncertain('Observation is stale')
            error=np.array(observation['feature'])-goal
            correction=-np.linalg.solve(jacobian,error)
            self.log('alignment_error',iteration=iteration,correction_local_m_rad=correction.tolist())
            if np.linalg.norm(correction[:2])<=servo['position_tolerance_m'] and abs(correction[2])<=math.radians(servo['yaw_tolerance_deg']):
                converged=True;break
            if iteration==servo['maximum_iterations']:break
            correction*=servo['gain'];length=np.linalg.norm(correction[:2])
            if length>servo['maximum_step_m']:correction[:2]*=servo['maximum_step_m']/length
            correction[2]=np.clip(correction[2],-math.radians(servo['maximum_step_yaw_deg']),math.radians(servo['maximum_step_yaw_deg']))
            target=with_delta(current,correction)
            # Integrate relative to the prior command so measured impedance
            # tracking bias cannot produce a permanent visual error floor.
            for axis in (12,13):target[axis]+=self.last_target[axis]-current[axis]
            target[14]=expected_z;total=delta_pose(origin,target)
            if np.linalg.norm(total[:2])>servo['maximum_total_correction_m'] or abs(total[2])>math.radians(servo['maximum_total_yaw_deg']):
                raise Uncertain('Required correction exceeds local search limits; reteach the nominal pose')
            self.move(normalized,target,'visual_alignment_correction',direct='alignment')
        if not converged:raise Uncertain('Visual alignment did not converge; no descent or closure')
        if normalized==self.restored_source and normalized in progress['completed_columns']:
            # Rearm only after fresh, stable four-brick evidence at the source.
            # Keep the previous delivery in history rather than erasing it.
            progress=self.progress()
            progress.setdefault('delivery_history',[]).append(dict(pose=normalized,
                previous_delivery=progress['completed_columns'].pop(normalized),
                replenishment_verified_run=str(self.directory),
                utc=datetime.now(timezone.utc).isoformat()))
            save(HERE/'progress.json',progress)
            self.log('replenished_source_visually_verified',pose=normalized)
        self.log('aligned',pose=normalized,actual_pose=self.capture()['state']['O_T_EE'])
        if not close:return dict(aligned=True,closed=False,pick_verified=False)
        aligned_command=self.last_target.copy()
        aligned_actual=self.capture()['state']['O_T_EE']
        if transfer:
            destination=self.conveyor_target()
            preview=run_arc(normalized,destination,self.capture(),self.raw,point,False,
                dict(stage='factory_vision_transfer_preflight',endpoint_position_tolerance_m=.0015))
            if preview:raise Uncertain('Destination preflight failed; no descent or closure')
        # One continuous guarded descent avoids repeated starts and accumulated
        # endpoint bias. The real-time controller retains its contact reflexes.
        grasp_z=point['xyz_m'][2];descent=self.config['descent']
        target=aligned_command.copy();target[14]=grasp_z
        at_grasp=self.move(normalized,target,'aligned_grasp_descent',direct='vertical',time_scale=descent['time_scale'])
        actual=at_grasp['state']['O_T_EE']
        if abs(actual[14]-grasp_z)>.0015 or math.dist(actual[12:14],aligned_actual[12:14])>.0015:
            raise Uncertain('Grasp-height or lateral verification failed; no closure')
        self.log('grasp_height_reached',actual_pose=actual,saved_grasp_z_m=grasp_z)
        self.progress('closing_unverified',normalized)
        if command_width('close',True):raise Uncertain('60 mm position closure did not verify; object may be held; no automatic release')
        self.progress('closed_unverified',normalized)
        self.log('position_closed',target_width_m=self.config['close_width_m'],pick_verified=False,
            note='Finger position alone does not prove that four bricks are held')
        if not transfer:return dict(aligned=True,closed=True,pick_verified=False)
        return self.transfer_held(normalized,self.verify_retention(normalized,observation),observation)

    def verify_retention(self,label,source_observation):
        capture=self.capture();grasp_pose=capture['state']['O_T_EE'];save(self.directory/'grasp_capture.json',capture)
        save(self.directory/'source_observation.json',source_observation)
        self.camera.first_frame=True;before=self.camera.frame();cv2.imwrite(str(self.directory/'before_lift.jpg'),before)
        lift=vertical_pose(grasp_pose);lift[14]+=.005
        raised=self.move(label,lift,'factory_vision_retention_verification_lift',direct='vertical')
        self.camera.first_frame=True;after=self.camera.frame();cv2.imwrite(str(self.directory/'after_lift.jpg'),after)
        save(self.directory/'retention_lift_capture.json',raised)
        try:evidence=retention_evidence(before,after)
        except Uncertain as error:
            self.progress('retention_uncertain',label)
            save(self.directory/'retention.json',dict(all_four_retained=False,error=str(error),
                actual_lift_m=raised['state']['O_T_EE'][14]-grasp_pose[14]))
            raise
        evidence['actual_lift_m']=raised['state']['O_T_EE'][14]-grasp_pose[14]
        save(self.directory/'retention.json',evidence)
        if not evidence['all_four_retained'] or evidence['actual_lift_m']<.0035:
            self.progress('retention_uncertain',label);raise Uncertain('Four-brick retention check failed; no transfer or automatic release')
        self.progress('retention_passed',label);self.log('four_brick_retention_verified',**evidence)
        return grasp_pose

    def transfer_held(self,normalized,grasp_pose,source_observation):
        if self.config.get('conveyor_placement_mode')!='speed_matched':
            raise Uncertain('Conveyor placement requires speed matching; no stationary fallback')
        from moving_conveyor.placement import transfer
        return transfer(self,normalized,grasp_pose,source_observation)

    def resume_transfer(self):
        active=self.progress()['active_operation']
        if not active or active['phase']!='retention_passed':raise Uncertain('Resume requires a recorded successful four-brick retention lift')
        run=Path(active['run']);evidence=json.loads((run/'retention.json').read_text())
        if not evidence['all_four_retained'] or evidence['actual_lift_m']<.0035:raise Uncertain('Retention record is not valid')
        previous=json.loads((run/'retention_lift_capture.json').read_text())['state']['O_T_EE']
        current=self.capture()['state']['O_T_EE']
        if math.dist(previous[12:15],current[12:15])>.0015:raise Uncertain('Arm moved since the verification lift; operator inspection required')
        hand=json.loads(subprocess.check_output([str(TEACHING/'gripper_width'),'--state'],text=True))
        if not self.policy['gripper']['close_target_m']-.001<=hand['width']<=self.policy['gripper']['close_contact_upper_m']:
            raise Uncertain('Measured hand width is outside the recorded contact range')
        grasp=json.loads((run/'grasp_capture.json').read_text())['state']['O_T_EE']
        observation=json.loads((run/'source_observation.json').read_text())
        self.log('resuming_verified_retention',pose=active['pose'],previous_run=str(run),hand=hand)
        return self.transfer_held(active['pose'],grasp,observation)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=['observe','calibrate','align','resume-transfer'])
    parser.add_argument('pose',nargs='?');parser.add_argument('--execute',action='store_true')
    parser.add_argument('--close',action='store_true',help='After alignment, descend to saved grasp height and position-close to 60 mm')
    parser.add_argument('--transfer',action='store_true',help='After closing, lift and deliver to the fixed conveyor target, then release and inspect')
    parser.add_argument('--source-placement-run',type=Path,help='Recover a misplaced column from the target recorded by this previous run')
    parser.add_argument('--restored-source',action='store_true',help='Inspect a replenished source; retain prior delivery history and rearm only after fresh four-brick alignment evidence')
    args=parser.parse_args();config=json.loads((HERE/'config.json').read_text())
    if args.pose is None:args.pose=f'{config["development_row"]}_0'
    if args.transfer and not args.close:parser.error('--transfer requires --close')
    directory=HERE/'runs'/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ');directory.mkdir(parents=True)
    save(directory/'request.json',dict(action=args.action,pose=args.pose,execute=args.execute,close=args.close,transfer=args.transfer,config=config))
    if args.action!='observe' and not args.execute:
        print('PREVIEW: no hardware connection. Use --execute for calibration or alignment; --close adds descent and 60 mm closure.');return 0
    session=None
    try:
        session=Session(config,args.execute,directory)
        if args.restored_source:
            if args.action!='align' or args.source_placement_run:raise Uncertain('Restored-source inspection requires ordinary source alignment')
            session.restored_source=resolve_pose(session.catalog,args.pose)[0]
        if args.action=='observe':result=session.observation('observation')
        elif args.action=='calibrate':result=session.calibrate()
        elif args.action=='resume-transfer':result=session.resume_transfer()
        else:
            override=None;history=None
            if args.source_placement_run:
                events=[json.loads(line) for line in (args.source_placement_run/'events.jsonl').read_text().splitlines()]
                candidates=[event for event in events if event['kind']=='motion_requested' and event['stage']=='factory_vision_transfer']
                if not candidates or candidates[-1]['label']!=args.pose:raise Uncertain('Previous placement does not match this source ID')
                override=candidates[-1]['target']
                observations=[event for event in events if event['kind']=='observation' and event.get('name')=='destination_verification']
                if not observations:raise Uncertain('Previous placement has no verified row observation')
                frame=cv2.imread(str(args.source_placement_run/'destination_verification/frame_2.jpg'))
                if frame is None:raise Uncertain('Previous placement image is unavailable')
                history=(frame,observations[-1])
            result=session.align(args.pose,args.close,args.transfer,override,history)
        save(directory/'outcome.json',dict(completed=True,result=result,grasp_verified=False))
        print('Completed; run log:',directory,flush=True);return 0
    except Exception as error:
        save(directory/'outcome.json',dict(completed=False,error=str(error),automatic_retry=False,automatic_recovery=False,grasp_verified=False))
        print('STOP:',error,'Run log:',directory,flush=True);return 1
    finally:
        if session:session.camera.close()


if __name__=='__main__':raise SystemExit(main())
