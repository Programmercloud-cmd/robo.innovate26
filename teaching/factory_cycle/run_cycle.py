#!/usr/bin/python3
"""Deterministic, resident four-brick factory cycle; no LLM stage decisions."""
import argparse
import copy
import csv
from datetime import datetime,timezone
import json
import hashlib
import math
import os
from pathlib import Path
import sys
import time

import cv2
import numpy as np

HERE=Path(__file__).resolve().parent;TEACHING=HERE.parent
sys.path.insert(0,str(TEACHING));sys.path.insert(0,str(TEACHING/'factory_vision'))
from move_xyz import capture_pose,rotation_error
from run import fingerprint,with_delta,delta_pose
from vision import detect,track_row,Uncertain,retention_evidence
from planner import Planner
from commissioning import stationary_profile_evidence
from runtime import Owner,Camera,Stop,save,PhaseClock


def conveyor_floor_evidence(packet,row,placement,mapping,now_ns,not_before_ns):
    """Read fresh observations of this destination, never display memory."""
    stamp=packet.get('frame_unix_ns',0)
    result=dict(clear=False,conveyor_row=row,frame_unix_ns=stamp)
    if (packet.get('camera_serial')!='03FB123F' or packet.get('stale') is not False or
            not isinstance(stamp,int) or not not_before_ns<=stamp<=now_ns or now_ns-stamp>500_000_000):
        return dict(result,reason='conveyor_camera_not_fresh_at_placement')
    if packet.get('registration',{}).get('valid') is not True:
        return dict(result,reason='conveyor_registration_uncertain')
    region=next((r for r in mapping.get('conveyor_observations',{}).get('regions',[]) if r.get('row')==row),None)
    xy=region.get('configured_target_xyz_m',[])[:2] if region else []
    if len(xy)!=2 or not np.isfinite(xy).all() or np.linalg.norm(np.array(xy)-placement[12:14])>.001:
        return dict(result,reason='conveyor_image_mapping_does_not_match_placement')
    expected_hash=hashlib.sha256(json.dumps(mapping,sort_keys=True,separators=(',',':')).encode()).hexdigest()
    if packet.get('layout_hash')!=expected_hash:return dict(result,reason='conveyor_image_mapping_not_loaded')
    ids={f'conveyor_r{row}_n{n}' for n in range(4)}
    slots=[s for s in packet.get('conveyor_slots',[]) if s.get('slot_id') in ids]
    if len(slots)!=4 or {s['slot_id'] for s in slots}!=ids:return dict(result,reason='conveyor_slots_unavailable')
    if any(s.get('status')=='occupied' for s in slots):return dict(result,reason='conveyor_occupied')
    if any(s.get('status')!='empty' or s.get('visible') is not True or
           not isinstance(s.get('confidence'),(int,float)) or not .5<=s['confidence']<=1 or
           s.get('observed_unix_ns')!=stamp for s in slots):
        return dict(result,reason='conveyor_visibility_uncertain',slots=slots)
    return dict(result,clear=True,reason='fresh_positive_floor_at_destination',slots=slots)


class Factory:
    def __init__(self,out,columns,resume=False,source_reset=False,home_layer=5,initial_state=None,placement_verification=True,empty_gripper_confirmed=False,task_execution=False,setup=None):
        self.out=out;self.columns=columns;self.owner=self.camera=None;self.timings=[]
        self.placement_verification_enabled=placement_verification and not task_execution
        self.task_execution=task_execution
        self.empty_gripper_confirmed=empty_gripper_confirmed or task_execution
        self.confirmed_picks=0;self.confirmed_placements=0
        self.visual_references={}
        self.settings=json.loads((HERE/'settings.json').read_text())
        self.planner=Planner(self.settings,capture_pose()['state'] if initial_state is None else initial_state,setup=setup);self.plans={};self.ids={}
        self.settings=self.planner.settings
        self.cfg=copy.deepcopy(self.planner.factory)
        if self.task_execution:self.cfg['fixed_pose_execution']=True
        self.stationary=self.cfg.get('conveyor_placement_mode')=='stationary_vertical'
        self.free_endpoint_tolerance=.0015 if self.stationary or task_execution else self.settings['endpoint_position_m']
        self.controller_path=HERE/('controller_conveyor_moving' if setup=='conveyor_moving' else 'controller')
        if not self.controller_path.is_file():raise Stop('Build the selected native controller: bash teaching/factory_cycle/build.sh --moving-conveyor')
        self.schedule=self.cfg.get('phase_schedule',{})
        if not self.stationary and self.schedule.get('enabled'):raise Stop('Phase synchronization is configured for stationary placement; restore the saved tracking profile for speed alignment')
        self.calib={} if self.task_execution else json.loads((HERE/'calibration.json').read_text())
        if self.task_execution and self.planner.setup:
            save(out/'commissioning_policy.json',dict(setup=self.planner.setup,
                historical_geometry_commissioning_gate=False,
                basis='Operator-authorized fixed-pose execution of explicitly supported factory profiles',
                full_model_and_native_trajectory_checks_required=True,
                physical_success_not_inferred=True))
        else:
            commissioned=json.loads((HERE/('stationary_verified.json' if self.stationary else 'empty_tracking_verified.json')).read_text())
            stopped=json.loads((HERE/'stop_verified.json').read_text())
            local_surface_correction=(commissioned['planner_identity']==self.planner.base_identity and
                0<self.planner.surface_offset<=.002 and self.cfg.get('conveyor_surface_height_evidence',{}).get('operator_confirmed_brick_contact'))
            profile_evidence=stationary_profile_evidence(self.planner,commissioned) if self.stationary else None
            if (not commissioned['completed'] or (commissioned['planner_identity']!=self.planner.identity and not local_surface_correction and not profile_evidence) or
                    commissioned['controller_sha256']!=hashlib.sha256(self.controller_path.read_bytes()).hexdigest() or
                    not stopped['completed'] or stopped['controller_sha256']!=commissioned['controller_sha256']):
                mode='stationary-placement' if self.stationary else 'moving-release'
                raise Stop(f'Current controller/geometry/speed needs elevated empty {mode} commissioning')
            if profile_evidence:save(out/'stationary_profile_validation.json',profile_evidence)
            if local_surface_correction:
                save(out/'surface_correction_validation.json',dict(offset_m=self.planner.surface_offset,
                    original_taught_pose_unchanged=True,source_grasp_heights_unchanged=True,
                    unchanged_controller_and_speed_profile=True,timing_evidence_reused_from=commissioned['run'],
                    corrected_trajectories_receive_full_model_and_native_workspace_checks=True,
                    physical_loaded_result_not_inferred=True))
        if not self.task_execution and self.calib['source_signature']!=fingerprint(self.cfg,self.planner.catalog):raise Stop('Fresh local camera commissioning is required')
        self.goal=None if self.task_execution else np.array(self.calib['target_feature'])
        self.J=None if self.task_execution else np.array(self.calib['image_jacobian'])
        # This reference identifies the actual5_0 row from this session's
        # operator-confirmed calibration. Fixed-pose execution uses this only
        # for coarse supervision; descent does not require visual tracking.
        origin_files=[] if self.task_execution else list(Path(self.calib['run']).glob('origin_*_overlay.jpg'))
        if origin_files:
            source=origin_files[0].with_name(origin_files[0].name.replace('_overlay',''))
            im=cv2.imread(str(source))
            if im is not None:
                reference,_,_=detect(im,self.cfg['detector'])
                self.visual_references['5_0']=(im,reference)
        self.inventory=dict(task_id='factory_columns_of_four',available={},completed={},active=None,prior_history='../factory_vision/progress.json; preserved, not assumed complete')
        prior=HERE/'inventory.json'
        self.prior_inventory_bytes=prior.read_bytes() if prior.exists() else None
        previous=json.loads(self.prior_inventory_bytes) if self.prior_inventory_bytes is not None else None
        self.reset_prior_active=previous.get('active') if source_reset and previous else None
        if previous and previous.get('active') and not resume and not source_reset and not self.empty_gripper_confirmed:
            raise Stop('Previous cycle has unresolved grasp/placement inventory; inspect before restarting')
        if previous is not None:self.inventory['prior_session_inventory']=previous
        if resume:self.inventory=json.loads(prior.read_text())
        self.home_layer=home_layer
        self.home=self.planner.home(home_layer);self.qhome=self.planner.solve(self.home,self.planner.home_seed)
        self.place=self.planner.placement(home_layer)
        self.clear=self.home.copy() if self.stationary else self.place.copy()
        if not self.stationary:self.clear[14]+=self.settings['observation_height_m']
        self.qclear=self.planner.solve(self.clear,self.qhome)
        self.hover=self.home.copy() if self.stationary else self.place.copy()
        if not self.stationary:self.hover[14]+=self.planner.moving['tracking_entry_height_m']
        self.qhover=self.planner.solve(self.hover,self.qhome)
        self.stroke=None if self.stationary else self.planner.tracking(self.hover,self.qhover)
        self.inspection=self.home.copy() if self.stationary else self.place.copy()
        if not self.stationary:
            self.inspection[13]+=.5*self.stroke['certificate']['stroke_m'];self.inspection[14]+=self.settings['observation_height_m']
        self.qinspection=self.planner.solve(self.inspection,self.qhome)
        scale=lambda name:self.schedule.get(name,1) if self.schedule.get('enabled') else 1
        self.common={'clear':self.planner.slower(self.planner.path('home_to_clear_view',self.home,self.clear,self.qhome),scale('row_motion_scale')),
            'return':self.planner.slower(self.planner.path('return_home',self.inspection,self.home,self.qinspection),scale('return_motion_scale'))}
        if self.stationary:
            descend=self.planner.path('stationary_vertical_placement',self.hover,self.place,self.qhover)
            self.common['place']=self.planner.slower(descend,scale('placement_motion_scale'))
            self.common['inspect']=self.planner.slower(self.planner.path('stationary_vertical_inspection',self.place,self.inspection,descend['q1']),scale('return_motion_scale'))
        else:
            self.common.update(track=self.stroke,inspect=self.planner.path('retreat_to_inspection',self.stroke['target'],self.inspection,self.stroke['q1']))
        self.inspection_recovery=[];previous=self.inspection;seed=self.qinspection
        for index,dy in enumerate((-.008,.008) if self.placement_verification_enabled else ()):
            view=self.inspection.copy();view[13]+=dy
            leg=self.planner.path('inspection_reframe_'+str(index),previous,view,seed)
            back=self.planner.path('inspection_reframe_home_'+str(index),view,self.home,leg['q1'])
            self.inspection_recovery.append(dict(view=view,path=leg,home=back))
            previous=view;seed=leg['q1']
        self.recoveries=[]
        for label in columns:
            grasp=self.planner.frame(label);obs=grasp.copy();obs[14]+=self.settings['observation_height_m']
            verify=grasp.copy();verify[14]+=self.settings['verification_lift_m']
            probe=verify.copy();probe[14]+=.004
            qobs=self.planner.solve(obs,self.qhome);qgrasp=self.planner.solve(grasp,qobs)
            qverify=self.planner.solve(verify,qgrasp);qprobe=self.planner.solve(probe,qverify);qlift=self.planner.solve(obs,qverify)
            paths=dict(approach=self.planner.path(label+'_approach',self.clear,obs,self.qclear),
                descend=self.planner.path(label+'_descend',obs,grasp,qobs),
                verification_lift=self.planner.path(label+'_verify_lift',grasp,verify,qgrasp),
                settling_probe=self.planner.path(label+'_settling_probe',verify,probe,qverify),
                lift_after_probe=self.planner.path(label+'_lift_after_probe',probe,obs,qprobe),
                full_lift=self.planner.path(label+'_fixed_full_lift',grasp,obs,qgrasp),
                lift=self.planner.path(label+'_lift',verify,obs,qverify),
                transfer=self.planner.path(label+'_transfer',obs,self.hover,qlift))
            for key in ('approach','descend'):paths[key]=self.planner.slower(paths[key],scale('row_motion_scale'))
            for key in ('full_lift','lift','lift_after_probe','transfer'):paths[key]=self.planner.slower(paths[key],scale('carry_motion_scale'))
            self.plans[label]=dict(grasp=grasp,observation=obs,paths=paths)

    def controller_manifest(self):
        manifest=copy.deepcopy(self.planner.manifest());manifest['commissioning_authorized']=True
        # The unchanged native binary hard-bounds this setting to1.5mm.
        # Grasp endpoints retain the original stricter gate in motion(). Task
        # placement warns above 1 mm within this existing bounded native guard.
        manifest['settings']['endpoint_position_m']=self.free_endpoint_tolerance
        manifest['execution_profile']=dict(task_execution=self.task_execution,
            setup=self.planner.setup,
            schedule_deadlines_enforced=bool(self.schedule.get('enabled') and not self.task_execution),
            visual_gates_enabled=not self.task_execution,
            conveyor_clearance_policy='operator_managed' if self.task_execution else 'fresh_fixed_camera_immediately_before_placement',
            cached_start_policy='Adapt the cached path to the live FCI command through the existing certified planner',
            free_endpoint_position_m=self.free_endpoint_tolerance,
            interaction_endpoint_position_m=self.settings['endpoint_position_m'],
            placement_accuracy_policy=('Warn above 1 mm within existing 1.5 mm controller guard; grasp remains strict' if self.task_execution else 'Strict measured endpoints'),
            measured_release_speed_enforced=not self.task_execution,
            certified_trajectory_geometry_and_derivative_limits_unchanged=True,
            profile_not_yet_physically_benchmarked=True)
        save(self.out/'execution_profile.json',manifest['execution_profile'])
        return manifest

    def acknowledge_empty_gripper(self):
        """CLI acknowledgement only: no earlier pick/placement success inferred."""
        self.owner.wait(lambda:self.owner.latest and time.monotonic()-self.owner.latest['hand_stamp']<.25,1)
        current=self.owner.state()['state']
        if current['robot_mode']!='Idle' or current['current_errors'] or max(map(abs,current['dq']))>=.01:
            raise Stop('Empty-gripper acknowledgement requires stationary Idle and no robot errors')
        # The explicit flag confirms physical emptiness. Establish the configured
        # open width through this same SDK owner before any arm movement/reset.
        opened=self.owner.hand(2);current=opened['state']
        opening=self.planner.motion['gripper']['approach_opening_m']
        if (not opened['hand_ok'] or abs(opened['hand_width']-opening)>.001 or
                time.monotonic()-opened['hand_stamp']>.25 or current['robot_mode']!='Idle' or
                current['current_errors'] or max(map(abs,current['dq']))>=.01):
            raise Stop('Empty-gripper acknowledgement lacks fresh fault-free open-hand verification')
        prior=HERE/'inventory.json'
        observed=prior.read_bytes() if prior.exists() else None
        if observed!=self.prior_inventory_bytes:
            raise Stop('Inventory changed during startup; pending operation was not cleared')
        if observed is not None:(self.out/'inventory_before_empty_gripper_confirmation.json').write_bytes(observed)
        previous=self.inventory.get('prior_session_inventory',{})
        acknowledgement=dict(argument='--execute-task' if self.task_execution else '--confirm-empty-gripper',utc=datetime.now(timezone.utc).isoformat(),
            confirmation='Operator confirms no bricks held and requests a fresh grasp execution',
            prior_active=previous.get('active'),previous_pick_or_placement_success_inferred=False,
            measured_open_width_m=opened['hand_width'],requested_columns=self.columns,
            robot_mode=current['robot_mode'],robot_errors=current['current_errors'])
        self.inventory['operator_empty_gripper_confirmation']=acknowledgement
        save(self.out/'empty_gripper_confirmation.json',acknowledgement)
        save(prior,self.inventory)
        print('EMPTY_GRIPPER_CONFIRMED; pending operation archived; earlier outcome not inferred',flush=True)

    def initialize(self):
        began=time.monotonic()
        if not self.task_execution:self.camera=Camera(self.cfg['camera'])
        manifest=self.controller_manifest()
        self.owner=Owner(manifest,self.out,controller=self.controller_path)
        if self.empty_gripper_confirmed:self.acknowledge_empty_gripper()
        if self.reset_prior_active:
            self.owner.wait(lambda:self.owner.latest and time.monotonic()-self.owner.latest['hand_stamp']<.25,1)
            snapshot=self.owner.state();s=snapshot['state']
            if (s['robot_mode']!='Idle' or s['current_errors'] or max(map(abs,s['dq']))>=.01 or
                    abs(snapshot['hand_width']-.07)>.001 or np.linalg.norm(np.array(s['O_T_EE'][12:15])-self.home[12:15])>.001):
                raise Stop('Source-reset acknowledgement requires stationary home, open hand, Idle and no errors')
            self.inventory['operator_source_reset']=dict(prior_active=self.reset_prior_active,
                confirmation='Operator explicitly confirmed selected source replaced and conveyor cleared',utc=datetime.now(timezone.utc).isoformat())
        current=self.owner.state()['state']
        actual=self.planner.model.fk(current['q_d']).homogeneous.reshape(-1,order='F').tolist()
        init=self.planner.path('initial_home',actual,self.home,current['q_d'])
        init_id=self.owner.load(init)
        self.ids['common']={k:self.owner.load(p) for k,p in self.common.items()}
        self.ids['inspection_recovery']=[dict(path=self.owner.load(p['path']),home=self.owner.load(p['home'])) for p in self.inspection_recovery]
        for label,item in self.plans.items():self.ids[label]={k:self.owner.load(p) for k,p in item['paths'].items()}
        # FCI refreshes q_d on begin(). Rebase/retime this startup leg through
        # the same live-command adapter as every subsequent ordinary motion.
        self.owner.begin();self.motion('initial_home',init,init_id);self.owner.finish()
        # Audit actual source presence before any pickup. This is setup evidence;
        # every cycle still obtains fresh alignment evidence of its selected row.
        if self.cfg.get('fixed_pose_execution'):
            for label in self.columns:self.inventory['available'][label]=dict(status='expected_at_fixed_taught_pose; pick not yet verified')
            save(HERE/'inventory.json',self.inventory)
            save(self.out/'initialization.json',dict(duration_s=time.monotonic()-began,planner_cache_identity=self.planner.identity,
                source_inventory='Fixed operator-confirmed layout; no completed picks inferred'))
            return
        self.owner.begin()
        pose=self.home;q=self.qhome
        for label,item in self.plans.items():
            path=self.planner.path(label+'_inventory',pose,item['observation'],q)
            id=self.owner.load(path);self.owner.go(id,path['duration_s'])
            prime_started=time.monotonic()
            for prime in range(3):
                try:
                    evidence,image=self.observe(label+'_initial_inventory_'+str(prime));break
                except Stop:
                    if prime==2 or time.monotonic()-prime_started>3:raise
            self.visual_references[label]=(image,evidence)
            self.inventory['available'][label]=dict(studs_per_brick=evidence['studs_per_brick'],run=str(self.out),verified_monotonic=time.monotonic())
            pose=item['observation'];q=path['q1']
        path=self.planner.path('inventory_return_home',pose,self.home,q);id=self.owner.load(path);self.owner.go(id,path['duration_s']);self.owner.finish()
        save(HERE/'inventory.json',self.inventory)
        save(self.out/'initialization.json',dict(duration_s=time.monotonic()-began,includes_source_inventory=True,
              planner_cache_identity=self.planner.identity,planned_paths=self.plans,common_paths=self.common))

    def observe(self,name):
        errors=[];observations=[];after=time.monotonic()
        detector=self.cfg.get('placement_detector',self.cfg['detector']) if '_placement' in name else self.cfg['detector']
        for attempt in range(6):
            f=self.camera.frame(after=after);image=f['image']
            began=time.monotonic();age_at_receive=began-f['captured_monotonic'];method='detect'
            try:
                label=name.split('_alignment_')[0].split('_initial_inventory')[0]
                reference=self.visual_references.get(label) if ('_alignment_' in name or '_initial_inventory' in name) else None
                if reference is None and observations:
                    reference=(observations[-1][1]['image'],observations[-1][0])
                if reference is None:d,overlay,rect=detect(image,detector)
                else:
                    try:method='track';d,overlay,rect=track_row(reference,image,detector)
                    except Uncertain:method='track_then_detect';d,overlay,rect=detect(image,detector)
                with (self.out/'vision_timing.ndjson').open('a') as timing:
                    timing.write(json.dumps(dict(name=name,attempt=attempt,method=method,capture_age_at_receive_s=age_at_receive,
                        computation_s=time.monotonic()-began,age_at_detection_s=time.monotonic()-f['captured_monotonic']))+'\n')
                if time.monotonic()-f['captured_monotonic']>self.settings['maximum_frame_age_s']:raise Uncertain('Detection evidence is stale')
                observations.append((d,f));after=f['captured_monotonic']+.001
                if len(observations)<2:continue
                delta=np.abs(np.array(observations[-1][0]['feature'])-observations[-2][0]['feature'])
                if max(delta[:2])>3 or delta[2]>math.radians(.5):raise Uncertain('Fresh detections are inconsistent')
                cv2.imwrite(str(self.out/(name+'.jpg')),image);cv2.imwrite(str(self.out/(name+'_overlay.jpg')),overlay)
                d.update(frame_sequence=f['sequence'],captured_monotonic=f['captured_monotonic'],frame_age_s=time.monotonic()-f['captured_monotonic'])
                save(self.out/(name+'.json'),d);return d,image
            except Uncertain as e:
                errors.append(str(e));after=time.monotonic();observations=[]
                if attempt in (0,5):cv2.imwrite(str(self.out/(name+f'_failed_{attempt}.jpg')),image)
        raise Stop('Bounded re-observation exhausted: '+str(errors))

    def phase(self,name,operation):
        began=time.monotonic();completed=False
        try:
            value=operation();completed=True;return value
        finally:
            self.timings.append(dict(phase=name,duration_s=time.monotonic()-began,completed=completed))
            save(self.out/'phase_timings.json',self.timings)

    def motion(self,name,path,id=None):
        if not path.get('tracking'):
            sample=self.owner.state();sent=sample['sent']
            joint_offset=max(abs(a-b) for a,b in zip(sent,path['q0']))
            # Tiny corrections can consume jerk headroom even below the old
            # 0.25 mrad patch threshold. Check the native bridge bound itself.
            T=path['duration_s'];difference=np.abs(np.array(sent)-path['q0'])
            bridge_needs_patch=False
            if joint_offset>1e-9:
                from scipy.interpolate import PPoly
                curve=PPoly(np.asarray(path['coefficients']).transpose(2,0,1),np.r_[path['knots_s'],T])
                for derivative,key,factor in ((1,'joint_velocity_rad_s',1.875),(2,'joint_acceleration_rad_s2',5.774),(3,'joint_jerk_rad_s3',60.)):
                    maximum=np.max(np.abs(curve.derivative(derivative)(np.linspace(0,T,1001))),axis=0)
                    if np.any(maximum+factor*difference/T**derivative>self.settings[key]):
                        bridge_needs_patch=True;break
            if joint_offset>.00025 or bridge_needs_patch:
                # A new FCI session anchors its command to measured q, which
                # need not equal the previous cached endpoint even at home.
                # Reuse the existing checked local-tube patch rather than
                # relaxing the native1mrad start guard or snapping the joints.
                start=self.planner.model.fk(sent).homogeneous.reshape(-1,order='F').tolist()
                if (not sample['control'] or sample['state']['current_errors'] or
                        max(map(abs,sample['state']['dq_d']))>=.01 or
                        np.linalg.norm(np.array(start[12:15])-path['start'][12:15])>self.free_endpoint_tolerance or
                        rotation_error(start,path['start'])>self.settings['endpoint_angle_rad']):
                    raise Stop('Cached path start differs from the stationary certified start; no path sent')
                original=path;path=self.planner.patch(path,start,path['target'],sent);id=None
                if bridge_needs_patch:
                    print(f'WARNING: FCI reference bridge automatically rebased and retimed '
                        f'{original["duration_s"]:.3f}s -> {path["duration_s"]:.3f}s; target and physical limits unchanged',flush=True)
                with (self.out/'cached_start_adjustments.ndjson').open('a') as log:
                    log.write(json.dumps(dict(phase=name,maximum_start_joint_offset_rad=joint_offset,
                        original_start_q=original['q0'],issued_start_q=sent,
                        patched_start_q=path['q0'],target_unchanged=path['target']==original['target'],
                        original_duration_s=original['duration_s'],issued_duration_s=path['duration_s'],
                        native_bridge_headroom_required_patch=bridge_needs_patch,
                        certificate=path['certificate']))+'\n')
        id=self.owner.load(path) if id is None else id
        state=self.owner.go(id,path['duration_s'])
        if name in ('descend','place'):
            try:state=self.interaction_endpoint(path,allow_placement_warning=name=='place' and self.task_execution)
            except Stop as error:
                # Keep the last measured evidence even when the interaction
                # gate rejects it. Never open/close on a failed endpoint.
                if self.owner.latest is not None:
                    self.log_endpoint(name,path,self.owner.latest,accepted=False,error=str(error))
                action='release' if name=='place' else 'close'
                raise Stop(f'{name.capitalize()} endpoint did not meet measured accuracy; no {action} command sent. '
                    'Inspect endpoint_diagnostics.ndjson; inventory remains pending') from error
        self.log_endpoint(name,path,state)
        return state

    def log_endpoint(self,name,path,state,accepted=True,error=None):
        m=self.planner.model;s=state['state'];xyz=lambda q:m.fk(q).translation.copy()
        requested=xyz(state['requested']);sent=xyz(state['sent']);desired=xyz(s['q_d']);measured=np.array(s['O_T_EE'][12:15])
        delta=measured-np.array(path['target'][12:15])
        diagnostic=dict(phase=name,accepted=accepted,error=error,target_xyz_m=path['target'][12:15],
            measured_xyz_m=measured.tolist(),measured_minus_target_m=delta.tolist(),
            measured_position_error_m=float(np.linalg.norm(delta)),
            position_tolerance_m=self.settings['endpoint_position_m'] if name in ('descend','place') else self.free_endpoint_tolerance,
            measured_angle_error_rad=rotation_error(s['O_T_EE'],path['target']),
            maximum_joint_speed_rad_s=max(map(abs,s['dq'])),
            estimated_base_wrench=s.get('O_F_ext_hat_K'),
            hand_width_m=state.get('hand_width'),robot_mode=s['robot_mode'],current_errors=s['current_errors'],
            sample_unix_ns=state.get('unix_ns'),
            target_to_requested_m=float(np.linalg.norm(np.array(path['target'][12:15])-requested)),
            requested_to_sent_m=float(np.linalg.norm(requested-sent)),
            sent_to_sdk_desired_m=float(np.linalg.norm(sent-desired)),
            sdk_desired_to_measured_m=float(np.linalg.norm(desired-measured)),
            model_measured_q_to_tcp_m=float(np.linalg.norm(xyz(s['q'])-measured)),
            sdk_desired_cartesian_field_to_fk_qd_m=float(np.linalg.norm(np.array(s['O_T_EE_d'][12:15])-desired)))
        with (self.out/'endpoint_diagnostics.ndjson').open('a') as f:f.write(json.dumps(diagnostic)+'\n')

    def placement_warning(self,detail,**evidence):
        warning=dict(type='placement_coordinate_warning',detail=detail,**evidence)
        with (self.out/'warnings.ndjson').open('a') as log:log.write(json.dumps(warning)+'\n')
        print('WARNING: placement coordinates may need updating:',detail,flush=True)

    def release_trace(self):
        # Ordered rows share one SDK owner. Its native files remain under the
        # first row's directory, even while subsequent rows log elsewhere.
        trace=max(self.owner.directory.glob('tracking_*.csv'),key=lambda p:p.stat().st_mtime)
        if trace.parent!=self.out:
            local=self.out/trace.name;local.write_bytes(trace.read_bytes());trace=local
        with trace.open() as stream:return trace,list(csv.DictReader(stream))

    def verify_release_metrics(self,metrics):
        if (not metrics or metrics['samples']<=0 or
                not 0<=metrics['release_start_t']<=metrics['opening_complete_t'] or
                metrics['maximum_sample_gap_s']>.020001):
            raise Stop('Actual gripper opening lacks complete fresh release telemetry')
        if not metrics['full_opening_speed_verified']:
            if not self.task_execution:raise Stop('Measured gripper +Y speed did not verify throughout actual opening')
            self.placement_warning('Gripper +Y release speed outside configured range; --execute-task speed gate disabled',
                release_speed_enforced=False,release_speed_verified=False,measured_release_metrics=metrics)

    def interaction_endpoint(self,path,allow_placement_warning=False):
        """Strict grasps; task placement may warn inside the native 1.5 mm guard."""
        stable_from=None
        tolerance=self.free_endpoint_tolerance if allow_placement_warning else self.settings['endpoint_position_m']
        def settled():
            nonlocal stable_from
            sample=self.owner.latest;now=time.monotonic()
            if sample is None:return False
            s=sample['state']
            valid=(now-sample['received_monotonic']<.15 and not s['current_errors'] and
                max(map(abs,s['dq']))<.01 and
                np.linalg.norm(np.array(s['O_T_EE'][12:15])-path['target'][12:15])<=tolerance and
                rotation_error(s['O_T_EE'],path['target'])<=self.settings['endpoint_angle_rad'])
            if not valid:stable_from=None;return False
            if stable_from is None:stable_from=now
            return now-stable_from>=self.settings['endpoint_stable_s']
        try:self.owner.wait(settled,self.settings['endpoint_timeout_s'])
        except Stop as error:raise Stop('Grasp/placement endpoint did not meet original measured accuracy; no gripper action') from error
        sample=self.owner.state()
        delta=np.array(sample['state']['O_T_EE'][12:15])-path['target'][12:15]
        if allow_placement_warning and np.linalg.norm(delta)>self.settings['endpoint_position_m']:
            self.placement_warning(f'Measured endpoint error {np.linalg.norm(delta)*1000:.3f} mm; continuing within the existing {tolerance*1000:.1f} mm guard',
                measured_minus_target_m=delta.tolist(),hard_position_limit_m=tolerance)
        return sample

    def active(self,label,phase):
        previous=self.inventory.get('active') or {}
        self.inventory['active']=dict(pose=label,phase=phase,run=str(self.out),origin_run=previous.get('origin_run',previous.get('run',str(self.out))),utc=datetime.now(timezone.utc).isoformat())
        if self.planner.setup:self.inventory['active']['setup']=self.planner.setup['name']
        save(HERE/'inventory.json',self.inventory)

    def verify_retention(self,label,before,lift_m):
        if lift_m<.0035:raise Stop('Measured verification lift is below3.5mm')
        errors=[]
        for attempt in range(3):
            frame=self.camera.frame(after=time.monotonic())['image'];cv2.imwrite(str(self.out/(label+f'_{attempt}.jpg')),frame)
            try:
                result=retention_evidence(before,frame);result['actual_lift_m']=lift_m
                save(self.out/(label+f'_{attempt}.json'),result)
                if result['all_four_retained']:return result,frame
                errors.append(result)
            except Uncertain as e:errors.append(str(e))
        save(self.out/(label+'_stop.json'),dict(errors=errors,inventory_advanced=False))
        raise Stop('Existing four-brick lift verification did not pass')

    def coarse_check(self,label):
        errors=[]
        for attempt in range(3):
            frame=self.camera.frame(after=time.monotonic());image=frame['image'];cv2.imwrite(str(self.out/(label+f'_coarse_{attempt}.jpg')),image)
            try:
                result,_,_=detect(image,self.cfg['detector'])
                correction=-np.linalg.solve(self.J,np.array(result['feature'])-self.goal)
            except Uncertain as e:errors.append(str(e));continue
            if np.linalg.norm(correction[:2])>.006 or abs(correction[2])>math.radians(3):raise Stop('Significant misalignment of fixed source pose')
            save(self.out/(label+'_coarse.json'),dict(result=result,delta_m_rad=correction.tolist(),fine_correction_applied=False))
            return result,image
        save(self.out/(label+'_coarse.json'),dict(status='No fine detection; use operator-confirmed fixed pose',observations=errors))
        return dict(fixed_pose=True),image

    def cycle(self,label):
        item=self.plans[label];paths=item['paths'];ids=self.ids[label];self.timings=[];self.recoveries=[]
        s=self.owner.state()
        if np.linalg.norm(np.array(s['state']['O_T_EE'][12:15])-self.home[12:15])>self.free_endpoint_tolerance or max(map(abs,s['state']['dq']))>.01 or abs(s['hand_width']-.07)>.001:
            raise Stop('Complete cycle must start stationary at home with70mm opening')
        start=time.monotonic();self.cycle_start_monotonic=start;self.cycle_event_mark=len(self.owner.events);state_start=len(self.owner.states)
        self.clock=PhaseClock(self.owner,start,self.schedule) if self.schedule.get('enabled') and not self.task_execution else None
        self.phase('control_begin',self.owner.begin)
        self.phase('home_departure' if self.task_execution else 'clearance_view',lambda:self.motion('clear',self.common['clear'],self.ids['common']['clear']))
        self.phase('approach',lambda:self.motion('approach',paths['approach'],ids['approach']))
        def align():
            began=time.monotonic();command=item['observation'].copy();correction_total=np.zeros(3)
            for attempt in range(7):
                evidence,image=self.observe(label+'_alignment_'+str(attempt))
                correction=-np.linalg.solve(self.J,np.array(evidence['feature'])-self.goal)
                if np.linalg.norm(correction[:2])<=.0007 and abs(correction[2])<=math.radians(.3):return command,evidence,image
                if time.monotonic()-began>self.settings['alignment_timeout_s']:raise Stop('Bounded visual alignment timed out')
                length=np.linalg.norm(correction[:2]);correction[:2]*=min(1,.0015/max(length,1e-9));correction[2]=np.clip(correction[2],-math.radians(.4),math.radians(.4))
                correction_total+=correction
                if np.linalg.norm(correction_total[:2])>.006 or abs(correction_total[2])>math.radians(3):raise Stop('Alignment exceeded taught-layout correction bounds')
                current=self.owner.state()['state'];q,command=self.planner.local_target(current['q_d'],command,correction)
                self.owner.servo(q,command)
            raise Stop('Alignment iteration bound')
        if self.task_execution:
            command=item['observation'].copy();source=dict(status='visual_gates_bypassed_by_execute_task');image=None
            save(self.out/(label+'_visual_policy.json'),dict(enabled=False,reason='--execute-task; execute taught poses without visual gating'))
        elif self.cfg.get('fixed_pose_execution'):
            source,image=self.phase('coarse_visual_supervision',lambda:self.coarse_check(label));command=item['observation'].copy()
        else:command,source,image=self.phase('visual_alignment',align)
        grasp=command.copy();grasp[14]=item['grasp'][14]
        if np.linalg.norm(np.array(command)-item['observation'])>1e-8:
            q=self.owner.state()['state']['q_d'];desc=self.planner.patch(paths['descend'],command,grasp,q)
            verify=grasp.copy();verify[14]+=self.settings['verification_lift_m']
            small=self.planner.patch(paths['verification_lift'],grasp,verify,desc['q1'])
            lift=self.planner.patch(paths['lift'],verify,command,small['q1'])
            transfer=self.planner.patch(paths['transfer'],command,self.hover,lift['q1'])
            use=dict(descend=desc,verification_lift=small,lift=lift,transfer=transfer);use_ids={k:None for k in use}
        else:use=paths;use_ids=ids
        self.phase('descent',lambda:self.motion('descend',use['descend'],use_ids['descend']))
        if self.clock:self.phase('row_synchronization',lambda:self.clock.at('row',self.schedule['row_at_s'],grasp,.069,.071))
        self.active(label,'closing_unverified')
        self.phase('close',lambda:self.owner.hand(1))
        self.active(label,'closed_unverified')
        return self.complete_held(label,use,use_ids,start,state_start)

    def complete_held(self,label,use,use_ids,start,state_start,recovery=False,operator_confirmed=False):
        hand=self.owner.state();before=None
        if not self.task_execution:
            before=self.camera.frame(after=time.monotonic())['image'];cv2.imwrite(str(self.out/(label+'_before_lift.jpg')),before)
        z0=self.plans[label]['grasp'][14] if operator_confirmed else self.owner.state()['state']['O_T_EE'][14]
        if self.cfg.get('fixed_pose_execution') and not operator_confirmed:
            minimum=self.cfg['minimum_loaded_contact_width_m'];maximum=self.planner.motion['gripper']['close_contact_upper_m']
            if not minimum<=hand['hand_width']<=maximum:raise Stop('Closure lacks four-brick-span contact; no transfer')
            self.active(label,'lifting_pick_unverified')
            lifted=self.phase('clearance_lift',lambda:self.motion('full_lift',use['full_lift'],use_ids['full_lift']))
            measured_lift=lifted['state']['O_T_EE'][14]-z0
            self.owner.wait(lambda:self.owner.latest and time.monotonic()-self.owner.latest['hand_stamp']<.25,.5)
            held=self.owner.state()
            if measured_lift<.045 or not minimum<=held['hand_width']<=maximum or abs(held['hand_width']-hand['hand_width'])>.001:
                raise Stop('Measured lift/contact retention is uncertain; no transfer')
            frame=None
            if not self.task_execution:
                frame=self.camera.frame(after=time.monotonic());cv2.imwrite(str(self.out/(label+'_clearance_visual_supervision.jpg')),frame['image'])
            retained=dict(method='Fixed four-brick layout; loaded contact width retained through measured full lift',
                measured_lift_m=measured_lift,initial_contact_width_m=hand['hand_width'],retained_contact_width_m=held['hand_width'],
                minimum_loaded_contact_width_m=minimum,frame_captured_monotonic=frame['captured_monotonic'] if frame else None,
                visual_supervision_enabled=not self.task_execution,
                image_based_individual_brick_count_verified=False)
            save(self.out/(label+'_retention.json'),retained);self.confirmed_picks+=1;self.active(label,'retention_verified')
            return self.carry_to_delivery(label,use,use_ids,start,state_start,recovery,operator_confirmed,hand)
        if operator_confirmed:
            small_lift=hand['state']['O_T_EE'][14]-z0
            proof=dict(method='operator_confirmation_after_recoverable_stop',all_four_held=True,
                automatic_visual_verification_passed=False,measured_lift_m=small_lift)
            small_image=before;save(self.out/(label+'_operator_pick_confirmation.json'),proof)
        else:
            self.active(label,'verification_lift_unverified')
            small=self.phase('verification_lift',lambda:self.motion('verification_lift',use['verification_lift'],use_ids['verification_lift']))
            small_lift=small['state']['O_T_EE'][14]-z0
            self.active(label,'lift_complete_pick_unverified')
            try:
                proof,small_image=self.phase('pick_verification',lambda:self.verify_retention(label+'_pick_verification',before,small_lift))
            except Stop as e:
                if str(e)!='Existing four-brick lift verification did not pass' or 'settling_probe' not in use:raise
                # One bounded upward probe after initial pad settling. The same
                # four-brick test and thresholds must independently pass.
                prior=self.owner.state();settled_before=self.camera.frame(after=time.monotonic())['image']
                self.active(label,'settling_probe_pick_unverified')
                probed=self.phase('settling_probe',lambda:self.motion('settling_probe',use['settling_probe'],use_ids['settling_probe']))
                if abs(probed['hand_width']-hand['hand_width'])>.001:raise Stop('Contact width changed during bounded pick recheck')
                delta=probed['state']['O_T_EE'][14]-prior['state']['O_T_EE'][14]
                proof,small_image=self.phase('pick_recheck',lambda:self.verify_retention(label+'_pick_recheck',settled_before,delta))
                small_lift=probed['state']['O_T_EE'][14]-z0
                use=dict(use,lift=use['lift_after_probe']);use_ids=dict(use_ids,lift=use_ids['lift_after_probe'])
        self.confirmed_picks+=1;self.active(label,'small_lift_verified')
        lifted=self.phase('clearance_lift',lambda:self.motion('lift',use['lift'],use_ids['lift']))
        measured_lift=lifted['state']['O_T_EE'][14]-z0
        if measured_lift<.045:raise Stop('Full measured clearance lift did not verify')
        if operator_confirmed:
            retained=dict(proof,additional_check='Measured clearance lift and unchanged contact width')
            if abs(lifted['hand_width']-hand['hand_width'])>.001:raise Stop('Contact width changed during clearance lift')
        else:
            retained,_=self.phase('clearance_retention',lambda:self.verify_retention(label+'_clearance_verification',small_image,measured_lift-small_lift))
        retained.update(measured_lift_m=measured_lift,contact_width_m=hand['hand_width'])
        save(self.out/(label+'_retention.json'),retained)
        self.active(label,'retention_verified')
        return self.carry_to_delivery(label,use,use_ids,start,state_start,recovery,operator_confirmed,hand)

    def carry_to_delivery(self,label,use,use_ids,start,state_start,recovery,operator_confirmed,hand):
        if recovery:
            self.phase('transfer_clearance',lambda:self.motion('transfer_clearance',use['transfer_clearance'],use_ids['transfer_clearance']))
        if recovery and np.linalg.norm(np.array(self.clear)-self.hover)<1e-9:arrived=self.owner.state()
        else:arrived=self.phase('transfer',lambda:self.motion('transfer',use['transfer'],use_ids['transfer']))
        if abs(arrived['hand_width']-hand['hand_width'])>.001:raise Stop('Contact width changed during carrying; no release')
        return self.finish_delivery(label,start,state_start,recovery,operator_confirmed)

    def check_conveyor_before_placement(self,label):
        # The wrist's held bricks obscure its old conveyor ROI. Use only the
        # independently registered fixed-camera destination slots, at arrival.
        began_ns=time.time_ns();until=time.monotonic()+.6
        evidence=dict(clear=False,reason='conveyor_camera_unavailable')
        while time.monotonic()<until:
            self.owner.state()  # Maintain fresh controller/fault supervision.
            try:
                mapping=json.loads((TEACHING/'external_layout/config.json').read_text())
                packet=json.loads((TEACHING/'external_layout/output/latest.json').read_text())
                evidence=conveyor_floor_evidence(packet,self.home_layer,self.place,mapping,time.time_ns(),began_ns)
                if evidence['clear']:
                    save(self.out/(label+'_conveyor_clearance.json'),dict(enabled=True,**evidence));return evidence
            except (OSError,ValueError,KeyError,TypeError) as error:
                evidence=dict(clear=False,reason='conveyor_camera_unavailable',detail=str(error))
            time.sleep(.03)
        save(self.out/(label+'_conveyor_clearance.json'),dict(enabled=True,**evidence))
        raise Stop('Conveyor not confirmed clear immediately before placement: '+evidence['reason'])

    def finish_delivery(self,label,start,state_start,recovery,operator_confirmed):
        clock=getattr(self,'clock',None) if not recovery else None
        stationary_delta=None
        if self.task_execution:
            save(self.out/(label+'_conveyor_clearance.json'),dict(enabled=False,clear=None,
                reason='--execute-task; operator manages conveyor clearing; no visual occupancy gate'))
        else:self.phase('preplacement_conveyor_clear_verification',lambda:self.check_conveyor_before_placement(label))
        if self.stationary:
            before_place=np.array(self.owner.state()['state']['O_T_EE'][12:14])
            self.active(label,'placement_descent_unverified_holding')
            self.phase('vertical_placement',lambda:self.motion('place',self.common['place'],self.ids['common']['place']))
            self.active(label,'placement_endpoint_verified_holding')
            if clock:self.phase('opening_synchronization',lambda:clock.at('opening_start',self.schedule['placing_at_s']-self.schedule['opening_lead_s'],self.place,self.cfg['minimum_loaded_contact_width_m'],self.planner.motion['gripper']['close_contact_upper_m']))
            self.active(label,'release_started_placement_unverified')
            tracked=self.phase('release',lambda:self.owner.hand(2))
            stationary_delta=(np.array(tracked['state']['O_T_EE'][12:14])-before_place).tolist()
            release_complete_s=time.monotonic()-start
        else:
            self.active(label,'speed_matched_placement_unverified')
            tracked=self.phase('speed_matched_placement',lambda:self.motion('track',self.common['track'],self.ids['common']['track']))
            release_complete_s=None
        if abs(tracked['hand_width']-.07)>.001 or not tracked['hand_ok']:raise Stop('Opening did not verify')
        self.active(label,'released_placement_unverified')
        if clock:self.phase('placing_synchronization',lambda:clock.at('placing',self.schedule['placing_at_s'],self.place,.069,.071))
        retreat_phase='retreat_to_inspection' if self.placement_verification_enabled else 'retreat_to_clearance'
        self.phase(retreat_phase,lambda:self.motion('inspect',self.common['inspect'],self.ids['common']['inspect']))
        if self.placement_verification_enabled:
            return_path,return_id=self.phase('placement_verification',lambda:self.verify_placement(label))
        else:
            save(self.out/(label+'_placement_verification.json'),dict(enabled=False,visually_verified=False,
                reason='Post-placement camera verification disabled by operator; measured release and final home checks remain required'))
            return_path,return_id=self.common['return'],self.ids['common']['return']
        self.phase('return_home',lambda:self.motion('return',return_path,return_id))
        metrics_mark=len(self.owner.events)
        self.phase('control_finish',self.owner.finish)
        if clock:self.phase('home_synchronization',lambda:clock.at('home',self.schedule['home_at_s'],self.home,.069,.071,
            position_tolerance_m=self.free_endpoint_tolerance))
        final=self.owner.state()
        if abs(final['hand_width']-.07)>.001 or final['state']['current_errors'] or np.linalg.norm(np.array(final['state']['O_T_EE'][12:15])-self.home[12:15])>self.free_endpoint_tolerance:
            raise Stop('Cycle final home/open/robot verification failed')
        metrics=None;measured_forward=None
        if not self.stationary:
            metrics=next((e for e in self.owner.events[metrics_mark:] if e['type']=='release_metrics'),None)
            self.verify_release_metrics(metrics)
            trace,tracking=self.release_trace()
            measured_forward=float(tracking[-1]['y'])-float(tracking[0]['y'])
            if not .05<=measured_forward<=.07:raise Stop('Measured conveyor forward travel is outside50–70mm')
            if self.task_execution:
                errors=[math.hypot(float(r['x'])-self.place[12],float(r['z'])-self.place[14]) for r in tracking
                    if r['release_requested']=='1' and r['opening_complete']=='0']
                if errors and max(errors)>self.settings['endpoint_position_m']:
                    self.placement_warning(f'Moving release X/Z deviation reached {max(errors)*1000:.3f} mm; native height/corridor checks remained enforced',
                        maximum_release_xz_error_m=max(errors),tracking_trace=str(trace))
        verified_placements=int(self.placement_verification_enabled)
        self.confirmed_placements+=verified_placements
        self.inventory['completed'][label]=dict(run=str(self.out),report=label+'_cycle.json',correct_picks=1,
            setup=self.planner.setup['name'] if self.planner.setup else None,
            correct_placements=verified_placements,release_verified=True,
            placement_verification_enabled=self.placement_verification_enabled,
            placement_visually_verified=self.placement_verification_enabled)
        self.inventory['available'].pop(label,None);self.inventory['active']=None
        save(HERE/'inventory.json',self.inventory)
        duration=time.monotonic()-start
        overhead=max(0.,duration-sum(t['duration_s'] for t in self.timings))
        result=dict(pose=label,completed=True,total_duration_s=None if recovery else duration,recovery_duration_s=duration if recovery else None,
            recovered_existing_grasp=recovery,full_cycle_benchmark=not recovery,target_s=self.schedule['home_at_s'] if clock else None if self.task_execution else 5,phase_timings=self.timings,
            task_execution=self.task_execution,schedule_deadlines_enforced=clock is not None,
            visual_gates_enabled=not self.task_execution,
            placement_mode=self.cfg['conveyor_placement_mode'],synchronized_markers=clock.events if clock else [],release_complete_actual_s=release_complete_s,
            operator_confirmed_pick=operator_confirmed,
            correct_picks=1,correct_placements=verified_placements,placements_released=1,
            placement_verification_enabled=self.placement_verification_enabled,
            placement_visually_verified=self.placement_verification_enabled,
            stops=0,measured_release_vy_m_s=None if metrics is None else dict(mean=metrics['mean_vy_m_s'],minimum=metrics['min_vy_m_s'],maximum=metrics['max_vy_m_s']),
            full_opening_release_metrics=metrics,
            measured_release_speed_enforced=not self.task_execution if not self.stationary else None,
            bounded_recoveries=self.recoveries,
            tracking_inventory_and_other_in_cycle_overhead_s=overhead,
            commanded_release_vy_m_s=0 if self.stationary else self.planner.moving['belt_velocity_base_m_s'][1],forward_stroke_m=0 if self.stationary else self.stroke['certificate']['stroke_m'],
            measured_forward_travel_m=measured_forward,
            measured_stationary_placement_xy_delta_m=stationary_delta,
            release_opening_latency_s=tracked['hand_latency_s'],inventory_advanced=True,
            verification=('Post-placement camera verification disabled by operator; measured release/open hand and final home checks passed; placement not visually verified' if not self.placement_verification_enabled else
                'Operator-confirmed held pick; measured clearance and contact-width checks; fresh destination; open hand; final home' if operator_confirmed else
                'Fixed four-brick row: loaded width retained through full lift; fresh camera supervision; fresh four-brick destination detection; open hand and final home' if self.cfg.get('fixed_pose_execution') else
                'Fixed taught poses; existing small-lift and full-clearance camera checks; fresh destination; open hand; final home'))
        save(self.out/(label+'_cycle.json'),result)
        print(json.dumps(result),flush=True);return result

    def check_placement(self,label,placed):
        offset=-np.linalg.solve(self.J,np.array(placed['feature'])-self.goal)
        view=placed.get('view_pose_O_T_EE',self.inspection)
        estimated_xy=np.array(view[12:14])+offset[:2]
        relative=estimated_xy-np.array(self.place[12:14])
        in_lane=np.linalg.norm(relative)<=.006 if self.stationary else abs(relative[0])<=.006 and -.006<=relative[1]<=self.planner.moving['maximum_forward_stroke_m']+.006
        if not in_lane or abs(offset[2])>math.radians(3):
            raise Stop('Placed four-brick row does not match delivery corridor')
        save(self.out/(label+'_placement_corridor.json'),dict(estimated_xy_m=estimated_xy.tolist(),
            relative_to_conveyor_origin_m=relative.tolist(),method='Local image Jacobian check of known conveyor lane; not a new camera extrinsic calibration'))

    def verify_placement(self,label):
        errors=[]
        if 'inspection_recovery' not in self.ids:
            self.ids['inspection_recovery']=[dict(path=self.owner.load(p['path']),home=self.owner.load(p['home'])) for p in self.inspection_recovery]
        for attempt in range(1+len(self.inspection_recovery)):
            if attempt:
                recovery=self.inspection_recovery[attempt-1];ids=self.ids['inspection_recovery'][attempt-1]
                self.motion('inspection_reframe',recovery['path'],ids['path'])
            try:placed,_=self.observe(label+'_placement_view_'+str(attempt))
            except Stop as e:
                if not str(e).startswith('Bounded re-observation exhausted:'):raise
                errors.append(str(e));self.recoveries.append(dict(stage='placement_framing',view=attempt,error=str(e)));continue
            placed['view_pose_O_T_EE']=self.owner.state()['state']['O_T_EE'];self.check_placement(label,placed)
            save(self.out/(label+'_placement.json'),placed)
            return (self.common['return'],self.ids['common']['return']) if attempt==0 else (self.inspection_recovery[attempt-1]['home'],self.ids['inspection_recovery'][attempt-1]['home'])
        raise Stop('Placement uncertain after bounded resident inspection views: '+str(errors))

    def resume_inspection(self,label):
        active=self.inventory.get('active')
        if not active or active['pose']!=label or active['phase']!='released_placement_unverified':raise Stop('No released placement awaiting inspection')
        prior=Path(active['run']);rows=[json.loads(l) for l in (prior/'controller.ndjson').read_text().splitlines()]
        metrics=next((r for r in rows if r['type']=='release_metrics'),None)
        if not metrics or not metrics['full_opening_speed_verified']:raise Stop('Original release speed verification did not complete')
        self.camera=Camera(self.cfg['camera']);manifest=self.planner.manifest();manifest['commissioning_authorized']=True;self.owner=Owner(manifest,self.out)
        self.owner.wait(lambda:self.owner.latest and self.owner.latest['hand_stamp']>0,1);now=self.owner.state();state=now['state']
        if state['robot_mode']!='Idle' or state['current_errors'] or abs(now['hand_width']-.07)>.001 or np.linalg.norm(np.array(state['O_T_EE'][12:15])-self.inspection[12:15])>.0015:
            raise Stop('Inspection recovery requires known inspection pose and open hand')
        began=time.monotonic();placed,_=self.phase('placement_verification',lambda:self.observe(label+'_placement'));self.check_placement(label,placed)
        start=self.planner.model.fk(state['q_d']).homogeneous.reshape(-1,order='F').tolist();path=self.planner.path(label+'_inspection_recovery_home',start,self.home,state['q_d']);id=self.owner.load(path)
        self.owner.begin();self.phase('return_home',lambda:self.motion('return_home',path,id));self.phase('control_finish',self.owner.finish)
        final=self.owner.state()
        if abs(final['hand_width']-.07)>.001 or np.linalg.norm(np.array(final['state']['O_T_EE'][12:15])-self.home[12:15])>.001:raise Stop('Final home/open check failed')
        self.confirmed_picks=self.confirmed_placements=1
        self.inventory['completed'][label]=dict(run=str(self.out),prior_run=str(prior),correct_picks=1,correct_placements=1,recovered=True,automatic_uninterrupted_cycle=False)
        self.inventory['available'].pop(label,None);self.inventory['active']=None;save(HERE/'inventory.json',self.inventory)
        report=dict(completed=True,pose=label,recovered_inspection=True,full_cycle_benchmark=False,total_duration_s=None,
            recovery_duration_s=time.monotonic()-began,phase_timings=self.timings,original_release_metrics=metrics,
            correct_picks=1,correct_placements=1,returned_home_open=True,prior_run=str(prior))
        save(self.out/(label+'_inspection_recovery.json'),report);print(json.dumps(report),flush=True);return report

    def resume_clear_delivery(self,label):
        active=self.inventory.get('active')
        if not active or active['pose']!=label or active['phase'] not in ('retention_verified','held_at_home_after_placement_stop'):
            raise Stop('Delivery recovery requires the recorded verified held pick')
        from_home=active['phase']=='held_at_home_after_placement_stop'
        rows=[json.loads(l) for l in (Path(active['run'])/'controller.ndjson').read_text().splitlines()]
        expected=next(r for r in reversed(rows) if r['type']=='state')
        self.camera=Camera(self.cfg['camera']);manifest=self.planner.manifest();manifest['commissioning_authorized']=True
        self.owner=Owner(manifest,self.out)
        self.owner.wait(lambda:self.owner.latest and self.owner.latest['hand_stamp']>0,1)
        now=self.owner.state();state=now['state']
        checkpoint=active['last_measured_xyz_m'] if from_home else expected['state']['O_T_EE'][12:15]
        expected_width=active['last_measured_hand_width_m'] if from_home else expected['hand_width']
        location=self.home if from_home else self.clear
        if (np.linalg.norm(np.array(state['O_T_EE'][12:15])-checkpoint)>.0015 or
                abs(now['hand_width']-expected_width)>.001 or
                np.linalg.norm(np.array(state['O_T_EE'][12:15])-location[12:15])>(.001+self.planner.surface_offset if from_home else .001) or
                state['robot_mode']!='Idle' or state['current_errors']):
            raise Stop('Arm/hand changed from recorded conveyor clearance stop')
        save(self.out/'operator_recovery_evidence.json',dict(all_four_held=True,placement_spot_clear=True,
            source='Prior held-pick and clearance confirmations; explicit request to execute placement',
            resumed_from_home=from_home,automatic_full_cycle=False))
        self.confirmed_picks=1
        start_pose=self.planner.model.fk(state['q_d']).homogeneous.reshape(-1,order='F').tolist()
        path=self.planner.path(label+'_resume_delivery_hover',start_pose,self.hover,state['q_d'])
        id=self.owner.load(path);self.ids['common']={k:self.owner.load(p) for k,p in self.common.items()}
        began=time.monotonic();state_start=len(self.owner.states);self.owner.begin()
        self.active(label,'held_delivery_transfer_unverified')
        arrived=self.phase('transfer',lambda:self.motion('transfer',path,id))
        if abs(arrived['hand_width']-now['hand_width'])>.001:raise Stop('Contact width changed before release')
        self.active(label,'retention_verified')
        return self.finish_delivery(label,began,state_start,True,True)

    def resume_closed(self,label,operator_confirmed=False):
        active=self.inventory.get('active')
        allowed=('lift_complete_pick_unverified',) if operator_confirmed else ('closing_unverified','closed_unverified')
        if not active or active['pose']!=label or active['phase'] not in allowed:
            raise Stop('Resume requires the recorded unverified closure for the selected source')
        rows=[json.loads(l) for l in (Path(active['run'])/'controller.ndjson').read_text().splitlines()]
        expected=next(r for r in reversed(rows) if r['type']=='state')
        self.camera=Camera(self.cfg['camera']);manifest=self.planner.manifest();manifest['commissioning_authorized']=True
        self.owner=Owner(manifest,self.out)
        self.owner.wait(lambda:self.owner.latest and self.owner.latest['hand_stamp']>0,1)
        now=self.owner.state();state=now['state'];grasp=self.plans[label]['grasp']
        expected_z=expected['state']['O_T_EE'][14] if operator_confirmed else grasp[14]
        if (np.linalg.norm(np.array(state['O_T_EE'][12:15])-expected['state']['O_T_EE'][12:15])>.0015 or
                abs(now['hand_width']-expected['hand_width'])>.001 or abs(state['O_T_EE'][14]-expected_z)>.001):
            raise Stop('Arm/hand changed from recorded closure; no automatic recovery move')
        start_pose=self.planner.model.fk(state['q_d']).homogeneous.reshape(-1,order='F').tolist()
        small=start_pose.copy();small[14]=grasp[14]+self.settings['verification_lift_m']
        lifted=small.copy();lifted[14]=grasp[14]+self.settings['observation_height_m']
        p1=self.planner.path(label+'_recovery_verify',start_pose,small,state['q_d'])
        p2=self.planner.path(label+'_recovery_lift',small,lifted,p1['q1'])
        p3=self.planner.path(label+'_recovery_transfer_high',lifted,self.clear,p2['q1'])
        p4=self.planner.path(label+'_recovery_hover',self.clear,self.hover,p3['q1'])
        use=dict(verification_lift=p1,lift=p2,transfer_clearance=p3,transfer=p4)
        if operator_confirmed:use['lift']=self.planner.path(label+'_confirmed_pick_clearance',start_pose,lifted,state['q_d'])
        ids={k:self.owner.load(p) for k,p in use.items()};self.ids['common']={k:self.owner.load(p) for k,p in self.common.items()}
        began=time.monotonic();state_start=len(self.owner.states);self.owner.begin()
        return self.complete_held(label,use,ids,began,state_start,recovery=True,operator_confirmed=operator_confirmed)

    def close(self):
        if self.owner:self.owner.close()
        if self.camera:self.camera.close()

    def recover_released_home(self,label):
        """Bounded safe retreat; never marks an uncertain delivery complete."""
        if not self.owner:
            manifest=self.controller_manifest()
            self.owner=Owner(manifest,self.out)
        elif self.owner.latest and self.owner.latest['control'] and self.owner.error is None:
            self.owner.finish()
        if self.owner.error:
            if self.owner.latest and self.owner.latest['control']:self.owner.send('stop',id=self.owner.next_id())
            expected=getattr(self.owner,'last_hand_request_id',None) if self.stationary else 1000000+self.ids['common']['track']
            self.owner.recover_idle(require_opening_done=True,opening_request_id=expected)
        self.owner.wait(lambda:self.owner.latest and self.owner.latest['hand_stamp']>0,1)
        current=self.owner.state();state=current['state']
        if state['robot_mode']!='Idle' or state['current_errors'] or abs(current['hand_width']-.07)>.001:
            raise Stop('Release retreat requires fault-free Idle and a measured open hand')
        xy=np.array(state['O_T_EE'][12:14])-self.place[12:14]
        if abs(xy[0])>.002 or not -.002<=xy[1]<=.072:raise Stop('Release retreat is outside conveyor corridor')
        pose=self.planner.model.fk(state['q_d']).homogeneous.reshape(-1,order='F').tolist();seed=state['q_d'];paths=[]
        if pose[14]<self.clear[14]-.001:
            clear=pose.copy();clear[14]=self.clear[14]
            leg=self.planner.path('stopped_release_vertical_clearance',pose,clear,seed);paths.append(leg);pose=clear;seed=leg['q1']
        paths.append(self.planner.path('stopped_release_home',pose,self.home,seed));ids=[self.owner.load(p) for p in paths]
        began=time.monotonic();self.owner.begin()
        for p,id in zip(paths,ids):self.motion('guarded_release_retreat',p,id)
        self.owner.finish();final=self.owner.state()
        self.active(label,'recovered_home_after_release_stop')
        self.inventory['active'].update(last_measured_xyz_m=final['state']['O_T_EE'][12:15],last_measured_hand_width_m=final['hand_width'])
        save(HERE/'inventory.json',self.inventory)
        result=dict(returned_home=True,duration_s=time.monotonic()-began,inventory_not_advanced=True,mode=final['state']['robot_mode'],errors=final['state']['current_errors'])
        save(self.out/'guarded_release_recovery.json',result);return result


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('columns',nargs='*',default=['5_0','5_1','5_2']);parser.add_argument('--execute',action='store_true');recovery=parser.add_mutually_exclusive_group();recovery.add_argument('--resume-closed',action='store_true');recovery.add_argument('--resume-confirmed-pick',action='store_true',help='Resume only after the operator confirms all four bricks are held following a stopped verification lift');recovery.add_argument('--resume-clear-delivery',action='store_true',help='Resume held delivery only after explicit operator confirmation that the occluded placement spot is clear');recovery.add_argument('--resume-inspection',action='store_true');recovery.add_argument('--source-reset-confirmed',action='store_true',help='After an unresolved stop: operator has physically restored selected sources and cleared conveyor; requires stationary home/open/Idle');args=parser.parse_args()
    if not args.execute:print('Preview only. --execute runs the deterministic resident cycle on specified stack5 columns.');return
    if not args.columns or any(x not in ('5_0','5_1','5_2') for x in args.columns) or len(set(args.columns))!=len(args.columns):raise Stop('Use distinct stack5 columns5_0,5_1,5_2')
    cv2.setNumThreads(1);os.sched_setaffinity(0,{3,4,5,9,10,11})
    out=HERE/'runs'/('cycles_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ'));out.mkdir(parents=True)
    factory=None;results=[];start=time.monotonic()
    try:
        factory=Factory(out,args.columns,resume=args.resume_closed or args.resume_confirmed_pick or args.resume_clear_delivery or args.resume_inspection,source_reset=args.source_reset_confirmed)
        if args.resume_closed or args.resume_confirmed_pick or args.resume_clear_delivery or args.resume_inspection:
            if len(args.columns)!=1:raise Stop('Recover only one recorded closed grasp at a time')
            initialized=time.monotonic()-start
            results.append(factory.resume_inspection(args.columns[0]) if args.resume_inspection else factory.resume_clear_delivery(args.columns[0]) if args.resume_clear_delivery else factory.resume_closed(args.columns[0],operator_confirmed=args.resume_confirmed_pick))
        else:
            factory.initialize();initialized=time.monotonic()-start
            for label in args.columns:results.append(factory.cycle(label))
        save(out/'outcome.json',dict(completed=True,initialization_s=initialized,cycles=results,correct_picks=len(results),correct_placements=len(results),stops=0))
    except Exception as e:
        recovery_result=None
        if factory and factory.owner and factory.inventory.get('active'):
            latest=factory.owner.latest
            if latest:
                active=factory.inventory['active']
                if any(e['type']=='release_metrics' and e.get('samples',0)>0 for e in factory.owner.events[getattr(factory,'cycle_event_mark',0):]):
                    active['phase']='release_started_placement_unverified'
                if latest['hand_done']>=1000000 and abs(latest['hand_width']-.07)<.001:
                    active['phase']='released_placement_unverified'
                active.update(last_stop=str(e),last_measured_xyz_m=latest['state']['O_T_EE'][12:15],
                    last_measured_hand_width_m=latest['hand_width'],run=str(out))
                save(HERE/'inventory.json',factory.inventory)
                if active['phase'] in ('release_started_placement_unverified','released_placement_unverified'):
                    try:
                        recovery_result=factory.recover_released_home(active['pose'])
                        factory.inventory['active']['last_stop']=str(e);save(HERE/'inventory.json',factory.inventory)
                    except Exception as recovery_error:
                        recovery_result=dict(returned_home=False,error=str(recovery_error),inventory_not_advanced=True)
        save(out/'outcome.json',dict(completed=False,error=str(e),cycles=results,
            correct_picks=factory.confirmed_picks if factory else 0,
            correct_placements=factory.confirmed_placements if factory else 0,
            phase_timings=factory.timings if factory else [],
            bounded_failure_recovery=recovery_result,
            total_attempt_duration_s=time.monotonic()-factory.cycle_start_monotonic if factory and hasattr(factory,'cycle_start_monotonic') else None,
            stops=1,unverified_inventory_not_advanced=True))
        print('RECOVERABLE STOP',e,'LOG',out,flush=True);raise
    finally:
        if factory:factory.close()


if __name__=='__main__':main()
