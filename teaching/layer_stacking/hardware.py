"""Existing SDK owner + certified transport + guarded slow outer admittance.

All layer-specific physical commissioning fields are required BEFORE SDK access.
The new native binary adds a callback-rate overload guard; the conveyor binary
is never rebuilt or substituted. No physical commissioning is asserted here.
"""
import copy
import hashlib
import json
from pathlib import Path
import sys
import time

import numpy as np

from .geometry import ROOT,rz
from .pipeline import RecoverableStop,save
from .vision import Uncertain,reference_key

HERE=Path(__file__).resolve().parent


def readiness(config):
    h=config['hardware'];missing=[]
    flags=('enabled','whole_layer_grasp_qualified','board_fixture_fixed','workspace_verified','minimum_release_retention_verified')
    for key in flags:
        if h.get(key) is not True:missing.append(key)
    required=('commissioning_id','source_poses','first_seated_O_T_EE','workspace_bounds_m','expected_F_T_EE',
              'minimum_insertion_m','seated_depth_tolerance_m','contact_reference_z_m','contact_force_N','seating_force_N',
              'maximum_force_N','maximum_lateral_force_N','maximum_torque_Nm','admittance_m_s_N',
              'maximum_vertical_speed_m_s','wrench_sign_for_downward_contact','bias_maximum_std_N',
              'bias_maximum_mean_deviation_N','bias_maximum_mean_deviation_Nm','free_space_wrench_bias_by_parity',
              'minimum_loaded_width_m','maximum_loaded_width_m','maximum_width_change_m')
    for key in required:
        if h.get(key) is None or h.get(key)=={}:missing.append(key)
    if not config['vision']['reference_file']:missing.append('vision.reference_file')
    if not (HERE/'controller').exists():missing.append('layer controller binary; run build.sh')
    if missing:raise RecoverableStop('Hardware disabled before SDK access; missing: '+', '.join(missing))
    if not 0<h['contact_force_N']<h['seating_force_N']<h['maximum_force_N']:
        raise RecoverableStop('Require contact < seating < hard force limits')
    if h['wrench_sign_for_downward_contact'] not in (-1,1):raise RecoverableStop('Measured wrench sign required')
    for key in required[5:]:
        if key in ('wrench_sign_for_downward_contact','free_space_wrench_bias_by_parity'):continue
        value=h[key]
        if not isinstance(value,(int,float)) or not np.isfinite(value) or value<=0:raise RecoverableStop('Invalid commissioned parameter '+key)
    gripper=json.loads((ROOT/'teaching/motion_config.json').read_text())['gripper']
    if not gripper['close_target_m']<h['minimum_loaded_width_m']<h['maximum_loaded_width_m']<=gripper['close_contact_upper_m']:
        raise RecoverableStop('Measured whole-layer contact window must reject empty closure and remain within native hand limits')
    for label in config['job']['sources']:
        if label not in h['source_poses']:raise RecoverableStop('Untaught whole-layer source '+label)
    for raw in [h['first_seated_O_T_EE'],*h['source_poses'].values()]:
        if np.asarray(raw).shape!=(16,) or not np.isfinite(raw).all():raise RecoverableStop('Invalid taught layer transform')
        pose=np.asarray(raw).reshape(4,4,order='F');R=pose[:3,:3]
        if (not np.allclose(pose[3],[0,0,0,1],atol=1e-8) or
                not np.allclose(R.T@R,np.eye(3),atol=1e-5) or np.linalg.det(R)<.99999 or
                np.linalg.norm(R[:,2]-[0,0,-1])>1e-4):
            raise RecoverableStop('Taught layer pose must be a proper vertical tool transform')
    bounds=h['workspace_bounds_m']
    if any(np.asarray(bounds[key]).shape!=(3,) or not np.isfinite(bounds[key]).all() for key in ('min','max')) or np.any(np.asarray(bounds['min'])>=bounds['max']):
        raise RecoverableStop('Invalid commissioned layer workspace')
    if np.asarray(h['expected_F_T_EE']).shape!=(16,) or not np.isfinite(h['expected_F_T_EE']).all():raise RecoverableStop('Invalid commissioned tool frame')
    ref=json.loads((ROOT/config['vision']['reference_file']).read_text())
    if ref.get('alignment_height_m')!=config['contact']['alignment_height_m'] or ref.get('verification_height_m')!=config['motion']['transport_clearance_m']:
        raise RecoverableStop('Image references must match the configured alignment and verification heights')
    if not h['contact_reference_z_m']<config['contact']['alignment_height_m']:raise RecoverableStop('Alignment/bias pose must be above measured contact')
    for parity in {k%2 for k in range(len(config['job']['sources']))}:
        bias=h['free_space_wrench_bias_by_parity'].get(str(parity))
        if np.asarray(bias).shape!=(6,) or not np.isfinite(bias).all():raise RecoverableStop('Missing measured free-space wrench baseline for orientation')
    for k in range(len(config['job']['sources'])):
        for stage in ('align','verify'):
            if reference_key(dict(index=k),stage) not in ref:raise RecoverableStop('Missing baseplate/parity/stage image reference')
    model=json.loads((ROOT/'digital_twin/isaac/environments/profiles.json').read_text())['fingers']['wide']
    if ref.get('wide_cad_sha256')!=model['cad_sha256'] or ref.get('cavity_height_m')!=model['ceiling']['target_cavity_height_m']:
        raise RecoverableStop('Wrist references do not match the current wide tool')
    camera=json.loads((ROOT/'teaching/factory_vision/config.json').read_text())['camera']
    camera_hash=hashlib.sha256(json.dumps(camera,sort_keys=True).encode()).hexdigest()
    if ref.get('camera_profile_sha256')!=camera_hash:raise RecoverableStop('Wrist reference camera settings changed')
    return ref


class Hardware:
    domain='hardware'
    def __init__(self,config,out,confirm_empty=False,reset_confirmed=False):
        self.references=readiness(config)
        if not confirm_empty:raise RecoverableStop('Real start requires --confirm-empty-gripper')
        self.config=config;self.h=config['hardware'];self.out=Path(out);self.owner=self.camera=None
        self.reset_confirmed=reset_confirmed
        self.bias=np.zeros(6);self.target=None;self.paths={};self.initialized=False;self.width_before=None
    def now(self):return time.monotonic()
    def wait(self,dt):time.sleep(dt)
    def contact_parameters(self):return self.h
    def preflight(self,plan):
        sys.path.insert(0,str(ROOT/'teaching/factory_cycle'));sys.path.insert(0,str(ROOT/'teaching'))
        from move_xyz import capture_pose
        from planner import Planner,digest
        state=capture_pose()['state']
        if state['current_errors'] or state['robot_mode']!='Idle' or max(map(abs,state['dq']))>=.01:raise RecoverableStop('Need stationary fault-free Idle before startup')
        if not np.allclose(state['F_T_EE'],self.h['expected_F_T_EE'],atol=1e-6,rtol=0):raise RecoverableStop('Tool frame differs from commissioning')
        settings=json.loads((ROOT/'teaching/factory_cycle/settings.json').read_text())
        self.planner=Planner(settings,state);bounds=self.h['workspace_bounds_m']
        self.planner.lower=np.array(bounds['min']);self.planner.upper=np.array(bounds['max'])
        self.planner.identity=digest(dict(parent=self.planner.identity,task='whole_layer_stacking',hardware=self.h))
        self.planner.cache_dir=HERE/'cache'/self.planner.identity;self.planner.cache_dir.mkdir(parents=True,exist_ok=True)
        previous=self.planner.model.fk(state['q_d']).homogeneous.reshape(-1,order='F').tolist();seed=state['q_d']
        clear_z=max(previous[14],max(raw[14]+self.config['motion']['approach_height_m'] for raw in self.h['source_poses'].values()),
            self.h['first_seated_O_T_EE'][14]+(len(plan['layers'])-1)*self.config['geometry']['layer_pitch_m']+self.config['motion']['transport_clearance_m'])
        for layer in plan['layers']:
            k=layer['index'];source=list(self.h['source_poses'][layer['source']]);target=np.array(self.h['first_seated_O_T_EE']).reshape(4,4,order='F').copy()
            target[2,3]+=k*self.config['geometry']['layer_pitch_m'];target[:3,:3]=rz(layer['yaw_rad'])@target[:3,:3]
            if np.linalg.norm(target[:3,2]-[0,0,-1])>1e-4:raise RecoverableStop('Taught target must be vertical')
            hover=source.copy();hover[14]+=self.config['motion']['approach_height_m']
            transport=hover.copy();transport[14]=clear_z
            departure=previous.copy();departure[14]=clear_z
            pre=target.reshape(-1,order='F').tolist();pre[14]+=self.config['contact']['alignment_height_m']
            target_hover=pre.copy();target_hover[14]=transport[14]
            inspection=target.reshape(-1,order='F').tolist();inspection[14]+=self.config['motion']['transport_clearance_m']
            paths={};p=previous;q=seed
            for name,end in (('departure_clearance',departure),('source_overhead',transport),('approach_source',hover),('grasp',source),('lift',hover),('lift_clearance',transport),('transfer',target_hover),('precontact',pre)):
                path=self.planner.path(f'layer_{k}_{name}',p,end,q);self.planner.preflight_path(path)
                paths[name]=path;p=end;q=path['q1']
            self.paths[k]=dict(paths=paths,target=target,hover=target_hover,inspection=inspection,source=source)
            previous=inspection;seed=self.planner.solve(previous,q)
        self.snapshot=state
        save(self.out/'hardware_preflight.json',dict(completed=True,initial_measured_state=state,
            paths={str(k):dict(target=v['target'].tolist(),source=v['source'],hover=v['hover'],
                certificates={name:path['certificate'] for name,path in v['paths'].items()}) for k,v in self.paths.items()},
            source_pose_authority='freshly taught task-specific whole-layer TCPs; simulation layer-bottom poses are not executed'))
    def initialize(self):
        from runtime import Owner,Camera
        manifest=self.planner.manifest();manifest['commissioning_authorized']=True;manifest['layer_contact']=self.h
        self.owner=Owner(manifest,self.out,controller=HERE/'controller')
        self.owner.hand(2)
        # Same SDK owner performs free motion and seating; no telemetry reader.
        camera_cfg=json.loads((ROOT/'teaching/factory_vision/config.json').read_text())['camera']
        self.camera=Camera(camera_cfg);self.initialized=True
    def source_present(self,layer):
        packet=json.loads((ROOT/'teaching/external_layout/output/latest.json').read_text())
        age=(time.time_ns()-packet['frame_unix_ns'])/1e9
        slots=[s for s in packet['slots'] if s['footprint_id']==layer['source']]
        if packet.get('stale') or packet.get('registration',{}).get('valid') is not True or not 0<=age<=.5 or len(slots)!=12 or any(s['status']!='occupied' or not s.get('visible') or s.get('confidence',0)<.5 or s.get('observed_unix_ns')!=packet['frame_unix_ns'] for s in slots):
            raise RecoverableStop('Fresh complete source layer not confirmed; no grasp')
        self.frozen_colors={s['slot_id']:s['color_id'] for s in slots}
        save(self.out/f"source_colors_{layer['index']}.json",dict(source=layer['source'],frame_unix_ns=packet['frame_unix_ns'],colors=self.frozen_colors,
            frozen_for_transport=True,provenance='external camera pixels; not updated while held'))
    def verify_departed_source(self,layer):
        not_before=time.time_ns();until=self.now()+self.config['vision']['source_clearance_timeout_s']
        while self.now()<until:
            self.owner.state()
            packet=json.loads((ROOT/'teaching/external_layout/output/latest.json').read_text())
            stamp=packet['frame_unix_ns'];slots=[s for s in packet['slots'] if s['footprint_id']==layer['source']]
            fresh=(not packet.get('stale') and packet.get('registration',{}).get('valid') is True and
                not_before<=stamp<=time.time_ns() and time.time_ns()-stamp<=500_000_000)
            if fresh and len(slots)==12:
                if any(s['status']=='occupied' and s.get('visible') and s.get('confidence',0)>=.5 and s.get('observed_unix_ns')==stamp for s in slots):
                    raise RecoverableStop('Partial-layer pick: bricks still observed at source')
                if all(s['status']=='empty' and s.get('visible') and s.get('confidence',0)>=.7 and s.get('observed_unix_ns')==stamp for s in slots):
                    save(self.out/f"source_departure_{layer['index']}.json",dict(frame_unix_ns=stamp,source=layer['source'],slots=slots));return
            time.sleep(.03)
        raise RecoverableStop('Source departure remains visually uncertain; no placement')
    def motion(self,name,layer):
        if not self.initialized:self.initialize()
        current=self.paths[layer['index']];self.target=current['target'];self.current_layer=layer
        if name=='approach_source':self.owner.begin()
        if name=='transfer':
            for key in ('lift_clearance','transfer'):self.go(current['paths'][key])
        elif name=='alignment_approach':self.go(current['paths']['precontact'])
        elif name=='approach_source':
            for key in ('departure_clearance','source_overhead','approach_source'):self.go(current['paths'][key])
        elif name=='retreat':
            self.owner.command('contact_guard',enabled=False)
            state=self.owner.state()['state'];start=self.planner.model.fk(state['q_d']).homogeneous.reshape(-1,order='F').tolist()
            end=start.copy();end[14]=current['inspection'][14]
            path=self.planner.path('layer_release_retreat',start,end,state['q_d']);self.planner.preflight_path(path);self.go(path)
        elif name=='home':
            state=self.owner.state()['state'];start=self.planner.model.fk(state['q_d']).homogeneous.reshape(-1,order='F').tolist()
            path=self.planner.path('layer_return_nominal_home',start,current['inspection'],state['q_d']);self.planner.preflight_path(path);self.go(path)
            self.owner.finish()
        else:self.go(current['paths'][name])
    def go(self,path):
        sample=self.owner.state();sent=sample['sent']
        if max(abs(a-b) for a,b in zip(sent,path['q0']))>.00025:
            start=self.planner.model.fk(sent).homogeneous.reshape(-1,order='F').tolist()
            if np.linalg.norm(np.array(start[12:15])-path['start'][12:15])>.0015:raise RecoverableStop('Cached transport start mismatch')
            path=self.planner.patch(path,start,path['target'],sent)
        ident=self.owner.load(path);self.owner.go(ident,path['duration_s'])
    def hand(self,opening):
        self.owner.hand(2 if opening else 1)
        if not opening:self.width_before=self.owner.state()['hand_width']
    def lift_verify(self,layer):
        before=self.owner.state();m=self.config['motion']
        if not self.h['minimum_loaded_width_m']<=before['hand_width']<=self.h['maximum_loaded_width_m']:raise RecoverableStop('No full-layer contact-width evidence')
        z=before['state']['O_T_EE'][14];self.motion('lift',layer);after=self.owner.state()
        if after['state']['O_T_EE'][14]-z<.045 or abs(after['hand_width']-before['hand_width'])>self.h['maximum_width_change_m']:
            raise RecoverableStop('Layer lift/retention uncertain')
    def state(self):
        if self.owner.error:raise RecoverableStop('Native controller stopped: '+str(self.owner.error))
        sample=self.owner.state();s=sample['state'];pose=np.array(s['O_T_EE']).reshape(4,4,order='F')
        local=np.r_[pose[:3,3]-self.target[:3,3],np.arctan2((pose[:3,:3]@self.target[:3,:3].T)[1,0],(pose[:3,:3]@self.target[:3,:3].T)[0,0])]
        wrench=np.array(s['O_F_ext_hat_K'])-self.bias;wrench[2]*=self.h['wrench_sign_for_downward_contact']
        width=sample['hand_width'];m=self.config['motion']
        grasp_valid=(abs(width-m['open_width_m'])<.001 or (self.h['minimum_loaded_width_m']<=width<=self.h['maximum_loaded_width_m'] and abs(width-self.width_before)<=self.h['maximum_width_change_m']))
        return dict(stamp_s=sample['received_monotonic'],valid=self.now()-sample['hand_stamp']<.25,
            fault=bool(s['current_errors']),grasp_valid=grasp_valid,local_pose=local.tolist(),wrench=wrench.tolist(),width_m=width)
    def frame(self,layer,stage):
        packet=self.camera.frame(after=self.now(),max_age=self.config['vision']['maximum_frame_age_s'])
        return dict(stamp_s=packet['captured_monotonic'],sequence=packet['sequence'],image=packet['image']),self.references[reference_key(layer,stage)]
    def move_local(self,pose):
        target=self.target.copy();target[:3,3]+=np.array(pose[:3]);target[:3,:3]=rz(pose[3])@target[:3,:3]
        sample=self.owner.state();q=self.planner.solve(target.reshape(-1,order='F').tolist(),sample['state']['q_d'])
        seed=np.array(sample['state']['q_d']);difference=np.abs(q-seed)
        if np.max(difference)>.05:raise RecoverableStop('Local IK branch jump')
        # Reuse the factory planner's small-segment Lipschitz argument rather
        # than recertifying an entire transfer for every micrometre correction.
        if self.planner.ignore_last_joint:difference[-1]=0
        budget=self.planner.lipschitz*float(difference.sum());count=max(8,int(np.ceil(budget/.02)))
        if count>512:raise RecoverableStop('Local correction certification budget exceeded')
        margin=budget/(2*count);low_full=low_fixed=float('inf');high=0.
        for u in np.linspace(0,1,count+1):
            m=self.planner.model.metrics((1-u)*seed+u*q)
            low_full=min(low_full,m['full_sigma_min']);low_fixed=min(low_fixed,m['fixed_elbow_sigma_min'])
            high=max(high,m['full_condition']*m['full_sigma_min'],m['fixed_elbow_condition']*m['fixed_elbow_sigma_min'])
        policy=self.planner.model.policy
        if (low_full-margin<policy['min_full_sigma'] or low_fixed-margin<policy['min_fixed_elbow_sigma'] or
                (high+margin)/max(min(low_full,low_fixed)-margin,1e-12)>policy['max_condition']):
            raise RecoverableStop('Local correction fails continuous singularity bounds')
        self.owner.servo(q,target.reshape(-1,order='F').tolist())
    def arm_contact_guard(self):
        samples=[];previous=0
        for _ in range(20):
            self.owner.wait(lambda:self.owner.latest and self.owner.latest['unix_ns']>previous,.2)
            value=self.owner.state();previous=value['unix_ns'];s=value['state']
            if max(map(abs,s['dq']))>.01:raise RecoverableStop('Wrench bias requires stationary held layer')
            samples.append(s['O_F_ext_hat_K'])
        if np.max(np.std(samples,axis=0)[:3])>self.h['bias_maximum_std_N']:raise RecoverableStop('Wrench bias is not stable')
        self.bias=np.mean(samples,axis=0)
        expected=np.array(self.h['free_space_wrench_bias_by_parity'][str(self.current_layer['index']%2)])
        if (np.linalg.norm((self.bias-expected)[:3])>self.h['bias_maximum_mean_deviation_N'] or
                np.linalg.norm((self.bias-expected)[3:])>self.h['bias_maximum_mean_deviation_Nm']):
            raise RecoverableStop('Possible pre-existing contact or changed payload; measured bias differs from qualified free-space baseline')
        self.owner.command('contact_guard',enabled=True,bias=self.bias.tolist())
        save(self.out/f"wrench_bias_{self.current_layer['index']}.json",dict(base_frame_bias=self.bias.tolist(),sample_count=len(samples),
            standard_deviation=np.std(samples,axis=0).tolist(),commissioning_id=self.h['commissioning_id']))
    def unload(self):
        start=self.state()['local_pose'][2];c=self.config['contact'];until=self.now()+2.
        while self.now()<until:
            state=self.state();force=state['wrench'][2]
            if force < -self.h['contact_force_N']:raise RecoverableStop('Tensile load while unloading newly seated layer')
            if force<=1.2*self.h['contact_force_N']:return
            pose=np.array(state['local_pose']);step=min(self.h['admittance_m_s_N']*(force-self.h['contact_force_N'])*c['tick_s'],self.h['maximum_vertical_speed_m_s']*c['tick_s'])
            pose[2]+=step
            if pose[2]-start>c['unload_lift_m']:raise RecoverableStop('Unable to unload within bounded displacement')
            self.move_local(pose)
        raise RecoverableStop('Bounded force unloading timed out')
    def verify(self,layer,delta,proof):
        v=self.config['vision']
        if np.linalg.norm(delta[:2])>v['position_tolerance_m'] or abs(delta[2])>v['yaw_tolerance_rad']:
            raise Uncertain('Released layer edges did not match qualified seated reference')
        s=self.owner.state()
        if s['state']['current_errors'] or abs(s['hand_width']-.07)>.001:raise RecoverableStop('Release/open-hand verification failed')
    def stop(self):
        if self.owner:self.owner.stop_and_wait()
    def close(self):
        if self.owner:self.owner.close()
        if self.camera:self.camera.close()
