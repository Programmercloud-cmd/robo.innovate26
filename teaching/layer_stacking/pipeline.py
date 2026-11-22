"""Resident whole-layer pick/alignment/admittance/release supervisor."""
import json
import math
from pathlib import Path
import time

import cv2
import numpy as np

from .vision import Uncertain,correction,overlay


class RecoverableStop(RuntimeError):pass


def save(path,value):
    path=Path(path);tmp=path.with_suffix('.tmp')
    tmp.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n');tmp.replace(path)


class LayerStacker:
    def __init__(self,config,plan,backend,out):
        self.config,self.plan,self.backend,self.out=config,plan,backend,Path(out)
        self.out.mkdir(parents=True,exist_ok=False);self.events=[];self.timings=[]
        self.shared_inventory=Path(__file__).with_name('inventory.json') if backend.domain=='hardware' else None
        self.shared_writes_enabled=False
        if self.shared_inventory and self.shared_inventory.exists():
            previous=json.loads(self.shared_inventory.read_text())
            if (previous.get('pending') or previous.get('completed')) and not getattr(backend,'reset_confirmed',False):
                raise RecoverableStop('Previous physical layer job unresolved or target occupied; inspect and use --reset-confirmed only after restoring sources and clearing target')
            save(self.out/'inventory_before_reset_confirmation.json',previous)
        self.inventory=dict(domain=backend.domain,available=list(config['job']['sources']),held=None,
            pending=None,completed=[],physical_inventory_authority=backend.domain=='hardware')
        self.started=backend.now();self.phase_name='idle'
        save(self.out/'plan.json',plan);save(self.out/'configuration.json',config);self.persist()

    def persist(self):
        save(self.out/'inventory.json',self.inventory)
        if self.shared_inventory and self.shared_writes_enabled:save(self.shared_inventory,self.inventory)

    def event(self,name,**data):
        packet=dict(t=self.backend.now()-self.started,phase=self.phase_name,event=name,**data)
        self.events.append(packet)
        with (self.out/'events.ndjson').open('a') as stream:stream.write(json.dumps(packet,allow_nan=False)+'\n')

    def phase(self,name,operation):
        self.phase_name=name;start=self.backend.now();self.event('begin')
        result=operation();elapsed=self.backend.now()-start
        self.timings.append(dict(phase=name,duration_s=elapsed));self.event('complete',duration_s=elapsed)
        save(self.out/'timings.json',self.timings);return result

    def state(self):
        s=self.backend.state();now=self.backend.now();v=self.config['vision']
        if not 0<=now-s['stamp_s']<=v['maximum_state_age_s'] or s['fault'] or not s['valid']:
            raise RecoverableStop('Stale/invalid/faulted measured state')
        if self.inventory['held'] and not s['grasp_valid']:raise RecoverableStop('Layer grasp/retention uncertain')
        if not np.isfinite(s['local_pose']).all():raise RecoverableStop('Invalid measured local pose')
        return s

    def observe(self,layer,stage):
        errors=[];v=self.config['vision']
        for attempt in range(v['reobservation_attempts']):
            try:
                self.state();packet,reference=self.backend.frame(layer,stage)
                if not 0<=self.backend.now()-packet['stamp_s']<=v['maximum_frame_age_s']:raise Uncertain('Stale/future camera frame')
                if packet['sequence']<=getattr(self,'last_frame',-1):raise Uncertain('Camera frame repeated')
                self.last_frame=packet['sequence']
                delta,evidence=correction(packet['image'],reference,v)
                label=f"{layer['index']}_{stage}_{packet['sequence']}"
                cv2.imwrite(str(self.out/(label+'.jpg')),overlay(packet['image'],evidence))
                self.event('vision',stage=stage,frame_sequence=packet['sequence'],captured_at_s=packet['stamp_s'],
                    frame_age_s=self.backend.now()-packet['stamp_s'],delta_xy_yaw=delta.tolist(),evidence=evidence)
                return delta
            except Uncertain as error:
                errors.append(str(error));self.event('reobserve',attempt=attempt,reason=str(error))
        raise RecoverableStop('Bounded edge re-observation exhausted: '+str(errors))

    def align(self,layer):
        v=self.config['vision'];stable=0
        for _ in range(v['maximum_iterations']):
            delta=self.observe(layer,'align')
            if np.linalg.norm(delta[:2])<=v['position_tolerance_m'] and abs(delta[2])<=v['yaw_tolerance_rad']:
                stable+=1
                if stable>=v['stable_frames']:return
                continue
            stable=0;distance=np.linalg.norm(delta[:2]);delta[:2]*=min(1.,v['alignment_step_translation_m']/max(distance,1e-12))
            delta[2]=np.clip(delta[2],-v['alignment_step_yaw_rad'],v['alignment_step_yaw_rad'])
            pose=np.asarray(self.state()['local_pose']);pose[[0,1,3]]+=delta
            if np.linalg.norm(pose[:2])>v['maximum_translation_correction_m'] or abs(pose[3])>v['maximum_yaw_correction_rad']:
                raise RecoverableStop('Visual correction exceeded the taught-pose envelope')
            self.backend.move_local(pose);self.state()
        raise RecoverableStop('Visual alignment iteration bound reached')

    def seat(self,layer):
        c=self.config['contact'];p=self.backend.contact_parameters();start=self.backend.now()
        self.backend.arm_contact_guard();stable=0;search=0;stall=0;last_z=math.inf;search_origin=None;searching=False
        try:
            while self.backend.now()-start<c['timeout_s']:
                s=self.state();pose=np.asarray(s['local_pose']);wrench=np.asarray(s['wrench'])
                if not np.isfinite(wrench).all():raise RecoverableStop('Nonfinite contact wrench')
                axial=float(wrench[2]);lateral=np.linalg.norm(wrench[:2]);moment=np.linalg.norm(wrench[3:])
                if np.linalg.norm(wrench[:3])>p['maximum_force_N'] or lateral>p['maximum_lateral_force_N'] or moment>p['maximum_torque_Nm']:
                    raise RecoverableStop('Measured seating force/torque limit')
                if pose[2]<-c['maximum_descent_m']:raise RecoverableStop('Seating exceeded bounded depth')
                depth=p['contact_reference_z_m']-pose[2]
                seated=(depth>=p['minimum_insertion_m'] and abs(pose[2])<=p['seated_depth_tolerance_m'] and
                    .9*p['seating_force_N']<=axial<=1.1*p['seating_force_N'])
                stable=stable+1 if seated else 0
                self.event('contact_sample',state_stamp_s=s['stamp_s'],local_pose=pose.tolist(),wrench=wrench.tolist(),stable_samples=stable)
                if stable>=c['stable_samples']:return dict(depth_m=depth,force_N=axial,compound_seating_criterion=True)
                stall=stall+1 if axial>=p['seating_force_N']*.85 and pose[2]>p['seated_depth_tolerance_m'] and abs(pose[2]-last_z)<.00002 else 0
                last_z=pose[2]
                if stall>=3:
                    searching=True
                    if search_origin is None:search_origin=pose[:2].copy()
                if searching:
                    desired=p['contact_force_N']
                    # First unload to light contact; lateral search never runs
                    # under the larger seating load.
                    if axial<=desired*1.5:
                        if search>=c['maximum_search_steps']:raise RecoverableStop('Bounded light-contact search exhausted')
                        search+=1;fraction=search/c['maximum_search_steps'];theta=2*math.pi*c['search_turns']*fraction
                        pose[:2]=search_origin+c['maximum_search_radius_m']*fraction*np.array([math.cos(theta),math.sin(theta)])
                        self.event('bounded_contact_search',step=search,radius_m=float(np.linalg.norm(pose[:2]-search_origin)))
                        searching=False;stall=0
                else:desired=p['seating_force_N']
                velocity=np.clip(p['admittance_m_s_N']*(axial-desired),-p['maximum_vertical_speed_m_s'],p['maximum_vertical_speed_m_s'])
                pose[2]+=float(velocity*c['tick_s'])
                if np.linalg.norm(pose[:2])>self.config['vision']['maximum_translation_correction_m']+c['maximum_search_radius_m']:
                    raise RecoverableStop('Contact correction left the total lateral envelope')
                self.backend.move_local(pose);self.backend.wait(c['tick_s'])
            raise RecoverableStop('Seating timed out without verified depth/load stability')
        finally:
            # Keep the native overload guard armed on failure; the owning
            # backend performs its bounded SDK stop before returning control.
            pass

    def verify_released(self,layer,proof):
        stable=0;errors=[];v=self.config['vision']
        for _ in range(v['stable_frames']+v['reobservation_attempts']-1):
            delta=self.observe(layer,'verify')
            try:self.backend.verify(layer,delta,proof)
            except Uncertain as error:
                stable=0;errors.append(str(error));self.event('placement_reobserve',reason=str(error));continue
            stable+=1
            if stable>=v['stable_frames']:return
        raise RecoverableStop('Released layer verification remains uncertain: '+str(errors))

    def run(self):
        result=None
        try:
            self.backend.preflight(self.plan);self.event('preflight_complete')
            for layer in self.plan['layers']:
                source=layer['source'];self.inventory['pending']=dict(source=source,layer=layer['index'],phase='before_pick');self.persist()
                self.phase('source_presence',lambda:self.backend.source_present(layer))
                self.phase('approach_source',lambda:self.backend.motion('approach_source',layer))
                # Hardware startup has now verified fresh Idle/open hand under
                # the exclusive owner. Only now may an acknowledged old job
                # be archived/replaced in the persistent physical ledger.
                self.shared_writes_enabled=True;self.persist()
                self.phase('bounded_grasp_descent',lambda:self.backend.motion('grasp',layer))
                self.inventory['pending']['phase']='closing_unverified';self.persist()
                self.phase('close',lambda:self.backend.hand(False))
                self.phase('lift_and_verify',lambda:self.backend.lift_verify(layer))
                self.inventory['held']=source;self.inventory['pending']['phase']='held';self.persist()
                self.phase('transfer_above_target',lambda:self.backend.motion('transfer',layer))
                self.phase('source_departure_verification',lambda:self.backend.verify_departed_source(layer))
                self.phase('alignment_approach',lambda:self.backend.motion('alignment_approach',layer))
                self.phase('edge_alignment',lambda:self.align(layer))
                proof=self.phase('compliant_seating',lambda:self.seat(layer))
                self.phase('unload',self.backend.unload)
                self.inventory['pending']['phase']='release_started_unverified';self.persist()
                self.phase('release',lambda:self.backend.hand(True))
                self.inventory['held']=None;self.inventory['pending']['phase']='released_placement_unverified';self.persist()
                self.phase('vertical_retreat',lambda:self.backend.motion('retreat',layer))
                self.phase('placement_verification',lambda:self.verify_released(layer,proof))
                self.phase('return_nominal_home',lambda:self.backend.motion('home',layer))
                self.inventory['available'].remove(source);self.inventory['held']=None
                self.inventory['completed'].append(dict(source=source,layer=layer['index'],domain=self.backend.domain,
                    qualified_seating_criteria_verified=self.backend.domain=='hardware',
                    independent_physical_interlock_check=False,proof=proof))
                self.inventory['pending']=None;self.persist();self.event('inventory_commit',source=source)
            result=dict(completed=True,domain=self.backend.domain,layers_completed=len(self.inventory['completed']),
                physical_interlocks_verified=0,
                qualified_physical_seatings=len(self.inventory['completed']) if self.backend.domain=='hardware' else 0,
                duration_s=self.backend.now()-self.started,stops=0,
                meaning='Synthetic software-loop exercise, not physical LEGO engagement' if self.backend.domain=='fixture' else 'Commissioned compound seating/release criteria passed')
        except Exception as error:
            self.event('recoverable_stop',reason=str(error))
            try:self.backend.stop()
            except Exception as stop_error:self.event('stop_confirmation_failed',reason=str(stop_error))
            if self.inventory['pending']:self.inventory['pending']['last_stop']=str(error)
            self.persist();result=dict(completed=False,domain=self.backend.domain,error=str(error),stops=1,
                layers_completed=len(self.inventory['completed']),physical_interlocks_verified=0,
                inventory_not_advanced_for_uncertain_layer=True)
        finally:
            try:self.backend.close()
            except Exception as error:
                result=dict(result or {},completed=False,shutdown_error=str(error),stops=1)
            save(self.out/'outcome.json',result)
        return result
