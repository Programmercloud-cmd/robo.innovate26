"""Persistent presentation state; raw camera observations remain separate.

The only execution input is the existing controller's inventory file, read-only.
Old completed jobs are baselined, never replayed into today's restored layout.
No SDK, motion command, authoritative inventory or coordinate writer exists here.
"""
import copy
import json
import time
from datetime import datetime
from pathlib import Path


def multiply(a,b):
    """Column-major homogeneous matrices, matching existing Franka poses."""
    return [sum(a[k*4+i]*b[j*4+k] for k in range(4)) for j in range(4) for i in range(4)]


def inverse_rigid(a):
    out=[0.]*16;out[15]=1.
    for i in range(3):
        for j in range(3):out[j*4+i]=a[i*4+j]
        out[12+i]=-sum(a[i*4+k]*a[12+k] for k in range(3))
    return out


def translation(xyz):
    return [1.,0.,0.,0.,0.,1.,0.,0.,0.,0.,1.,0.,*xyz,1.]


class DisplayState:
    FROZEN_MODES=('pending_grasp','held','held_unverified','releasing')

    def __init__(self,path,layout,positions,job,policy=None):
        self.path=Path(path);self.layout=layout;self.positions=positions;self.job=job
        self.policy=policy or {};self.manual_updates=self.policy.get('manual_camera_updates') is True
        self.conveyor_geometry={s['slot_id']:s for s in layout.get('conveyor_slots',[])}
        self.geometry={s['slot_id']:s for s in layout['slots']+layout.get('conveyor_slots',[])}
        self.groups={}
        for s in layout['slots']:
            if s.get('motion_pose_id'):self.groups.setdefault(s['motion_pose_id'],[]).append(s['slot_id'])
        self.data=dict(schema='fr3.display_memory.v1',states={},last_occupied={},columns={},
                       previous_completed=None,last_event=None,revision=0)
        if self.path.exists():
            saved=json.loads(self.path.read_text())
            if saved.get('schema')=='fr3.display_memory.v1':self.data=saved
        self.last_inventory_mtime=None;self.last_frame=None;self.last_stale=None;self.last_save=0.
        self.color_candidates={}
        self.memory_changed=False

    def observe_inventory(self,inventory):
        completed={k:v.get('run') for k,v in inventory.get('completed',{}).items()}
        if self.data['previous_completed'] is None:
            self.data['previous_completed']=completed
            self.data['reset_marker']=json.dumps(inventory.get('operator_source_reset'),sort_keys=True)
            # Baseline historical active state too; a fresh phase change or a
            # persisted frozen payload is needed to interpret a live transfer.
            active=inventory.get('active')
            self.data['last_event']=self.event_key(active) if active else None
            return
        active=inventory.get('active')
        if active and self.event_key(active)!=self.data['last_event']:
            self.data['last_event']=self.event_key(active)
            label,phase=active['pose'],active['phase']
            if label in self.groups:
                if phase in ('closing_unverified','closed_unverified'):
                    self.clear_conveyor_if_confirmed(active)
                    self.freeze(label,active,'pending_grasp')
                elif phase in ('lifting_pick_unverified','verification_lift_unverified','lift_complete_pick_unverified'):
                    # The controller has completed closure and started its lift;
                    # render the user's requested assumed rigid grasp, clearly
                    # distinct from the later retention-verified event.
                    self.freeze(label,active,'held_unverified')
                elif phase in ('retention_verified','held_at_home_after_placement_stop'):
                    self.freeze(label,active,'held')
                elif phase=='released_placement_unverified':
                    self.freeze(label,active,'conveyor_unverified')
                elif phase=='release_started_placement_unverified':
                    self.freeze(label,active,'releasing')
        for label,run in completed.items():
            if run!=self.data['previous_completed'].get(label) and label in self.groups:
                verified=inventory['completed'][label].get('placement_visually_verified',True)
                self.freeze(label,dict(run=run,pose=label,phase='completed'),
                            'conveyor_verified' if verified else 'conveyor_unverified')
        # Only an explicit controller source reset/new available entry reopens
        # a previously owned source to fresh camera observations.
        reset=json.dumps(inventory.get('operator_source_reset'),sort_keys=True)
        if reset!=self.data.get('reset_marker') and inventory.get('operator_source_reset'):
            for label in inventory.get('available',{}):
                if (active or {}).get('pose')!=label:self.data['columns'].pop(label,None)
        self.data['reset_marker']=reset
        self.data['previous_completed']=completed

    @staticmethod
    def event_key(active):
        if not active:return None
        return '|'.join(str(active.get(k,'')) for k in ('run','pose','phase','utc'))

    def source_exclusions(self):
        """Read-only controller ownership mask for camera registration/sampling."""
        labels={label for label,column in self.data['columns'].items()
                if (column['mode'] in self.FROZEN_MODES if self.manual_updates else column['mode']!='pending_grasp')}
        return dict(schema='fr3.source_exclusions.v1',observed_unix_ns=time.time_ns(),
                    stale=False,inventory_source='read_only_factory_inventory',
                    slot_ids=sorted(sid for label in labels for sid in self.groups.get(label,[])))

    def clear_conveyor_if_confirmed(self,active):
        if self.job.get('conveyor_occupancy_policy')!='cleared_between_deliveries':return
        try:
            path=Path(active['run'])/'prepick_conveyor_clear.json'
            proof=json.loads(path.read_text())
            phase_ns=int(datetime.fromisoformat(active['utc'].replace('Z','+00:00')).timestamp()*1e9)
            if proof.get('clear') is not True or not 0<=phase_ns-path.stat().st_mtime_ns<=30_000_000_000:return
        except (OSError,ValueError,KeyError):return
        for column in self.data['columns'].values():
            if column['mode'].startswith('conveyor'):
                column.update(mode='departed_conveyor',departure_basis='controller fresh pre-pick conveyor-clear evidence',departure_run=active['run'])

    def freeze(self,label,event,mode):
        run=str(event.get('origin_run',event.get('run','')))
        column=self.data['columns'].get(label)
        if column is None or column['run']!=run:
            column=dict(label=label,run=run,slots=[],mode=mode,
                        basis='controller-reported state; rigid attachment/display placement assumed')
            grasp=self.positions['points']['point_'+label]['O_T_EE']
            for sid in self.groups[label]:
                known=self.data['states'].get(sid,{})
                if known.get('status')!='occupied':known=self.data['last_occupied'].get(sid,{})
                g=self.geometry[sid]
                column['slots'].append(dict(slot_id=sid,color_id=known.get('color_id'),
                    last_observed_unix_ns=known.get('last_observed_unix_ns'),
                    confidence=known.get('confidence',0.),display_size_m=g['display_size_m'],
                    relative_to_tcp=multiply(inverse_rigid(grasp),translation(g['display_base_xyz_m']))))
            self.data['columns'][label]=column
        column['mode']=mode;column['controller_phase']=event.get('phase');column['updated_unix_ns']=time.time_ns()
        place=self.job['conveyor_target_O_T_EE'].copy()
        place[14]+=self.job.get('conveyor_surface_height_offset_m',0.)
        try:
            geometry=json.loads((Path(event.get('run',run))/'display_geometry.json').read_text())
            candidate=geometry['placement_O_T_EE']
            if geometry.get('source')==label and len(candidate)==16 and all(isinstance(v,(int,float)) for v in candidate):
                place=candidate
        except (OSError,ValueError,KeyError,TypeError):pass
        column['placement_O_T_EE']=place
        if mode.startswith('conveyor'):
            for s in column['slots']:s['placed_O_T_body']=multiply(place,s['relative_to_tcp'])

    def accept_camera(self,raw,frame_ns,force_confirmation=False):
        """Bounded camera updates; uncertain/occluded samples never erase memory."""
        sid=raw['slot_id'];previous=self.data['states'].get(sid,{})
        if raw['status'] not in ('occupied','empty') or raw.get('visible') is not True:
            self.color_candidates.pop(sid,None);return False
        conflict=(previous.get('status') in ('occupied','empty') and
                  (previous.get('status'),previous.get('color_id'))!=(raw['status'],raw.get('color_id')))
        if self.manual_updates and (conflict or force_confirmation):
            signature=(raw['status'],raw.get('color_id'));candidate=self.color_candidates.get(sid,{})
            count=candidate.get('count',0)+1 if candidate.get('signature')==signature else 1
            minimum=self.policy.get('minimum_change_confidence',.5)
            self.color_candidates[sid]=dict(signature=signature,count=count if raw['confidence']>=minimum else 0)
            needed=self.policy.get('empty_confirmation_frames',3) if raw['status']=='empty' else self.policy.get('change_confirmation_frames',5)
            if raw['confidence']<minimum or count<needed:return False
        elif not self.manual_updates and previous.get('status')=='occupied' and raw['status']=='occupied' and previous.get('color_id')!=raw['color_id']:
            candidate=self.color_candidates.get(sid,{})
            count=candidate.get('count',0)+1 if candidate.get('color_id')==raw['color_id'] else 1
            self.color_candidates[sid]=dict(color_id=raw['color_id'],count=count if raw['confidence']>=.85 else 0)
            if raw['confidence']<.85 or count<8:return False
        self.color_candidates.pop(sid,None)
        known=dict(status=raw['status'],color_id=raw.get('color_id'),confidence=raw['confidence'],
                   last_observed_unix_ns=raw.get('observed_unix_ns',frame_ns),evidence='camera')
        if (previous.get('status'),previous.get('color_id'),previous.get('evidence'))!=(known['status'],known['color_id'],known['evidence']):
            self.memory_changed=True
        self.data['states'][sid]=known
        if raw['status']=='occupied':self.data['last_occupied'][sid]=known.copy()
        return True

    def seed_conveyor(self):
        """Release assumptions persist only until independently seen at the belt."""
        if not self.manual_updates:return
        for column in self.data['columns'].values():
            if column['mode'] not in ('conveyor_verified','conveyor_unverified') or column.get('conveyor_seeded_run')==column['run']:continue
            try:row=int(column['label'].split('_')[0])
            except (ValueError,KeyError):continue
            for n,source in enumerate(column['slots']):
                sid=f'conveyor_r{row}_n{n}'
                if sid not in self.conveyor_geometry or source.get('color_id') is None:continue
                self.data['states'][sid]=dict(status='occupied',color_id=source['color_id'],
                    confidence=source.get('confidence',0.),last_observed_unix_ns=None,evidence='controller_release_assumption')
                self.memory_changed=True
                self.color_candidates.pop(sid,None)
            column['conveyor_seeded_run']=column['run']

    def conveyor_display(self,packet,stale):
        observed={s['slot_id']:s for s in packet.get('conveyor_slots',[])};result=[]
        frozen_rows={int(c['label'].split('_')[0]) for c in self.data['columns'].values() if c['mode'] in self.FROZEN_MODES}
        for sid,g in self.conveyor_geometry.items():
            raw=observed.get(sid,{});known=self.data['states'].get(sid,dict(status='uncertain',color_id=None,confidence=0.,last_observed_unix_ns=None))
            current=(not stale and raw.get('visible') is True and raw.get('status') in ('occupied','empty') and
                     g['conveyor_row'] not in frozen_rows and known.get('evidence')=='camera' and
                     (known['status'],known.get('color_id'))==(raw['status'],raw.get('color_id')))
            result.append(dict(slot_id=sid,**known,display_base_xyz_m=g['display_base_xyz_m'],display_size_m=g['display_size_m'],
                observation_status='stale' if stale else raw.get('status','uncertain'),confirmed_in_current_frame=current,
                provenance='current_camera' if current else 'controller_release_assumption' if known.get('evidence')=='controller_release_assumption' else 'last_confirmed_camera_unverified'))
        return result

    def update(self,packet,inventory=None):
        self.memory_changed=False
        if packet.get('frame_unix_ns',0)==0 and packet.get('stale',True):
            # A reconnect that fails before its first frame is not a new camera
            # epoch. Preserve the last real exposure time while still forwarding
            # controller ownership events during the outage.
            packet=copy.deepcopy(packet)
            packet['frame_unix_ns']=self.data.get('last_camera_unix_ns',0)
            packet['camera_unavailable_before_first_frame']=True
        elif packet.get('frame_unix_ns',0)>0:
            self.data['last_camera_unix_ns']=max(self.data.get('last_camera_unix_ns',0),packet['frame_unix_ns'])
        previous_event=self.data['last_event'];before=copy.deepcopy(self.data['columns'])
        if inventory is not None:self.observe_inventory(inventory)
        self.seed_conveyor()
        stale=packet.get('stale',True) or not 0<=(time.time_ns()-packet['frame_unix_ns'])/1e9<=1.
        fresh_frame=packet['frame_unix_ns']!=self.last_frame
        if fresh_frame and not stale:
            for s in packet['slots']:
                sid=s['slot_id'];label=self.geometry[sid].get('motion_pose_id')
                column=self.data['columns'].get(label)
                if column and (not self.manual_updates or column['mode'] in self.FROZEN_MODES):continue
                reopen=column is not None and sid not in column.get('camera_reopened_slots',[])
                if self.accept_camera(s,packet['frame_unix_ns'],reopen) and reopen:
                    column.setdefault('camera_reopened_slots',[]).append(sid)
            if self.manual_updates:
                frozen_rows={int(c['label'].split('_')[0]) for c in self.data['columns'].values() if c['mode'] in self.FROZEN_MODES}
                for s in packet.get('conveyor_slots',[]):
                    g=self.conveyor_geometry.get(s['slot_id'])
                    if g and g['conveyor_row'] not in frozen_rows:self.accept_camera(s,packet['frame_unix_ns'])
        output=copy.deepcopy(packet);output['stale']=bool(stale);display=[]
        for raw in packet['slots']:
            sid=raw['slot_id'];g=self.geometry[sid];column=self.data['columns'].get(g.get('motion_pose_id'))
            reopened=(self.manual_updates and column is not None and column['mode'] not in self.FROZEN_MODES and sid in column.get('camera_reopened_slots',[]))
            known=self.data['states'].get(sid,dict(status='uncertain',color_id=None,confidence=0.,last_observed_unix_ns=None))
            d=dict(slot_id=sid,**known,display_base_xyz_m=g['display_base_xyz_m'],display_size_m=g['display_size_m'])
            current=(not stale and raw['status'] in ('occupied','empty') and raw.get('visible') is True and (column is None or reopened))
            current=current and (known.get('status'),known.get('color_id'))==(raw['status'],raw.get('color_id'))
            conflict=(raw['status']=='occupied' and known.get('status')=='occupied' and raw.get('color_id')!=known.get('color_id'))
            current=current and not conflict
            d.update(observation_status='stale' if stale else raw['status'],confirmed_in_current_frame=current,
                     provenance='current_camera' if current else 'last_confirmed_camera_unverified')
            if conflict:d['provenance']='color_conflict_retaining_last_confirmed'
            if column and column['mode']!='pending_grasp' and not reopened:
                d.update(status='empty',color_id=None,confidence=0.,confirmed_in_current_frame=False,
                         provenance='source_vacated_by_controller_pick',ownership=column['mode'])
            elif column and not reopened:d['provenance']='frozen_pending_grasp_confirmation'
            display.append(d)
        changed=fresh_frame or stale!=self.last_stale or before!=self.data['columns'] or previous_event!=self.data['last_event']
        if changed:self.data['revision']+=1
        output['display_slots']=display
        output['display_columns']=[copy.deepcopy(c) for c in self.data['columns'].values() if c['mode']!='departed_conveyor']
        if self.manual_updates and 'conveyor_slots' in packet:
            output['display_conveyor_slots']=self.conveyor_display(packet,stale)
            for c in output['display_columns']:
                if c['mode'] in ('conveyor_verified','conveyor_unverified'):c['render_as_payload']=False
        output['display_departed_columns']=[dict(label=c['label'],run=c['run'],departure_basis=c.get('departure_basis'))
            for c in self.data['columns'].values() if c['mode']=='departed_conveyor']
        output['display_revision']=self.data['revision']
        output['display_semantics']='Last confirmed colors retained through uncertainty; controller-owned payloads frozen. Not execution inventory.'
        self.last_frame=packet['frame_unix_ns'];self.last_stale=stale
        if changed and (self.memory_changed or time.monotonic()-self.last_save>1 or before!=self.data['columns']):
            temp=self.path.with_suffix('.tmp');temp.write_text(json.dumps(self.data));temp.replace(self.path);self.last_save=time.monotonic()
        return output


class InventoryObserver:
    def __init__(self,path):self.path=Path(path);self.mtime=None
    def read(self):
        try:
            mtime=self.path.stat().st_mtime_ns
            if mtime==self.mtime:return None
            value=json.loads(self.path.read_text());self.mtime=mtime;return value
        except (OSError,ValueError):return None
