"""Synthetic state-transition checks; no camera/SDK commands or inventory writes."""
import copy
import json
import tempfile
import time
import unittest
from datetime import datetime,timezone
from pathlib import Path
from display_state import DisplayState,multiply,translation

ROOT=Path(__file__).resolve().parents[2]


class DisplayTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.layout=json.loads((ROOT/'digital_twin/perception/display_layout.json').read_text())
        self.positions=json.loads((ROOT/'teaching/positions.json').read_text())
        self.job=json.loads((ROOT/'teaching/factory_vision/config.json').read_text())
        self.state=DisplayState(Path(self.tmp.name)/'state.json',self.layout,self.positions,self.job)
        self.source=[s['slot_id'] for s in self.layout['slots'] if s.get('motion_pose_id')=='5_0']
        self.inventory=dict(active=None,completed={'5_0':{'run':'historical_cycle'}},available={})
        self.packet=dict(schema='fr3.external_layout.v1',frame_unix_ns=time.time_ns(),stale=False,slots=[])
        for s in self.layout['slots']:
            occupied=s['slot_id'] in self.source
            self.packet['slots'].append(dict(slot_id=s['slot_id'],status='occupied' if occupied else 'uncertain',color_id=300121 if occupied else None,
                visible=occupied,confidence=.9 if occupied else 0.,observed_unix_ns=self.packet['frame_unix_ns']))
    def tick(self,inventory=None,occlude=False,color=300121):
        p=copy.deepcopy(self.packet);p['frame_unix_ns']=time.time_ns()
        for s in p['slots']:
            s['observed_unix_ns']=p['frame_unix_ns']
            if s['slot_id'] in self.source:
                s.update(status='uncertain' if occlude else 'occupied',color_id=None if occlude else color,visible=not occlude)
        return self.state.update(p,inventory)
    def event(self,phase):
        self.inventory['active']=dict(pose='5_0',run='new_cycle',phase=phase,utc=phase)
        return self.inventory
    def test_old_completed_cycle_does_not_empty_restored_source(self):
        p=self.tick(self.inventory)
        self.assertFalse(p['display_columns']);self.assertEqual(sum(s['status']=='occupied' for s in p['display_slots']),4)
    def test_occlusion_retains_color_and_original_observation_time(self):
        first=self.tick(self.inventory);second=self.tick(occlude=True)
        a=[s for s in first['display_slots'] if s['slot_id'] in self.source]
        b=[s for s in second['display_slots'] if s['slot_id'] in self.source]
        self.assertEqual([s['last_observed_unix_ns'] for s in a],[s['last_observed_unix_ns'] for s in b])
        self.assertTrue(all(s['color_id']==300121 and not s['confirmed_in_current_frame'] for s in b))
    def test_freeze_grasp_source_empty_and_release_frozen_payload(self):
        self.tick(self.inventory);self.tick(self.event('closing_unverified'))
        p=self.tick(self.event('retention_verified'),color=6487946)
        self.assertTrue(all(s['status']=='empty' for s in p['display_slots'] if s['slot_id'] in self.source))
        self.assertTrue(all(s['color_id']==300121 for s in p['display_columns'][0]['slots']))
        p=self.tick(self.event('released_placement_unverified'),occlude=True)
        self.assertEqual(p['display_columns'][0]['mode'],'conveyor_unverified')
        self.assertTrue(all('placed_O_T_body' in s for s in p['display_columns'][0]['slots']))
        p=self.tick(color=6487946)
        self.assertTrue(all(s['status']=='empty' for s in p['display_slots'] if s['slot_id'] in self.source))
    def test_stale_camera_retains_display_but_not_freshness(self):
        self.tick(self.inventory);p=copy.deepcopy(self.packet);p['stale']=True;p['frame_unix_ns']=time.time_ns()-3_000_000_000
        out=self.state.update(p)
        self.assertTrue(out['stale']);self.assertTrue(all(not s['confirmed_in_current_frame'] for s in out['display_slots']))
        self.assertEqual(sum(s['status']=='occupied' for s in out['display_slots']),4)
    def test_lift_start_attaches_assumed_column_before_retention_verification(self):
        self.tick(self.inventory);p=self.tick(self.event('lifting_pick_unverified'),occlude=True)
        self.assertEqual(p['display_columns'][0]['mode'],'held_unverified')
        self.assertTrue(all(s['status']=='empty' for s in p['display_slots'] if s['slot_id'] in self.source))
    def test_registration_excludes_departed_sources_until_explicit_reset(self):
        self.tick(self.inventory);self.tick(self.event('closing_unverified'))
        self.assertEqual(self.state.source_exclusions()['slot_ids'],[])
        self.tick(self.event('retention_verified'),occlude=True)
        self.assertEqual(self.state.source_exclusions()['slot_ids'],sorted(self.source))
        self.inventory.update(active=None,operator_source_reset={'confirmed':True},available={'5_0':{}})
        self.tick(self.inventory)
        self.assertEqual(self.state.source_exclusions()['slot_ids'],[])
    def test_weak_color_disagreement_does_not_change_confirmed_color(self):
        self.tick(self.inventory)
        for _ in range(10):out=self.tick(color=6135191)
        # .9 is deliberately strong enough only after eight agreeing frames.
        self.assertTrue(all(s['color_id']==6135191 for s in out['display_slots'] if s['slot_id'] in self.source))
        for s in self.packet['slots']:
            if s['slot_id'] in self.source:s['confidence']=.6
        for _ in range(12):out=self.tick(color=6487946)
        self.assertTrue(all(s['color_id']==6135191 for s in out['display_slots'] if s['slot_id'] in self.source))
    def test_attachment_at_taught_grasp_reproduces_source_geometry(self):
        self.tick(self.inventory);p=self.tick(self.event('retention_verified'),occlude=True)
        frozen=p['display_columns'][0]['slots'][0];pose=multiply(self.positions['points']['point_5_0']['O_T_EE'],frozen['relative_to_tcp'])
        expected=next(s['display_base_xyz_m'] for s in self.layout['slots'] if s['slot_id']==frozen['slot_id'])
        for actual,target in zip(pose[12:15],expected):self.assertAlmostEqual(actual,target,places=8)
    def test_persisted_payload_stays_frozen_after_display_restart(self):
        self.tick(self.inventory);self.tick(self.event('retention_verified'),occlude=True)
        again=DisplayState(self.state.path,self.layout,self.positions,self.job)
        out=again.update(self.packet,self.inventory)
        self.assertEqual(out['display_columns'][0]['mode'],'held')
        self.assertTrue(all(s['status']=='empty' for s in out['display_slots'] if s['slot_id'] in self.source))
    def test_failed_reconnect_preserves_real_timestamp_and_forwards_pick(self):
        before=self.tick(self.inventory);p=copy.deepcopy(self.packet);p.update(frame_unix_ns=0,stale=True)
        out=self.state.update(p,self.event('retention_verified'))
        self.assertEqual(out['frame_unix_ns'],before['frame_unix_ns'])
        self.assertTrue(out['stale']);self.assertEqual(out['display_columns'][0]['mode'],'held')
    def test_controller_clear_retires_payload_without_reopening_source(self):
        self.tick(self.inventory);self.tick(self.event('released_placement_unverified'),occlude=True)
        run=Path(self.tmp.name)/'next_run';run.mkdir();(run/'prepick_conveyor_clear.json').write_text(json.dumps({'clear':True}))
        self.inventory['active']=dict(pose='5_1',run=str(run),phase='closing_unverified',utc=datetime.now(timezone.utc).isoformat())
        out=self.tick(self.inventory,occlude=True)
        self.assertEqual(out['display_departed_columns'][0]['label'],'5_0')
        self.assertEqual(self.state.data['columns']['5_0']['mode'],'departed_conveyor')
        self.assertTrue(all(s['status']=='empty' for s in out['display_slots'] if s['slot_id'] in self.source))


class ManualDisplayTests(unittest.TestCase):
    def setUp(self):
        DisplayTests.setUp(self)
        self.policy=dict(manual_camera_updates=True,minimum_change_confidence=.5,
                         change_confirmation_frames=5,empty_confirmation_frames=3)
        self.state=DisplayState(self.state.path,self.layout,self.positions,self.job,policy=self.policy)
        self.packet['conveyor_slots']=[dict(slot_id=f'conveyor_r{r}_n{n}',status='uncertain',
            color_id=None,confidence=0.,visible=False) for r in range(6) for n in range(4)]
        self.inventory['completed']={}
        self.tick(self.inventory)

    tick=DisplayTests.tick
    event=DisplayTests.event

    def conveyor(self,status,color=None):
        for s in self.packet['conveyor_slots']:
            if s['slot_id'].startswith('conveyor_r5_'):
                s.update(status=status,color_id=color,confidence=.9 if status!='uncertain' else 0.,visible=status!='uncertain')

    def test_released_source_and_conveyor_accept_manual_color_exchange(self):
        self.tick(self.event('retention_verified'));self.tick(self.event('released_placement_unverified'))
        self.inventory['active']=None
        self.assertEqual(self.state.source_exclusions()['slot_ids'],[])
        self.conveyor('occupied',6135191)
        for _ in range(5):out=self.tick(self.inventory,color=6487946)
        self.assertTrue(all(s['color_id']==6487946 for s in out['display_slots'] if s['slot_id'] in self.source))
        self.assertTrue(all(s['color_id']==6135191 for s in out['display_conveyor_slots'] if s['slot_id'].startswith('conveyor_r5_')))
        self.assertFalse(out['display_columns'][0]['render_as_payload'])

    def test_active_grasp_freezes_source_colors_and_belt_updates(self):
        self.conveyor('occupied',6135191)
        for _ in range(5):self.tick()
        self.tick(self.event('retention_verified'))
        self.conveyor('occupied',4211201)
        for _ in range(10):out=self.tick(color=6487946)
        self.assertEqual(self.state.source_exclusions()['slot_ids'],sorted(self.source))
        self.assertTrue(all(s['color_id']==300121 for s in out['display_columns'][0]['slots']))
        self.assertTrue(all(s['color_id']==6135191 for s in out['display_conveyor_slots'] if s['slot_id'].startswith('conveyor_r5_')))

    def test_uncertainty_does_not_remove_belt_bricks_but_positive_floor_does(self):
        self.tick(self.event('retention_verified'));self.tick(self.event('released_placement_unverified'))
        self.inventory['active']=None
        for _ in range(8):out=self.tick(self.inventory,occlude=True)
        self.assertTrue(all(s['status']=='occupied' for s in out['display_conveyor_slots'] if s['slot_id'].startswith('conveyor_r5_')))
        self.conveyor('empty')
        for _ in range(3):out=self.tick(occlude=True)
        self.assertTrue(all(s['status']=='empty' for s in out['display_conveyor_slots'] if s['slot_id'].startswith('conveyor_r5_')))
        self.conveyor('uncertain')
        for _ in range(8):out=self.tick(occlude=True)
        self.assertTrue(all(s['status']=='empty' for s in out['display_conveyor_slots'] if s['slot_id'].startswith('conveyor_r5_')))

    def test_stale_floor_evidence_cannot_erase_a_conveyor_payload(self):
        self.tick(self.event('retention_verified'));self.tick(self.event('released_placement_unverified'))
        self.conveyor('empty');p=copy.deepcopy(self.packet);p.update(stale=True,frame_unix_ns=time.time_ns()-3_000_000_000)
        for _ in range(8):out=self.state.update(p)
        self.assertTrue(all(s['status']=='occupied' for s in out['display_conveyor_slots'] if s['slot_id'].startswith('conveyor_r5_')))

    def test_manual_conveyor_removal_is_persisted_before_a_restart(self):
        self.tick(self.event('retention_verified'));self.tick(self.event('released_placement_unverified'))
        self.conveyor('empty')
        for _ in range(3):self.tick()
        self.state=DisplayState(self.state.path,self.layout,self.positions,self.job,policy=self.policy)
        self.conveyor('uncertain');out=self.tick(occlude=True)
        self.assertTrue(all(s['status']=='empty' for s in out['display_conveyor_slots'] if s['slot_id'].startswith('conveyor_r5_')))

    def test_reopened_source_survives_display_restart_without_inventory_reset(self):
        self.tick(self.event('retention_verified'));self.tick(self.event('released_placement_unverified'))
        for _ in range(5):self.tick(color=6487946)
        self.state=DisplayState(self.state.path,self.layout,self.positions,self.job,policy=self.policy)
        out=self.tick(occlude=True)
        self.assertTrue(all(s['status']=='occupied' and s['color_id']==6487946 for s in out['display_slots'] if s['slot_id'] in self.source))


if __name__=='__main__':unittest.main()
