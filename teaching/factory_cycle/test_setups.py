"""Focused setup and execution-mode regressions; never connects to hardware."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
import subprocess
from unittest.mock import patch

import numpy as np

from commissioning import stationary_profile_evidence
from planner import Planner,placement_profile
from run_cycle import Factory,HERE,TEACHING
from setups import resolve_setup


class SetupTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.settings=json.loads((HERE/'settings.json').read_text())
        cls.factory=json.loads((TEACHING/'factory_vision/config.json').read_text())
        cls.motion=json.loads((TEACHING/'motion_config.json').read_text())
        catalog=json.loads((TEACHING/'positions.json').read_text())
        p=catalog['points']['point_5_0']
        cls.state=dict(F_T_EE=p['tool_frames']['F_T_EE'],q_d=p['q_rad'])
        cls.planners={name:Planner(cls.settings,cls.state,setup=name) for name in ('tabletop','conveyor','conveyor_moving')}

    def test_exact_offsets_and_unchanged_sources_for_all_columns(self):
        table,conveyor=(self.planners[n] for n in ('tabletop','conveyor'))
        for row in range(6):
            for col in range(3):
                self.assertEqual(table.frame(f'{row}_{col}'),conveyor.frame(f'{row}_{col}'))
            for method in ('placement','home'):
                a=np.array(getattr(table,method)(row));b=np.array(getattr(conveyor,method)(row))
                expected=np.zeros(16);expected[12:15]=[.025,0,.055]
                if method=='placement':expected[14]+=conveyor.setup['placement_height_correction_m']
                np.testing.assert_allclose(b-a,expected,atol=1e-15)
        self.assertEqual(table.motion['gripper'],conveyor.motion['gripper'])
        self.assertEqual(table.settings['observation_height_m'],conveyor.settings['observation_height_m'])
        self.assertNotEqual(table.cache_dir,conveyor.cache_dir)
        self.assertAlmostEqual(table.placement(5)[14],.00707379262894392)
        self.assertAlmostEqual(table.home(5)[14],.05542837847024202)
        self.assertAlmostEqual(conveyor.placement(5)[14],conveyor.setup['placement_height_teaching']['world_z_m'])
        self.assertAlmostEqual(conveyor.home(5)[14],.11042837847024203)

    def test_repeated_selection_does_not_mutate_inputs_or_accumulate(self):
        before=copy.deepcopy((self.settings,self.factory,self.motion))
        for name in ('conveyor','tabletop','conveyor','tabletop'):
            settings,factory,motion,info=resolve_setup(name,self.settings,self.factory,self.motion)
            expected=self.planners[name]
            self.assertEqual(factory['conveyor_target_O_T_EE'],expected.factory['conveyor_target_O_T_EE'])
            self.assertEqual(settings,expected.settings)
            self.assertEqual(motion['workspace_bounds_m']['max'][2],.5)
        self.assertEqual(before,(self.settings,self.factory,self.motion))

    def test_reject_wrong_profile_tracking_and_unsafe_limits(self):
        for name in ('unknown',):
            with self.assertRaises(ValueError):resolve_setup(name,self.settings,self.factory,self.motion)
        moving=dict(self.factory,conveyor_placement_mode='speed_matched')
        with self.assertRaises(ValueError):resolve_setup('conveyor',self.settings,moving,self.motion)
        for value in (1.1,float('nan'),0):
            settings=dict(self.settings,joint_velocity_rad_s=value)
            with self.assertRaises(ValueError):resolve_setup('tabletop',settings,self.factory,self.motion)

    def test_controller_evidence_reused_only_with_unchanged_limits(self):
        commissioned=json.loads((HERE/'stationary_verified.json').read_text())
        for name in ('tabletop','conveyor'):
            planner=self.planners[name]
            self.assertTrue(stationary_profile_evidence(planner,commissioned))
            with patch.object(planner,'settings',dict(planner.settings,joint_velocity_rad_s=.9)):
                self.assertIsNone(stationary_profile_evidence(planner,commissioned))

    def test_moving_profile_reuses_conveyor_geometry_and_matches_through_release(self):
        stationary=self.planners['conveyor'];moving=self.planners['conveyor_moving'];cfg=moving.moving
        for row in range(6):
            self.assertEqual(stationary.placement(row),moving.placement(row))
            self.assertEqual(stationary.home(row),moving.home(row))
            for col in range(3):self.assertEqual(stationary.frame(f'{row}_{col}'),moving.frame(f'{row}_{col}'))
        self.assertNotEqual(stationary.cache_dir,moving.cache_dir)
        self.assertEqual(cfg['belt_velocity_base_m_s'],[0.,.02,0.])
        self.assertEqual(moving.motion['gripper'],stationary.motion['gripper'])
        self.assertFalse(moving.factory['phase_schedule']['enabled'])
        start=cfg['descent_time_s']+cfg['contact_settle_time_s'];end=start+cfg['matched_release_time_s']
        offsets,velocity=placement_profile(np.linspace(start,end,100),cfg)
        np.testing.assert_allclose(velocity[:,1],.02,atol=1e-12)
        np.testing.assert_allclose(velocity[:,2],0.,atol=1e-12)
        np.testing.assert_allclose(offsets[:,2],0.,atol=1e-12)
        duration=cfg['forward_travel_m']/.02+cfg['acceleration_time_s']
        positions,speeds=placement_profile(np.linspace(0,duration,1001),cfg)
        self.assertAlmostEqual(positions[-1,1],.066)
        self.assertGreaterEqual(np.min(np.diff(positions[:,1])),-1e-12)
        self.assertTrue(np.any((speeds[:,1]>0)&(speeds[:,2]<0)))
        np.testing.assert_allclose(speeds[[0,-1]],0.,atol=1e-12)
        self.assertGreater(cfg['matched_release_time_s'],cfg['measured_release_duration_bound_s']+cfg['release_command_margin_s'])

    def test_task_preflight_never_opens_sdk_or_camera_or_changes_inventory(self):
        inventory=HERE/'inventory.json'
        before=inventory.read_bytes() if inventory.exists() else None
        with tempfile.TemporaryDirectory() as temp, \
                patch('run_cycle.capture_pose',side_effect=AssertionError('SDK capture forbidden')), \
                patch('run_cycle.Owner',side_effect=AssertionError('SDK owner forbidden')), \
                patch('run_cycle.Camera',side_effect=AssertionError('Camera forbidden')), \
                patch('run_cycle.stationary_profile_evidence',side_effect=AssertionError('Historical commissioning gate forbidden')):
            for name in ('tabletop','conveyor','conveyor_moving'):
                out=Path(temp)/name;out.mkdir()
                f=Factory(out,['5_0'],initial_state=self.state,task_execution=True,setup=name)
                self.assertEqual(f.stationary,name!='conveyor_moving')
                self.assertFalse(f.placement_verification_enabled)
                self.assertIsNone(f.owner)
                self.assertIsNone(f.camera)
                if not f.stationary:
                    manifest=f.controller_manifest();manifest_path=out/'controller_manifest.json'
                    manifest_path.write_text(json.dumps(manifest))
                    def native_validate():
                        return subprocess.run([str(f.controller_path),str(manifest_path),'--validate-only'],
                            capture_output=True,text=True,timeout=3)
                    result=native_validate()
                    self.assertEqual(result.returncode,0,result.stdout+result.stderr)
                    self.assertIn('configuration_validated',result.stdout)
                    self.assertIn('warning-only',result.stdout)
                    manifest['execution_profile']['task_execution']=False;manifest_path.write_text(json.dumps(manifest))
                    result=native_validate()
                    self.assertEqual(result.returncode,0,result.stdout)
                    self.assertIn('guard enforced',result.stdout)
                    manifest['maximum_release_vy_m_s']=.026;manifest_path.write_text(json.dumps(manifest))
                    result=native_validate()
                    self.assertNotEqual(result.returncode,0)
                    self.assertIn('Invalid measured release speed bounds',result.stdout)
                selected=f.common['place'] if f.stationary else f.common['track']
                f.planner.preflight_path(selected)
                self.assertEqual(f.controller_path.name,'controller_conveyor_moving' if not f.stationary else 'controller')
                bad=copy.deepcopy(selected);bad['duration_s']=11.
                with self.assertRaises(RuntimeError):f.planner.preflight_path(bad)
                bad=copy.deepcopy(selected);bad['coefficients'][0][0][0]=float('nan')
                with self.assertRaises(RuntimeError):f.planner.preflight_path(bad)
                bad=copy.deepcopy(selected);bad['coefficients'][0][0][4]+=100.
                with self.assertRaises(RuntimeError):f.planner.preflight_path(bad)
                if not f.stationary:
                    bad=copy.deepcopy(selected);bad['release_deadline']=bad['duration_s']+.1
                    with self.assertRaises(RuntimeError):f.planner.preflight_path(bad)
        self.assertEqual(before,inventory.read_bytes() if inventory.exists() else None)


if __name__=='__main__':unittest.main()
