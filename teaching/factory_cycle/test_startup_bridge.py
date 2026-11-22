"""Replay the measured Idle-to-FCI reference change without hardware."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
from scipy.interpolate import PPoly

from planner import Planner
from run_cycle import Factory,HERE
from runtime import Stop


class StartupOwner:
    def __init__(self,recording,settings):
        self.recording=recording;self.settings=settings;self.latest=copy.deepcopy(recording['idle'])
        self.paths={};self.serial=10;self.executed=[]

    def state(self):return self.latest

    def load(self,path):
        self.serial+=1;self.paths[self.serial]=path;return self.serial

    def begin(self):self.latest=copy.deepcopy(self.recording['fci'])

    def finish(self):self.latest['control']=False;self.latest['state']['robot_mode']='Idle'

    def go(self,id,duration):
        path=self.paths[id];T=path['duration_s'];difference=np.abs(np.array(self.latest['sent'])-path['q0'])
        curve=PPoly(np.array(path['coefficients']).transpose(2,0,1),np.r_[path['knots_s'],T])
        for d,key,factor in ((1,'joint_velocity_rad_s',1.875),(2,'joint_acceleration_rad_s2',5.774),(3,'joint_jerk_rad_s3',60.)):
            maximum=np.max(np.abs(curve.derivative(d)(np.linspace(0,T,1001))),axis=0)
            if np.any(maximum+factor*difference/T**d>self.settings[key]):
                raise Stop('Reference bridge exceeds derivative headroom')
        self.executed.append(path)
        for key in ('q','q_d'):self.latest['state'][key]=path['q1']
        for key in ('requested','sent'):self.latest[key]=path['q1']
        for key in ('O_T_EE','O_T_EE_d'):self.latest['state'][key]=path['target']
        return self.latest


class StartupBridgeTests(unittest.TestCase):
    def test_recorded_startup_is_rebased_and_retimed_in_both_execution_modes(self):
        recording=json.loads((HERE/'testdata/startup_fci_refresh_20261028.json').read_text())
        settings=json.loads((HERE/'settings.json').read_text())
        for task in (False,True):
            with self.subTest(task_execution=task),tempfile.TemporaryDirectory() as temp:
                f=Factory.__new__(Factory);f.out=Path(temp);f.task_execution=task
                f.planner=Planner(settings,recording['idle']['state'],setup='conveyor_moving')
                f.settings=f.planner.settings;f.home=recording['target'];f.controller_path=HERE/'controller_conveyor_moving'
                f.free_endpoint_tolerance=.0015;f.cfg={'fixed_pose_execution':True,'camera':{}}
                f.empty_gripper_confirmed=False;f.reset_prior_active=None;f.controller_manifest=lambda:{}
                f.common={};f.plans={};f.ids={};f.inspection_recovery=[];f.columns=['5_0'];f.inventory={'available':{}}
                owner=StartupOwner(recording,settings)
                start=f.planner.model.fk(recording['idle']['state']['q_d']).homogeneous.reshape(-1,order='F').tolist()
                old=f.planner.path('initial_home',start,f.home,recording['idle']['state']['q_d'])
                owner.begin()
                with self.assertRaisesRegex(Stop,'Reference bridge exceeds derivative headroom'):
                    owner.go(owner.load(old),old['duration_s'])
                owner.latest=copy.deepcopy(recording['idle'])
                with patch('run_cycle.Owner',return_value=owner),patch('run_cycle.Camera'),patch('run_cycle.save'):
                    f.initialize()
                self.assertEqual(len(owner.executed),1)
                issued=owner.executed[0]
                np.testing.assert_allclose(issued['q0'],recording['fci']['sent'],atol=1e-12)
                self.assertEqual(issued['target'],old['target'])
                self.assertGreater(issued['duration_s'],old['duration_s'])
                f.planner.preflight_path(issued)
                self.assertAlmostEqual(f.settings['joint_jerk_rad_s3'],40.)


if __name__=='__main__':unittest.main()
