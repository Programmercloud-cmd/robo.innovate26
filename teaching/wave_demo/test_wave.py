"""Focused geometry, continuity and native preflight checks; never opens SDK."""
import copy
import json
from pathlib import Path
import subprocess
import tempfile
import unittest

import numpy as np
from scipy.interpolate import PPoly

from teaching.wave_demo.plan import HERE,Plan,wave_angles


class WaveTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cfg=json.loads((HERE/'config.json').read_text());cls.plan=Plan(cls.cfg);cls.manifest=cls.plan.build()

    @staticmethod
    def curve(path):return PPoly(np.array(path['coefficients']).transpose(2,0,1),np.r_[path['knots_s'],path['duration_s']])

    def test_world_y_wave_tilt_and_smooth_boundaries(self):
        path=self.manifest['paths'][3];curve=self.curve(path);times=np.linspace(0,path['duration_s'],1001)
        poses=[self.plan.model.fk(q) for q in curve(times)]
        xyz=np.array([p.translation for p in poses]);neutral=self.plan.model.fk(self.cfg['upright_q_rad']).rotation
        relative=[p.rotation@neutral.T for p in poses]
        angles=np.degrees(np.arctan2([r[2,1] for r in relative],[r[1,1] for r in relative]))
        np.testing.assert_allclose(np.array([p.rotation[:,2] for p in poses]),np.tile([1,0,0],(len(poses),1)),atol=1e-8)
        self.assertGreater(angles.max(),33.9);self.assertLess(angles.min(),-33.9)
        self.assertLessEqual(np.max(np.abs(angles)),34.001)
        self.assertLess(np.ptp(xyz[:,0]),1e-8)
        for d in (1,2):np.testing.assert_allclose(curve([0,path['duration_s']],d),0.,atol=1e-7)
        self.assertGreaterEqual(path['certificate']['min_full_sigma'],.15)
        self.assertLessEqual(path['certificate']['max_full_condition'],35.)

    def test_home_gate_vertical_ascent_and_return(self):
        paths=self.manifest['paths'];home=self.cfg['home_5_O_T_EE']
        np.testing.assert_allclose(paths[-1]['target'],home,atol=2e-8)
        lift=paths[1];curve=self.curve(lift)
        xyz=np.array([self.plan.model.fk(q).translation for q in curve(np.linspace(0,lift['duration_s'],501))])
        np.testing.assert_allclose(xyz[:,:2],np.tile(home[12:14],(len(xyz),1)),atol=2e-6)
        self.assertAlmostEqual(xyz[-1,2],.4)
        invalid=copy.deepcopy(self.cfg);invalid['commanded_tilt_deg']=36
        with self.assertRaises(ValueError):Plan(invalid)
        with self.assertRaisesRegex(ValueError,'vertical column'):Plan(self.cfg).build(self.cfg['seed_q_rad'])
        single=copy.deepcopy(self.cfg);single['cycles']=1;Plan(single)
        angles=np.degrees(wave_angles(np.linspace(0,1,1001),single))
        self.assertGreater(angles.max(),33.9);self.assertLess(angles.min(),-33.9)

    def test_recorded_elevated_start_descends_inside_home_column(self):
        recorded=json.loads((HERE/'recorded_column_start.json').read_text());plan=Plan(self.cfg)
        qhome=plan.approach_home(recorded['q_d']);path=plan.paths[0];curve=self.curve(path)
        positions=np.array([plan.model.fk(q).translation for q in curve(np.linspace(0,path['duration_s'],501))])
        home=np.array(self.cfg['home_5_O_T_EE'][12:15])
        self.assertGreater(positions[0,2],home[2]+.2)
        self.assertLessEqual(np.max(np.abs(positions[:,:2]-home[:2])),.05)
        np.testing.assert_allclose(positions[-1],home,atol=2e-8)
        np.testing.assert_allclose(qhome,self.manifest['paths'][1]['q0'],atol=1e-7)
        # Also exercise native entry-column checks against this actual failure.
        plan.paths.extend(copy.deepcopy(self.manifest['paths'][1:]));manifest=plan.manifest()
        with tempfile.TemporaryDirectory() as temp:
            p=Path(temp)/'manifest.json';p.write_text(json.dumps(manifest))
            result=subprocess.run([str(HERE/'controller'),str(p),'--check'],capture_output=True,text=True,timeout=30)
            self.assertEqual(result.returncode,0,result.stderr)

    def test_native_check_and_safety_rejection_without_sdk(self):
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'manifest.json';path.write_text(json.dumps(self.manifest))
            def native():return subprocess.run([str(HERE/'controller'),str(path),'--check'],capture_output=True,text=True,timeout=30)
            result=native();self.assertEqual(result.returncode,0,result.stderr);self.assertIn('no SDK connection',result.stdout)
            bad=copy.deepcopy(self.manifest);bad['config']['maximum_measured_tilt_deg']=36;path.write_text(json.dumps(bad))
            result=native();self.assertNotEqual(result.returncode,0);self.assertIn('Unsafe demo limits',result.stderr)
            bad=copy.deepcopy(self.manifest);bad['config']['limits']['min_full_sigma']=.01;path.write_text(json.dumps(bad))
            result=native();self.assertNotEqual(result.returncode,0);self.assertIn('Unsafe demo limits',result.stderr)


if __name__=='__main__':unittest.main()
