"""Focused closed-loop software checks; all images/wrenches are synthetic."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import numpy as np

from .fixture import Fixture
from .geometry import ROOT,build_plan
from .hardware import Hardware
from .pipeline import LayerStacker,RecoverableStop
from .vision import correction,Uncertain,fit_calibration

HERE=Path(__file__).resolve().parent


class LayersTests(unittest.TestCase):
    def config(self):return json.loads((HERE/'config.json').read_text())
    def exercise(self,**kwargs):
        config=self.config();backend=Fixture(config,**kwargs)
        with tempfile.TemporaryDirectory() as directory:
            runner=LayerStacker(config,build_plan(config),backend,Path(directory)/'run');result=runner.run()
            return result,copy.deepcopy(runner.inventory),runner.events

    def test_compact_geometry_all_twelve_sources_and_alternating_grid(self):
        plan=build_plan(self.config());self.assertEqual(len(plan['sources']),12)
        self.assertEqual(plan['footprint_studs'],[12,8])
        for k,layer in enumerate(plan['layers']):
            self.assertEqual(len(layer['bricks']),12)
            studs={tuple(p) for b in layer['bricks'] for p in b['studs']}
            self.assertEqual(len(studs),96)
            extent=[max(x[i] for x in studs)-min(x[i] for x in studs)+1 for i in (0,1)]
            self.assertEqual(extent,[12,8] if k==0 else [8,12])
            self.assertTrue(all(1<=x<=30 for p in studs for x in p))
            self.assertEqual(layer['potential_stud_matches'],96 if k==0 else 64)
            if k:self.assertTrue(any(b['support']['cantilever_clutch_validation_required'] for b in layer['bricks']))
        bad=self.config();bad['geometry']['target_center_stud']=[2.5,2.5]
        with self.assertRaises(ValueError):build_plan(bad)

    def test_full_two_layer_loop_and_transient_observation_recovery(self):
        inventory=ROOT/'teaching/factory_cycle/inventory.json';before=inventory.read_bytes()
        for failures in ((),(1,2)):
            result,inv,events=self.exercise(fail_vision_frames=failures)
            self.assertTrue(result['completed'],result);self.assertEqual(len(inv['completed']),2)
            self.assertEqual(result['physical_interlocks_verified'],0)
            self.assertIsNone(inv['pending']);self.assertIsNone(inv['held'])
            self.assertTrue(any(e['phase']=='edge_alignment' for e in events))
        self.assertEqual(before,inventory.read_bytes())

    def test_jam_and_wrong_release_do_not_advance_inventory(self):
        for kwargs in (dict(jam=True),dict(bad_release=True),dict(fail_vision_frames=(1,2,3))):
            result,inv,events=self.exercise(**kwargs)
            self.assertFalse(result['completed']);self.assertEqual(len(inv['completed']),0)
            self.assertEqual(len(inv['available']),2);self.assertIsNotNone(inv['pending'])
            for e in events:
                if e['event']=='bounded_contact_search':self.assertLessEqual(e['radius_m'],.000400001)

    def test_stale_images_and_uncertain_grasp_stop(self):
        c=self.config();plan=build_plan(c)
        for failure in ('image','grasp','source_departure'):
            backend=Fixture(c)
            if failure=='image':
                original=backend.frame
                def stale(*args):
                    packet,reference=original(*args);packet['stamp_s']-=10;return packet,reference
                backend.frame=stale
            else:
                def bad(*args):raise RecoverableStop('uncertain grasp')
                if failure=='grasp':backend.lift_verify=bad
                else:backend.verify_departed_source=bad
            with tempfile.TemporaryDirectory() as directory:
                runner=LayerStacker(c,plan,backend,Path(directory)/'run');result=runner.run()
                self.assertFalse(result['completed']);self.assertFalse(runner.inventory['completed'])

    def test_vision_subpixel_correction_and_held_slip_guard(self):
        c=self.config();b=Fixture(c);layer=build_plan(c)['layers'][0]
        frame,reference=b.frame(layer,'align');delta,_=correction(frame['image'],reference,c['vision'])
        np.testing.assert_allclose(delta[:2],-b.error[:2],atol=.00012)
        self.assertLess(abs(delta[2]+b.error[2]),.0015)
        image=frame['image'].copy();image[60:210,80:250]=35
        with self.assertRaises(Uncertain):correction(image,reference,c['vision'])

    def test_hardware_missing_calibration_blocks_before_sdk_access(self):
        with patch('subprocess.Popen',side_effect=AssertionError('No hardware subprocess allowed')):
            with self.assertRaisesRegex(RecoverableStop,'before SDK access'):
                Hardware(self.config(),Path('/tmp/unused'),confirm_empty=True)

    def test_force_overload_stops_before_any_inventory_commit(self):
        c=self.config();backend=Fixture(c);backend.force=lambda:6.
        with tempfile.TemporaryDirectory() as directory:
            runner=LayerStacker(c,build_plan(c),backend,Path(directory)/'run');result=runner.run()
            self.assertFalse(result['completed']);self.assertIn('force/torque',result['error'])
            self.assertFalse(runner.inventory['completed']);self.assertIsNotNone(runner.inventory['pending'])


if __name__=='__main__':unittest.main()
