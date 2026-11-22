"""Replay measured endpoint evidence; no SDK connection or robot commands."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
import io
from contextlib import redirect_stdout
from unittest.mock import patch
from types import SimpleNamespace

from planner import Planner
from run_cycle import Factory,HERE
from runtime import Stop


class RecordedOwner:
    def __init__(self,samples):
        self.samples=copy.deepcopy(samples);self.latest=self.samples[0];self.now=0.

    def state(self):return self.latest

    def go(self,id,duration):return self.latest

    def wait(self,predicate,timeout):
        start=self.samples[0]['unix_ns']
        for sample in self.samples:
            self.now=(sample['unix_ns']-start)/1e9
            if self.now>timeout:break
            sample['received_monotonic']=self.now;self.latest=sample
            if predicate():return
        raise Stop('SDK owner response timed out')


class EndpointTests(unittest.TestCase):
    def test_shared_sdk_owner_release_trace_is_available_for_later_rows(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Factory.__new__(Factory);first=Path(temp)/'5_0';first.mkdir()
            f.out=Path(temp)/'5_1';f.out.mkdir();f.owner=SimpleNamespace(directory=first)
            original=first/'tracking_3.csv';original.write_text('y,z\n0.1,0.06\n0.166,0.11\n')
            trace,rows=f.release_trace()
            self.assertEqual(trace.parent,f.out)
            self.assertEqual(trace.read_bytes(),original.read_bytes())
            self.assertEqual(float(rows[-1]['y'])-float(rows[0]['y']),.066)

    def test_task_placement_warns_for_recorded_error_but_grasp_and_hard_bound_stop(self):
        evidence=json.loads((HERE/'testdata/loaded_endpoint_20261028.json').read_text())['placement']
        settings=json.loads((HERE/'settings.json').read_text())
        with tempfile.TemporaryDirectory() as temp:
            f=Factory.__new__(Factory);f.out=Path(temp);f.settings=settings;f.task_execution=True
            f.free_endpoint_tolerance=.0015
            f.planner=Planner(settings,evidence['samples'][0]['state'],setup='conveyor')
            path=dict(target=evidence['target'],q0=evidence['samples'][0]['sent'],duration_s=1.)
            f.owner=RecordedOwner(evidence['samples'])
            with patch('run_cycle.time.monotonic',lambda:f.owner.now),redirect_stdout(io.StringIO()) as output:
                f.motion('place',path,id=1)
            self.assertIn('WARNING:',output.getvalue())
            self.assertEqual(len((f.out/'warnings.ndjson').read_text().splitlines()),1)
            f.owner=RecordedOwner(evidence['samples'])
            with patch('run_cycle.time.monotonic',lambda:f.owner.now),self.assertRaisesRegex(Stop,'no close command sent'):
                f.motion('descend',path,id=1)
            far=copy.deepcopy(evidence['samples'])
            for sample in far:sample['state']['O_T_EE'][14]=path['target'][14]+.0016
            f.owner=RecordedOwner(far)
            with patch('run_cycle.time.monotonic',lambda:f.owner.now),self.assertRaisesRegex(Stop,'no release command sent'):
                f.motion('place',path,id=1)

    def test_recorded_source_passes_but_loaded_placement_stops_before_release(self):
        recorded=json.loads((HERE/'testdata/loaded_endpoint_20261028.json').read_text())
        settings=json.loads((HERE/'settings.json').read_text())
        with tempfile.TemporaryDirectory() as temp:
            for case,phase in (('source','descend'),('placement','place')):
                evidence=recorded[case];f=Factory.__new__(Factory)
                f.out=Path(temp)/case;f.out.mkdir();f.settings=settings;f.task_execution=False
                f.owner=RecordedOwner(evidence['samples']);f.free_endpoint_tolerance=.0015
                f.planner=Planner(settings,evidence['samples'][0]['state'],setup='conveyor')
                path=dict(target=evidence['target'],q0=f.owner.latest['sent'],duration_s=1.)
                with patch('run_cycle.time.monotonic',lambda:f.owner.now):
                    if case=='source':f.motion(phase,path,id=1)
                    else:
                        with self.assertRaisesRegex(Stop,'no release command sent'):
                            f.motion(phase,path,id=1)
                diagnostic=json.loads((f.out/'endpoint_diagnostics.ndjson').read_text())
                self.assertEqual(diagnostic['accepted'],case=='source')
                self.assertEqual(diagnostic['position_tolerance_m'],.001)
                if case=='placement':
                    self.assertGreater(diagnostic['measured_position_error_m'],.001)
                    self.assertLess(diagnostic['hand_width_m'],.0655)
                    self.assertLess(diagnostic['estimated_base_wrench'][2],-10.)

    def test_failed_placement_preserves_holding_phase(self):
        f=Factory.__new__(Factory);f.stationary=True;f.task_execution=True
        f.out=Path('/unused');f.common={'place':{}};f.ids={'common':{'place':1}}
        f.owner=RecordedOwner([dict(state={'O_T_EE':[0.]*16})]);phases=[]
        f.active=lambda label,phase:phases.append(phase)
        f.motion=lambda *args:(_ for _ in ()).throw(Stop('endpoint rejected'))
        f.phase=lambda name,operation:operation()
        with patch('run_cycle.save'),self.assertRaisesRegex(Stop,'endpoint rejected'):
            f.finish_delivery('5_0',0.,0,False,True)
        self.assertEqual(phases,['placement_descent_unverified_holding'])


if __name__=='__main__':unittest.main()
