"""Invented tensor-buffer/path tests; no actual Tensor API or scene."""
import importlib.util
from pathlib import Path
import sys
from types import ModuleType,SimpleNamespace
import unittest
from unittest.mock import patch
import numpy as np

SOURCE=Path(__file__).resolve().parents[1]/'experiments/control_alignment_20261026/batched_source.py'
spec=importlib.util.spec_from_file_location('batched_source_test_module',SOURCE)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


class Tensor:
    def __init__(self,array):self.array=array
    def __getitem__(self,index):return Tensor(self.array[index])
    def detach(self):return self
    def cpu(self):return self
    def numpy(self):return self.array


def fixture(paths=None):
    names=['brick_a','brick_b','brick_c'];expected=['/World/'+n for n in names]
    old=np.array([[1.,2.,3.,1.,0.,0.,0.],[4.,5.,6.,1.,0.,0.,0.],[7.,8.,9.,1.,0.,0.,0.]],np.float32)
    scene={name:SimpleNamespace(root_physx_view=SimpleNamespace(prim_paths=[path]),data=SimpleNamespace(root_link_pose_w=Tensor(old[i:i+1]))) for i,(name,path) in enumerate(zip(names,expected))}
    actual=[expected[2],expected[0],expected[1]] if paths is None else paths
    raw=np.array([[7.,8.,9.,0.,0.,0.,-1.],[1.,2.,3.,0.,0.,0.,1.],[4.,5.,6.,0.,0.,0.,1.]],np.float32)
    view=SimpleNamespace(prim_paths=actual,count=len(actual),get_transforms=lambda:Tensor(raw))
    calls=[]
    simulation=SimpleNamespace(set_subspace_roots=lambda value:calls.append(('roots',value)),
        create_rigid_body_view=lambda value:(calls.append(('paths',value)) or view))
    tensor_api=SimpleNamespace(create_simulation_view=lambda frontend:(calls.append(('frontend',frontend)) or simulation))
    omni=ModuleType('omni');physics=ModuleType('omni.physics');physics.tensors=tensor_api;omni.physics=physics
    return names,scene,raw,view,calls,{'omni':omni,'omni.physics':physics,'omni.physics.tensors':tensor_api}


class BatchedTests(unittest.TestCase):
    def test_actual_path_permutation_world_frame_and_sign(self):
        names,scene,raw,view,calls,modules=fixture()
        with patch.dict(sys.modules,modules):reader=module.BatchedSourcePoses(scene,list(reversed(names)))
        self.assertEqual(reader.proof['names'],names);self.assertEqual(reader.proof['row_indices_in_name_order'],[1,2,0])
        self.assertEqual(calls,[('frontend','torch'),('roots','/'),('paths',['/World/'+n for n in names])])
        self.assertTrue(reader.proof['equivalence_passed']);self.assertEqual(reader.proof['maximum_angle_error_rad'],0.)
        np.testing.assert_array_equal(reader.read()[:,0],[1,4,7])
        np.testing.assert_array_equal(reader.read()[2,3:],[-1,0,0,0])  # Preserve raw quaternion sign.

    def test_snapshots_do_not_alias_reused_tensor_buffer(self):
        names,scene,raw,view,calls,modules=fixture()
        with patch.dict(sys.modules,modules):reader=module.BatchedSourcePoses(scene,names)
        first=reader.read();proof=np.asarray(reader.proof['new_poses_world_wxyz']).copy();raw[:,0]+=100
        np.testing.assert_array_equal(first[:,0],[1,4,7]);np.testing.assert_array_equal(proof[:,0],[1,4,7])
        np.testing.assert_array_equal(reader.read()[:,0],[101,104,107])
        first[:]=99;np.testing.assert_array_equal(reader.read()[:,0],[101,104,107])

    def test_missing_duplicate_actor_paths_rejected(self):
        for paths in (['/World/brick_a','/World/brick_a','/World/brick_c'],['/World/brick_a','/World/brick_b'],['/World/brick_a','/World/brick_b','/World/other']):
            names,scene,raw,view,calls,modules=fixture(paths)
            with patch.dict(sys.modules,modules):
                with self.assertRaises(RuntimeError):module.BatchedSourcePoses(scene,names)
        names,scene,raw,view,calls,modules=fixture();scene[names[1]].root_physx_view.prim_paths=['/World/brick_a']
        with patch.dict(sys.modules,modules):
            with self.assertRaises(RuntimeError):module.BatchedSourcePoses(scene,names)
            with self.assertRaises(ValueError):module.BatchedSourcePoses(scene,[names[0],names[0]])

    def test_startup_discrepancy_and_invalid_readback(self):
        for mutation in ('position','angle','nan','zero'):
            names,scene,raw,view,calls,modules=fixture()
            if mutation=='position':raw[1,0]+=.001
            elif mutation=='angle':raw[1,3]=1e-5
            elif mutation=='nan':raw[0,0]=float('nan')
            else:raw[0,3:]=0
            with patch.dict(sys.modules,modules):
                with self.assertRaises(RuntimeError):module.BatchedSourcePoses(scene,names)
        q=np.array([[1.,0.,0.,0.],[0.,1.,0.,0.]])
        np.testing.assert_allclose(module.quaternion_angles(q,-q),0,atol=0)


if __name__=='__main__':unittest.main()
