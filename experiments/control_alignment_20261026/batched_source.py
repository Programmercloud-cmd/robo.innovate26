"""Read-only world-pose batching with explicit name/path/row correspondence."""
import copy
import numpy as np


class SourcePoseEquivalenceError(RuntimeError):
    """Preserve the failed comparison without weakening its acceptance gate."""
    def __init__(self,proof):
        super().__init__('Batched/per-asset startup source poses differ')
        self.proof=copy.deepcopy(proof)


def _array(value):
    if hasattr(value,'detach'):value=value.detach()
    if hasattr(value,'cpu'):value=value.cpu()
    if hasattr(value,'numpy'):value=value.numpy()
    return np.asarray(value)


def _poses(value,count):
    result=np.array(_array(value),dtype=np.float64,copy=True)
    if result.shape!=(count,7) or not np.isfinite(result).all() or np.any(np.linalg.norm(result[:,3:],axis=1)==0):
        raise RuntimeError('Finite nonzero-quaternion source poses required')
    return result


def quaternion_angles(a,b):
    """Float64, sign-invariant quaternion angles, without acos near-one loss."""
    a=np.asarray(a,dtype=np.float64);b=np.asarray(b,dtype=np.float64)
    a=a/np.linalg.norm(a,axis=1)[:,None];b=b/np.linalg.norm(b,axis=1)[:,None]
    distance=np.minimum(np.linalg.norm(a-b,axis=1),np.linalg.norm(a+b,axis=1))
    return 4*np.arcsin(np.clip(distance/2,0,1))


class BatchedSourcePoses:
    def __init__(self,scene,names):
        names=list(names)
        if not names or any(not isinstance(n,str) or not n for n in names) or len(set(names))!=len(names):
            raise ValueError('Unique source names required')
        self.names=sorted(names);expected=[]
        for name in self.names:
            paths=list(map(str,scene[name].root_physx_view.prim_paths))
            if len(paths)!=1 or not paths[0].startswith('/'):raise RuntimeError('Exact single actor root path required')
            expected.append(paths[0])
        if len(set(expected))!=len(expected):raise RuntimeError('Duplicate expected source root path')
        from omni.physics import tensors
        self.simulation_view=tensors.create_simulation_view('torch')
        self.simulation_view.set_subspace_roots('/')
        self.view=self.simulation_view.create_rigid_body_view(expected)
        actual=list(map(str,self.view.prim_paths));count=len(self.names)
        if self.view.count!=count or len(actual)!=count or len(set(actual))!=count or set(actual)!=set(expected):
            raise RuntimeError('Batched source actor paths differ from the exact requested set')
        lookup={path:i for i,path in enumerate(actual)}
        self.indices=np.asarray([lookup[path] for path in expected],dtype=np.int64)
        # Both readers execute now, with no simulation step or actor writes.
        old=_poses(np.stack([_array(scene[name].data.root_link_pose_w[0]) for name in self.names]),count)
        new=self.read();position_error=np.linalg.norm(old[:,:3]-new[:,:3],axis=1)
        angle_error=quaternion_angles(old[:,3:],new[:,3:])
        passed=bool(np.max(position_error)<=1e-9 and np.max(angle_error)<=1e-7)
        self.proof={'names':self.names.copy(),'expected_root_paths':expected,'actual_view_prim_paths':actual,
            'row_indices_in_name_order':self.indices.tolist(),'count':count,'subspace_roots':'/','frame':'world',
            'getter_layout':'xyz_xyzw','returned_layout':'xyz_wxyz','snapshot_copy':True,
            'no_step_called_by_reader':True,'old_reader':'scene[name].data.root_link_pose_w[0]',
            'old_poses_world_wxyz':old.tolist(),'new_poses_world_wxyz':new.tolist(),
            'maximum_position_error_m':float(np.max(position_error)),'maximum_angle_error_rad':float(np.max(angle_error)),
            'position_tolerance_m':1e-9,'angle_tolerance_rad':1e-7,'equivalence_passed':passed}
        if not passed:raise SourcePoseEquivalenceError(self.proof)

    def read(self):
        raw=_array(self.view.get_transforms())
        if raw.shape!=(len(self.names),7):raise RuntimeError('Unexpected batched transform shape')
        reordered=raw[self.indices][:,[0,1,2,6,3,4,5]]
        return _poses(reordered,len(self.names))
