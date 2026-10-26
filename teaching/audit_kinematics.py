#!/usr/bin/python3
"""Audit both task-equivalent yaw choices for all stored Cartesian targets; no motion."""
from datetime import datetime,timezone
import hashlib
import json
import math
from pathlib import Path
import numpy as np
from kinematics import Kinematics

HERE=Path(__file__).resolve().parent


def yaw_variant(pose,degrees):
    matrix=np.array(pose).reshape((4,4),order='F').copy()
    a=math.radians(degrees);c,s=math.cos(a),math.sin(a)
    # Tool-axis symmetry preserves the approach direction even for a taught
    # pose with a small residual tilt. A world-Z turn would reverse that tilt.
    matrix[:3,:3]=matrix[:3,:3]@np.array([[c,-s,0],[s,c,0],[0,0,1]])
    return matrix.reshape(-1,order='F').tolist()


def catalog_signature(raw,policy,model_sha):
    return hashlib.sha256(raw+json.dumps(policy,sort_keys=True).encode()+model_sha.encode()).hexdigest()


def main():
    raw=(HERE/'positions.json').read_bytes();catalog=json.loads(raw)
    if catalog.get('task_id')!='factory_columns_of_four':raise RuntimeError('Not the factory catalog')
    if not catalog['points']:
        print('No factory poses recorded; no robot connection or motion.');return
    policy=json.loads((HERE/'motion_config.json').read_text())['singularity_avoidance']
    points=list(catalog['points'].items())
    model=Kinematics(points[0][1]['tool_frames']['F_T_EE'],policy)
    seeds=[np.array(point['q_rad']) for _,point in points];entries={}
    for name,point in points:
        candidates=[]
        for yaw in ([0,180] if policy['allow_yaw180_for_bricks'] else [0]):
            pose=yaw_variant(point['O_T_EE'],yaw)
            for seed in [np.array(point['q_rad'])]+seeds:
                result=model.solve(pose,seed)
                if result:
                    result.update(effective_O_T_EE=pose,yaw_variant_deg=yaw,
                                  shoulder_sign=1 if result['q'][1]>0 else -1)
                    candidates.append(result);break
        identifier=str(int(name.removeprefix('point_')))
        entries[identifier]=dict(candidates=candidates,status='model_feasible' if candidates else 'blocked',
            canonical_O_T_EE=point['O_T_EE'],physical_validation=False)
    result=dict(created_utc=datetime.now(timezone.utc).isoformat(),
        source_signature=catalog_signature(raw,policy,model.model_sha256),
        model_sha256=model.model_sha256,poses=entries,count=len(entries),
        model_feasible_count=sum(bool(p['candidates']) for p in entries.values()),
        scope='Current factory endpoints only; complete motion paths still require independent preflight')
    (HERE/'kinematic_catalog.json').write_text(json.dumps(result,indent=2)+'\n')
    print('Factory endpoint audit:',result['model_feasible_count'],'/',result['count'])


if __name__=='__main__':main()
