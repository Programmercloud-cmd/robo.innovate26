"""Serialize the prepared timed spline for an independent C++ model/runtime check."""
import json
from pathlib import Path
import sys
import numpy as np
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent))
from kinematics import Kinematics


def freeze(plan_path):
    path=Path(plan_path);plan=json.loads(path.read_text());motion=json.loads((HERE.parent/'motion_config.json').read_text())
    cfg=plan['config'];model=Kinematics(plan['tool_F_T_EE'],motion['singularity_avoidance']);fixed=plan['fixed_model']
    authorized=int(cfg['execution_enabled'] and cfg['release_timing_physically_validated'])
    current=json.loads((HERE/'config.json').read_text())
    if cfg['belt_velocity_base_m_s']!=current['belt_velocity_base_m_s']:
        raise ValueError('Cached conveyor velocity is stale; prepare a fresh trajectory')
    values=[2,len(plan['joint_coefficients']),cfg['acceleration_time_s'],cfg['matched_release_time_s'],cfg['belt_velocity_base_m_s'][1],cfg['preplace_height_m'],motion['max_callback_gap_s'],cfg['release_opening_m'],cfg['release_speed_m_s'],motion['control_cpu'],authorized]
    values+=plan['conveyor_O_T_EE']+plan['tool_F_T_EE']+plan['q0']+model.lower.tolist()+model.upper.tolist()
    values+=motion['workspace_bounds_m']['min']+motion['workspace_bounds_m']['max']+[motion['singularity_avoidance']['characteristic_length_m']]
    values+=np.array(fixed['joint_origins']).reshape(-1).tolist()+fixed['joint7_to_ee']
    for time,coeff in zip(plan['knots_s'][:-1],plan['joint_coefficients']):values+=[time]+np.array(coeff).reshape(-1).tolist()
    output=path.with_name('frozen_input.txt');output.write_text('\n'.join(format(x,'.17g') for x in values)+'\n');return output


if __name__=='__main__':print(freeze(sys.argv[1]))
