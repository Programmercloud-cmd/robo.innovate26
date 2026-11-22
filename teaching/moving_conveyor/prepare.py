#!/usr/bin/python3
"""Prepare a bounded conveyor placement stroke. No connection or command to hardware."""
import argparse
from datetime import datetime,timezone
import hashlib
import json
import math
from pathlib import Path
import sys
import numpy as np
import pinocchio as pin
from scipy.interpolate import CubicSpline

HERE=Path(__file__).resolve().parent
TEACHING=HERE.parent
sys.path.insert(0,str(TEACHING))
from kinematics import Kinematics
from safe_motion import fixed_parameters,spline_bounds


def smoother(u):return u*u*u*(10+u*(-15+6*u))
def integrated_smoother(u):return 2.5*u**4-3*u**5+u**6


def profile(t,config):
    """Y is monotone; contact starts at full belt speed; release precedes braking."""
    ramp=config['acceleration_time_s'];hold=config['matched_release_time_s'];speed=config['belt_velocity_base_m_s'][1]
    height=config['preplace_height_m'];t=np.asarray(t,dtype=float)
    y=np.zeros_like(t);vy=np.zeros_like(t);z=np.zeros_like(t);vz=np.zeros_like(t)
    first=t<ramp;middle=(t>=ramp)&(t<=ramp+hold);last=t>ramp+hold
    u=np.clip(t[first]/ramp,0,1)
    y[first]=speed*ramp*integrated_smoother(u);vy[first]=speed*smoother(u)
    z[first]=height*(1-smoother(u));vz[first]=-height/ramp*(30*u*u*(1-u)**2)
    y[middle]=speed*ramp*.5+speed*(t[middle]-ramp);vy[middle]=speed
    u=np.clip((t[last]-ramp-hold)/ramp,0,1)
    y[last]=speed*ramp*.5+speed*hold+speed*ramp*(u-integrated_smoother(u));vy[last]=speed*(1-smoother(u))
    return np.column_stack([np.zeros_like(t),y,z]),np.column_stack([np.zeros_like(t),vy,vz])


def prepare(layer,loaded_width,output,initial_capture=None,*,elevated_timing_test=False):
    config=json.loads((HERE/'config.json').read_text());motion=json.loads((TEACHING/'motion_config.json').read_text())
    factory=json.loads((TEACHING/'factory_vision/config.json').read_text());catalog_raw=(TEACHING/'positions.json').read_bytes();catalog=json.loads(catalog_raw)
    if not 0<=layer<=5:raise ValueError('Layer must be 0..5')
    v=config['belt_velocity_base_m_s']
    if (len(v)!=3 or not all(math.isfinite(x) for x in v) or
            v[0]!=0 or v[2]!=0 or not 0<v[1]<=.12):
        raise ValueError('Conveyor velocity must be exclusively world +Y, at most 0.12 m/s')
    if not config['loaded_width_min_m']<=loaded_width<=config['loaded_width_max_m']:raise ValueError('Loaded width outside the measured four-brick contact range')
    ramp=config['acceleration_time_s'];hold=config['matched_release_time_s'];duration=2*ramp+hold
    estimated_open=max((config['release_opening_m']-loaded_width)/config['release_speed_m_s'],
        config.get('measured_release_duration_bound_s',0))+config['release_command_margin_s']
    if estimated_open>hold:raise ValueError('Opening estimate exceeds the matching-speed window')
    conveyor=np.array(factory['conveyor_target_O_T_EE']).reshape(4,4,order='F').copy()
    reference_layer=config['conveyor_reference_layer']
    conveyor[:3,3]+=(layer-reference_layer)*np.array(catalog['factory_source_grid']['config']['row_step_in_base_m'])
    home=conveyor.copy();home[2,3]+=config['home_height_m']
    reference_conveyor=conveyor.copy()
    if elevated_timing_test:
        # A free-space arm/hand timing test 50 mm above the placement plane;
        # this is a commanded offset, not a claim about contact/table height.
        conveyor[2,3]+=.05
    hover=conveyor.copy();hover[2,3]+=config['preplace_height_m']
    seed=catalog['points'][catalog['factory_source_grid']['config']['anchor']]['q_rad']
    tool=catalog['points'][catalog['factory_source_grid']['config']['anchor']]['tool_frames']['F_T_EE']
    state=None
    if initial_capture:
        state=json.loads(Path(initial_capture).read_text())['state'];seed=state['q'];tool=state['F_T_EE']
        if state['robot_mode']!='Idle' or state['current_errors']:raise ValueError('Fresh hover capture must be fault-free Idle')
    model=Kinematics(tool,motion['singularity_avoidance'])
    if state:
        actual=model.fk(seed)
        if np.linalg.norm(actual.translation-np.array(state['O_T_EE'][12:15]))>.0001:raise ValueError('Captured hover/model mismatch')
        if np.linalg.norm(actual.translation[:2]-conveyor[:2,3])>.002:raise ValueError('Captured hover outside conveyor XY tolerance')
        config['preplace_height_m']=float(actual.translation[2]-conveyor[2,3])
        if not .0005<config['preplace_height_m']<.01:raise ValueError('Captured final descent height is invalid')
        # Keep X fixed and move only +Y from the measured hover. Start exactly
        # from measured q, removing the old nominal-IK starting discontinuity.
        conveyor[:2,3]=actual.translation[:2];hover[:3,3]=actual.translation
        initial=np.array(seed);start_rotation=actual.rotation.copy()
    else:
        fit=model.solve(hover.reshape(-1,order='F').tolist(),seed,max_evaluations=250)
        if not fit:raise ValueError('No margin-valid IK for the hover pose')
        initial=np.array(fit['q']);start_rotation=conveyor[:3,:3].copy()
    rotation_delta=pin.log3(start_rotation.T@conveyor[:3,:3])
    if np.linalg.norm(rotation_delta)>.03491:raise ValueError('Hover tool tilt differs by over two degrees')
    q=initial.copy();elbow=q[2]
    u=np.linspace(0,1,10001);d1=30*u*u*(1-u)**2;d2=60*u*(1-u)*(1-2*u);d3=60-360*u+360*u*u
    h=config['preplace_height_m'];speed=v[1]
    acceleration=np.column_stack([speed/ramp*d1,-h/ramp**2*d2]);jerk=np.column_stack([speed/ramp**2*d2,-h/ramp**3*d3])
    jerk_hull=math.hypot(speed/ramp**2*(10/math.sqrt(3)),60*h/ramp**3)
    snap_hull=math.hypot(60*speed/ramp**3,360*h/ramp**4);half_dt=ramp/(2*(len(u)-1))
    cart_accel_bound=float(np.linalg.norm(acceleration,axis=1).max())+jerk_hull*half_dt
    cart_jerk_bound=float(np.linalg.norm(jerk,axis=1).max())+snap_hull*half_dt
    if cart_accel_bound>config['maximum_tracking_acceleration_m_s2'] or cart_jerk_bound>config['maximum_tracking_jerk_m_s3']:
        raise ValueError('Local Cartesian derivative limits exceeded')
    knots=np.linspace(0,duration,491);offsets,velocities=profile(knots,config);qs=[]
    for i,offset in enumerate(offsets):
        target=conveyor.copy();target[:3,3]+=offset
        target[:3,:3]=start_rotation@pin.exp3(rotation_delta*smoother(min(1.,knots[i]/ramp)))
        fit=model.solve_elbow(target.reshape(-1,order='F').tolist(),q,None,elbow)
        if not fit:raise ValueError(f'No continuous IK at {knots[i]:.4f}s')
        q=np.array(fit['q']);qs.append(q)
    # Endpoint derivatives must match the preceding/following stationary phases.
    spline=CubicSpline(knots,np.asarray(qs),axis=0,bc_type=((1,np.zeros(7)),(1,np.zeros(7))))
    bounds=spline_bounds(spline);planner=motion['joint_planner']
    if np.any(bounds['lower']<model.lower-1e-9) or np.any(bounds['upper']>model.upper+1e-9):raise ValueError('Continuous joint bound violated')
    maximum_v=float(bounds['d1'].max());maximum_a=float(bounds['d2'].max());maximum_j=float(bounds['d3'].max())
    if maximum_v>motion['max_joint_velocity_rad_s']*planner['joint_velocity_headroom']:raise ValueError(f'Joint speed too high: {maximum_v:.3f}')
    if maximum_a>planner['max_joint_acceleration_rad_s2']:raise ValueError(f'Joint acceleration too high: {maximum_a:.3f}')
    if maximum_j>config.get('max_joint_jerk_rad_s3',planner['max_joint_jerk_rad_s3']):raise ValueError(f'Joint jerk too high: {maximum_j:.3f}')
    ts=np.linspace(0,duration,10001);offset,velocity=profile(ts,config)
    if offset[-1,1]>config['maximum_forward_stroke_m']-config['commanded_stroke_margin_m']+1e-10 or np.min(np.diff(offset[:,1]))< -1e-10:raise ValueError('Forward stroke bound violated')
    path=conveyor[:3,3]+offset
    lower=np.array(motion['workspace_bounds_m']['min']);upper=np.array(motion['workspace_bounds_m']['max'])
    if np.any(path<lower-1e-9) or np.any(path>upper+1e-9) or np.any(home[:3,3]<lower) or np.any(home[:3,3]>upper+1e-9):raise ValueError('Prepared stroke/home exceeds workspace')
    reach=sum(np.linalg.norm(p.translation) for p in model.model.jointPlacements[1:])+np.linalg.norm(model.model.frames[model.ee].placement.translation)
    lipschitz=math.sqrt(7)*math.sqrt((4*reach/model.policy['characteristic_length_m'])**2+1)
    uncertainty=lipschitz*float(bounds['d1'].sum())*duration/(2*(len(ts)-1))
    minimum_full=minimum_fixed=math.inf;condition=0;largest_sigma=0;position_error=0;rotation_error=0;vy_error=0
    for t,expected,q,dq in zip(ts,path,spline(ts),spline(ts,1)):
        metrics=model.metrics(q);minimum_full=min(minimum_full,metrics['full_sigma_min']);minimum_fixed=min(minimum_fixed,metrics['fixed_elbow_sigma_min'])
        condition=max(condition,metrics['full_condition'],metrics['fixed_elbow_condition'])
        largest_sigma=max(largest_sigma,metrics['full_condition']*metrics['full_sigma_min'],metrics['fixed_elbow_condition']*metrics['fixed_elbow_sigma_min'])
        actual=model.fk(q);position_error=max(position_error,float(np.linalg.norm(actual.translation-expected)))
        rotation_target=start_rotation@pin.exp3(rotation_delta*smoother(min(1.,t/ramp)))
        rotation_error=max(rotation_error,float(np.linalg.norm(pin.log3(actual.rotation.T@rotation_target))))
        if ramp+.005<t<ramp+hold-.005:
            twist=model.jacobian(q)@dq;vy_error=max(vy_error,abs(float(twist[1])-speed))
    if minimum_full-uncertainty<model.policy['min_full_sigma'] or minimum_fixed-uncertainty<model.policy['min_fixed_elbow_sigma']:raise ValueError('Cannot certify the continuous singularity margin')
    condition_bound=(largest_sigma+uncertainty)/(min(minimum_full,minimum_fixed)-uncertainty)
    if condition_bound>model.policy['max_condition'] or position_error>.0001 or rotation_error>.001 or vy_error>.0005:raise ValueError('Joint curve does not reproduce the requested tool motion')
    certificate=dict(continuous_min_full_sigma=minimum_full-uncertainty,continuous_min_fixed_sigma=minimum_fixed-uncertainty,
        jacobian_variation_bound=uncertainty,maximum_sampled_condition=condition,continuous_condition_bound=condition_bound,max_joint_velocity_rad_s=maximum_v,
        max_joint_acceleration_rad_s2=maximum_a,max_joint_jerk_rad_s3=maximum_j,maximum_model_position_error_m=position_error,
        maximum_model_orientation_error_rad=rotation_error,maximum_matched_speed_error_m_s=vy_error,
        continuous_cartesian_acceleration_bound_m_s2=cart_accel_bound,continuous_cartesian_jerk_bound_m_s3=cart_jerk_bound)
    certificate['local_joint_jerk_limit_rad_s3']=config.get('max_joint_jerk_rad_s3',planner['max_joint_jerk_rad_s3'])
    result=dict(schema_version=1,layer=layer,created_utc=datetime.now(timezone.utc).isoformat(),
        hardware_motion_commands_sent=False,tabletop_only=True,config=config,certificate=certificate,
        home_O_T_EE=home.reshape(-1,order='F').tolist(),hover_O_T_EE=hover.reshape(-1,order='F').tolist(),
        reference_conveyor_O_T_EE=reference_conveyor.reshape(-1,order='F').tolist(),initial_capture=str(initial_capture) if initial_capture else None,
        elevated_empty_hand_timing_test=elevated_timing_test,
        conveyor_O_T_EE=conveyor.reshape(-1,order='F').tolist(),stroke_m=float(offset[-1,1]),duration_s=duration,
        release_start_s=ramp,release_deadline_s=ramp+hold,release_estimate_s=estimated_open,
        q0=initial.tolist(),knots_s=knots.tolist(),joint_coefficients=spline.c.transpose(1,2,0).tolist(),
        tool_F_T_EE=tool,fixed_model=fixed_parameters(model),catalog_sha256=hashlib.sha256(catalog_raw).hexdigest(),
        limitations=['No real motion executed','Gripper release duration is estimated and must be measured before enabling this trajectory','Tabletop placeholder: physical belt surface height and conveyor synchronization remain uncommissioned'])
    output.mkdir(parents=True,exist_ok=True);(output/'plan.json').write_text(json.dumps(result,indent=2)+'\n')
    columns=np.column_stack([ts,path,velocity,spline(ts),spline(ts,1)])
    np.savetxt(output/'trajectory.csv',columns,delimiter=',',header='t_s,x_m,y_m,z_m,vx_m_s,vy_m_s,vz_m_s,'+','.join(f'q{i+1}' for i in range(7))+','+','.join(f'dq{i+1}' for i in range(7)),comments='')
    summary={key:result[key] for key in ('layer','stroke_m','duration_s','release_start_s','release_deadline_s','release_estimate_s','certificate','limitations')}
    (output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary,indent=2))
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--layer',type=int,default=5);parser.add_argument('--loaded-width-mm',type=float,default=65.2)
    parser.add_argument('--output',type=Path,default=HERE/'prepared_layer5');parser.add_argument('--initial-capture',type=Path)
    args=parser.parse_args();prepare(args.layer,args.loaded_width_mm/1000,args.output,args.initial_capture)
