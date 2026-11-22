"""Model-derived overhead wave using the full seven-joint Jacobian."""
import copy
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np
import pinocchio as pin
from scipy.interpolate import PPoly

HERE=Path(__file__).resolve().parent;TEACHING=HERE.parent
sys.path.insert(0,str(TEACHING));sys.path.insert(0,str(TEACHING/'factory_cycle'))
from kinematics import Kinematics
from safe_motion import fixed_parameters
from planner import quintic,polynomial_bounds,smooth


class WaveModel(Kinematics):
    def acceptable(self,q):
        m=self.metrics(q);p=self.policy
        # The wave is a seven-joint trajectory, not a fixed-elbow Cartesian
        # chart. A singular six-column submatrix does not make the arm singular.
        return (m['full_sigma_min']>=p['min_full_sigma'] and
            m['full_condition']<=p['max_condition'] and
            m['joint_limit_margin_rad']>=0 and
            m['elbow_extension_margin_rad']>=p['min_elbow_extension_margin_rad'])


def wave_angles(s,cfg):
    s=np.asarray(s);duration=cfg['cycles']*cfg['period_s'];ramp=min(cfg['envelope_ramp_s'],duration/4)/duration
    envelope=smooth(np.clip(s/ramp,0,1))*smooth(np.clip((1-s)/ramp,0,1))
    return math.radians(cfg['commanded_tilt_deg'])*envelope*np.sin(2*math.pi*cfg['cycles']*s)


class Plan:
    def __init__(self,cfg,tool=None):
        self.cfg=copy.deepcopy(cfg);lim=cfg['limits']
        if (cfg['wave_joint_index']!=3 or not 0<cfg['commanded_tilt_deg']<cfg['maximum_measured_tilt_deg']<=35 or
                not isinstance(cfg['cycles'],int) or not 1<=cfg['cycles']<=8 or
                not 4<=cfg['period_s']<=12 or not 0<cfg['envelope_ramp_s']<=cfg['cycles']*cfg['period_s'] or
                not 0<cfg['home_gate_half_width_m']<=.05):raise ValueError('Invalid wave angle, timing or home gate')
        if not (0<lim['joint_velocity_rad_s']<=1 and 0<lim['joint_acceleration_rad_s2']<=2 and
                0<lim['joint_jerk_rad_s3']<=40 and lim['min_full_sigma']>=.15 and lim['max_full_condition']<=35):
            raise ValueError('Invalid wave motion or singularity limits')
        policy=dict(characteristic_length_m=.3,joint_limit_margin_rad=lim['joint_limit_margin_rad'],
            min_full_sigma=lim['min_full_sigma'],min_fixed_elbow_sigma=0.,max_condition=lim['max_full_condition'],
            min_abs_joint2_rad=0.,min_abs_joint5_rad=0.,min_elbow_extension_margin_rad=lim['min_elbow_extension_margin_rad'],
            ik_position_tolerance_m=2e-5,ik_rotation_tolerance_rad=.0002)
        self.model=WaveModel(tool or cfg['tool_F_T_EE'],policy);self.paths=[]
        self.lower=np.array(cfg['workspace_bounds_m']['min']);self.upper=np.array(cfg['workspace_bounds_m']['max'])
        reach=sum(np.linalg.norm(p.translation) for p in self.model.model.jointPlacements[1:])+np.linalg.norm(self.model.model.frames[self.model.ee].placement.translation)
        self.lipschitz=math.sqrt(7)*math.sqrt((4*reach/.3)**2+1)

    def pose(self,q):return self.model.fk(q).homogeneous.reshape(-1,order='F').tolist()

    def add(self,name,qs,minimum_seconds=1.,wave=False):
        qs=np.asarray(qs);curve=quintic(np.linspace(0,1,len(qs)),qs);bounds=polynomial_bounds(curve);lim=self.cfg['limits']
        if np.any(bounds['lower']<self.model.lower) or np.any(bounds['upper']>self.model.upper):raise ValueError(name+': joint bounds')
        seconds=max(minimum_seconds,float(bounds['d1'].max())/(.8*lim['joint_velocity_rad_s']),
            math.sqrt(float(bounds['d2'].max())/lim['joint_acceleration_rad_s2']),
            (float(bounds['d3'].max())/lim['joint_jerk_rad_s3'])**(1/3))
        minimum_full=math.inf;largest=0.;positions=[];max_v=max_w=0.
        neutral_rotation=self.model.fk(self.cfg['upright_q_rad']).rotation if wave else None
        count=max(1000,math.ceil(self.lipschitz*float(bounds['d1'].sum())/.012))
        if count>80000:raise ValueError('Wave certification exceeds bounded budget')
        uncertainty=self.lipschitz*float(bounds['d1'].sum())/(2*count)
        for q,dq in zip(curve(np.linspace(0,1,count+1)),curve.derivative(1)(np.linspace(0,1,count+1))):
            m=self.model.metrics(q);frame=self.model.fk(q);p=frame.translation;positions.append(p.copy())
            minimum_full=min(minimum_full,m['full_sigma_min']);largest=max(largest,m['full_sigma_min']*m['full_condition'])
            if not self.model.acceptable(q) or np.any(p<self.lower) or np.any(p>self.upper):raise ValueError(name+': workspace/joint/full-Jacobian guard')
            if name=='home_5_entry':
                home=np.array(self.cfg['home_5_O_T_EE'][12:15]);half=self.cfg['home_gate_half_width_m']
                if np.any(np.abs(p[:2]-home[:2])>half) or p[2]<home[2]-half:
                    raise ValueError('Entry path leaves the vertical home-5 column')
            v=self.model.jacobian(q)@dq;max_v=max(max_v,np.linalg.norm(v[:3]));max_w=max(max_w,np.linalg.norm(v[3:]))
            if wave:
                orientation=frame.rotation
                if np.linalg.norm(pin.log3(neutral_rotation.T@orientation))>math.radians(35):raise ValueError('Planned wave exceeds 35 degrees from neutral')
                if np.linalg.norm(orientation[:,2]-[1,0,0])>1e-6:raise ValueError('Wave fingers must remain along world +X')
        if minimum_full-uncertainty<lim['min_full_sigma'] or (largest+uncertainty)/(minimum_full-uncertainty)>lim['max_full_condition']:
            raise ValueError(name+': continuous full-Jacobian margin cannot be certified')
        seconds=max(seconds,max_v/lim['translation_velocity_m_s'],max_w/lim['rotation_velocity_rad_s'])*1.05
        if seconds>120:raise ValueError('Wave phase exceeds duration bound')
        coeff=curve.c.transpose(1,2,0).copy()
        for power in range(6):coeff[:,:,power]/=seconds**(5-power)
        path=dict(name=name,wave=wave,duration_s=seconds,knots_s=(curve.x[:-1]*seconds).tolist(),
            coefficients=coeff.tolist(),q0=qs[0].tolist(),q1=qs[-1].tolist(),target=self.pose(qs[-1]),
            certificate=dict(samples=count+1,min_full_sigma=minimum_full-uncertainty,
                max_full_condition=(largest+uncertainty)/(minimum_full-uncertainty),
                maximum_translation_speed_m_s=max_v/seconds,maximum_rotation_speed_rad_s=max_w/seconds,
                xyz_min_m=np.min(positions,axis=0).tolist(),xyz_max_m=np.max(positions,axis=0).tolist()))
        self.paths.append(path);return path

    def approach_home(self,start_q=None):
        home=self.cfg['home_5_O_T_EE'];seed=np.array(self.cfg['seed_q_rad'])
        fit=self.model.solve_elbow(home,seed,None,seed[2])
        if fit is None:raise ValueError('No regular home-5 solution')
        qhome=np.array(fit['q']);start=qhome if start_q is None else np.asarray(start_q)
        first=self.model.fk(start);half=self.cfg['home_gate_half_width_m']
        if (np.any(np.abs(first.translation[:2]-home[12:14])>half) or
                first.translation[2]<home[14]-half or first.translation[2]>self.upper[2]):
            raise ValueError('Start outside home-5 vertical column: XY +/-50 mm, Z from home-50 mm to demo ceiling')
        entry=[start.copy()];q=start.copy();target=self.model.target(home)
        turn=pin.log3(first.rotation.T@target.rotation)
        for s in np.linspace(0,1,101)[1:]:
            u=smooth(s);pose=np.eye(4);pose[:3,3]=(1-u)*first.translation+u*target.translation
            pose[:3,:3]=first.rotation@pin.exp3(u*turn);frame=pose.reshape(-1,order='F').tolist()
            elbow=(1-u)*start[2]+u*qhome[2]
            fit=self.model.solve_elbow(frame,q,None,elbow)
            if fit is None:fit=self.model.solve(frame,q)
            if fit is None:raise ValueError('No continuous regular approach inside home-5 column')
            q=np.array(fit['q']);entry.append(q.copy())
        self.add('home_5_entry',entry);return entry[-1].copy()

    def build(self,start_q=None):
        home=self.cfg['home_5_O_T_EE'];qhome=self.approach_home(start_q)
        lift=[];q=qhome.copy()
        for s in np.linspace(0,1,81):
            pose=home.copy();pose[14]+=(self.cfg['raised_world_z_m']-home[14])*smooth(s)
            fit=self.model.solve_elbow(pose,q,None,qhome[2])
            if fit is None:raise ValueError('No continuous vertical ascent through home 5')
            q=np.array(fit['q']);lift.append(q.copy())
        self.add('vertical_ascent_home_5',lift);qlift=q.copy();upright=np.array(self.cfg['upright_q_rad'])
        neutral=self.model.fk(upright)
        if np.linalg.norm(neutral.rotation[:,2]-[1,0,0])>1e-6:raise ValueError('Configured neutral fingers must point world +X')
        axis=self.model.jacobian(upright)[3:,self.cfg['wave_joint_index']]
        if np.linalg.norm(axis-[1,0,0])>1e-6:raise ValueError('Wave hinge must be world X for a world Y sweep')
        ascent=np.array([(1-smooth(s))*qlift+smooth(s)*upright for s in np.linspace(0,1,101)])
        self.add('straighten_to_upright',ascent)
        waving=np.tile(upright,(601,1));waving[:,3]-=wave_angles(np.linspace(0,1,601),self.cfg)
        waving[[0,-1]]=upright
        self.add('wave_world_y',waving,self.cfg['cycles']*self.cfg['period_s'],wave=True)
        self.add('return_from_upright',ascent[::-1]);self.add('vertical_descent_home_5',lift[::-1])
        return self.manifest()

    def manifest(self):
        return dict(task='isolated_empty_hand_wave',config=self.cfg,tool=self.cfg['tool_F_T_EE'],
            lower_q=self.model.lower.tolist(),upper_q=self.model.upper.tolist(),fixed_model=fixed_parameters(self.model),
            characteristic_length_m=.3,model_sha256=self.model.model_sha256,paths=self.paths,
            planned_duration_s=sum(p['duration_s'] for p in self.paths),model_only=True,physical_motion_verified=False)
