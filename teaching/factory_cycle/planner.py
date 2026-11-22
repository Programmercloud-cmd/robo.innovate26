"""Resident fixed-layout planner. Cache once; small corrections use a local bound."""
import hashlib
import json
import math
from pathlib import Path
import sys
import time

import numpy as np
import pinocchio as pin
from scipy.interpolate import CubicSpline,make_interp_spline,PPoly,BPoly

HERE=Path(__file__).resolve().parent
TEACHING=HERE.parent
sys.path.insert(0,str(TEACHING))
from kinematics import Kinematics
from safe_motion import fixed_parameters,spline_bounds
from move_xyz import vertical_pose,resolve_pose
from setups import resolve_setup


def digest(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':')).encode()).hexdigest()


def smooth(s):
    return s*s*s*(10+s*(-15+6*s))


def placement_profile(t,cfg):
    """Independent smooth Y/Z curves: descend, release at speed, then lift."""
    t=np.asarray(t,dtype=float);ramp=cfg['acceleration_time_s']
    speed=cfg['belt_velocity_base_m_s'][1];stroke=cfg['forward_travel_m']
    duration=stroke/speed+ramp;contact=cfg['descent_time_s']
    release=contact+cfg['contact_settle_time_s'];lift_at=release+cfg['matched_release_time_s']
    if not (0<ramp<contact and lift_at+ramp<duration and .05<=stroke<=.07):
        raise RuntimeError('Invalid curved conveyor phase durations or forward stroke')
    u=np.clip(t/ramp,0,1);w=np.clip((t-(duration-ramp))/ramp,0,1)
    integral=lambda a:2.5*a**4-3*a**5+a**6
    y=speed*(ramp*integral(u)+np.maximum(t-ramp,0)-ramp*integral(w))
    vy=speed*(smooth(u)-smooth(w))
    down=np.clip(t/contact,0,1);up=np.clip((t-lift_at)/(duration-lift_at),0,1)
    h=cfg['tracking_entry_height_m'];clear=cfg['tracking_clearance_height_m']
    z=h*(1-smooth(down))+clear*smooth(up)
    vz=-h/contact*30*down**2*(1-down)**2+clear/(duration-lift_at)*30*up**2*(1-up)**2
    return np.column_stack([np.zeros_like(t),y,z]),np.column_stack([np.zeros_like(t),vy,vz])


def bezier(control,u):
    p=np.array(control,dtype=float,copy=True)
    for n in range(len(p)-1,0,-1):
        p[:n]=(1-u)*p[:n]+u*p[1:n+1]
    return p[0]


def quintic(knots,values):
    """C2 stationary ends, avoiding acceleration jumps into the SDK limiter."""
    values=np.asarray(values);zero=np.zeros(values.shape[1]);coeff=[];breaks=None
    bs=make_interp_spline(knots,values,k=5,bc_type=([(1,zero),(2,zero)],[(1,zero),(2,zero)]))
    for j in range(values.shape[1]):
        p=PPoly.from_spline((bs.t,bs.c[:,j],bs.k));valid=np.diff(p.x)>1e-12
        coeff.append(p.c[:,valid]);breaks=np.r_[p.x[:-1][valid],p.x[1:][valid][-1]]
    return PPoly(np.stack(coeff,axis=-1),breaks)


def polynomial_bounds(p):
    h=BPoly.from_power_basis(p).c
    result=dict(lower=h.min(axis=(0,1)),upper=h.max(axis=(0,1)))
    for d in range(1,4):result['d'+str(d)]=np.abs(BPoly.from_power_basis(p.derivative(d)).c).max(axis=(0,1))
    return result


class Planner:
    def __init__(self,settings,state,setup=None):
        began=time.perf_counter()
        self.settings=settings
        self.motion=json.loads((TEACHING/'motion_config.json').read_text())
        self.factory=json.loads((TEACHING/'factory_vision/config.json').read_text())
        self.moving=json.loads((TEACHING/'moving_conveyor/config.json').read_text())
        self.catalog=json.loads((TEACHING/'positions.json').read_text())
        self.setup=None
        if setup is not None:
            self.settings,self.factory,self.motion,self.setup=resolve_setup(setup,settings,self.factory,self.motion)
            settings=self.settings
            self.moving.update(self.setup['moving_parameters'])
        self.model=Kinematics(state['F_T_EE'],self.motion['singularity_avoidance'])
        self.tool=state['F_T_EE']
        self.home_seed=np.asarray(state['q_d'])
        self.elbow=float(self.home_seed[2])
        self.lower=np.array(self.motion['workspace_bounds_m']['min'])
        self.upper=np.array(self.motion['workspace_bounds_m']['max'])
        self.cache={}
        reach=sum(np.linalg.norm(p.translation) for p in self.model.model.jointPlacements[1:])+np.linalg.norm(self.model.model.frames[self.model.ee].placement.translation)
        self.lipschitz=math.sqrt(7)*math.sqrt((4*reach/self.model.policy['characteristic_length_m'])**2+1)
        self.ignore_last_joint=np.linalg.norm(self.model.model.frames[self.model.ee].placement.translation[:2])<1e-10
        factory_motion={key:self.factory[key] for key in ('destination_mode','conveyor_placement_mode','conveyor_target_O_T_EE')}
        identity_data=dict(spline_schema=4,settings=settings,tool=self.tool,model=self.model.model_sha256,
            policy=self.model.policy,geometry=self.catalog,factory=factory_motion,moving=self.moving,
            motion=self.motion,setup=self.setup)
        self.base_identity=digest(identity_data)
        self.surface_offset=float(self.factory.get('conveyor_surface_height_offset_m',0))
        if not math.isfinite(self.surface_offset) or not 0<=self.surface_offset<=.002:
            raise RuntimeError('Conveyor surface correction exceeds the bounded2mm local correction')
        if self.surface_offset:factory_motion['conveyor_surface_height_offset_m']=self.surface_offset
        self.identity=digest(identity_data)
        self.cache_dir=HERE/'cache'/self.identity
        self.cache_dir.mkdir(parents=True,exist_ok=True)
        self.initialization_s=time.perf_counter()-began

    def manifest(self):
        fixed=fixed_parameters(self.model)
        return dict(commissioning_authorized=False,tool=self.tool,lower_q=self.model.lower.tolist(),upper_q=self.model.upper.tolist(),
            lower_xyz=self.lower.tolist(),upper_xyz=self.upper.tolist(),fixed_model=fixed,
            characteristic_length=self.model.policy['characteristic_length_m'],
            settings=self.settings,gripper=self.motion['gripper'],
            min_full=self.model.policy['min_full_sigma'],min_fixed=self.model.policy['min_fixed_elbow_sigma'],
            max_condition=self.model.policy['max_condition'],
            identity=self.identity,model_sha256=self.model.model_sha256,
            release_speed_m_s=self.moving['release_speed_m_s'],
            minimum_release_vy_m_s=self.moving['minimum_release_vy_m_s'],maximum_release_vy_m_s=self.moving['maximum_release_vy_m_s'],
            conveyor_speed_m_s=self.moving['belt_velocity_base_m_s'][1])

    def solve(self,pose,seed):
        fit=self.model.solve_elbow(pose,seed,None,self.elbow)
        if fit is None:
            raise RuntimeError('No regular fixed-layout IK solution')
        return np.asarray(fit['q'])

    def frame(self,label):
        _,point,_=resolve_pose(self.catalog,label)
        return vertical_pose(point['O_T_EE'])

    def placement(self,layer):
        p=self.factory['conveyor_target_O_T_EE'].copy()
        p[14]+=self.surface_offset
        grid=self.catalog['factory_source_grid']
        step=np.array(grid.get('conveyor_row_step_in_base_m',grid['config']['row_step_in_base_m']))
        p[12:15]=(np.array(p[12:15])+(layer-self.moving['conveyor_reference_layer'])*step).tolist()
        # Keep every conveyor destination on the same X as placement0.
        p[12]=self.factory['conveyor_target_O_T_EE'][12]
        # Conveyor placement is directly beside the taught row, rather than
        # using the old nominal78.2mm pitch. Home shares this exact Y.
        p[13]=self.frame(f'{layer}_0')[13]
        return p

    def home(self,layer):
        p=self.placement(layer)
        # Use the absolute post-grasp lift height. Conveyor surface correction
        # belongs to placement only; it must not raise this transfer plane.
        p[14]=(self.frame(f'{layer}_0')[14]+self.settings['observation_height_m']
               +self.settings.get('home_z_offset_m',0.))
        return p

    def path(self,name,start,target,seed,*,departure=0,arrival=0):
        """One time parameterization; no distance floor or double smootherstep."""
        started=time.perf_counter()
        key=digest(dict(name=name,start=start,target=target,seed=np.round(seed,9).tolist(),departure=departure,arrival=arrival))
        if key in self.cache:return self.cache[key]
        pathfile=self.cache_dir/(key+'.json')
        if pathfile.exists():
            result=json.loads(pathfile.read_text());self.cache[key]=result;return result
        a=self.model.target(start);b=self.model.target(target)
        control=np.array([a.translation,a.translation+[0,0,departure],a.translation+[0,0,departure],
                          b.translation+[0,0,arrival],b.translation+[0,0,arrival],b.translation])
        if np.any(control<self.lower-1e-9) or np.any(control>self.upper+1e-9):
            raise RuntimeError('Fixed path leaves the taught workspace')
        rotation=pin.log3(a.rotation.T@b.rotation)
        knots=np.linspace(0,1,65);q=np.array(seed);qs=[];poses=[]
        for s in knots:
            u=smooth(s);p=np.eye(4);p[:3,:3]=a.rotation@pin.exp3(rotation*u)
            p[:3,3]=(1-u)*a.translation+u*b.translation if departure==arrival==0 else bezier(control,u)
            q=self.solve(p.reshape(-1,order='F').tolist(),q);qs.append(q.copy());poses.append(p[:3,3])
        curve=quintic(knots,qs)
        bound=polynomial_bounds(curve);cfg=self.settings
        position=CubicSpline(knots,np.array(poses),axis=0,bc_type=((1,np.zeros(3)),(1,np.zeros(3))))
        xyz=spline_bounds(position)
        times=[float(bound['d1'].max())/(cfg['joint_velocity_rad_s']*.85),
               math.sqrt(float(bound['d2'].max())/cfg['joint_acceleration_rad_s2']),
               (float(bound['d3'].max())/cfg['joint_jerk_rad_s3'])**(1/3),
               float(np.linalg.norm(xyz['d1']))/cfg['translation_velocity_m_s'],
               math.sqrt(float(np.linalg.norm(xyz['d2']))/cfg['translation_acceleration_m_s2']),
               (float(np.linalg.norm(xyz['d3']))/cfg['translation_jerk_m_s3'])**(1/3),
               1.875*float(np.linalg.norm(rotation))/.3]
        duration=max(.04,max(times)*1.04)
        # A Lipschitz bound covers all intervals, not just sampled postures.
        count=max(128,math.ceil(self.lipschitz*float(bound['d1'].sum())/.025))
        uncertainty=self.lipschitz*float(bound['d1'].sum())/(2*count)
        min_full=min_fixed=math.inf;max_condition=0
        for q in curve(np.linspace(0,1,count+1)):
            metrics=self.model.metrics(q);min_full=min(min_full,metrics['full_sigma_min']);min_fixed=min(min_fixed,metrics['fixed_elbow_sigma_min']);max_condition=max(max_condition,metrics['full_condition'],metrics['fixed_elbow_condition'])
        if min_full-uncertainty<self.model.policy['min_full_sigma'] or min_fixed-uncertainty<self.model.policy['min_fixed_elbow_sigma']:
            raise RuntimeError('Cannot certify cached path singularity margin')
        if np.any(bound['lower']<self.model.lower) or np.any(bound['upper']>self.model.upper):raise RuntimeError('Cached curve joint bound')
        samples=curve(np.linspace(0,1,257));expected=np.array([(1-smooth(s))*a.translation+smooth(s)*b.translation if departure==arrival==0 else bezier(control,smooth(s)) for s in np.linspace(0,1,257)])
        error=max(np.linalg.norm(self.model.fk(q).translation-p) for q,p in zip(samples,expected))
        if error>.0005:raise RuntimeError('Cached spline leaves fixed Cartesian path')
        coeff=curve.c.transpose(1,2,0).copy()
        for power in range(6):coeff[:,:,power]/=duration**(5-power)
        result=dict(name=name,key=key,kind='path',duration_s=duration,knots_s=(curve.x[:-1]*duration).tolist(),departure=departure,arrival=arrival,
            coefficients=coeff.tolist(),q0=np.asarray(qs[0]).tolist(),q1=np.asarray(qs[-1]).tolist(),
            cartesian_derivative_bounds=[float(np.linalg.norm(xyz['d'+str(d)]))/duration**d*1.02 for d in range(1,4)],
            start=start,target=target,certificate=dict(min_full=min_full-uncertainty,min_fixed=min_fixed-uncertainty,
                maximum_sampled_condition=max_condition,cartesian_error_m=error,intervals=count,
                q_speed=float(bound['d1'].max())/duration,q_acc=float(bound['d2'].max())/duration**2,
                q_jerk=float(bound['d3'].max())/duration**3),initialization_s=time.perf_counter()-started)
        pathfile.write_text(json.dumps(result)+'\n');self.cache[key]=result;return result

    def slower(self,path,scale):
        """Reuse the certified curve; stretch time without increasing derivatives."""
        if not math.isfinite(scale) or scale<1 or path.get('tracking'):raise RuntimeError('Only non-tracking paths may be slowed')
        if scale==1:return path
        result=dict(path);result['duration_s']=path['duration_s']*scale
        if result['duration_s']>10:raise RuntimeError('Scheduled path exceeds controller path-duration bound')
        result['knots_s']=(np.array(path['knots_s'])*scale).tolist()
        c=np.asarray(path['coefficients']).copy()
        for power in range(6):c[:,:,power]/=scale**(5-power)
        result['coefficients']=c.tolist();result['time_scale']=scale
        result['certificate']=dict(path['certificate'])
        for d,key in enumerate(('q_speed','q_acc','q_jerk'),1):
            if key in result['certificate']:result['certificate'][key]/=scale**d
        if 'cartesian_derivative_bounds' in path:result['cartesian_derivative_bounds']=[v/scale**d for d,v in enumerate(path['cartesian_derivative_bounds'],1)]
        result['key']=digest(dict(parent=path.get('key'),duration_s=result['duration_s']))
        return result

    def preflight_path(self,path):
        """Mirror native load-time checks offline, cached by full path content."""
        marker=self.cache_dir/('preflight_'+digest(path)+'.json')
        if marker.exists():return json.loads(marker.read_text())
        duration=path['duration_s'];knots=np.asarray(path['knots_s']);coeff=np.asarray(path['coefficients'])
        if (not math.isfinite(duration) or not 0<duration<=10 or
                knots.ndim!=1 or not 0<len(knots)<=1024 or knots[0]!=0 or
                not np.isfinite(knots).all() or np.any(np.diff(knots)<=0) or knots[-1]>=duration or
                coeff.shape!=(len(knots),7,6) or not np.isfinite(coeff).all()):
            raise RuntimeError('Invalid path dimensions, coefficients or duration')
        curve=PPoly(coeff.transpose(2,0,1),np.r_[knots,duration])
        times=np.linspace(0,duration,1001);qs=curve(times)
        if np.any(qs<self.model.lower) or np.any(qs>self.model.upper):raise RuntimeError('Preflight joint bound')
        for d,key,headroom in ((1,'joint_velocity_rad_s',.86),(2,'joint_acceleration_rad_s2',1.001),(3,'joint_jerk_rad_s3',1.001)):
            if np.max(np.abs(curve.derivative(d)(times)))>self.settings[key]*headroom:
                raise RuntimeError('Preflight path derivative limit')
        for q in qs:
            if not self.model.acceptable(q):raise RuntimeError('Preflight joint/singularity protection')
            xyz=self.model.fk(q).translation
            if np.any(xyz<self.lower-1e-9) or np.any(xyz>self.upper+1e-9):raise RuntimeError('Preflight path workspace')
        if (np.linalg.norm(self.model.fk(path['q1']).translation-np.asarray(path['target'][12:15]))>.0001 or
                np.max(np.abs(qs[0]-path['q0']))>1e-7 or np.max(np.abs(qs[-1]-path['q1']))>1e-7):
            raise RuntimeError('Preflight endpoint model mismatch')
        release_speed_error=None
        if path.get('tracking'):
            origin=np.asarray(path['tracking_origin']);cfg=self.moving
            release_at=cfg['descent_time_s']+cfg['contact_settle_time_s']
            deadline=release_at+cfg['matched_release_time_s']
            if (origin.shape!=(3,) or not np.isfinite(origin).all() or
                    abs(path['release_at']-release_at)>1e-9 or
                    abs(path['release_deadline']-deadline)>1e-9 or
                    not 0<release_at<deadline<duration or
                    abs(duration-(cfg['forward_travel_m']/cfg['belt_velocity_base_m_s'][1]+cfg['acceleration_time_s']))>1e-9):
                raise RuntimeError('Preflight tracking release timing/origin')
            expected,_=placement_profile(times,cfg)
            release_speed_error=0.
            for t,q,dq,p in zip(times,qs,curve.derivative(1)(times),expected):
                if np.linalg.norm(self.model.fk(q).translation-origin-p)>.0001:
                    raise RuntimeError('Preflight tracking curve/model mismatch')
                if release_at<=t<=deadline:
                    vy=float((self.model.jacobian(q)@dq)[1])
                    release_speed_error=max(release_speed_error,abs(vy-cfg['belt_velocity_base_m_s'][1]))
            if release_speed_error>.0005:raise RuntimeError('Preflight tracking release speed')
            if not .05<=expected[-1,1]<cfg['maximum_forward_stroke_m']-cfg['commanded_stroke_margin_m']:
                raise RuntimeError('Preflight tracking stroke')
        result=dict(completed=True,name=path['name'],samples=1001,duration_s=duration,
                    tracking=bool(path.get('tracking')),maximum_release_speed_error_m_s=release_speed_error,
                    native_load_checks_still_required=True)
        marker.write_text(json.dumps(result)+'\n');return result

    def local_target(self,q,pose,delta):
        """Resident IK plus two endpoint metrics and one analytic small-tube bound."""
        p=self.model.target(pose);p.translation+=p.rotation@np.array([delta[0],delta[1],0.])
        p.rotation=pin.exp3(np.array([0.,0.,delta[2]]))@p.rotation
        target=self.solve(p.homogeneous.reshape(-1,order='F').tolist(),q)
        first=self.model.metrics(q);last=self.model.metrics(target)
        difference=np.abs(target-np.array(q))
        if self.ignore_last_joint:difference[-1]=0
        uncertainty=self.lipschitz*float(difference.sum())/2
        if min(first['full_sigma_min'],last['full_sigma_min'])-uncertainty<self.model.policy['min_full_sigma'] or min(first['fixed_elbow_sigma_min'],last['fixed_elbow_sigma_min'])-uncertainty<self.model.policy['min_fixed_elbow_sigma']:
            # Subdivide this tiny stationary-to-stationary joint segment. The
            # same Lipschitz bound then covers small intervals instead of
            # pessimistically applying the entire correction to both endpoints.
            count=max(8,math.ceil(self.lipschitz*float(difference.sum())/.02))
            if count>512:raise RuntimeError('Visual correction exceeds bounded local certification budget')
            margin=self.lipschitz*float(difference.sum())/(2*count)
            low_full=low_fixed=math.inf;high_sigma=0
            for u in np.linspace(0,1,count+1):
                m=self.model.metrics((1-u)*np.array(q)+u*target)
                low_full=min(low_full,m['full_sigma_min']);low_fixed=min(low_fixed,m['fixed_elbow_sigma_min'])
                high_sigma=max(high_sigma,m['full_condition']*m['full_sigma_min'],m['fixed_elbow_condition']*m['fixed_elbow_sigma_min'])
            if (low_full-margin<self.model.policy['min_full_sigma'] or low_fixed-margin<self.model.policy['min_fixed_elbow_sigma'] or
                    (high_sigma+margin)/(min(low_full,low_fixed)-margin)>self.model.policy['max_condition']):
                raise RuntimeError('Visual correction fails subdivided continuous singularity checks')
        return target,p.homogeneous.reshape(-1,order='F').tolist()

    def tracking(self,hover,q0):
        key=digest(dict(kind='tracking',hover=hover,q0=np.asarray(q0).tolist()))
        pathfile=self.cache_dir/(key+'.json')
        if pathfile.exists():return json.loads(pathfile.read_text())
        cfg=self.moving;duration=cfg['forward_travel_m']/cfg['belt_velocity_base_m_s'][1]+cfg['acceleration_time_s']
        release_at=cfg['descent_time_s']+cfg['contact_settle_time_s'];release_deadline=release_at+cfg['matched_release_time_s']
        conveyor=hover.copy();conveyor[14]-=cfg['tracking_entry_height_m']
        lift_time=duration-release_deadline;ramp=cfg['acceleration_time_s'];speed=cfg['belt_velocity_base_m_s'][1]
        zv=1.875*max(cfg['tracking_entry_height_m']/cfg['descent_time_s'],cfg['tracking_clearance_height_m']/lift_time)
        za=(10/math.sqrt(3))*max(cfg['tracking_entry_height_m']/cfg['descent_time_s']**2,cfg['tracking_clearance_height_m']/lift_time**2)
        zj=60*max(cfg['tracking_entry_height_m']/cfg['descent_time_s']**3,cfg['tracking_clearance_height_m']/lift_time**3)
        cart_bounds=[math.hypot(speed,zv),math.hypot(1.875*speed/ramp,za),math.hypot((10/math.sqrt(3))*speed/ramp**2,zj)]
        limits=[self.settings['translation_velocity_m_s'],self.settings['translation_acceleration_m_s2'],self.settings['translation_jerk_m_s3']]
        if any(a>b for a,b in zip(cart_bounds,limits)):raise RuntimeError('Curved conveyor Cartesian derivative bound')
        boundaries=[0,cfg['acceleration_time_s'],cfg['descent_time_s'],release_at,release_deadline,duration-cfg['acceleration_time_s'],duration]
        knots=np.unique(np.r_[np.linspace(0,duration,301),boundaries]);offset,velocity=placement_profile(knots,cfg)
        world=np.array(conveyor[12:15])+offset
        if np.any(world<self.lower-1e-9) or np.any(world>self.upper+1e-9):raise RuntimeError('Curved conveyor path workspace')
        qs=[];q=np.array(q0)
        for p in offset:
            target=conveyor.copy();target[12:15]=(np.array(conveyor[12:15])+p).tolist()
            q=self.solve(target,q);qs.append(q.copy())
        curve=quintic(knots,qs);bounds=polynomial_bounds(curve)
        for derivative,limit in ((1,self.settings['joint_velocity_rad_s']*.85),
                (2,self.settings['joint_acceleration_rad_s2']),(3,self.settings['joint_jerk_rad_s3'])):
            if bounds['d'+str(derivative)].max()>limit:raise RuntimeError('Tracking joint derivative exceeds configured bound')
        if np.any(bounds['lower']<self.model.lower) or np.any(bounds['upper']>self.model.upper):raise RuntimeError('Tracking joint bounds')
        count=max(500,math.ceil(self.lipschitz*float(bounds['d1'].sum())*duration/.025))
        uncertainty=self.lipschitz*float(bounds['d1'].sum())*duration/(2*count)
        minimum_full=minimum_fixed=math.inf;largest=0;error=0;speed_error=0
        times=np.linspace(0,duration,count+1);expected,_=placement_profile(times,cfg)
        for t,p,q,dq in zip(times,expected,curve(times),curve(times,1)):
            m=self.model.metrics(q);minimum_full=min(minimum_full,m['full_sigma_min']);minimum_fixed=min(minimum_fixed,m['fixed_elbow_sigma_min'])
            largest=max(largest,m['full_condition']*m['full_sigma_min'],m['fixed_elbow_condition']*m['fixed_elbow_sigma_min'])
            fk=self.model.fk(q);error=max(error,float(np.linalg.norm(fk.translation-np.array(conveyor[12:15])-p)))
            if release_at<=t<=release_deadline:
                speed_error=max(speed_error,abs(float((self.model.jacobian(q)@dq)[1])-cfg['belt_velocity_base_m_s'][1]))
        if (minimum_full-uncertainty<self.model.policy['min_full_sigma'] or minimum_fixed-uncertainty<self.model.policy['min_fixed_elbow_sigma']
                or (largest+uncertainty)/(min(minimum_full,minimum_fixed)-uncertainty)>self.model.policy['max_condition']
                or error>.0001 or speed_error>.0005):raise RuntimeError('Tracking path certificate failed')
        stroke=float(offset[-1,1])
        if not 0<stroke<cfg['maximum_forward_stroke_m']-cfg['commanded_stroke_margin_m']:raise RuntimeError('Conveyor stroke bound')
        target=conveyor.copy();target[13]+=stroke;target[14]+=cfg['tracking_clearance_height_m']
        result=dict(name='speed_matched_release',key=key,kind='path',tracking=True,duration_s=duration,
                    release_at=release_at,release_deadline=release_deadline,contact_at=cfg['descent_time_s'],
                    tracking_origin=conveyor[12:15],knots_s=curve.x[:-1].tolist(),coefficients=curve.c.transpose(1,2,0).tolist(),
                    q0=qs[0].tolist(),q1=qs[-1].tolist(),target=target,
                    certificate=dict(min_full=minimum_full-uncertainty,min_fixed=minimum_fixed-uncertainty,
                        cartesian_derivative_bounds=cart_bounds,
                        maximum_model_error_m=error,maximum_speed_error_m_s=speed_error,stroke_m=stroke))
        pathfile.write_text(json.dumps(result)+'\n');return result

    def patch(self,base,start,target,q0):
        """Patch a cached leg in a certified joint tube, retaining C2 endpoints."""
        T=base['duration_s'];q1=self.solve(target,base['q1'])
        d0=np.array(q0)-base['q0'];d1=q1-base['q1'];delta=d1-d0
        diff=np.maximum(np.abs(d0),np.abs(d1))
        if self.ignore_last_joint:diff[-1]=0
        uncertainty=self.lipschitz*float(diff.sum())
        certificate=base['certificate']
        if min(certificate['min_full']-uncertainty-self.model.policy['min_full_sigma'],
               certificate['min_fixed']-uncertainty-self.model.policy['min_fixed_elbow_sigma'])<0:
            # A materially larger posture change receives a new full certificate;
            # normal tiny corrections use only the cached tube below.
            return self.path(base['name']+'_large_patch',start,target,q0,
                             departure=base.get('departure',0),arrival=base.get('arrival',0))
        coefficients=np.asarray(base['coefficients']).copy()
        global_power=np.array([d0,np.zeros(7),np.zeros(7),10*delta/T**3,-15*delta/T**4,6*delta/T**5])
        for i,t0 in enumerate(base['knots_s']):
            for degree in range(6):
                local=sum(global_power[k]*math.comb(k,degree)*t0**(k-degree) for k in range(degree,6))
                coefficients[i,:,5-degree]+=local
        p=PPoly(coefficients.transpose(2,0,1),np.r_[base['knots_s'],T]);b=polynomial_bounds(p)
        if np.any(b['lower']<self.model.lower) or np.any(b['upper']>self.model.upper):raise RuntimeError('Patched path joint bounds')
        factor=max(1,float(b['d1'].max())/(self.settings['joint_velocity_rad_s']*.85),
            math.sqrt(float(b['d2'].max())/self.settings['joint_acceleration_rad_s2']),
            (float(b['d3'].max())/self.settings['joint_jerk_rad_s3'])**(1/3))
        # Conservative serial-chain perturbation bounds preserve the Cartesian
        # speed/acceleration/jerk headroom when patching tiny joint offsets.
        original=PPoly(np.asarray(base['coefficients']).transpose(2,0,1),np.r_[base['knots_s'],T])
        ob=polynomial_bounds(original);V,A,J=[float(ob['d'+str(d)].sum()) for d in (1,2,3)]
        D=float(np.maximum(np.abs(d0),np.abs(d1)).sum());change=float(np.abs(delta).sum())
        dv,da,dj=1.875*change/T,10/math.sqrt(3)*change/T**2,60*change/T**3
        reach=sum(np.linalg.norm(v.translation) for v in self.model.model.jointPlacements[1:])+np.linalg.norm(self.model.model.frames[self.model.ee].placement.translation)
        perturb=[reach*(2*D*V+dv),reach*(D*(2*A+4*V*V)+4*V*dv+da),
                 reach*(D*(2*J+12*V*A+8*V**3)+6*dv*(A+2*V*V)+6*V*da+dj)]
        limits=[self.settings['translation_velocity_m_s'],self.settings['translation_acceleration_m_s2'],self.settings['translation_jerk_m_s3']]
        factor=max(factor,*[((a+e)/limit)**(1/d) for d,(a,e,limit) in enumerate(zip(base['cartesian_derivative_bounds'],perturb,limits),1)])
        # Check Cartesian bounds and endpoint orientation on the patched curve.
        for q in p(np.linspace(0,T,129)):
            fk=self.model.fk(q)
            if np.any(fk.translation<self.lower-1e-9) or np.any(fk.translation>self.upper+1e-9):raise RuntimeError('Patched path workspace')
            if np.linalg.norm(fk.rotation[:,2]-np.array([0,0,-1]))>.005:raise RuntimeError('Patched tool tilt')
        for degree in range(6):coefficients[:,:,degree]/=factor**(5-degree)
        result=dict(base);result.update(coefficients=coefficients.tolist(),duration_s=T*factor,
            knots_s=(np.array(base['knots_s'])*factor).tolist(),start=start,target=target,q0=list(q0),q1=q1.tolist(),
            certificate=dict(certificate,min_full=certificate['min_full']-uncertainty,
                min_fixed=certificate['min_fixed']-uncertainty,patch_tube_joint_sum_rad=float(diff.sum()),
                patch_jacobian_bound=uncertainty))
        return result
