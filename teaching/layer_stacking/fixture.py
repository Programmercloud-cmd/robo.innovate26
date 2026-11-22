"""Synthetic pixel/wrench fixture for software development, NOT contact physics."""
import math
import cv2
import numpy as np
from .pipeline import RecoverableStop
from .vision import feature,Uncertain,reference_key


class Fixture:
    domain='fixture'
    def __init__(self,config,fail_vision_frames=(),jam=False,bad_release=False):
        self.config=config;self.p=dict(config['development_fixture']);self.t=0.;self.seq=0
        self.pose=np.array([0.,0.,config['contact']['alignment_height_m'],0.]);self.width=.07;self.closed=False;self.jam=jam;self.bad_release=bad_release
        self.fail_vision=set(fail_vision_frames);self.error=np.array(self.p['position_error_xy_yaw']);self.guard=False
        self.references={};self.trajectory=[];self.current_layer=None

    def now(self):return self.t
    def wait(self,dt):self.t+=dt
    def preflight(self,plan):
        if any(len(layer['bricks'])!=12 for layer in plan['layers']):raise RecoverableStop('Not a complete layer')
    def source_present(self,layer):self.wait(.1)
    def verify_departed_source(self,layer):self.wait(.1)
    def motion(self,name,layer):
        self.current_layer=layer
        if name=='transfer':self.pose=np.array([0.,0.,self.config['contact']['alignment_height_m'],0.]);self.error=np.array(self.p['position_error_xy_yaw'])
        if name=='retreat':self.guard=False
        self.trajectory.append(dict(t=self.t,stage=name,layer=layer['index'],source=layer['source'],local_pose=self.pose.tolist()))
        self.wait(.2)
    def hand(self,opening):self.width=.07 if opening else .0638;self.closed=not opening;self.wait(.3)
    def lift_verify(self,layer):
        if not self.closed:raise RecoverableStop('Fixture empty pick')
        self.motion('lift',layer)
    def contact_parameters(self):return self.p
    def arm_contact_guard(self):self.guard=True
    def force(self):
        error=self.error+self.pose[[0,1,3]];z=self.pose[2]
        if not self.closed or z>self.p['contact_reference_z_m']:return 0.
        if self.jam or np.linalg.norm(error[:2])>.00025 or abs(error[2])>.002:
            return max(0.,self.p['stiffness_N_m']*(self.p['contact_reference_z_m']-z))
        return .5+self.p['stiffness_N_m']*max(0.,.0002-z)
    def state(self):
        return dict(stamp_s=self.t,valid=True,fault=False,grasp_valid=self.closed if self.width<.069 else True,
            local_pose=self.pose.tolist(),wrench=[0.,0.,self.force(),0.,0.,0.],width_m=self.width)
    def move_local(self,pose):self.pose=np.array(pose,dtype=float);self.wait(.1)
    def draw(self,error):
        image=np.full((480,640,3),35,np.uint8);J=np.diag([-6000.,-6000.,1.]);shift=J@error
        corner=np.array([310.,245.])+shift[:2];angle=shift[2]
        u=np.array([math.cos(angle),math.sin(angle)]);v=np.array([-math.sin(angle),math.cos(angle)])
        def line(a,b,color):cv2.line(image,tuple(np.round(a*16).astype(int)),tuple(np.round(b*16).astype(int)),color,2,cv2.LINE_AA,shift=4)
        line(corner,corner+130*u,(220,220,220));line(corner,corner+105*v,(220,220,220))
        line(np.array([110,90]),np.array([230,90]),(200,180,160));line(np.array([110,90]),np.array([110,190]),(200,180,160))
        return image
    def frame(self,layer,stage):
        self.seq+=1;self.wait(.03);key=reference_key(layer,stage)
        if key not in self.references:
            reference=dict(edges=dict(receiver_u=[[320,245],[430,245]],receiver_v=[[310,255],[310,340]],
                held_u=[[120,90],[220,90]],held_v=[[110,100],[110,180]]),image_jacobian=np.diag([-6000.,-6000.,1.]).tolist(),angle_feature_scale_px=150.)
            f,_=feature(self.draw(np.zeros(3)),reference,self.config['vision']);reference['target_feature']=f.tolist();self.references[key]=reference
        error=self.error+self.pose[[0,1,3]] if stage=='align' else (np.array([.001,0.,0.]) if self.bad_release else np.zeros(3))
        image=self.draw(error)
        if self.seq in self.fail_vision:image[:]=35
        return dict(sequence=self.seq,stamp_s=self.t,image=image),self.references[key]
    def unload(self):self.pose[2]+=self.config['contact']['unload_lift_m'];self.wait(.2);self.guard=False
    def verify(self,layer,delta,proof):
        if np.linalg.norm(delta[:2])>self.config['vision']['position_tolerance_m'] or abs(delta[2])>self.config['vision']['yaw_tolerance_rad']:
            raise Uncertain('Post-release fixture geometry did not verify')
    def stop(self):self.guard=False
    def close(self):pass
