"""Fixed configured virtual RGB-D, unchanged R3 and conservative local checks."""
import hashlib,json,math
from pathlib import Path
import sys
import numpy as np
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]
sys.path.insert(0,str(ROOT/'experiments/stage1_20261025'))
from experiments.stage1_20261025 import online
from experiments.stage1_20261025.calibration import build
from experiments.stage2_exposed_tip_20261025.model import save

class Sensors:
    def __init__(self,sim,scene,geometry,cameras,output,protocol):
        import torch
        from pxr import Gf,UsdGeom,UsdLux
        from isaaclab.sim import get_current_stage
        from experiments.final_workcell_20261024 import render_core
        self.sim,self.scene,self.output,self.protocol=sim,scene,output,protocol
        self.camera=scene['overview'];self.index=0;self.config=json.loads(cameras.read_text())
        self.calibration=build(geometry,cameras);nominal=self.calibration['views']['camera_0']
        self.cal={'K':np.asarray(self.calibration['K']), 'R':np.asarray(nominal['R_world_to_camera']),'t':np.asarray(nominal['t_world_to_camera_m'])}
        view=next(v for v in self.config['views'] if v['id']=='diagnostic_table_top')
        center=np.asarray(nominal['eye_world_m'])-view['eye_offset_m'];pose=render_core.camera_pose(center,view)
        self.camera.set_world_poses(positions=torch.tensor([pose['eye_world_m']],dtype=torch.float32),orientations=torch.tensor([pose['quaternion_wxyz_opengl']],dtype=torch.float32),convention='opengl');self.camera.reset()
        stage=get_current_stage();light=self.config['lighting'];UsdLux.DomeLight.Get(stage,'/World/Light').GetIntensityAttr().Set(light['dome_intensity'])
        key=UsdLux.RectLight.Define(stage,'/World/ExperimentKey');key.CreateWidthAttr(light['key_width_m']);key.CreateHeightAttr(light['key_height_m']);key.CreateNormalizeAttr(light['key_normalize']);key.CreateColorAttr(Gf.Vec3f(*light['key_color_linear']));key.CreateIntensityAttr(light['key_intensity'])
        lp=render_core.camera_pose((0,0,0),{'eye_offset_m':light['key_position_table_m'],'target_offset_m':light['key_target_table_m'],'up_world':(0,0,1)})
        xf=UsdGeom.Xformable(key.GetPrim());xf.AddTranslateOp().Set(Gf.Vec3d(*lp['eye_world_m']));q=lp['quaternion_wxyz_opengl'];xf.AddOrientOp().Set(Gf.Quatf(q[0],Gf.Vec3f(*q[1:])))
        for _ in range(self.config['render_warmup_frames']):sim.render()
        upstream=ROOT/'experiments/stage1_20261025/upstream'
        for name,digest in protocol['perception']['upstream_sha256'].items():
            if hashlib.sha256((upstream/name).read_bytes()).hexdigest()!=digest:raise RuntimeError('Frozen R3 changed:'+name)
        self.method=online.load('pick_R3',upstream/'method.py');self.depth_module=online.load('pick_depth',upstream/'depth_module.py');self.Evidence=online.load('pick_evidence',upstream/'evidence.py').Evidence
        self.method_config=json.loads((upstream/'config.json').read_text());self.depth_config=json.loads((upstream/'depth_config.json').read_text());self.depth_config['source_roi_world_xy_m']={'x':[-.128,.128],'y':[-.128,.128]}
        self.workspace=json.loads((ROOT/protocol['perception']['workspace']).read_text())
        save(output/'online/calibration.json',self.calibration)

    def capture(self):
        import cv2
        clock=float(self.sim.current_time);self.sim.render();self.camera.update(0.,force_recompute=True)
        if float(self.sim.current_time)!=clock:raise RuntimeError('Render-only camera advanced physics')
        rgb=self.camera.data.output['rgb'][0,:,:,:3].cpu().numpy().copy();depth=self.camera.data.output['distance_to_image_plane'][0,:,:,0].cpu().numpy().copy()
        folder=self.output/'online/observations'/f'frame_{self.index:04d}';folder.mkdir(parents=True);self.index+=1
        cv2.imwrite(str(folder/'rgb.png'),cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR));np.savez_compressed(folder/'depth.npz',depth=depth)
        return rgb,depth,{'frame_id':folder.name,'measurement_time_s':clock,'rgb':str((folder/'rgb.png').relative_to(self.output)),'depth':str((folder/'depth.npz').relative_to(self.output)),'depth_array_key':'depth'}

    def cloud(self,depth):
        valid=np.isfinite(depth)&(depth>0);y,x=np.nonzero(valid);K=self.cal['K'];rays=np.c_[(x-K[0,2])/K[0,0],(y-K[1,2])/K[1,1],np.ones(len(x))]
        return (depth[valid,None]*rays-self.cal['t'])@self.cal['R']

    def select(self,current_tcp):
        import cv2
        rgb,depth,frame=self.capture();filtered,_=online.filter_workspace_depth(depth,self.cal,self.workspace)
        cv2.setRNGSeed(26);result,_,_=self.method.analyze(filtered,cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR),self.cal,self.depth_module,self.depth_config,self.method_config,self.Evidence,'camera_0','observed')
        save(self.output/'online/r3_initial.json',result);candidates=[online.candidate_contract(c) for c in result['candidates'] if c['decision']=='present']
        candidates.sort(key=lambda c:(float(np.linalg.norm(np.asarray(c['table_xy_m'])-current_tcp[:2])),c['candidate_id']))
        self.initial_depth=depth;self.initial_cloud=self.cloud(filtered);self.frame=frame
        return candidates,frame

    def local_height(self,candidate):
        xy=np.asarray(candidate['table_xy_m']);yaw=candidate['yaw_table_rad'];c,s=np.cos(yaw),np.sin(yaw);axes=np.array([[c,s],[-s,c]])
        cloud=self.initial_cloud;local=(cloud[:,:2]-xy)@axes.T
        selected=cloud[(abs(local[:,0])<=.014)&(abs(local[:,1])<=.006)&(abs(cloud[:,2]-candidate['estimated_body_top_from_selected_band_m'])<=.003)]
        support={'supported':False,'count':len(selected),'method':'Fixed eroded2x4 interior upper-depth quantiles; engineering support, not calibrated covariance'}
        if len(selected)<64:return None,support
        upper,p99=np.quantile(selected[:,2],[.9,.99]);upper_count=int(np.count_nonzero(abs(selected[:,2]-upper)<=.0005))
        support.update(upper_90_m=float(upper),upper_99_m=float(p99),upper_support_count=upper_count,spread_m=float(p99-upper))
        support['supported']=bool(p99-upper<=self.protocol['engineering_envelopes']['local_upper_depth_Z_m'] and upper_count>=32)
        return float(upper) if support['supported'] else None,support

    def clearance(self,poses,parts,current_parts):
        """Sample inflated tool boxes against observed ray-free space; unknown fails."""
        from scipy.spatial.transform import Rotation
        K,R,t=self.cal['K'],self.cal['R'],self.cal['t'];depth=self.initial_depth;unknown=occupied=0;minimum=float('inf');samples=0
        current=[]
        for v in current_parts:
            from scipy.spatial import ConvexHull
            current.append(ConvexHull(v).equations)
        envelope=self.protocol['engineering_envelopes']
        for pose in poses:
            q=np.asarray(pose[3:]);rot=Rotation.from_quat(q[[1,2,3,0]]).as_matrix()
            for vertices in parts:
                v=np.asarray(vertices);lo,hi=v.min(0),v.max(0)
                grid=np.asarray([[x,y,z] for x in (lo[0],(lo[0]+hi[0])/2,hi[0]) for y in (lo[1],(lo[1]+hi[1])/2,hi[1]) for z in (lo[2],(lo[2]+hi[2])/2,hi[2])])
                relative=grid@rot.T
                # Pointwise working envelope, so proximal metal extent does not
                # incorrectly inflate the distinct distal contact band.
                angle=envelope['final_TCP_tracking_angle_rad'];yaw=envelope['yaw_rad']
                norm=np.linalg.norm(relative,axis=1)
                angular=np.column_stack([np.sin(angle)*np.linalg.norm(np.delete(relative,j,axis=1),axis=1)+(1-np.cos(angle))*norm for j in range(3)])
                angular[:,:2]+=np.sin(yaw)*abs(relative[:,[1,0]])+(1-np.cos(yaw))*abs(relative[:,:2])
                position=np.array([envelope['XY_m'],envelope['XY_m'],envelope['local_upper_depth_Z_m']])+envelope['final_TCP_tracking_position_m']
                inflation=angular+position
                signs=np.asarray([[x,y,z] for x in (-1,1) for y in (-1,1) for z in (-1,1)])
                world=(relative[:,None,:]+inflation[:,None,:]*signs[None,:,:]+pose[:3]).reshape(-1,3)
                camera=world@R.T+t;uv=camera[:,:2]/camera[:,2,None];uv=uv@K[:2,:2].T+K[:2,2]
                for point,optical,pixel in zip(world,camera,uv):
                    samples+=1
                    # Known current tool volume is proprioceptive self geometry.
                    if any(np.max(h[:,:3]@point+h[:,3])<=1e-8 for h in current):continue
                    if not np.isfinite(pixel).all():unknown+=1;continue
                    x,y=np.rint(pixel).astype(int)
                    if optical[2]<=0 or not(0<=x<depth.shape[1] and 0<=y<depth.shape[0]) or not np.isfinite(depth[y,x]):unknown+=1;continue
                    gap=float(depth[y,x]-optical[2]);minimum=min(minimum,gap)
                    if gap<=0:occupied+=1
        return {'passed':unknown==0 and occupied==0,'unknown_samples':unknown,'nonfree_samples':occupied,'samples':samples,'minimum_observed_ray_clearance_m':minimum if np.isfinite(minimum) else None,'method':'Fixed inflated box corners/midplanes over2mm/1deg path samples; conditional observed ray-space screen, not global collision certification'}
