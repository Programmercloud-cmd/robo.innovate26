"""Author display-only fixed ROIs from existing row poses + current floor image.
No device/SDK access. The source image must be a coherent shared-file capture.
"""
from pathlib import Path
import hashlib,json
import cv2
import numpy as np
from perception import load_config,compile_layout,transform,pixels
from registration import WholeFootprintRegistration
R=Path(__file__).resolve().parent
p=R/'config.json';c=json.loads(p.read_text());positions_path=R.parents[1]/'teaching/positions.json'
# R.parent = teaching; repo = R.parents[1]
positions_path=R.parents[1]/'teaching/positions.json';factory_path=R.parents[1]/'teaching/factory_vision/config.json';moving_path=R.parents[1]/'teaching/moving_conveyor/config.json'
positions=json.loads(positions_path.read_text());factory=json.loads(factory_path.read_text());moving=json.loads(moving_path.read_text())
raw=R/'captures/conveyor_current.jpg';packet_path=raw.with_suffix('.json');packet=json.loads(packet_path.read_text());im=cv2.imread(str(raw));cfg=load_config(p);layout=compile_layout(cfg);registration=WholeFootprintRegistration(cfg,cv2.imread(str(R/cfg['reference_image'])),layout);ex=packet['source_exclusions']['slot_ids'];fit=registration.estimate(im,ex)
if not fit['valid']:raise ValueError('Current floor evidence requires valid registration')
floor=registration.canonical(im,fit);floor_path=R/'captures/conveyor_floor_reference_canonical.jpg';cv2.imwrite(str(floor_path),floor);floor=cv2.imread(str(floor_path));floor_sha=hashlib.sha256(floor_path.read_bytes()).hexdigest()
length,width=c['geometry']['brick_body_st_m'];target=np.array(factory['conveyor_target_O_T_EE'][12:15]);step=np.array(positions['factory_source_grid']['conveyor_row_step_in_base_m']);base=moving['conveyor_reference_layer'];items=[]
for row in range(6):
 f=next(f for f in layout['footprints'] if f['row']==row and f['bank']==0)
 h=cv2.getPerspectiveTransform(np.array([[0,0],[3*length,0],[3*length,4*width],[0,4*width]],np.float32),np.array(f['top_plane_quad_px'],np.float32))
 source=np.array(positions['points'][f'point_{row}_0']['O_T_EE'][12:15]);destination=target+(row-base)*step;delta=destination-source;s=length/2-delta[0];t=2*width+delta[1]
 slots=[]
 for n in range(4):
  yc=t+(n-1.5)*width;body=transform(h,[(s-length/2,yc-width/2),(s+length/2,yc-width/2),(s+length/2,yc+width/2),(s-length/2,yc+width/2)])
  margin=.30;sample=transform(h,[(s-(.5-margin)*length,yc-(.5-margin)*width),(s+(.5-margin)*length,yc-(.5-margin)*width),(s+(.5-margin)*length,yc+(.5-margin)*width),(s-(.5-margin)*length,yc+(.5-margin)*width)])
  v=pixels(floor,sample);hsv=cv2.cvtColor(v.reshape(-1,1,3),cv2.COLOR_BGR2HSV).reshape(-1,3) if v is not None else None
  fraction=float(((hsv[:,1]<45)&(hsv[:,2]>150)).mean()) if hsv is not None else 0
  empty={'material_reference_verified':fraction>=.98,'polygon_px':sample,'reference_polygon_px':sample,'reference_image':'captures/'+floor_path.name,'reference_image_sha256':floor_sha,'provenance':'Independently exposed conveyor floor in current captured EMPTYlane; not absence-of-color inference; canonicalwarp from same frame. Conservative center window, no robot-world calibration.','observed_white_fraction':fraction}
  slots.append({'slot_id':f'conveyor_r{row}_n{n}','footprint_id':f'conveyor_r{row}','row':row,'bank':None,'m':0,'n':n,'motion_pose_id':None,'conveyor_pose_row':row,'location_type':'conveyor','sample_polygon_px':sample,'brick_top_quad_px':body,'surface_status':'registered_flat_visible_top','sample_projection_verified':True,'empty_reference':empty,'execution_authority':False,'display_only':True})
 body_quad=transform(h,[(s-length/2,t-2*width),(s+length/2,t-2*width),(s+length/2,t+2*width),(s-length/2,t+2*width)])
 items.append({'footprint_id':f'conveyor_r{row}','row':row,'top_plane_quad_px':body_quad,'configured_target_xyz_m':destination.tolist(),'target_minus_source_xyz_m':delta.tolist(),'source_anchor':f'point_{row}_0','source_local_column_center_st_m':[s,t],'slots':slots,'projection_scope':'Existing source-row top-plane homography extrapolated to configured placement target; display only; cloth/floor parallax unmeasured. Empty windows separately observed on exposed current floor.'})
lane_profiles=[slot['empty_reference']['reference_polygon_px'] for region in items for slot in region['slots'] if slot['empty_reference']['material_reference_verified']]
for region in items:
 for slot in region['slots']:
  slot['empty_reference']['material_reference_polygons_px']=lane_profiles
  slot['empty_reference']['material_profile_provenance']='Independently verified exposed white-paper lane samples from the same BEFORE-exchange baseline; no after-image or color-absence baseline learning.'
c['conveyor_observations']={'schema':'fr3.conveyor_observations.config.v1','regions':items,'provenance':{'raw_image':'captures/'+raw.name,'raw_sha256':hashlib.sha256(raw.read_bytes()).hexdigest(),'raw_packet':'captures/'+packet_path.name,'frame_sequence':packet['frame_sequence'],'frame_unix_ns':packet['frame_unix_ns'],'reference_to_image':fit['reference_to_image'],'positions_sha256':hashlib.sha256(positions_path.read_bytes()).hexdigest(),'factory_config_sha256':hashlib.sha256(factory_path.read_bytes()).hexdigest(),'moving_config_sha256':hashlib.sha256(moving_path.read_bytes()).hexdigest(),'legacy_numeric_poses_7_to_13_used':False,'world_coordinates_modified':False}}
c['allow_display_empty_when_guard_unknown']=True;c['classification']['unknown_guard_empty_minimum_floor_fraction']=.98;c['classification']['unknown_guard_empty_maximum_reference_difference']=15;c['classification']['unknown_guard_empty_minimum_confidence']=.8
c['display_policy']={'manual_camera_updates':True,'minimum_change_confidence':.5,'change_confirmation_frames':5,'empty_confirmation_frames':3}
# Explicit source floor windows: matching material from independently visible
# row-local paper margin, never label occupied source pixels as an empty state.
empty_references={}
for slot in layout['slots']:
 row=slot['row'];f=next(f for f in layout['footprints'] if f['row']==row and f['bank']==0);quad=np.array(f['top_plane_quad_px']);y0=float(quad[:,1].min()+8);y1=float(quad[:,1].max()-8)
 material=[[598,y0],[606,y0],[606,y1],[598,y1]];v=pixels(floor,material);hsv=cv2.cvtColor(v.reshape(-1,1,3),cv2.COLOR_BGR2HSV).reshape(-1,3) if v is not None else None;fraction=float(((hsv[:,1]<45)&(hsv[:,2]>150)).mean()) if hsv is not None else 0
 empty_references[slot['slot_id']]={'material_reference_verified':fraction>=.98,'polygon_px':slot['sample_polygon_px'],'reference_polygon_px':material,'reference_image':'captures/'+floor_path.name,'reference_image_sha256':floor_sha,'provenance':'Independently visible row-local white-paper margin MATERIAL; this is NOT an empty-state label of the occupied source slot. Explicit slot-interior floor observation must match material; unresolved parallax/occlusion remains uncertain.','observed_material_white_fraction':fraction}
c['empty_references']=empty_references
c['retired_static_occlusion_masks']={'polygons_px':c['static_occlusion_polygons_px'],'polylines_px':c.get('static_occlusion_polylines_px',[]),'reason':'Fresh conveyor_current image shows arm/lamp/crossingcable absent from source tops and conveyor lane. Replaced obsolete masks with current registered pigment/floor evidence + local image motion/foreground checks; no robot execution authority.','evidence_frame_unix_ns':packet['frame_unix_ns']}
c['static_occlusion_polygons_px']=[];c['static_occlusion_polylines_px']=[]
# Local outside-column paper strips detect hands entering each known lane ROI.
regions=[x for x in c['white_gap_regions'] if not x.get('conveyor_floor_guard')]
for region in items:
 q=np.array(region['top_plane_quad_px'],np.float32);h=cv2.getPerspectiveTransform(np.array([[0,0],[length,0],[length,4*width],[0,4*width]],np.float32),q)
 for a,b in ((-.008,-.004),(length+.004,length+.008)):
  poly=transform(h,[(a,0),(b,0),(b,4*width),(a,4*width)]);v=pixels(floor,poly)
  if v is None:continue
  hsv=cv2.cvtColor(v.reshape(-1,1,3),cv2.COLOR_BGR2HSV).reshape(-1,3);fraction=float(((hsv[:,1]<45)&(hsv[:,2]>150)).mean())
  if fraction>.98:regions.append({'polygon_px':poly,'affected_footprints':[region['footprint_id']],'reference_white_erosion_px':1,'reference_image':'captures/'+floor_path.name,'reference_image_sha256':floor_sha,'conveyor_floor_guard':True})
c['white_gap_regions']=regions
for name in ('config.json','config_v2_candidate.json'):(R/name).write_text(json.dumps(c,indent=2)+'\n')
print(json.dumps({'conveyor_white_fractions':{s['slot_id']:round(s['empty_reference']['observed_white_fraction'],3) for x in items for s in x['slots']},'source_material_verified':sum(x['material_reference_verified'] for x in empty_references.values()),'conveyor_gap_guards':sum(x.get('conveyor_floor_guard',False) for x in regions)},indent=2))
