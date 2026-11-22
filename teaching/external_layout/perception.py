"""Fixed external-camera brick patches. No robot imports or inventory writes."""
from collections import deque
import hashlib
import json
import math
from pathlib import Path
import time
from registration import WholeFootprintRegistration,registered_debug_layout
from conveyor_refinement import ConveyorRefinement
import cv2
import numpy as np


COLORS={300121:'red',6117418:'maroon',6487946:'orange-brown',6135191:'tan',4211201:'dark-brown'}


def slot_id(row,bank,m,n):return f'external_r{row}_c{bank}_m{m}_n{n}'


def load_config(path):
    config=json.loads(Path(path).read_text())
    if config.get('schema')!='fr3.external_layout.config.v1':raise ValueError('External layout schema mismatch')
    if config['camera']['serial']!='03FB123F':raise ValueError('External Logitech identity required')
    config['layout_hash']=hashlib.sha256(json.dumps(config,sort_keys=True,separators=(',',':')).encode()).hexdigest()
    config['_config_dir']=str(Path(path).resolve().parent)
    return config


def transform(h,points):
    return cv2.perspectiveTransform(np.asarray(points,np.float32)[None],h)[0].tolist()


def sample_polygon(h,m,n,config):
    length,width=config['geometry']['brick_body_st_m']
    margin=config['classification']['sample_margin_fraction']
    s0,s1=(m+margin)*length,(m+1-margin)*length
    t0,t1=(n+margin)*width,(n+1-margin)*width
    return transform(h,[(s0,t0),(s1,t0),(s1,t1),(s0,t1)])


def compile_layout(config):
    slots=[];footprints=[]
    length,width=config['geometry']['brick_body_st_m']
    gap=config['geometry']['gap_m'];side=config['geometry']['side_margin_m'];end=config['geometry']['end_margin_m']
    all_st=[];all_px=[]
    for footprint in config['footprints']:
        reference=footprint.get('top_plane_landmarks')
        if reference:
            origin=np.array([side+footprint['bank']*(3*length+gap),end+footprint['row']*(4*width+gap)])
            all_st.extend((np.asarray(reference['footprint_st_m'])+origin).tolist());all_px.extend(reference['image_px'])
    global_h,_=cv2.findHomography(np.asarray(all_st,float),np.asarray(all_px,float),0) if len(all_st)>=4 else (None,None)
    residuals=np.linalg.norm(np.asarray(transform(global_h,all_st))-all_px,axis=1) if global_h is not None else None
    for footprint in config['footprints']:
        row,bank=footprint['row'],footprint['bank']
        reference=footprint.get('top_plane_landmarks')
        h=None
        if reference is not None:
            a=np.asarray(reference['footprint_st_m'],np.float32);b=np.asarray(reference['image_px'],np.float32)
            if a.shape!=(4,2) or b.shape!=(4,2):raise ValueError('Four explicitly paired top-plane landmarks required')
            h=cv2.getPerspectiveTransform(a,b)
            if not np.isfinite(h).all() or abs(np.linalg.det(h))<1e-12:raise ValueError('Degenerate top-plane registration')
        verified=h is not None
        if h is None and global_h is not None:
            origin=[side+bank*(3*length+gap),end+row*(4*width+gap)]
            h=global_h@np.asarray([[1,0,origin[0]],[0,1,origin[1]],[0,0,1]],float)
        item={key:footprint[key] for key in ('footprint_id','row','bank','surface_status')}
        item['top_plane_quad_px']=transform(h,[(0,0),(3*length,0),(3*length,4*width),(0,4*width)]) if h is not None else None
        item['top_plane_landmarks']=reference
        item['projection_scope']='observed_local_flat_top_landmarks' if verified else 'expected_extrapolation_only_unverified_height'
        footprints.append(item)
        for m in range(3):
            for n in range(4):
                identity=slot_id(row,bank,m,n)
                slots.append({'slot_id':identity,'footprint_id':footprint['footprint_id'],'row':row,'bank':bank,'m':m,'n':n,
                              'motion_pose_id':f'{row}_{m}' if bank==0 else None,
                              'display_only':True,'execution_authority':False,
                              'center_layout_st_m':[side+bank*(3*length+gap)+(m+.5)*length,end+row*(4*width+gap)+(n+.5)*width],
                              'body_size_st_m':[length,width],'sample_polygon_px':sample_polygon(h,m,n,config) if h is not None else None,
                              'brick_top_quad_px':transform(h,[(m*length,n*width),((m+1)*length,n*width),((m+1)*length,(n+1)*width),(m*length,(n+1)*width)]) if h is not None else None,
                              'sample_projection_verified':verified,
                              'surface_status':footprint['surface_status'],
                              'empty_reference':config.get('empty_references',{}).get(identity)})
    if len(slots)!=144 or len({s['slot_id'] for s in slots})!=144:raise ValueError('Exactly twelve explicit4x3 footprints required')
    result={'schema':'fr3.external_layout.static.v1','layout_hash':config['layout_hash'],'camera_serial':config['camera']['serial'],
            'mapping_confirmed':config['mapping_confirmed'],'geometry':config['geometry'],'index_convention':config['index_convention'],
            'footprints':footprints,'slots':slots,'execution_authority':False,'inventory_authority':False}
    conveyor=config.get('conveyor_observations',{})
    result['conveyor_slots']=[slot for region in conveyor.get('regions',[]) for slot in region['slots']]
    result['conveyor_regions']=[{k:v for k,v in region.items() if k!='slots'} for region in conveyor.get('regions',[])]
    if result['conveyor_slots'] and (len(result['conveyor_slots'])!=24 or len({x['slot_id'] for x in result['conveyor_slots']})!=24):raise ValueError('Exactly six four-brick conveyor regions required')
    result['expected_projection_fit']={'landmark_count':len(all_st),'rms_px':float(np.sqrt(np.mean(residuals**2))) if residuals is not None else None,
                                       'max_px':float(np.max(residuals)) if residuals is not None else None,
                                       'scope':'Expected outline fromallflat-top landmarks; no unverified/piled-surface observations or floor reuse'}
    return result


def pixels(image,polygon):
    if polygon is None:return None
    p=np.asarray(polygon,np.float32)
    if p.shape!=(4,2) or not np.isfinite(p).all():return None
    if np.any(p[:,0]<1) or np.any(p[:,1]<1) or np.any(p[:,0]>=image.shape[1]-1) or np.any(p[:,1]>=image.shape[0]-1):return None
    x0,y0=np.floor(p.min(axis=0)).astype(int);x1,y1=np.ceil(p.max(axis=0)).astype(int)+1
    mask=np.zeros((y1-y0,x1-x0),np.uint8);cv2.fillConvexPoly(mask,np.round(p-[x0,y0]).astype(np.int32),255)
    values=image[y0:y1,x0:x1][mask>0]
    return values if len(values)>=12 else None


def color_feature(values):
    hsv=cv2.cvtColor(values.reshape(-1,1,3),cv2.COLOR_BGR2HSV).reshape(-1,3)
    selected=(hsv[:,1]>55)&(hsv[:,2]>28)&(hsv[:,2]<250)
    coverage=float(selected.mean())
    if selected.sum()<10:return None,coverage
    # Highlights lower saturation and can brighten a dark brick into another
    # class. Prefer actual pigment pixels rather than the brightest pixels.
    saturated=selected.copy();saturated[selected]=hsv[selected,1]>=np.percentile(hsv[selected,1],35)
    lab=cv2.cvtColor(values.reshape(-1,1,3),cv2.COLOR_BGR2LAB).reshape(-1,3)
    warm_hue=(hsv[:,0].astype(float)+90)%180-90
    feature=np.r_[np.median(lab[saturated],axis=0),np.median(warm_hue[saturated]),np.median(hsv[saturated,1])]
    return feature.astype(float),coverage


def build_palette(config,reference,slot_identity=None):
    models={}
    for identity,entry in config['color_references'].items():
        features=[]
        scopes=entry.get('prototype_slot_scopes',[None]*len(entry['polygons_px']))
        if len(scopes)!=len(entry['polygons_px']):raise ValueError('Color reference scope count mismatch')
        for polygon,scope in zip(entry['polygons_px'],scopes):
            if scope is not None and slot_identity not in scope:continue
            sample=pixels(reference,polygon)
            if sample is None:raise ValueError('Invalid color-reference sample polygon')
            feature,coverage=color_feature(sample)
            if feature is None or coverage<.6:raise ValueError('Color reference lacks positive colored material')
            features.append(feature)
        if not features:raise ValueError('Every class requires an unscoped color reference')
        models[int(identity)]=np.asarray(features)
    if set(models)!=set(COLORS):raise ValueError('Five explicit LEGO color references required')
    return models


class Classifier:
    def __init__(self,config,reference):
        self.config=config;self.reference=reference;self.palette=build_palette(config,reference)
        scoped_ids={identity for entry in config['color_references'].values() for scope in entry.get('prototype_slot_scopes',[]) if scope is not None for identity in scope}
        self.slot_palettes={identity:build_palette(config,reference,identity) for identity in scoped_ids}
        self.temporal={};self.occlusion=np.zeros(reference.shape[:2],np.uint8)
        self.previous_gray=None;self.previous_raw_gray=None;self.motion_until={};self.footprint_foreground_until={}
        self.reference_gray=cv2.cvtColor(reference,cv2.COLOR_BGR2GRAY)
        self.gap_regions=[];self.gap_reference_grays=[];self.gap_reference_whites=[]
        reference_hsv=cv2.cvtColor(reference,cv2.COLOR_BGR2HSV)
        reference_white=(reference_hsv[:,:,1]<config['classification']['empty_maximum_saturation'])&(reference_hsv[:,:,2]>config['classification']['empty_minimum_value'])
        for region in config.get('white_gap_regions',[]):
            gap_reference=reference
            if region.get('reference_image'):
                path=Path(config['_config_dir'])/region['reference_image']
                if hashlib.sha256(path.read_bytes()).hexdigest()!=region['reference_image_sha256']:raise ValueError('Gap image evidence hash mismatch')
                gap_reference=cv2.imread(str(path))
                if gap_reference is None or gap_reference.shape!=reference.shape:raise ValueError('Gap reference image size mismatch')
            gap_hsv=cv2.cvtColor(gap_reference,cv2.COLOR_BGR2HSV)
            reference_white=(gap_hsv[:,:,1]<config['classification']['empty_maximum_saturation'])&(gap_hsv[:,:,2]>config['classification']['empty_minimum_value'])
            mask=np.zeros(reference.shape[:2],np.uint8);cv2.fillPoly(mask,[np.asarray(region['polygon_px'],np.int32)],255)
            paper=mask.astype(bool)&reference_white
            erosion=region.get('reference_white_erosion_px',0)
            if not isinstance(erosion,int) or isinstance(erosion,bool) or not 0<=erosion<=2:raise ValueError('Gap reference erosion must be0..2pixels')
            if erosion:
                paper=cv2.erode(paper.astype(np.uint8),np.ones((2*erosion+1,2*erosion+1),np.uint8)).astype(bool)
                if int(paper.sum())<20:raise ValueError('Gap erosion leaves insufficient positive paper evidence')
            self.gap_regions.append((paper,tuple(region['affected_footprints'])))
            self.gap_reference_grays.append(cv2.cvtColor(gap_reference,cv2.COLOR_BGR2GRAY));self.gap_reference_whites.append(reference_white)
        self.empty_images={};verified_floor_images={}
        floor_entries=dict(config.get('empty_references',{}));floor_entries.update({slot['slot_id']:slot['empty_reference'] for region in config.get('conveyor_observations',{}).get('regions',[]) for slot in region['slots']})
        for identity,entry in floor_entries.items():
            if entry.get('reference_image'):
                p=Path(config['_config_dir'])/entry['reference_image'];key=(str(p),entry.get('reference_image_sha256'))
                if key not in verified_floor_images:
                    if hashlib.sha256(p.read_bytes()).hexdigest()!=entry.get('reference_image_sha256'):raise ValueError('Empty reference image hash mismatch')
                    im=cv2.imread(str(p))
                    if im is None or im.shape!=reference.shape:raise ValueError('Empty reference image shape mismatch')
                    verified_floor_images[key]=im
                self.empty_images[identity]=verified_floor_images[key]
        for polygon in config['static_occlusion_polygons_px']:
            cv2.fillPoly(self.occlusion,[np.asarray(polygon,np.int32)],255)
        for line in config.get('static_occlusion_polylines_px',[]):
            cv2.polylines(self.occlusion,[np.asarray(line['points'],np.int32)],False,255,int(line['thickness_px']))
        self.last_registration={'status':'not_observed','valid':False}
        self.stack_registration=WholeFootprintRegistration(config,reference,compile_layout(config)) if config.get('full_stack_registration') else None
        self.conveyor_refinement=ConveyorRefinement(config.get('conveyor_refinement',{'maximum_corner_shift_px':18,'minimum_rectangularity':.8,'minimum_assignment_margin_px':5}))
        self.last_conveyor_refinement=[]
        self.column_owner_labels=np.zeros(reference.shape[:2],np.uint8)
        self.current_segmentation_mask=None;self.allow_empty=True;self.strict_camera_empty=False;self.guard_status='not_observed';self.debug_layout=None

    def check_registration(self,gray):
        check=self.config.get('registration_checks')
        if not check:return {'status':'registration_unconfigured','valid':False,'matches':[]}
        radius=int(check['search_radius_px']);matches=[]
        for rectangle in check['landmark_rectangles_px']:
            x0,y0,x1,y1=map(int,rectangle)
            template=self.reference_gray[y0:y1,x0:x1]
            sx0,sy0=max(0,x0-radius),max(0,y0-radius);sx1,sy1=min(gray.shape[1],x1+radius),min(gray.shape[0],y1+radius)
            scores=cv2.matchTemplate(gray[sy0:sy1,sx0:sx1],template,cv2.TM_CCOEFF_NORMED)
            _,score,_,point=cv2.minMaxLoc(scores)
            matches.append({'score':float(score),'shift_px':[int(sx0+point[0]-x0),int(sy0+point[1]-y0)]})
        good=[m for m in matches if m['score']>=check['minimum_match_score']]
        shifted=[m for m in good if math.hypot(*m['shift_px'])>check['maximum_translation_px']]
        if len(shifted)>=check['minimum_good_landmarks']:status='registration_changed'
        elif len(good)-len(shifted)<check['minimum_good_landmarks']:status='registration_uncertain'
        else:status='registered'
        return {'status':status,'valid':status=='registered','matches':matches,'scope':'fixed image landmark check only; no automatic warp or robot-world calibration'}

    def uncertain(self,reason,visible=False):return {'status':'uncertain','color_id':None,'confidence':0.,'visible':visible,'reason':reason}

    def update_column_owners(self,image,layout):
        """A registered column's visible pixels belong to that column only."""
        self.column_owner_labels.fill(0)
        for evidence in self.last_conveyor_refinement:
            body=evidence.get('observed_colored_body_quad_px')
            if body is None:continue
            identity=evidence['footprint_id'];slots=[s for s in layout.get('conveyor_slots',[]) if s['footprint_id']==identity]
            if not slots:continue
            mask=np.zeros(image.shape[:2],np.uint8)
            if evidence['status']=='observed_full_column':
                cv2.fillPoly(mask,[np.round(body).astype(np.int32)],1)
            else:
                # Retain only current colored parts, never a disappeared
                # column's historical empty silhouette.
                for part in slots:
                    values=pixels(image,part['sample_polygon_px'])
                    feature,coverage=color_feature(values) if values is not None else (None,0.)
                    if feature is not None and coverage>=self.config['classification']['minimum_color_coverage']:
                        cv2.fillPoly(mask,[np.round(part['brick_top_quad_px']).astype(np.int32)],1)
            self.column_owner_labels|=mask*np.uint8(1<<slots[0]['row'])

    def independent_pixels(self,image,polygon,slot):
        values=pixels(image,polygon)
        if values is None or slot.get('location_type')!='conveyor':return values,1.
        owners=pixels(self.column_owner_labels,polygon)
        if owners is None:return None,0.
        own_bit=1<<slot['row'];independent=(owners&np.uint8(63^own_bit))==0
        return values[independent],float(independent.mean())

    def classify(self,image,slot):
        c=self.config['classification']
        segmented=True
        if self.current_segmentation_mask is not None and slot.get('location_type')!='conveyor':
            material=pixels(self.current_segmentation_mask,slot['sample_polygon_px'])
            segmented=material is not None and float((material>0).mean())>=c['minimum_color_coverage']
        has_top=slot['surface_status']=='registered_flat_visible_top'
        empty=slot.get('empty_reference')
        if not has_top and not empty:return self.uncertain(slot['surface_status'])
        polygon=slot['sample_polygon_px'] if has_top else empty['polygon_px'];values,independent_fraction=self.independent_pixels(image,polygon,slot)
        if values is None:return self.uncertain('sample_outside_frame')
        blocked=pixels(self.occlusion,polygon)
        if blocked is None or np.any(blocked):return self.uncertain('fixed_robot_lamp_or_cable_occlusion')
        feature,coverage=color_feature(values) if has_top and len(values)>=12 else (None,0.)
        coverage*=independent_fraction
        if feature is not None and coverage>=c['minimum_color_coverage']:
            if not segmented:return self.uncertain('no_visible_segmented_slot_material',True)
            distances=sorted((float(np.min(np.linalg.norm((feature-model)/np.asarray(c['feature_scales']),axis=1))),identity)
                             for identity,model in self.slot_palettes.get(slot['slot_id'],self.palette).items())
            distance,identity=distances[0];margin=distances[1][0]-distance
            if distance<=c['maximum_color_distance'] and margin>=c['minimum_color_margin']:
                quality=float(np.clip((1-distance/c['maximum_color_distance'])*min(1,margin)*coverage,0,1))
                if quality<c['minimum_occupied_confidence']:return self.uncertain('color_confidence_below_threshold',True)
                return {'status':'occupied','color_id':identity,'confidence':quality,'visible':True,'reason':'positive_colored_top_material',
                        'color_distance':distance,'color_margin':margin,'independent_pixel_fraction':independent_fraction}
            return self.uncertain('color_reference_ambiguous',True)
        if slot.get('current_full_column_body'):return self.uncertain('observed_column_body_color_uncertain',True)
        if not self.allow_empty:return self.uncertain('robot_guard_unknown_empty_disabled',True)
        if not empty or empty.get('material_reference_verified') is not True:return self.uncertain('no_positive_empty_reference',True)
        floor,floor_independent_fraction=self.independent_pixels(image,empty['polygon_px'],slot);before=pixels(self.empty_images.get(slot['slot_id'],self.reference),empty.get('reference_polygon_px',empty['polygon_px']))
        empty_occlusion=pixels(self.occlusion,empty['polygon_px'])
        if empty_occlusion is None or np.any(empty_occlusion):return self.uncertain('empty_floor_occluded')
        if floor is None or before is None:return self.uncertain('empty_reference_clipped')
        if len(floor)<12 or floor_independent_fraction<c['minimum_color_coverage']:return self.uncertain('neighbor_column_occlusion')
        hsv=cv2.cvtColor(floor.reshape(-1,1,3),cv2.COLOR_BGR2HSV).reshape(-1,3)
        floor_fraction=float(((hsv[:,1]<c['empty_maximum_saturation'])&(hsv[:,2]>c['empty_minimum_value'])).mean())
        profiles=empty.get('material_reference_polygons_px',[empty.get('reference_polygon_px',empty['polygon_px'])])
        profile_values=[pixels(self.empty_images.get(slot['slot_id'],self.reference),poly) for poly in profiles]
        medians=[np.median(value.astype(float),axis=0) for value in profile_values if value is not None]
        distances=[float(np.max(np.abs(np.median(floor.astype(float),axis=0)-value))) for value in medians]
        if not distances:return self.uncertain('material_profile_unavailable')
        profile_index=int(np.argmin(distances));difference=distances[profile_index]
        minimum_floor=max(c['empty_minimum_floor_fraction'],c.get('unknown_guard_empty_minimum_floor_fraction',1.)) if self.strict_camera_empty else c['empty_minimum_floor_fraction']
        maximum_difference=min(c['empty_maximum_reference_difference'],c.get('unknown_guard_empty_maximum_reference_difference',0.)) if self.strict_camera_empty else c['empty_maximum_reference_difference']
        if floor_fraction>=minimum_floor and difference<=maximum_difference:
            quality=float(np.clip(floor_fraction*(1-.2*difference/max(1,maximum_difference)),0,1))
            minimum_confidence=max(c['minimum_empty_confidence'],c.get('unknown_guard_empty_minimum_confidence',1.)) if self.strict_camera_empty else c['minimum_empty_confidence']
            if quality<minimum_confidence:return self.uncertain('empty_confidence_below_threshold',True)
            return {'status':'empty','color_id':None,'confidence':quality,
                    'visible':True,'reason':'verified_exposed_floor_reference','floor_white_fraction':floor_fraction,'floor_material_difference':difference,'floor_material_profile_index':profile_index,'floor_independent_pixel_fraction':floor_independent_fraction}
        return self.uncertain('shadow_foreground_or_changed_floor',True)

    def update(self,image,layout,frame_unix_ns,guard,now_unix_ns=None,excluded_ids=(),exclusions_valid=True):
        observed=[]
        now=time.time_ns() if now_unix_ns is None else now_unix_ns
        stamp=guard.get('observed_unix_ns');q=guard.get('q_measured');reference=self.config.get('registration_reference_q')
        guard_age_ok=isinstance(stamp,int) and not isinstance(stamp,bool) and 0<=now-stamp<=int(self.config['guard_stale_after_s']*1e9)
        q_ok=(isinstance(q,list) and len(q)==7 and isinstance(reference,list) and len(reference)==7
              and np.isfinite(q).all() and max(abs(a-b) for a,b in zip(q,reference))<=self.config['maximum_registration_joint_delta_rad'])
        frame_fresh=isinstance(frame_unix_ns,int) and 0<=now-frame_unix_ns<=int(self.config['stale_after_s']*1e9)
        c=self.config['classification'];seconds=now*1e-9
        self.column_owner_labels.fill(0)
        raw_gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        raw_delta=cv2.absdiff(raw_gray,self.previous_raw_gray) if self.previous_raw_gray is not None and self.previous_raw_gray.shape==raw_gray.shape else None
        if self.stack_registration is not None:
            self.last_registration=self.stack_registration.estimate(image,excluded_ids) if exclusions_valid else {'valid':False,'status':'source_exclusions_unavailable'}
            self.debug_layout=registered_debug_layout(layout,self.last_registration)
            if self.last_registration['valid']:
                image=self.stack_registration.canonical(image,self.last_registration)
                self.current_segmentation_mask=self.stack_registration.segment(image)
                layout,conveyor_transforms,self.last_conveyor_refinement=self.conveyor_refinement.apply(image,layout,frame_unix_ns)
                self.debug_layout=registered_debug_layout(layout,self.last_registration)
                self.update_column_owners(image,layout)
            else:self.current_segmentation_mask=None;conveyor_transforms={};self.last_conveyor_refinement=[]
        else:
            conveyor_transforms={}
            self.last_registration=self.check_registration(cv2.cvtColor(image,cv2.COLOR_BGR2GRAY));self.debug_layout=layout
        gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        # Motion must compare sensor pixels in a common RAW coordinate frame.
        # Re-estimated reference warps otherwise turn fit jitter into fake motion.
        raw_slots={s['slot_id']:s for s in self.debug_layout['slots']+self.debug_layout.get('conveyor_slots',[])}
        fresh_guard=guard_age_ok and guard.get('stale') is False
        known_moving=fresh_guard and guard.get('robot_moving') is True
        known_safe=fresh_guard and q_ok and guard.get('robot_moving') is False
        camera_only=(self.config.get('robot_guard_mode')=='camera_evidence_when_unknown' and self.stack_registration is not None and self.last_registration['valid'])
        self.strict_camera_empty=not known_safe and camera_only
        self.allow_empty=known_safe or (camera_only and self.config.get('allow_display_empty_when_guard_unknown') is True)
        self.guard_status='fresh_moving_veto' if known_moving else 'fresh_stationary_known_pose' if known_safe else 'unknown_camera_evidence_only' if camera_only else 'unknown_veto'
        safe=(self.config.get('operator_setup_ready') is True and self.config['mapping_confirmed'] and not known_moving and (known_safe or camera_only) and frame_fresh and self.last_registration['valid'] and exclusions_valid)
        hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
        nonpaper=(hsv[:,:,1]>c['empty_maximum_saturation'])|(hsv[:,:,2]<c['empty_minimum_value'])
        for gap_index,((mask,affected),gap_reference_gray) in enumerate(zip(self.gap_regions,self.gap_reference_grays)):
            local_h=next((conveyor_transforms[identity] for identity in affected if identity in conveyor_transforms),None)
            if local_h is not None:
                from registration import points
                region=self.config['white_gap_regions'][gap_index];poly=points(local_h,region['polygon_px']);moved=np.zeros(gray.shape,np.uint8);cv2.fillPoly(moved,[np.round(poly).astype(np.int32)],255)
                mask=(moved>0)&self.gap_reference_whites[gap_index];erosion=region.get('reference_white_erosion_px',0)
                if erosion:mask=cv2.erode(mask.astype(np.uint8),np.ones((2*erosion+1,2*erosion+1),np.uint8)).astype(bool)
                if int(mask.sum())<20:
                    for identity in affected:self.footprint_foreground_until[identity]=seconds+c['motion_settle_s']
            foreign=(cv2.absdiff(gray,gap_reference_gray)>c['gap_difference_threshold'])&nonpaper
            if mask.any() and float(foreign[mask].mean())>c['gap_change_fraction']:
                for footprint in affected:self.footprint_foreground_until[footprint]=seconds+c['motion_settle_s']
        for slot in layout['slots']+layout.get('conveyor_slots',[]):
            identity=slot['slot_id']
            raw_slot=raw_slots.get(identity)
            motion=pixels(raw_delta,raw_slot['brick_top_quad_px']) if raw_delta is not None and raw_slot is not None else None
            if motion is not None and float((motion>c['motion_difference_threshold']).mean())>c['maximum_motion_fraction']:
                self.motion_until[identity]=seconds+c['motion_settle_s']
            moving=seconds<self.footprint_foreground_until.get(slot['footprint_id'],0) or seconds<self.motion_until.get(identity,0)
            hard_reason=('operator_setup_quarantine' if self.config.get('operator_setup_ready') is not True else 'mapping_pending' if not self.config['mapping_confirmed'] else self.last_registration['status'] if not self.last_registration['valid'] else 'stale_camera_frame' if not frame_fresh else 'robot_guard_stale_or_moving_or_pose_changed')
            if not exclusions_valid:hard_reason='source_exclusions_unavailable'
            raw=self.classify(image,slot) if safe and not moving else self.uncertain('local_foreground_or_roi_motion' if safe else hard_reason)
            if identity in excluded_ids:raw=self.uncertain('transferred_source')
            history=self.temporal.setdefault(identity,deque(maxlen=self.config['temporal']['window_frames']))
            if raw['status']=='uncertain':
                if raw['visible']:history.append(('uncertain',None,0.))
                else:history.clear()
                result=raw
            else:
                history.append((raw['status'],raw['color_id'],raw['confidence']))
                needed=self.config['temporal']['empty_frames'] if raw['status']=='empty' else self.config['temporal']['occupied_frames']
                consistent=sum(item[:2]==(raw['status'],raw['color_id']) for item in history)>=needed
                result=raw if consistent else self.uncertain('bounded_temporal_confirmation_pending',raw['visible'])
            observed.append({key:slot[key] for key in ('slot_id','footprint_id','motion_pose_id')}|result|{'observed_unix_ns':frame_unix_ns,'location_type':slot.get('location_type','source')}|({'conveyor_pose_row':slot['row'],'n':slot['n']} if slot.get('location_type')=='conveyor' else {}))
        self.previous_gray=gray;self.previous_raw_gray=raw_gray
        return observed


def overlay(image,layout,slots,guard):
    result=image.copy();by_id={s['slot_id']:s for s in slots}
    colors={300121:(25,25,230),6117418:(45,20,100),6487946:(20,105,170),6135191:(140,180,210),4211201:(35,45,70)}
    for footprint in layout['footprints']+layout.get('conveyor_regions',[]):
        polygon=footprint['top_plane_quad_px']
        if polygon is not None:
            cv2.polylines(result,[np.round(polygon).astype(np.int32)],True,(180,180,0),1)
            origin=tuple(np.round(np.asarray(polygon)[0]).astype(int));cv2.putText(result,footprint['footprint_id'],origin,cv2.FONT_HERSHEY_SIMPLEX,.35,(255,255,0),1)
    for slot in layout['slots']+layout.get('conveyor_slots',[]):
        polygon=slot['sample_polygon_px'];state=by_id[slot['slot_id']]
        if polygon is None:continue
        p=np.round(polygon).astype(np.int32)
        color=colors.get(state['color_id'],(0,220,0) if state['status']=='empty' else (80,80,80))
        cv2.polylines(result,[p],True,color,1)
        point=tuple(np.round(np.asarray(polygon).mean(axis=0)).astype(int))
        label=f"conv{slot['row']}:{slot['n']}" if slot.get('location_type')=='conveyor' else f"{slot['row']}/{slot['bank']}:{slot['m']},{slot['n']}"
        cv2.putText(result,label,point,cv2.FONT_HERSHEY_SIMPLEX,.24,color,1)
    for index,slot in enumerate(layout['slots']):
        state=by_id[slot['slot_id']];name=COLORS.get(state['color_id'],'--')
        label=f"r{slot['row']}b{slot['bank']}m{slot['m']}n{slot['n']} {state['status'][:3]} {name} {state['confidence']:.2f}"
        cv2.putText(result,label,(900+(index//72)*190,65+(index%72)*8),cv2.FONT_HERSHEY_SIMPLEX,.23,(30,230,230),1)
    cv2.putText(result,'DISPLAY ONLY  '+('ROBOT / GUARD UNCERTAIN' if guard.get('stale') or guard.get('robot_moving') else 'fixed visible top patches'),(15,25),cv2.FONT_HERSHEY_SIMPLEX,.55,(0,255,255),1)
    return result
