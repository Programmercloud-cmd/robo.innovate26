"""Bounded image-only registration from complete known LEGO footprints.

Segments pigment bodies only within the existing layout neighborhood. No world
reconstruction, inventory inference, or fallback color/position fabrication.
"""
import cv2
import numpy as np


def points(h,xy):
    return cv2.perspectiveTransform(np.asarray(xy,np.float32)[None],h)[0]


def ordered_quad(contour):
    hull=cv2.convexHull(contour)
    polygon=cv2.approxPolyDP(hull,.025*cv2.arcLength(hull,True),True).reshape(-1,2)
    if len(polygon)!=4:return None
    center=polygon.mean(axis=0)
    polygon=polygon[np.argsort(np.arctan2(polygon[:,1]-center[1],polygon[:,0]-center[0]))]
    polygon=np.roll(polygon,-np.argmin(polygon.sum(axis=1)),axis=0)
    return polygon.astype(np.float32)


class WholeFootprintRegistration:
    def __init__(self,config,reference,layout):
        self.config=config['full_stack_registration'];self.shape=reference.shape[:2];self.layout=layout
        self.reference_gray=cv2.cvtColor(reference,cv2.COLOR_BGR2GRAY)
        self.neighborhood=np.zeros(self.shape,np.uint8)
        for f in layout['footprints']:
            if f['top_plane_quad_px'] is not None:
                cv2.fillConvexPoly(self.neighborhood,np.round(f['top_plane_quad_px']).astype(np.int32),255)
        radius=max(self.config['maximum_translation_px'],self.config.get('corroborated_maximum_translation_px',0))+12
        self.neighborhood=cv2.dilate(self.neighborhood,np.ones((2*radius+1,2*radius+1),np.uint8))
        self.reference_mask=self.segment(reference)
        reference_components=self.components(self.reference_mask)
        self.reference_bodies={}
        for f in layout['footprints']:
            expected=np.asarray(f['top_plane_quad_px'],float)
            if expected.shape!=(4,2):continue
            center=expected.mean(axis=0);area=abs(cv2.contourArea(expected.astype(np.float32)))
            candidates=[]
            for body in reference_components:
                distance=float(np.linalg.norm(body['center']-center))
                ratio=body['area']/area
                if distance<self.config['reference_center_tolerance_px'] and .65<ratio<1.6:
                    candidates.append((distance,body))
            if len(candidates)==1:self.reference_bodies[f['footprint_id']]=candidates[0][1]

    def segment(self,image):
        hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
        warm=(hsv[:,:,0]<=self.config['maximum_warm_hue'])|(hsv[:,:,0]>=170)
        # No closing across narrow paper gaps: that can join neighboring rows.
        return ((hsv[:,:,1]>=self.config['minimum_pigment_saturation'])&(hsv[:,:,2]>=self.config['minimum_pigment_value'])&warm&(self.neighborhood>0)).astype(np.uint8)*255

    def components(self,mask):
        contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        result=[]
        for contour in contours:
            area=float(cv2.contourArea(contour))
            if area<self.config['minimum_component_area_px']:continue
            quad=ordered_quad(contour)
            if quad is None:continue
            box=cv2.boundingRect(contour)
            rectangularity=area/max(1.,abs(cv2.contourArea(quad)))
            if rectangularity<self.config['minimum_rectangularity']:continue
            result.append({'quad':quad,'center':quad.mean(axis=0),'area':area,'size':np.asarray(box[2:],float),'rectangularity':rectangularity})
        return result

    def corroborate_extended_shift(self,image,h):
        """A fixed table tape corner independently checks the proposed image fit."""
        c=self.config;check=c.get('extended_shift_landmark')
        if not check:return {'valid':False,'reason':'independent_landmark_unconfigured'}
        x0,y0,x1,y1=check['rectangle_px'];radius=int(check['search_radius_px'])
        gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY);template=self.reference_gray[y0:y1,x0:x1]
        sx0,sy0=max(0,x0-radius),max(0,y0-radius);sx1,sy1=min(gray.shape[1],x1+radius),min(gray.shape[0],y1+radius)
        scores=cv2.matchTemplate(gray[sy0:sy1,sx0:sx1],template,cv2.TM_CCOEFF_NORMED)
        _,score,_,peak=cv2.minMaxLoc(scores)
        measured=np.array([sx0+peak[0]+(x1-x0)/2,sy0+peak[1]+(y1-y0)/2],float)
        center=np.array([(x0+x1)/2,(y0+y1)/2]);predicted=points(h,[center])[0]
        residual=float(np.linalg.norm(measured-predicted))
        # A camera shift along the row direction must stay well below one row
        # pitch. The fixed-corner agreement separately prevents aliasing rows.
        pitches=[];row_axes=[]
        for bank in (0,1):
            fs=sorted([f for f in self.layout['footprints'] if f['bank']==bank],key=lambda f:f['row'])
            centers=np.array([np.mean(f['top_plane_quad_px'],axis=0) for f in fs])
            for a,b in zip(centers,centers[1:]):
                direction=b-a;length=float(np.linalg.norm(direction));pitches.append(length);row_axes.append(direction/length)
        layout_centers=np.array([np.mean(f['top_plane_quad_px'],axis=0) for f in self.layout['footprints']])
        displacement=points(h,layout_centers)-layout_centers
        maximum_row_displacement=max(float(np.max(np.abs(displacement@axis))) for axis in row_axes)
        row_limit=float(min(pitches)*check['maximum_row_pitch_fraction'])
        valid=score>=check['minimum_match_score'] and residual<=check['maximum_transform_residual_px'] and maximum_row_displacement<row_limit
        return {'valid':bool(valid),'score':float(score),'landmark':'fixed_right_tape_cross_corner','rectangle_px':check['rectangle_px'],'measured_center_px':measured.tolist(),'predicted_center_px':predicted.tolist(),'transform_residual_px':residual,'maximum_row_axis_displacement_px':maximum_row_displacement,'row_axis_limit_px':row_limit,'minimum_reference_row_pitch_px':min(pitches)}

    def corroborate_depleted_layout(self,image,h,good_footprints):
        """Partial source banks require two independent, fixed 2D corners."""
        c=self.config.get('depletion_fallback')
        if not c:return {'valid':False,'reason':'depletion_fallback_unconfigured'}
        canonical=cv2.warpPerspective(image,np.linalg.inv(h),(self.shape[1],self.shape[0]))
        gray=cv2.cvtColor(canonical,cv2.COLOR_BGR2GRAY);radius=c['local_search_radius_px'];matches=[]
        for landmark in c['fixed_corner_patches']:
            x0,y0,x1,y1=landmark['rectangle_px'];sx0,sy0=max(0,x0-radius),max(0,y0-radius);sx1,sy1=min(gray.shape[1],x1+radius),min(gray.shape[0],y1+radius)
            template=self.reference_gray[y0:y1,x0:x1];scoremap=cv2.matchTemplate(gray[sy0:sy1,sx0:sx1],template,cv2.TM_CCOEFF_NORMED);_,score,_,peak=cv2.minMaxLoc(scoremap)
            residual=[int(sx0+peak[0]-x0),int(sy0+peak[1]-y0)];valid=score>=c['minimum_corner_score'] and np.linalg.norm(residual)<=c['maximum_corner_residual_px']
            matches.append({'landmark':landmark['name'],'rectangle_px':landmark['rectangle_px'],'score':float(score),'canonical_residual_px':residual,'valid':bool(valid),'center_reference_px':[(x0+x1)/2,(y0+y1)/2]})
        supported=[m for m in matches if m['valid']]
        separation=max((float(np.linalg.norm(np.asarray(a['center_reference_px'])-b['center_reference_px'])) for i,a in enumerate(supported) for b in supported[i+1:]),default=0.)
        anchors=np.array([np.mean(f['top_plane_quad_px'],axis=0) for f in good_footprints]+[m['center_reference_px'] for m in supported])
        span=np.ptp(anchors,axis=0).tolist() if len(anchors) else [0.,0.]
        valid=len(supported)>=2 and separation>=c['minimum_corner_separation_px'] and all(a>=b for a,b in zip(span,c['minimum_anchor_span_px']))
        return {'valid':bool(valid),'matches':matches,'corner_separation_px':separation,'combined_anchor_span_px':span,'scope':'Independent immutable table/taped-border image corners only; no world/extrinsic authority'}

    def estimate(self,image,excluded_ids=()):
        c=self.config
        output={'status':'insufficient_full_stack_support','valid':False,'scope':'bounded known-layout full pigment bodies; image-only, no world-coordinate authority','supported_footprints':[],'segments':[],'limits':c}
        if image.shape[:2]!=self.shape:return output|{'status':'camera_image_size_changed'}
        excluded=set(excluded_ids)
        excluded_footprints={s['footprint_id'] for s in self.layout['slots'] if s['slot_id'] in excluded}
        current_mask=self.segment(image);bodies=self.components(current_mask)
        matches=[];used=set()
        for identity,ref in self.reference_bodies.items():
            if identity in excluded_footprints:continue
            candidates=[]
            for index,body in enumerate(bodies):
                size_ratio=body['size']/ref['size'];area_ratio=body['area']/ref['area']
                distance=float(np.linalg.norm(body['center']-ref['center']))
                if distance<=max(c['maximum_translation_px'],c.get('corroborated_maximum_translation_px',0))+8 and np.all((size_ratio>.84)&(size_ratio<1.18)) and .72<area_ratio<1.3:
                    candidates.append((distance,index,body))
            candidates.sort(key=lambda x:x[0])
            if not candidates:continue
            if len(candidates)>1 and candidates[1][0]-candidates[0][0]<c['minimum_assignment_margin_px']:continue
            distance,index,body=candidates[0]
            if index in used:continue
            used.add(index);matches.append((identity,ref,body))
        fallback_min=c.get('depletion_fallback',{}).get('minimum_full_stacks',c['minimum_full_stacks'])
        if len(matches)<min(c['minimum_full_stacks'],fallback_min):return output|{'candidate_footprints':[x[0] for x in matches]}
        source=np.asarray([ref['center'] for _,ref,_ in matches]);target=np.asarray([body['center'] for _,_,body in matches])
        affine,inliers=cv2.estimateAffinePartial2D(source,target,method=cv2.RANSAC,ransacReprojThreshold=c['maximum_fit_residual_px'],maxIters=500,confidence=.99)
        if affine is None:return output|{'status':'ambiguous_stack_geometry'}
        inliers=inliers[:,0].astype(bool);selected=[x for x,keep in zip(matches,inliers) if keep]
        initial_ids=[x[0] for x in selected];rejected=[]
        h=np.vstack([affine,[0.,0.,1.]])
        # Shape outliers must not continue biasing the transform subsequently
        # checked against fixed scene corners.
        for _ in range(2):
            clean=[]
            for identity,ref,body in selected:
                error=float(np.max(np.linalg.norm(points(h,ref['quad'])-body['quad'],axis=1)))
                if error<=c['maximum_shape_residual_px']:clean.append((identity,ref,body))
                else:rejected.append({'footprint_id':identity,'current_body_quad_px':body['quad'].tolist(),'shape_residual_px':error,'full_stack_anchor':False,'reason':'shape_rejected_before_refit'})
            if len(clean)<min(c['minimum_full_stacks'],fallback_min):return output|{'status':'full_stack_shape_ambiguous','segments':rejected}
            if len(clean)==len(selected):break
            selected=clean
            src=np.array([ref['center'] for _,ref,_ in selected]);dst=np.array([body['center'] for _,_,body in selected])
            affine,mask=cv2.estimateAffinePartial2D(src,dst,method=cv2.RANSAC,ransacReprojThreshold=c['maximum_fit_residual_px'],maxIters=500,confidence=.99)
            if affine is None or mask is None:return output|{'status':'shape_filtered_refit_uncertain','segments':rejected}
            retained=[]
            for item,keep in zip(selected,mask[:,0]):
                if keep:retained.append(item)
                else:rejected.append({'footprint_id':item[0],'current_body_quad_px':item[2]['quad'].tolist(),'full_stack_anchor':False,'reason':'centre_outlier_after_shape_refit'})
            selected=retained
            if len(selected)<min(c['minimum_full_stacks'],fallback_min):return output|{'status':'shape_filtered_refit_uncertain','segments':rejected}
            h=np.vstack([affine,[0.,0.,1.]])
        source=np.array([ref['center'] for _,ref,_ in selected]);target=np.array([body['center'] for _,_,body in selected])
        good_footprints=[f for f in self.layout['footprints'] if f['footprint_id'] in [x[0] for x in selected]]
        rows={f['row'] for f in good_footprints};banks={f['bank'] for f in good_footprints}
        if len(selected)<min(c['minimum_full_stacks'],fallback_min) or len(rows)<c['minimum_row_span']:return output|{'status':'insufficient_spatial_stack_support'}
        normally_supported=len(selected)>=c['minimum_full_stacks'] and len(banks)>=2
        affine_detail=None;corroboration=None
        if not normally_supported:
            corroboration=self.corroborate_depleted_layout(image,h,good_footprints)
            refine=c.get('bounded_affine_refinement')
            if not corroboration['valid'] and refine:
                matches_fixed=corroboration['matches'];fit_corner=next((x for x in matches_fixed if x['landmark']==refine['fit_corner']),None)
                # Only one scene landmark is fitted; the other corner remains
                # an independent held-out check, along with all stack shapes.
                if fit_corner is not None and all(x['score']>=c['depletion_fallback']['minimum_corner_score'] for x in matches_fixed):
                    reference_point=np.asarray(fit_corner['center_reference_px']);observed=points(h,[reference_point+fit_corner['canonical_residual_px']])[0]
                    fit_source=np.vstack([source,reference_point]);fit_target=np.vstack([target,observed])
                    candidate,mask=cv2.estimateAffine2D(fit_source,fit_target,method=cv2.RANSAC,ransacReprojThreshold=c['maximum_fit_residual_px'],maxIters=500,confidence=.99)
                    if candidate is not None and np.isfinite(candidate).all() and mask is not None and mask.all():
                        proposed=np.vstack([candidate,[0.,0.,1.]])
                        u,values,vt=np.linalg.svd(candidate[:,:2]);rotation_matrix=u@vt;strain=rotation_matrix.T@candidate[:,:2]
                        polar_rotation=float(np.degrees(np.arctan2(rotation_matrix[1,0],rotation_matrix[0,0])))
                        layout_points=np.concatenate([np.asarray(f['top_plane_quad_px']) for f in self.layout['footprints']])
                        deviation=float(np.max(np.linalg.norm(points(proposed,layout_points)-points(h,layout_points),axis=1)))
                        fit_error=float(np.max(np.linalg.norm(points(proposed,fit_source)-fit_target,axis=1)))
                        shape_error=max(float(np.max(np.linalg.norm(points(proposed,ref['quad'])-body['quad'],axis=1))) for _,ref,body in selected)
                        proof=self.corroborate_depleted_layout(image,proposed,good_footprints)
                        accepted=(np.linalg.det(candidate[:,:2])>0 and np.all(np.abs(values-1)<=c['maximum_scale_change']) and abs(polar_rotation)<=c['maximum_rotation_deg'] and abs(float(strain[0,1]))<=refine['maximum_shear'] and deviation<=refine['maximum_source_region_adjustment_px'] and fit_error<=c['maximum_fit_residual_px'] and shape_error<=c['maximum_shape_residual_px'] and proof['valid'])
                        affine_detail={'accepted':bool(accepted),'fit_corner':refine['fit_corner'],'held_out_corner':refine['held_out_corner'],'fit_residual_px_max':fit_error,'singular_scales':values.tolist(),'polar_rotation_deg':polar_rotation,'shear':float(strain[0,1]),'source_region_adjustment_px_max':deviation,'shape_residual_px_max':shape_error,'independent_corner_proof':proof}
                        if accepted:h=proposed;corroboration=proof
        linear=h[:2,:2];u,values,vt=np.linalg.svd(linear);rotation_matrix=u@vt
        rotation=float(np.degrees(np.arctan2(rotation_matrix[1,0],rotation_matrix[0,0])));scale=float(np.sqrt(np.prod(values)))
        layout_corners=np.concatenate([np.asarray(f['top_plane_quad_px']) for f in self.layout['footprints']]);maximum_shift=float(np.max(np.linalg.norm(points(h,layout_corners)-layout_corners,axis=1)))
        residual=np.linalg.norm(points(h,source)-target,axis=1)
        output.update({'supported_footprints':[x[0] for x in selected],'excluded_anchor_footprints':sorted(excluded_footprints),'reference_to_image':h.tolist(),'image_to_reference':np.linalg.inv(h).tolist(),'scale':scale,'rotation_deg':rotation,'maximum_layout_shift_px':maximum_shift,'fit_residual_px_max':float(np.max(residual)),'candidate_footprints':[x[0] for x in matches],'shape_filtered_refit':{'initial_footprints':initial_ids,'retained_footprints':[x[0] for x in selected]}})
        if affine_detail is not None:output['bounded_affine_refinement']=affine_detail
        translation_limit=c['maximum_translation_px']
        if maximum_shift>translation_limit:
            extension=self.corroborate_extended_shift(image,h);output['extended_shift_corroboration']=extension
            if extension['valid']:translation_limit=max(translation_limit,c.get('corroborated_maximum_translation_px',0))
        output['effective_translation_limit_px']=translation_limit
        if maximum_shift>translation_limit or abs(rotation)>c['maximum_rotation_deg'] or np.any(np.abs(values-1)>c['maximum_scale_change']):return output|{'status':'camera_deviation_out_of_bounds'}
        if output['fit_residual_px_max']>c['maximum_fit_residual_px']:return output|{'status':'stack_fit_residual_exceeded'}
        output['segments']=rejected
        for identity,ref,body in selected:
            error=float(np.max(np.linalg.norm(points(h,ref['quad'])-body['quad'],axis=1)))
            output['segments'].append({'footprint_id':identity,'current_body_quad_px':body['quad'].tolist(),'shape_residual_px':error,'full_stack_anchor':error<=c['maximum_shape_residual_px']})
            if error>c['maximum_shape_residual_px']:return output|{'status':'full_stack_shape_ambiguous'}
        if not normally_supported:
            output['depletion_fixed_corners']=corroboration
            if not corroboration['valid']:return output|{'status':'depleted_layout_landmarks_uncertain'}
        status='registered_full_stacks' if normally_supported else 'registered_depleted_layout_bounded_affine' if affine_detail and affine_detail['accepted'] else 'registered_depleted_layout_fixed_corners'
        return output|{'valid':True,'status':status}

    def canonical(self,image,registration):
        h=np.asarray(registration['image_to_reference'],float)
        return cv2.warpPerspective(image,h,(self.shape[1],self.shape[0]),flags=cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT)


def registered_debug_layout(layout,registration):
    import copy
    result=copy.deepcopy(layout)
    if not registration.get('valid'):return result
    h=np.asarray(registration['reference_to_image'],float)
    for f in result['footprints']+result.get('conveyor_regions',[]):
        if f['top_plane_quad_px'] is not None:f['top_plane_quad_px']=points(h,f['top_plane_quad_px']).tolist()
    for slot in result['slots']+result.get('conveyor_slots',[]):
        for key in ('sample_polygon_px','brick_top_quad_px'):
            if slot[key] is not None:slot[key]=points(h,slot[key]).tolist()
    return result
