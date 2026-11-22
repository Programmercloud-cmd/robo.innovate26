"""Persistent bounded pixel refinement of known four-brick conveyor columns."""
import copy
import cv2
import numpy as np
from registration import ordered_quad,points


class ConveyorRefinement:
    def __init__(self,config):
        self.config=config;self.retained={};self.evidence={}

    def apply(self,image,layout,frame_ns):
        result=dict(layout);result['conveyor_slots']=copy.deepcopy(layout.get('conveyor_slots',[]));result['conveyor_regions']=copy.deepcopy(layout.get('conveyor_regions',[]))
        hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
        pigment=((hsv[:,:,1]>=90)&(hsv[:,:,2]>=28)&((hsv[:,:,0]<=30)|(hsv[:,:,0]>=170))).astype(np.uint8)*255
        diagnostics=[];transforms={}
        c=self.config
        for region in result['conveyor_regions']:
            identity=region['footprint_id'];expected=np.asarray(region['top_plane_quad_px'],np.float32);radius=c['maximum_corner_shift_px']
            x0,y0=np.maximum(0,np.floor(expected.min(0)-radius).astype(int));x1,y1=np.minimum([image.shape[1],image.shape[0]],np.ceil(expected.max(0)+radius).astype(int))
            mask=np.zeros(pigment.shape,np.uint8);mask[y0:y1,x0:x1]=pigment[y0:y1,x0:x1]
            mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
            contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            expected_width=np.mean([np.linalg.norm(expected[1]-expected[0]),np.linalg.norm(expected[2]-expected[3])]);expected_height=np.mean([np.linalg.norm(expected[3]-expected[0]),np.linalg.norm(expected[2]-expected[1])]);candidates=[]
            for contour in contours:
                area=cv2.contourArea(contour)
                if area<.65*abs(cv2.contourArea(expected)):continue
                quad=ordered_quad(contour)
                if quad is None or area/max(1.,abs(cv2.contourArea(quad)))<c['minimum_rectangularity']:continue
                width=np.mean([np.linalg.norm(quad[1]-quad[0]),np.linalg.norm(quad[2]-quad[3])]);height=np.mean([np.linalg.norm(quad[3]-quad[0]),np.linalg.norm(quad[2]-quad[1])])
                if not .8<width/expected_width<1.2 or not .88<height/expected_height<1.15:continue
                shift=float(np.max(np.linalg.norm(quad-expected,axis=1)))
                if shift>radius:continue
                candidates.append((float(np.linalg.norm(quad.mean(0)-expected.mean(0))),quad,shift))
            candidates.sort(key=lambda a:a[0]);current_full=False
            calibrated_row=str(region['row']) in c.get('body_front_face_fraction_by_row',{})
            if calibrated_row and candidates and (len(candidates)==1 or candidates[1][0]-candidates[0][0]>=c['minimum_assignment_margin_px']):
                _,quad,shift=candidates[0]
                body_h=cv2.getPerspectiveTransform(expected,quad)
                trim=c.get('body_front_face_fraction_by_row',{}).get(str(region['row']),0.)
                if not 0<=trim<=.2:raise ValueError('Measured body front-face fraction outside bounded range')
                sampling_quad=quad.copy();sampling_quad[2]=quad[1]+(quad[2]-quad[1])*(1-trim);sampling_quad[3]=quad[0]+(quad[3]-quad[0])*(1-trim)
                h=cv2.getPerspectiveTransform(expected,sampling_quad)
                self.retained[identity]={'body':body_h,'sampling':h};self.evidence[identity]={'observed_unix_ns':frame_ns,'maximum_corner_shift_px':shift,'observed_colored_body_quad_px':quad.tolist(),'observed_sampling_quad_px':sampling_quad.tolist(),'measured_front_face_fraction':trim};current_full=True
            retained=self.retained.get(identity)
            if retained is not None:
                h=retained['sampling'];transforms[identity]=retained['body'];region['top_plane_quad_px']=points(h,expected).tolist()
                for slot in result['conveyor_slots']:
                    if slot['footprint_id']!=identity:continue
                    for key in ('sample_polygon_px','brick_top_quad_px'):slot[key]=points(h,slot[key]).tolist()
                    empty=slot.get('empty_reference')
                    if empty:
                        for key in ('polygon_px','reference_polygon_px'):empty[key]=points(h,empty[key]).tolist()
                    slot['current_full_column_body']=current_full
                diagnostics.append({'footprint_id':identity,'status':'observed_full_column' if current_full else 'retained_for_partial_or_empty_column','reference_to_refined':h.tolist(),'reference_to_body':retained['body'].tolist(),**self.evidence[identity]})
            else:diagnostics.append({'footprint_id':identity,'status':'nominal_body_to_top_unmeasured' if candidates else 'nominal_no_full_column_evidence'})
        return result,transforms,diagnostics
