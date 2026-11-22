"""Fixed-layout segmentation checks; synthetic warps test limits, not calibration."""
import copy,json
from unittest.mock import patch
from pathlib import Path
import unittest
import cv2
import numpy as np
from perception import Classifier,compile_layout,load_config
from registration import WholeFootprintRegistration,points
from run import update_source_exclusions
R=Path(__file__).resolve().parent
class RegistrationTests(unittest.TestCase):
 def setUp(self):
  self.c=load_config(R/'config.json');self.layout=compile_layout(self.c);self.image=cv2.imread(str(R/self.c['reference_image']));self.m=WholeFootprintRegistration(self.c,self.image,self.layout)
  self.excluded=[f'external_r5_c0_m2_n{n}' for n in range(4)]+[f'external_r0_c0_m0_n{n}' for n in range(4)]
 def test_exclusions_stale_retains_and_fresh_explicit_reset_clears(self):
  known={s['slot_id'] for s in self.layout['slots']};now=10**10
  d={'schema':'fr3.source_exclusions.v1','slot_ids':self.excluded,'observed_unix_ns':now,'stale':False}
  ids,seen,fresh=update_source_exclusions(set(),False,d,now,known)
  self.assertTrue(fresh);self.assertEqual(ids,set(self.excluded))
  ids,seen,fresh=update_source_exclusions(ids,seen,{**d,'slot_ids':[]},now+3*10**9,known)
  self.assertFalse(fresh);self.assertEqual(ids,set(self.excluded))
  ids,seen,fresh=update_source_exclusions(ids,seen,{**d,'slot_ids':[]},now,known)
  self.assertTrue(fresh);self.assertEqual(ids,set())
 def test_small_shift_recovers_and_excluded_columns_are_not_anchors(self):
  h=np.array([[1,0,12],[0,1,-7],[0,0,1]],float);im=cv2.warpPerspective(self.image,h,(1280,720));reg=self.m.estimate(im,self.excluded)
  self.assertTrue(reg['valid'],reg);self.assertNotIn('external_r5_c0',reg['supported_footprints']);self.assertNotIn('external_r0_c0',reg['supported_footprints'])
  self.assertLess(np.linalg.norm(points(np.array(reg['reference_to_image']),[[650,300]])-points(h,[[650,300]])),1.5)
 def test_extended_shift_needs_independent_fixed_corner(self):
  h=np.array([[1,0,-35],[0,1,0],[0,0,1]],float);im=cv2.warpPerspective(self.image,h,(1280,720))
  reg=self.m.estimate(im,self.excluded);self.assertTrue(reg['valid'],reg)
  self.assertTrue(reg['extended_shift_corroboration']['valid']);self.assertEqual(reg['effective_translation_limit_px'],36)
  im[450:640,700:930]=(215,215,215)
  rejected=self.m.estimate(im,self.excluded);self.assertFalse(rejected['valid'],rejected)
  self.assertFalse(rejected.get('extended_shift_corroboration',{}).get('valid',False))
 def test_cached_actual_camera_change_recovers(self):
  reg=self.m.estimate(cv2.imread(str(R/'captures/segmentation_current.jpg')),self.excluded)
  self.assertTrue(reg['valid'],reg);self.assertGreater(len(reg['supported_footprints']),4)
 def test_large_or_one_row_pitch_shift_is_rejected(self):
  for dx,dy in ((40,0),(0,65),(0,-65)):
   shifted=cv2.warpAffine(self.image,np.float32([[1,0,dx],[0,1,dy]]),(1280,720));self.assertFalse(self.m.estimate(shifted,self.excluded)['valid'])
 def test_blank_and_all_stacks_occluded_cannot_register(self):
  self.assertFalse(self.m.estimate(np.full_like(self.image,220),self.excluded)['valid'])
  im=self.image.copy();im[15:535,390:820]=(105,145,190)
  self.assertFalse(self.m.estimate(im,self.excluded)['valid'])
 def test_actual_depletion_needs_two_independent_fixed_corners(self):
  image=cv2.imread(str(R/'captures/depleted_measured.jpg'));reg=self.m.estimate(image,[])
  self.assertTrue(reg['valid'],reg);self.assertEqual(reg['status'],'registered_depleted_layout_fixed_corners')
  self.assertGreaterEqual(len(reg['supported_footprints']),4);self.assertTrue(reg['depletion_fixed_corners']['valid'])
  h=np.array(reg['reference_to_image']);poly=points(h,[[780,496],[880,496],[880,588],[780,588]])
  occluded=image.copy();cv2.fillConvexPoly(occluded,np.round(poly).astype(np.int32),(210,210,210))
  rejected=self.m.estimate(occluded,[]);self.assertFalse(rejected['valid'],rejected)
  self.assertFalse(rejected.get('depletion_fixed_corners',{}).get('valid',False))
 def test_actual_native_shape_rejection_affine_and_heldout_corner(self):
  # Published features precede raw.jpg JPEG re-encoding; keep that measured
  # distinction explicit rather than expecting bit-identical contours.
  packet=json.loads((R/'captures/returned_conveyor_native_packet.json').read_text());image=cv2.imread(str(R/'captures/returned_conveyor_current.jpg'));bodies=[]
  for segment in packet['registration']['segments']:
   q=np.array(segment['current_body_quad_px'],np.float32);box=cv2.boundingRect(q)
   bodies.append({'quad':q,'center':q.mean(0),'area':abs(cv2.contourArea(q)),'size':np.array(box[2:],float),'rectangularity':1.})
  with patch.object(self.m,'components',return_value=bodies):reg=self.m.estimate(image,[])
  self.assertTrue(reg['valid'],reg);self.assertEqual(reg['status'],'registered_depleted_layout_bounded_affine')
  self.assertNotIn('external_r0_c0',reg['supported_footprints']);detail=reg['bounded_affine_refinement'];self.assertTrue(detail['accepted'])
  self.assertEqual(detail['held_out_corner'],'fixed_right_tape_cross_corner');self.assertLessEqual(detail['fit_residual_px_max'],2.5)
  self.assertTrue(all(abs(s-1)<=.035 for s in detail['singular_scales']))
  h=np.array(reg['reference_to_image']);poly=points(h,[[780,496],[880,496],[880,588],[780,588]])
  occluded=image.copy();cv2.fillConvexPoly(occluded,np.round(poly).astype(np.int32),(210,210,210))
  with patch.object(self.m,'components',return_value=bodies):rejected=self.m.estimate(occluded,[])
  self.assertFalse(rejected['valid'],rejected)
 def test_partial_departed_columns_not_filled_and_other_slots_visible(self):
  im=self.image.copy()
  for s in self.layout['slots']:
   if s['slot_id'] in self.excluded:cv2.fillConvexPoly(im,np.round(s['brick_top_quad_px']).astype(np.int32),(210,210,210))
  m=Classifier(self.c,self.image);guard={'observed_unix_ns':10**9,'stale':True,'robot_moving':True}
  for _ in range(5):rows=m.update(im,self.layout,10**9,guard,10**9,self.excluded)
  self.assertTrue(m.last_registration['valid']);self.assertEqual(m.guard_status,'unknown_camera_evidence_only')
  self.assertTrue(all(s['reason']=='transferred_source' and not s['visible'] for s in rows if s['slot_id'] in self.excluded))
  self.assertTrue(any(s['status']=='occupied' for s in rows if s['motion_pose_id']=='5_0'))
  self.assertTrue(m.strict_camera_empty)
  self.assertTrue(all(s['confidence']>=self.c['classification']['unknown_guard_empty_minimum_confidence'] for s in rows if s['status']=='empty'))
 def test_fresh_moving_guard_remains_a_veto(self):
  m=Classifier(self.c,self.image);guard={'observed_unix_ns':10**9,'stale':False,'robot_moving':True,'q_measured':self.c['registration_reference_q']}
  for _ in range(4):rows=m.update(self.image,self.layout,10**9,guard,10**9,self.excluded)
  self.assertTrue(all(s['status']=='uncertain' for s in rows));self.assertEqual(m.guard_status,'fresh_moving_veto')
if __name__=='__main__':unittest.main()
