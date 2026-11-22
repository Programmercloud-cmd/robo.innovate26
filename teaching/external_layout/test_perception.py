"""Focused cached-image/software checks, not independently labeled accuracy."""
import copy,json,os,tempfile
from pathlib import Path
import unittest
from unittest.mock import patch
import cv2
import numpy as np
from perception import Classifier,color_feature,compile_layout,load_config
from run import close_capture_safely,publish_stopped
ROOT=Path(__file__).resolve().parent
class PerceptionTests(unittest.TestCase):
 def setUp(self):
  self.config=load_config(ROOT/'config_v2_candidate.json');self.layout=compile_layout(self.config)
  self.image=cv2.imread(str(ROOT/self.config['reference_image']))
  self.target=next(s for s in self.layout['slots'] if s['slot_id']=='external_r5_c0_m0_n1')
 def guard(self,now):return {'observed_unix_ns':now,'stale':False,'robot_moving':False,'q_measured':self.config['registration_reference_q']}
 def test_all144_ids_and_display_only_mapping(self):
  self.assertEqual(len({s['slot_id'] for s in self.layout['slots']}),144)
  self.assertTrue(all(s['sample_polygon_px'] for s in self.layout['slots']))
  self.assertTrue(all(s['motion_pose_id'] is None for s in self.layout['slots'] if s['bank']==1))
  self.assertFalse(self.layout['execution_authority'])
 def test_red_wrap(self):
  hsv=np.array([[[179,230,170]]]*30+[[[0,230,170]]]*30+[[[1,230,170]]]*30,np.uint8)
  f,c=color_feature(cv2.cvtColor(hsv,cv2.COLOR_HSV2BGR).reshape(-1,3));self.assertLess(abs(f[3]),2);self.assertEqual(c,1)
 def test_operator_pair_reference_is_measured_pixels_not_neighbor_copy(self):
  m=Classifier(self.config,self.image)
  slots={s['slot_id']:s for s in self.layout['slots']}
  anchor=m.classify(self.image,slots['external_r0_c1_m2_n0'])
  self.assertEqual(anchor['color_id'],6487946)
  target=slots['external_r0_c1_m2_n1']
  self.assertEqual(len(m.palette[6487946]),3)
  self.assertEqual(len(m.slot_palettes[target['slot_id']][6487946]),4)
  self.assertNotIn('external_r0_c1_m2_n0',m.slot_palettes)
  result=m.classify(self.image,target)
  self.assertIn(result['color_id'],(6487946,None))
  dark=self.image.copy();cv2.fillConvexPoly(dark,np.round(target['brick_top_quad_px']).astype(np.int32),(25,25,25))
  self.assertEqual(m.classify(dark,target)['status'],'uncertain')
 def test_empty_requires_positive_paper_dark_uncertain(self):
  m=Classifier(self.config,self.image);q=np.round(self.target['brick_top_quad_px']).astype(np.int32)
  white=self.image.copy();from perception import pixels
  material=pixels(m.empty_images[self.target['slot_id']],self.target['empty_reference']['reference_polygon_px']);color=np.median(material,axis=0).astype(int)
  cv2.fillConvexPoly(white,q,tuple(map(int,color)));self.assertEqual(m.classify(white,self.target)['status'],'empty')
  cv2.fillConvexPoly(white,q,(25,25,25));self.assertEqual(m.classify(white,self.target)['status'],'uncertain')
 def test_stale_guard_pose_setup_and_frame_clear(self):
  now=10**9
  for g in ({'stale':True},{'robot_moving':True},{'observed_unix_ns':0},{'q_measured':[v+.03 for v in self.config['registration_reference_q']]}):
   m=Classifier(self.config,self.image);rows=m.update(self.image,self.layout,now,{**self.guard(now),**g},now)
   self.assertTrue(all(s['status']=='uncertain' for s in rows))
  m=Classifier(self.config,self.image);rows=m.update(self.image,self.layout,0,self.guard(now),now)
  self.assertTrue(all(s['reason']=='stale_camera_frame' for s in rows))
 def test_registration_shift_invalidates_without_autowarp(self):
  m=Classifier(self.config,self.image);gray=cv2.cvtColor(self.image,cv2.COLOR_BGR2GRAY)
  self.assertTrue(m.check_registration(gray)['valid'])
  shifted=cv2.warpAffine(gray,np.float32([[1,0,0],[0,1,20]]),(gray.shape[1],gray.shape[0]))
  self.assertFalse(m.check_registration(shifted)['valid'])
 def test_gap_foreground_is_local_not_all_scene(self):
  m=Classifier(self.config,self.image);im=self.image.copy();r=self.config['white_gap_regions'][0]
  cv2.fillPoly(im,[np.array(r['polygon_px'],np.int32)],(80,130,170))
  for k in range(5):
   now=10**9+k*10**9;rows=m.update(im,self.layout,now,self.guard(now),now)
  affected=[s for s in rows if s['footprint_id'] in r['affected_footprints']]
  self.assertTrue(all(s['reason']=='local_foreground_or_roi_motion' for s in affected))
  self.assertTrue(any(s['status']=='occupied' for s in rows if s['footprint_id']=='external_r5_c1'))
 def test_gap_border_shift_is_not_foreground_but_interior_is(self):
  m=Classifier(self.config,self.image);im=cv2.imread(str(ROOT/'captures/gap_false_occlusion.jpg'))
  index=next(i for i,r in enumerate(self.config['white_gap_regions']) if r.get('reference_white_erosion_px')==2)
  mask,affected=m.gap_regions[index];self.assertGreaterEqual(mask.sum(),20)
  hsv=cv2.cvtColor(im,cv2.COLOR_BGR2HSV);gray=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY);c=self.config['classification']
  foreign=(cv2.absdiff(gray,m.reference_gray)>c['gap_difference_threshold'])&((hsv[:,:,1]>c['empty_maximum_saturation'])|(hsv[:,:,2]<c['empty_minimum_value']))
  self.assertLess(float(foreign[mask].mean()),c['gap_change_fraction'])
  changed=self.image.copy();changed[mask]=(60,100,160)
  now=10**9;rows=m.update(changed,self.layout,now,self.guard(now),now)
  self.assertTrue(all(s['reason']=='local_foreground_or_roi_motion' for s in rows if s['footprint_id'] in affected))
 def test_current_uncertainty_never_returns_old_color(self):
  m=Classifier(self.config,self.image);positive={'status':'occupied','color_id':300121,'confidence':.9,'visible':True,'reason':'test'}
  uncertain=m.uncertain('color_reference_ambiguous',True)
  with patch.object(m,'classify',return_value=positive):
   for k in range(3):rows=m.update(self.image,self.layout,10**9,self.guard(10**9),10**9)
  self.assertTrue(any(s['status']=='occupied' for s in rows))
  with patch.object(m,'classify',return_value=uncertain):rows=m.update(self.image,self.layout,10**9,self.guard(10**9),10**9)
  self.assertTrue(all(s['status']=='uncertain' for s in rows))
  with patch.object(m,'classify',return_value=positive):rows=m.update(self.image,self.layout,10**9,self.guard(10**9),10**9)
  self.assertTrue(any(s['status']=='occupied' for s in rows))
 def test_enodev_cleanup_and_stale_publish(self):
  class Mapping:
   closed=False
   def close(self):self.closed=True
  class Fake:
   streaming=True
   def close(self):raise OSError(19,'No such device')
  c=Fake();mapping=Mapping();c.maps=[mapping];c.fd=os.open('/dev/null',os.O_RDONLY);fd=c.fd
  errors=close_capture_safely(c);self.assertTrue(errors);self.assertTrue(mapping.closed);self.assertEqual(c.fd,-1)
  with self.assertRaises(OSError):os.fstat(fd)
  with tempfile.TemporaryDirectory() as directory:
   p=Path(directory)
   with open(p/'publish.lock','a') as lock:publish_stopped(None,self.layout,p,lock,errors)
   d=json.loads((p/'latest.json').read_text());self.assertTrue(d['stale']);self.assertEqual(len(d['slots']),144)
   self.assertTrue(all(s['status']=='uncertain' and not s['visible'] for s in d['slots']))
if __name__=='__main__':unittest.main()
