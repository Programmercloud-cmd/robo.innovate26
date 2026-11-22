"""Actual occupied-column regression and bounded partial/occlusion behavior."""
from pathlib import Path
import unittest
import cv2
import numpy as np
from perception import Classifier,load_config,compile_layout
from registration import points
R=Path(__file__).resolve().parent
class ColumnRefinementTests(unittest.TestCase):
 def setUp(self):
  self.c=load_config(R/'config.json');self.layout=compile_layout(self.c);self.ref=cv2.imread(str(R/self.c['reference_image']));self.raw=cv2.imread(str(R/'captures/manual_column_row2.jpg'));self.m=Classifier(self.c,self.ref)
 def test_actual_four_colors_and_source_empty(self):
  for k in range(6):rows=self.m.update(self.raw,self.layout,10**9+k*10**9,{'stale':True,'robot_moving':True},10**9+k*10**9,[])
  conveyor=[s for s in rows if s.get('conveyor_pose_row')==2]
  self.assertEqual([s['color_id'] for s in conveyor],[6135191,6487946,6117418,6487946]);self.assertTrue(all(s['status']=='occupied' for s in conveyor))
  self.assertTrue(all(s['status']=='empty' and s['confidence']>=.8 for s in rows if s['motion_pose_id']=='2_0'))
 def test_current_three_measured_top_grids(self):
  raw=cv2.imread(str(R/'captures/three_columns_current.jpg'))
  for k in range(6):rows=self.m.update(raw,self.layout,10**9+k*10**9,{'stale':True,'robot_moving':True},10**9+k*10**9,[])
  expected={2:[6135191,6487946,6117418,6487946],3:[4211201,6487946,6135191,300121],4:[6117418,6487946,6135191,6487946]}
  for row,colors in expected.items():
   actual=[s for s in rows if s.get('conveyor_pose_row')==row]
   self.assertEqual([s['color_id'] for s in actual],colors);self.assertTrue(all(s['confidence']>=.5 for s in actual))
 def test_partial_column_keeps_pixel_alignment(self):
  registration=self.m.stack_registration.estimate(self.raw,[]);canonical=self.m.stack_registration.canonical(self.raw,registration)
  a,transforms,diags=self.m.conveyor_refinement.apply(canonical,self.layout,10**9)
  target=next(s for s in a['conveyor_slots'] if s['slot_id']=='conveyor_r2_n0');partial=canonical.copy();mask=np.zeros(partial.shape[:2],np.uint8);cv2.fillConvexPoly(mask,np.round(target['brick_top_quad_px']).astype(np.int32),255);mask=cv2.dilate(mask,np.ones((5,5),np.uint8));partial[mask>0]=(215,215,215)
  b,after,diags=self.m.conveyor_refinement.apply(partial,self.layout,2*10**9)
  row=next(x for x in diags if x['footprint_id']=='conveyor_r2');self.assertEqual(row['status'],'retained_for_partial_or_empty_column')
  self.assertTrue(np.array_equal(transforms['conveyor_r2'],after['conveyor_r2']))
  for old,new in zip(a['conveyor_slots'],b['conveyor_slots']):
   if old['row']==2:self.assertTrue(np.array_equal(old['sample_polygon_px'],new['sample_polygon_px']))
 def test_foreground_on_preserved_outside_paper_still_vetoes(self):
  z=self.m.stack_registration.estimate(self.raw,[]);canonical=self.m.stack_registration.canonical(self.raw,z);_,transforms,_=self.m.conveyor_refinement.apply(canonical,self.layout,10**9)
  index=next(i for i,r in enumerate(self.c['white_gap_regions']) if r.get('conveyor_floor_guard') and r['affected_footprints']==['conveyor_r2'])
  region=self.c['white_gap_regions'][index];mask=np.zeros(canonical.shape[:2],np.uint8);cv2.fillPoly(mask,[np.round(points(transforms['conveyor_r2'],region['polygon_px'])).astype(np.int32)],255)
  mask=(mask>0)&self.m.gap_reference_whites[index];mask=cv2.erode(mask.astype(np.uint8),np.ones((5,5),np.uint8)).astype(bool);self.assertGreaterEqual(mask.sum(),20)
  canonical[mask]=(60,100,170);changed=cv2.warpPerspective(canonical,np.array(z['reference_to_image']),(1280,720))
  for k in range(4):rows=self.m.update(changed,self.layout,10**9+k*10**9,{'stale':True,'robot_moving':True},10**9+k*10**9,[])
  self.assertTrue(all(s['status']=='uncertain' and s['reason']=='local_foreground_or_roi_motion' for s in rows if s.get('conveyor_pose_row')==2))
if __name__=='__main__':unittest.main()
