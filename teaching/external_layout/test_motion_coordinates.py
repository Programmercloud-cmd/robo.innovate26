"""Motion uses unchanged sensor coordinates, independently of registration gauge."""
import copy
from pathlib import Path
import unittest
from unittest.mock import patch
import cv2
import numpy as np
from perception import Classifier,compile_layout,load_config
R=Path(__file__).resolve().parent
class RawMotionTests(unittest.TestCase):
 def setUp(self):
  self.c=load_config(R/'config.json');self.layout=compile_layout(self.c);self.raw=cv2.imread(str(R/'captures/three_columns_current.jpg'));self.m=Classifier(self.c,cv2.imread(str(R/self.c['reference_image'])))
  self.base=self.m.stack_registration.estimate(self.raw,[]);self.assertTrue(self.base['valid'])
  # Isolate the motion-coordinate contract; real paper occlusion behavior has
  # its own unchanged actual-image/foreign-material regression tests.
  self.m.gap_regions=[];self.m.gap_reference_grays=[]
 def reg(self,index):
  result=copy.deepcopy(self.base);h=np.array(result['reference_to_image']);h[0,2]+=2 if index%2 else 0
  result['reference_to_image']=h.tolist();result['image_to_reference']=np.linalg.inv(h).tolist();return result
 def test_same_raw_alternate_valid_fits_does_not_invent_motion(self):
  with patch.object(self.m.stack_registration,'estimate',side_effect=lambda *_:self.reg(self.step)):
   for self.step in range(8):
    t=10**9+self.step*200_000_000;rows=self.m.update(self.raw,self.layout,t,{'stale':True,'robot_moving':True},t,[])
    if self.step>=3:
     target=next(s for s in rows if s['slot_id']=='conveyor_r3_n0')
     self.assertEqual(target['status'],'occupied');self.assertEqual(target['color_id'],4211201)
     self.assertNotEqual(target['reason'],'local_foreground_or_roi_motion')
 def test_actual_raw_foreground_change_still_triggers_settle(self):
  with patch.object(self.m.stack_registration,'estimate',return_value=self.base):
   for k in range(5):
    t=10**9+k*200_000_000;self.m.update(self.raw,self.layout,t,{'stale':True,'robot_moving':True},t,[])
   target=next(s for s in self.m.debug_layout['conveyor_slots'] if s['slot_id']=='conveyor_r3_n0')
   hand=self.raw.copy();cv2.fillConvexPoly(hand,np.round(target['brick_top_quad_px']).astype(np.int32),(110,155,195))
   t=2*10**9;rows=self.m.update(hand,self.layout,t,{'stale':True,'robot_moving':True},t,[])
   result=next(s for s in rows if s['slot_id']=='conveyor_r3_n0')
   self.assertEqual(result['status'],'uncertain');self.assertEqual(result['reason'],'local_foreground_or_roi_motion')
if __name__=='__main__':unittest.main()
