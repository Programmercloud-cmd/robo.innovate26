"""Actual neighboring-column overlap: independent pixels, never count forcing."""
from pathlib import Path
import copy,json
import unittest
import cv2
import numpy as np
from perception import Classifier,compile_layout,load_config,pixels
from registration import points
R=Path(__file__).resolve().parent
class NeighborOwnershipTests(unittest.TestCase):
 def setUp(self):
  self.c=load_config(R/'config.json');self.layout=compile_layout(self.c);self.ref=cv2.imread(str(R/self.c['reference_image']));self.model=Classifier(self.c,self.ref)
  self.packet=json.loads((R/'captures/neighbor_overlap_current.json').read_text());raw=cv2.imread(str(R/'captures/neighbor_overlap_current.jpg'));self.image=cv2.warpPerspective(raw,np.array(self.packet['registration']['image_to_reference']),(1280,720))
  self.target=next(s for s in self.layout['conveyor_slots'] if s['slot_id']=='conveyor_r1_n3');self.owner=next(x for x in self.packet['conveyor_refinement'] if x['footprint_id']=='conveyor_r2');self.model.last_conveyor_refinement=[self.owner]
 def test_actual_borrowed_tan_becomes_independent_positive_floor(self):
  before=self.model.classify(self.image,self.target);self.assertEqual(before['color_id'],6135191)
  self.model.strict_camera_empty=True;self.model.update_column_owners(self.image,self.layout)
  values,fraction=self.model.independent_pixels(self.image,self.target['sample_polygon_px'],self.target)
  self.assertLess(fraction,self.c['classification']['minimum_color_coverage'])
  after=self.model.classify(self.image,self.target);self.assertEqual(after['status'],'empty');self.assertGreaterEqual(after['confidence'],.8)
  self.assertEqual(after['floor_white_fraction'],1.);self.assertEqual(after['floor_independent_pixel_fraction'],1.)
  # An owner retains its own pixels, rather than being suppressed with the ghost.
  own=copy.deepcopy(next(s for s in self.layout['conveyor_slots'] if s['slot_id']=='conveyor_r2_n0'));h=np.array(self.owner['reference_to_refined'])
  for key in ('sample_polygon_px','brick_top_quad_px'):own[key]=points(h,own[key]).tolist()
  self.assertEqual(self.model.classify(self.image,own)['color_id'],6135191)
 def test_independent_legitimate_row1_material_is_not_forced_empty(self):
  self.model.strict_camera_empty=True;self.model.update_column_owners(self.image,self.layout)
  values=pixels(self.ref,self.c['color_references']['300121']['polygons_px'][0]);red=tuple(map(int,np.median(values,axis=0)))
  occupied=self.image.copy();cv2.fillConvexPoly(occupied,np.round(self.target['brick_top_quad_px']).astype(np.int32),red)
  self.assertNotEqual(self.model.classify(occupied,self.target)['status'],'empty')
  separate=next(s for s in self.layout['conveyor_slots'] if s['slot_id']=='conveyor_r1_n2');cv2.fillConvexPoly(occupied,np.round(separate['brick_top_quad_px']).astype(np.int32),red)
  result=self.model.classify(occupied,separate);self.assertEqual(result['status'],'occupied');self.assertEqual(result['color_id'],300121)
if __name__=='__main__':unittest.main()
