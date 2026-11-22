"""Known conveyor regions and honest evidence transitions; no hardware access."""
import copy,fcntl,json,tempfile
from pathlib import Path
import unittest
import cv2
import numpy as np
from perception import Classifier,compile_layout,load_config,pixels
from run import publish_stopped
R=Path(__file__).resolve().parent
class ConveyorTests(unittest.TestCase):
 def setUp(self):
  self.c=load_config(R/'config.json');self.layout=compile_layout(self.c);self.ref=cv2.imread(str(R/self.c['reference_image']));self.m=Classifier(self.c,self.ref);self.slot=self.layout['conveyor_slots'][5]
  self.floor=cv2.imread(str(R/self.slot['empty_reference']['reference_image']))
 def test_twenty_four_fixed_ids_from_actual_row_targets(self):
  self.assertEqual(len(self.layout['slots']),144);self.assertEqual(len(self.layout['conveyor_slots']),24)
  self.assertEqual({s['slot_id'] for s in self.layout['conveyor_slots']},{f'conveyor_r{r}_n{n}' for r in range(6) for n in range(4)})
  regions=self.c['conveyor_observations']['regions']
  for a,b in zip(regions,regions[1:]):self.assertAlmostEqual(b['configured_target_xyz_m'][1]-a['configured_target_xyz_m'][1],.0782)
  self.assertFalse(self.c['conveyor_observations']['provenance']['legacy_numeric_poses_7_to_13_used'])
 def test_pixel_color_empty_and_unknown_not_neighbor_copy(self):
  self.m.strict_camera_empty=True;self.m.allow_empty=True
  self.assertEqual(self.m.classify(self.floor,self.slot)['status'],'empty')
  redref=self.c['color_references']['300121']['polygons_px'][0];values=pixels(self.ref,redref);red=tuple(map(int,np.median(values,axis=0)))
  colored=self.floor.copy();cv2.fillConvexPoly(colored,np.round(self.slot['brick_top_quad_px']).astype(np.int32),red)
  self.assertEqual(self.m.classify(colored,self.slot)['color_id'],300121)
  dark=self.floor.copy();cv2.fillConvexPoly(dark,np.round(self.slot['brick_top_quad_px']).astype(np.int32),(20,20,20))
  self.assertEqual(self.m.classify(dark,self.slot)['status'],'uncertain')
  unverified=copy.deepcopy(self.slot);unverified['empty_reference']['material_reference_verified']=False
  self.assertEqual(self.m.classify(self.floor,unverified)['status'],'uncertain')
 def test_fresh_capture_floor_maps_under_registered_camera_shift(self):
  raw=cv2.imread(str(R/'captures/conveyor_current.jpg'));d=json.loads((R/'captures/conveyor_current.json').read_text());t=d['frame_unix_ns']
  for _ in range(5):rows=self.m.update(raw,self.layout,t,{'stale':True,'robot_moving':True},t,[])
  conveyor=[s for s in rows if s['location_type']=='conveyor'];self.assertEqual(len(conveyor),24)
  self.assertTrue(all(s['status']=='empty' and s['confidence']>=.8 for s in conveyor))
  self.assertEqual(self.m.guard_status,'unknown_camera_evidence_only');self.assertTrue(self.m.strict_camera_empty)
 def test_actual_removed_lane_uses_only_pre_exchange_material_profiles(self):
  raw=cv2.imread(str(R/'captures/conveyor_removed_current.jpg'));d=json.loads((R/'captures/conveyor_removed_current.json').read_text());t=d['frame_unix_ns']
  for row in d.get('conveyor_refinement',[]):
   if row.get('reference_to_refined'):
    identity=row['footprint_id'];self.m.conveyor_refinement.retained[identity]={'sampling':np.array(row['reference_to_refined']),'body':np.array(row['reference_to_body'])}
    self.m.conveyor_refinement.evidence[identity]={k:row[k] for k in ('observed_unix_ns','maximum_corner_shift_px','observed_colored_body_quad_px','observed_sampling_quad_px','measured_front_face_fraction')}
  for _ in range(5):rows=self.m.update(raw,self.layout,t,{'stale':True,'robot_moving':True},t,[])
  conveyor=[s for s in rows if s['location_type']=='conveyor'];self.assertEqual(len(conveyor),24)
  self.assertTrue(all(s['status']=='empty' and s['floor_white_fraction']>=.98 and s['floor_material_difference']<=15 and s['confidence']>=.8 for s in conveyor))
  self.assertTrue(all('PRE-exchange' in s['empty_reference']['material_profile_provenance'] for s in self.layout['conveyor_slots']))
 def test_stopped_producer_invalidates_both_collections(self):
  with tempfile.TemporaryDirectory() as directory:
   p=Path(directory)
   with open(p/'publish.lock','a') as lock:publish_stopped(None,self.layout,p,lock,[])
   d=json.loads((p/'latest.json').read_text());self.assertEqual(len(d['conveyor_slots']),24)
   self.assertTrue(all(s['status']=='uncertain' and not s['visible'] for s in d['conveyor_slots']))
if __name__=='__main__':unittest.main()
