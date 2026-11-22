"""Bounded camera FIFO draining and true end-of-work expiry, no device access."""
import unittest
from unittest.mock import patch
from run import drain_latest_frame,finalize_packet_age
class FakeCapture:
 fd=42
 def __init__(self,queue):self.queue=list(queue);self.timeouts=[]
 def read(self,timeout):self.timeouts.append(timeout);return self.queue.pop(0)
class CaptureIOTests(unittest.TestCase):
 def readiness(self,capture):return lambda *_:([capture.fd] if capture.queue else [],[],[])
 def test_latest_buffer_preserves_kernel_sequence_and_stamp(self):
  first=(10,100.1,'old');capture=FakeCapture([(11,100.2,'middle'),(12,100.3,'latest')])
  with patch('run.select.select',side_effect=self.readiness(capture)):frame,discarded=drain_latest_frame(capture,first)
  self.assertEqual(frame,(12,100.3,'latest'));self.assertEqual(discarded,2);self.assertEqual(capture.timeouts,[0,0])
 def test_drain_is_bounded_and_empty_queue_does_not_read(self):
  capture=FakeCapture([(n,n/10,'frame') for n in range(1,100)])
  with patch('run.select.select',side_effect=self.readiness(capture)):frame,discarded=drain_latest_frame(capture,(0,0.,'first'))
  self.assertEqual(discarded,6);self.assertEqual(frame[0],6);self.assertEqual(len(capture.queue),93)
  empty=FakeCapture([])
  with patch('run.select.select',side_effect=self.readiness(empty)):frame,discarded=drain_latest_frame(empty,(7,123.,'single'))
  self.assertEqual(frame,(7,123.,'single'));self.assertEqual(discarded,0);self.assertEqual(empty.timeouts,[])
 def test_late_publication_expires_both_arrays_without_timestamp_refresh(self):
  slot={'status':'occupied','color_id':300121,'confidence':.9,'visible':True};packet={'frame_unix_ns':123456,'capture_age_at_processing_s':.02,'stale':False,'slots':[dict(slot)],'conveyor_slots':[dict(slot)]}
  self.assertTrue(finalize_packet_age(packet,100.,.5,100.71,999999));self.assertEqual(packet['frame_unix_ns'],123456);self.assertAlmostEqual(packet['frame_age_at_publish_s'],.71)
  self.assertTrue(all(s['status']=='uncertain' and not s['visible'] and s['reason']=='stale_camera_frame' for s in packet['slots']+packet['conveyor_slots']))
 def test_fresh_publication_preserves_evidence(self):
  packet={'frame_unix_ns':123456,'stale':False,'slots':[{'status':'occupied'}],'conveyor_slots':[{'status':'empty'}]}
  self.assertFalse(finalize_packet_age(packet,100.,.5,100.14,999999));self.assertEqual(packet['slots'][0]['status'],'occupied');self.assertEqual(packet['conveyor_slots'][0]['status'],'empty');self.assertEqual(packet['frame_unix_ns'],123456)
if __name__=='__main__':unittest.main()
