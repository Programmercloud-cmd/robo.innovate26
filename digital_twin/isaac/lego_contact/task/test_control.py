import dataclasses
import math
import unittest
try:
    from .control import VerticalResidualInterface,make_handoff_frame,downward_nominal,vertical_error
except ImportError:
    from control import VerticalResidualInterface,make_handoff_frame,downward_nominal,vertical_error
from test_residual_interface import synthetic_contract,measured
from residual_interface import Pose,RecoverableStop,WrenchBias,yaw_rotation,multiply

BIAS=WrenchBias((0.,)*6,"synthetic",.5,True)


class VerticalTests(unittest.TestCase):
    def test_world_vertical_independent_of_yaw(self):
        for yaw in (0.,.5,math.pi):
            frame=make_handoff_frame((0.,0.,0.),yaw)
            nominal=frame.compose(downward_nominal())
            self.assertEqual(vertical_error(nominal.rotation),0.)
            VerticalResidualInterface(synthetic_contract(),frame,downward_nominal(),BIAS,.01)

    def test_tilted_task_frame_and_tilted_nominal_rejected(self):
        angle=.02
        tilt=((1.,0.,0.),(0.,math.cos(angle),-math.sin(angle)),(0.,math.sin(angle),math.cos(angle)))
        with self.assertRaises(RecoverableStop):
            VerticalResidualInterface(synthetic_contract(),Pose((0,0,0),tilt),downward_nominal(),BIAS,.01)
        with self.assertRaises(RecoverableStop):
            VerticalResidualInterface(synthetic_contract(),Pose((0,0,0)),Pose((0,0,0),multiply(tilt,downward_nominal().rotation)),BIAS,.01)

    def test_actual_tilt_latches_stop(self):
        adapter=VerticalResidualInterface(synthetic_contract(),Pose((0,0,0)),downward_nominal(),BIAS,.01)
        state=measured(O_T_EE=downward_nominal())
        adapter.establish_handoff(state,1.)
        angle=.02
        tilt=((1.,0.,0.),(0.,math.cos(angle),-math.sin(angle)),(0.,math.sin(angle),math.cos(angle)))
        wrong=Pose((0,0,0),multiply(tilt,downward_nominal().rotation))
        with self.assertRaises(RecoverableStop):
            adapter.step(measured(stamp=1.01,sequence=2,O_T_EE=wrong),1.01)
        self.assertIn("tilt",adapter.stopped)

    def test_joint_candidate_requires_forward_kinematics(self):
        adapter=VerticalResidualInterface(synthetic_contract(),Pose((0,0,0)),downward_nominal(),BIAS,.01)
        adapter.establish_handoff(measured(O_T_EE=downward_nominal()),1.)
        adapter.set_action((0,0,0,0),1.,0,1.)
        request=adapter.step(measured(stamp=1.01,sequence=2,O_T_EE=downward_nominal()),1.01)
        with self.assertRaises(RecoverableStop):
            adapter.accept_ik_candidate(request,(0.,)*7,.5)
        self.assertIn("FK",adapter.stopped)


if __name__=="__main__":
    unittest.main()
