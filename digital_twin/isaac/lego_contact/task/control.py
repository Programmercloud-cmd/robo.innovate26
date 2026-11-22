"""Task-specific vertical-only residual target adapter. No hardware/IK solver."""
import math
try:
    from ..residual_interface import Pose, RecoverableStop, ResidualInterface, norm, sub, rotation_angle, multiply, transpose, yaw_rotation
except ImportError:
    from residual_interface import Pose, RecoverableStop, ResidualInterface, norm, sub, rotation_angle, multiply, transpose, yaw_rotation


def vertical_error(rotation):
    """Angle of configured EE +Z axis from world down, independent of yaw."""
    return math.acos(max(-1.,min(1.,-rotation[2][2])))


def make_handoff_frame(target_xyz_O, target_yaw_O):
    """Horizontal task frame, even when the measured baseplate is tilted.

    Brick pose and plate normal remain separate contact-model inputs. This
    intentionally does not rotate the tool into an unmeasured tilted plate.
    """
    return Pose(tuple(target_xyz_O),yaw_rotation(float(target_yaw_O)))


class VerticalResidualInterface(ResidualInterface):
    """World-vertical invariant in addition to existing independent guards."""
    def __init__(self,contract,taught_O_T_M,nominal_M_T_EE,bias,measured_vertical_tolerance_rad):
        tolerance=measured_vertical_tolerance_rad
        if tolerance is None or not math.isfinite(tolerance) or not 0<tolerance<math.pi/2:
            raise RecoverableStop("uncommissioned measured vertical tracking tolerance")
        # A horizontal yaw frame makes every policy yaw world vertical. It is
        # not a tolerance relaxation for a tilted plate or taught nominal tool.
        if any(abs(taught_O_T_M.rotation[i][2]-(i==2))>1e-10 for i in range(3)):
            raise RecoverableStop("task residual yaw axis must be world vertical")
        world_nominal=taught_O_T_M.compose(nominal_M_T_EE)
        if vertical_error(world_nominal.rotation)>1e-7:
            raise RecoverableStop("nominal tool must be exactly world vertical")
        self.vertical_tolerance=tolerance
        super().__init__(contract,taught_O_T_M,nominal_M_T_EE,bias)

    def _check_vertical(self,state):
        if vertical_error(state.O_T_EE.rotation)>self.vertical_tolerance:
            self.stopped="measured tool tilt exceeds vertical-only task tolerance"
            raise RecoverableStop(self.stopped)

    def establish_handoff(self,state,now):
        self._check_vertical(state)
        return super().establish_handoff(state,now)

    def step(self,state,now):
        self._check_vertical(state)
        request=super().step(state,now)
        if vertical_error(request.O_T_EE.rotation)>1e-7:
            self.stopped="generated tool tilt violates vertical-only actions"
            raise RecoverableStop(self.stopped)
        return request

    def accept_ik_candidate(self,request,q,singularity_metric,*,candidate_O_T_EE=None):
        """Require FK evidence from the external model before accepting joints.

        The caller must compute this pose from exactly q using the configured
        tool frame. This API does not itself implement or certify that solver.
        """
        if not isinstance(candidate_O_T_EE,Pose):
            self.stopped="candidate FK required for vertical-only task"
            raise RecoverableStop(self.stopped)
        if vertical_error(candidate_O_T_EE.rotation)>1e-7:
            self.stopped="candidate joint FK tilts the tool"
            raise RecoverableStop(self.stopped)
        if (norm(sub(candidate_O_T_EE.position,request.O_T_EE.position))>self.limits.tracking_position_tolerance_m
                or rotation_angle(multiply(candidate_O_T_EE.rotation,transpose(request.O_T_EE.rotation)))>self.limits.tracking_angle_tolerance_rad):
            self.stopped="candidate FK differs from requested target"
            raise RecoverableStop(self.stopped)
        return super().accept_ik_candidate(request,q,singularity_metric)


def downward_nominal():
    return Pose((0.,0.,0.),((1.,0.,0.),(0.,-1.,0.),(0.,0.,-1.)))
