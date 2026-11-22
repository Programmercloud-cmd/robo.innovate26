"""Shared, SDK-free residual *target* contract; no actuator or identified servo.

T_A_B maps B into A. Actor inputs use the taught (possibly imperfect) M in
both domains. The discrete limiter bounds target differences, not robot motion;
continuous IK, callback interpolation and the actual impedance loop remain external.
No configuration, including a synthetic test profile, enables hardware here.
"""
from dataclasses import dataclass
import math


class RecoverableStop(RuntimeError):
    """Execution must stop; inventory/outcome must not advance."""


def vector(value, n, name="vector"):
    try:
        result = tuple(float(x) for x in value)
    except (TypeError, ValueError) as exc:
        raise RecoverableStop("invalid " + name) from exc
    if len(result) != n or not all(math.isfinite(x) for x in result):
        raise RecoverableStop("invalid " + name)
    return result


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def norm(a):
    return math.sqrt(dot(a, a))


def cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])


def add(a, b):
    return tuple(x + y for x, y in zip(a, b))


def sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def transpose(r):
    return tuple(zip(*r))


def rotate(r, v):
    return tuple(dot(row, v) for row in r)


def multiply(a, b):
    return tuple(tuple(dot(row, col) for col in transpose(b)) for row in a)


IDENTITY = ((1., 0., 0.), (0., 1., 0.), (0., 0., 1.))


def yaw_rotation(angle):
    c, s = math.cos(angle), math.sin(angle)
    return ((c, -s, 0.), (s, c, 0.), (0., 0., 1.))


def rotation_angle(r):
    return math.acos(max(-1., min(1., (sum(r[i][i] for i in range(3)) - 1.) / 2.)))


@dataclass(frozen=True)
class Pose:
    position: tuple
    rotation: tuple = IDENTITY

    def __post_init__(self):
        object.__setattr__(self, "position", vector(self.position, 3, "position"))
        try:
            r = tuple(vector(row, 3, "rotation") for row in self.rotation)
        except TypeError as exc:
            raise RecoverableStop("invalid rotation") from exc
        if len(r) != 3 or any(abs(dot(r[i], r[j]) - (i == j)) > 1e-6
                              for i in range(3) for j in range(3)) or dot(r[0], cross(r[1], r[2])) < .999999:
            raise RecoverableStop("rotation must be proper orthonormal")
        object.__setattr__(self, "rotation", r)

    def compose(self, other):
        return Pose(add(self.position, rotate(self.rotation, other.position)), multiply(self.rotation, other.rotation))

    def inverse(self):
        r = transpose(self.rotation)
        return Pose(rotate(r, tuple(-x for x in self.position)), r)


@dataclass(frozen=True)
class WrenchBias:
    """Bias in K, at K origin, under matching payload/filter/profile conditions."""
    wrench_K: tuple
    profile_id: str
    calibrated_at_s: float
    valid: bool


@dataclass(frozen=True)
class MeasuredState:
    """Measured inputs only. No true brick pose, engagement or contact labels."""
    sequence: int
    stamp_s: float
    O_T_EE: Pose
    O_T_K: Pose
    twist_O_at_EE: tuple
    q: tuple
    dq: tuple
    wrench_K: tuple
    wrench_stamp_s: float
    wrench_valid: bool
    wrench_profile_id: str
    gripper_width_m: float
    hand_stamp_s: float
    grasp_valid: bool
    singularity_metric: float
    valid: bool = True
    fault: bool = False
    controller_ready: bool = True


def wrench_at_M(state, taught_O_T_M, bias):
    """Subtract K bias BEFORE rotating and shifting K-origin moment to M.

    Preserves libfranka's signed estimated-wrench convention; no absolute value
    or sign inversion. Moment_M = R_M_K moment_K + p_M_K cross force_M.
    """
    if not bias.valid or bias.profile_id != state.wrench_profile_id:
        raise RecoverableStop("wrench bias/profile invalid")
    if not math.isfinite(bias.calibrated_at_s) or bias.calibrated_at_s > state.wrench_stamp_s:
        raise RecoverableStop("wrench predates bias calibration")
    corrected = sub(vector(state.wrench_K, 6, "wrench"), vector(bias.wrench_K, 6, "bias"))
    M_T_K = taught_O_T_M.inverse().compose(state.O_T_K)
    force = rotate(M_T_K.rotation, corrected[:3])
    moment = add(rotate(M_T_K.rotation, corrected[3:]), cross(M_T_K.position, force))
    return force + moment


class Limits:
    """All numeric values must be explicitly commissioned; no safety defaults."""
    def __init__(self, contract):
        try:
            c = contract["control_contract"]
            self.action = vector(c["maximum_action_xyz_m"], 3) + (float(c["maximum_action_yaw_rad"]),)
            self.travel = vector(c["maximum_total_travel_xyz_m"], 3) + (float(c["maximum_total_yaw_rad"]),)
            derivative = c["velocity_acceleration_jerk_limits"]
            self.velocity = vector(derivative["velocity"], 4)
            self.acceleration = vector(derivative["acceleration"], 4)
            self.jerk = vector(derivative["jerk"], 4)
            self.force = float(c["force_torque_limits"]["force_N"])
            self.torque = float(c["force_torque_limits"]["torque_Nm"])
            self.timeout = float(c["episode_timeout_s"])
            self.policy_rate = float(c["policy_rate_hz"])
            s = c["supervisor_limits"]
            for name in ("max_state_age_s", "max_wrench_age_s", "max_hand_age_s", "max_policy_age_s",
                         "max_step_s", "minimum_singularity_metric", "max_joint_step_rad",
                         "handoff_linear_speed_m_s", "handoff_angular_speed_rad_s", "handoff_joint_speed_rad_s",
                         "handoff_position_tolerance_m", "handoff_angle_tolerance_rad",
                         "tracking_position_tolerance_m", "tracking_angle_tolerance_rad"):
                setattr(self, name, float(s[name]))
            for name, size in (("joint_lower_rad", 7), ("joint_upper_rad", 7),
                               ("joint_velocity_rad_s", 7), ("workspace_lower_m", 3), ("workspace_upper_m", 3)):
                setattr(self, name, vector(s[name], size, name))
        except (KeyError, TypeError, ValueError) as exc:
            raise RecoverableStop("missing/uncommissioned residual limits") from exc
        positive = self.action + self.travel + self.velocity + self.acceleration + self.jerk + self.joint_velocity_rad_s
        positive += (self.force, self.torque, self.timeout, self.policy_rate)
        positive += tuple(getattr(self, k) for k in vars(self) if k.startswith(("max_", "minimum_", "handoff_", "tracking_")))
        if not all(math.isfinite(x) and x > 0 for x in positive):
            raise RecoverableStop("limits must be finite and positive")
        if any(a >= b for a, b in zip(self.joint_lower_rad, self.joint_upper_rad)) or any(
                a >= b for a, b in zip(self.workspace_lower_m, self.workspace_upper_m)):
            raise RecoverableStop("invalid bounds")


def age(now, stamp, maximum, label):
    if not math.isfinite(now) or not math.isfinite(stamp) or not 0 <= now - stamp <= maximum:
        raise RecoverableStop("stale/future " + label)


def validate_state(state, now, limits, taught_O_T_M, bias):
    if not state.valid or state.fault or not state.controller_ready:
        raise RecoverableStop("invalid/fault/not-ready robot state")
    if not state.grasp_valid:
        raise RecoverableStop("uncertain grasp")
    if not state.wrench_valid or not math.isfinite(state.singularity_metric) or state.singularity_metric < limits.minimum_singularity_metric:
        raise RecoverableStop("invalid wrench or singularity")
    age(now, state.stamp_s, limits.max_state_age_s, "state")
    age(now, state.wrench_stamp_s, limits.max_wrench_age_s, "wrench")
    age(now, state.hand_stamp_s, limits.max_hand_age_s, "hand")
    q, dq = vector(state.q, 7, "q"), vector(state.dq, 7, "dq")
    vector(state.twist_O_at_EE, 6, "twist")
    if not math.isfinite(state.gripper_width_m) or state.gripper_width_m < 0:
        raise RecoverableStop("invalid measured gripper width")
    if any(not lo < x < hi for x, lo, hi in zip(q, limits.joint_lower_rad, limits.joint_upper_rad)):
        raise RecoverableStop("joint position bound")
    if any(abs(x) > limit for x, limit in zip(dq, limits.joint_velocity_rad_s)):
        raise RecoverableStop("joint velocity bound")
    if any(not lo <= x <= hi for x, lo, hi in zip(state.O_T_EE.position, limits.workspace_lower_m, limits.workspace_upper_m)):
        raise RecoverableStop("workspace bound")
    wrench = wrench_at_M(state, taught_O_T_M, bias)
    if norm(wrench[:3]) > limits.force or norm(wrench[3:]) > limits.torque:
        raise RecoverableStop("force/torque bound")
    return wrench


def actor_observation(state, now, taught_O_T_M, commanded_M_T_EE, previous_action, bias, limits):
    """Whitelisted, deployment-available actor fields; never accepts hidden truth."""
    wrench = validate_state(state, now, limits, taught_O_T_M, bias)
    M_T_EE = taught_O_T_M.inverse().compose(state.O_T_EE)
    r = transpose(taught_O_T_M.rotation)
    error_rotation = multiply(transpose(M_T_EE.rotation), commanded_M_T_EE.rotation)
    return {
        "tcp_position_M": M_T_EE.position,
        "tcp_rotation_M": sum(M_T_EE.rotation, ()),
        "tcp_twist_M_at_EE": rotate(r, state.twist_O_at_EE[:3]) + rotate(r, state.twist_O_at_EE[3:]),
        "setpoint_position_error_M": sub(commanded_M_T_EE.position, M_T_EE.position),
        "setpoint_rotation_error_EE": sum(error_rotation, ()),
        "previous_action": vector(previous_action, 4, "previous action"),
        "estimated_wrench_M_at_M": wrench,
        "gripper_width_m": state.gripper_width_m,
        "ages_s": (now-state.stamp_s, now-state.wrench_stamp_s, now-state.hand_stamp_s),
        "valid": True,
    }


class LocalLimiter:
    """Local quintic target interpolation, not an actuator trajectory certificate.

    Unchanged goals finish their frozen segment with zero velocity/acceleration.
    Changed goals preserve current p/v/a. Bernstein convex-hull bounds certify
    each segment's envelope and derivatives; infeasible braking stops explicitly.
    External hardware still needs a commissioned controlled-stop path and IK.
    """
    def __init__(self, position):
        self.position = vector(position, 4)
        self.velocity = (0.,)*4
        self.acceleration = (0.,)*4
        self.travel = (0.,)*4
        self.target = None
        self.controls = None
        self.elapsed = 0.

    @staticmethod
    def _split(points, t):
        rows = [tuple(points)]
        while len(rows[-1]) > 1:
            rows.append(tuple((1.-t)*a+t*b for a, b in zip(rows[-1], rows[-1][1:])))
        return tuple(row[0] for row in rows), tuple(row[-1] for row in reversed(rows))

    @staticmethod
    def _derivative(points, duration):
        return tuple((len(points)-1)*(b-a)/duration for a, b in zip(points, points[1:]))

    @classmethod
    def _pieces(cls, points, depth=3):
        if depth == 0:
            return [points]
        left, right = cls._split(points, .5)
        return cls._pieces(left, depth-1)+cls._pieces(right, depth-1)

    @classmethod
    def _certified(cls, controls, duration, limits):
        for i, points in enumerate(controls):
            # Subdivision tightens the convex hull without sampled-only checks.
            for piece in cls._pieces(points):
                if any(abs(p) > limits.action[i]+1e-12 for p in piece):
                    return False
                derivative = piece
                for bound in (limits.velocity[i], limits.acceleration[i], limits.jerk[i]):
                    derivative = cls._derivative(derivative, duration/8.)
                    if any(abs(x) > bound+1e-12 for x in derivative):
                        return False
        return True

    def _plan(self, target, dt, limits):
        duration = dt
        while True:
            controls = tuple((p, p+v*duration/5., p+2*v*duration/5.+a*duration**2/20., goal, goal, goal)
                             for p, v, a, goal in zip(self.position, self.velocity, self.acceleration, target))
            if self._certified(controls, duration, limits):
                return controls, duration
            if duration >= limits.timeout:
                raise RecoverableStop("no certified local interpolation: insufficient braking room or time")
            duration = min(limits.timeout, duration*1.15)

    def advance(self, target, dt, limits):
        if not 0 < dt <= limits.max_step_s:
            raise RecoverableStop("invalid interpolation interval")
        target = vector(target, 4, "target")
        if any(abs(x) > bound for x, bound in zip(target, limits.action)):
            raise RecoverableStop("target outside local envelope")
        if target != self.target:
            controls, duration = self._plan(target, dt, limits)
            elapsed = 0.
        else:
            controls, duration, elapsed = self.controls, self.duration, self.elapsed
        next_elapsed = min(duration, elapsed+dt)
        t0, t1 = elapsed/duration, next_elapsed/duration
        p_next, v_next, a_next, path_bound = [], [], [], []
        for points in controls:
            velocity_points = self._derivative(points, duration)
            acceleration_points = self._derivative(velocity_points, duration)
            p_next.append(self._split(points, t1)[0][-1])
            v_next.append(self._split(velocity_points, t1)[0][-1])
            a_next.append(self._split(acceleration_points, t1)[0][-1])
            prefix = self._split(points, t1)[0]
            interval = self._split(prefix, t0/t1)[1] if t1 else prefix
            # Control polygon bounds true absolute travel, including a turn
            # between samples; endpoint displacement alone would undercount it.
            path_bound.append(sum(abs(b-a) for a, b in zip(interval, interval[1:])))
        travel = add(self.travel, path_bound)
        if any(x > budget for x, budget in zip(travel, limits.travel)):
            raise RecoverableStop("cumulative residual travel budget")
        self.position, self.velocity, self.acceleration, self.travel = tuple(p_next), tuple(v_next), tuple(a_next), travel
        self.target, self.controls, self.duration, self.elapsed = target, controls, duration, next_elapsed
        return self.position


@dataclass(frozen=True)
class TargetRequest:
    sequence: int
    stamp_s: float
    dt_s: float
    residual_M: tuple
    M_T_EE: Pose
    O_T_EE: Pose
    ik_seed_q: tuple


class ResidualInterface:
    """Latched supervisor and target adapter; no SDK or actuator dependency."""
    def __init__(self, contract, taught_O_T_M, nominal_M_T_EE, bias):
        self.limits = Limits(contract)
        self.taught, self.nominal, self.bias = taught_O_T_M, nominal_M_T_EE, bias
        self.stopped = "fresh stationary handoff required"
        self.started = None

    @staticmethod
    def require_hardware_execution():
        raise RecoverableStop("hardware execution unsupported: actuator surrogate/interface unimplemented and unidentified")

    def _latch(self, operation):
        try:
            return operation()
        except RecoverableStop as exc:
            self.stopped = str(exc)
            raise

    def establish_handoff(self, state, now):
        def reset():
            if self.started is not None:
                if self.stopped is None:
                    raise RecoverableStop("cannot reset an active residual episode")
                if state.sequence <= self.last_sequence or state.stamp_s <= self.last_stamp:
                    raise RecoverableStop("handoff must use new measured state")
            validate_state(state, now, self.limits, self.taught, self.bias)
            relative = self.taught.inverse().compose(state.O_T_EE)
            error = sub(relative.position, self.nominal.position)
            angle = rotation_angle(multiply(relative.rotation, transpose(self.nominal.rotation)))
            if norm(error) > self.limits.handoff_position_tolerance_m or angle > self.limits.handoff_angle_tolerance_rad:
                raise RecoverableStop("handoff differs from taught nominal")
            if (norm(state.twist_O_at_EE[:3]) > self.limits.handoff_linear_speed_m_s
                    or norm(state.twist_O_at_EE[3:]) > self.limits.handoff_angular_speed_rad_s
                    or any(abs(x) > self.limits.handoff_joint_speed_rad_s for x in state.dq)):
                raise RecoverableStop("handoff not stationary")
            delta_r = multiply(relative.rotation, transpose(self.nominal.rotation))
            initial = error + (math.atan2(delta_r[1][0], delta_r[0][0]),)
            if any(abs(x) > b for x, b in zip(initial, self.limits.action)):
                raise RecoverableStop("handoff outside residual envelope")
            self.limiter = LocalLimiter(initial)
            self.started = self.last_time = now
            self.last_sequence, self.last_stamp = state.sequence, state.stamp_s
            self.policy_stamp, self.policy_sequence = now, -1
            self.requested = self.previous_action = (0.,)*4
            self.ik_seed = vector(state.q, 7)
            self.request_sequence = 0
            self.accepted_request_sequence = 0
            self.last_request = None
            self.stopped = None
        return self._latch(reset)

    def set_action(self, normalized_absolute_action, stamp_s, sequence, now):
        def update():
            if self.stopped:
                raise RecoverableStop(self.stopped)
            age(now, stamp_s, self.limits.max_policy_age_s, "policy")
            if sequence <= self.policy_sequence or stamp_s < self.policy_stamp:
                raise RecoverableStop("nonprogressing policy action")
            action = vector(normalized_absolute_action, 4, "action")
            self.previous_action = tuple(max(-1., min(1., x)) for x in action)
            self.requested = tuple(x*b for x, b in zip(self.previous_action, self.limits.action))
            self.policy_stamp, self.policy_sequence = stamp_s, sequence
        return self._latch(update)

    def step(self, state, now):
        def advance():
            if self.stopped:
                raise RecoverableStop(self.stopped)
            if now-self.started > self.limits.timeout:
                raise RecoverableStop("episode timeout")
            age(now, self.policy_stamp, self.limits.max_policy_age_s, "policy")
            validate_state(state, now, self.limits, self.taught, self.bias)
            if state.sequence <= self.last_sequence or state.stamp_s <= self.last_stamp:
                raise RecoverableStop("nonprogressing measured state")
            if self.last_request is not None:
                if norm(sub(state.O_T_EE.position, self.last_request.O_T_EE.position)) > self.limits.tracking_position_tolerance_m or rotation_angle(
                        multiply(state.O_T_EE.rotation, transpose(self.last_request.O_T_EE.rotation))) > self.limits.tracking_angle_tolerance_rad:
                    raise RecoverableStop("tracking discrepancy")
            dt = now-self.last_time
            residual = self.limiter.advance(self.requested, dt, self.limits)
            pose = Pose(add(self.nominal.position, residual[:3]), multiply(yaw_rotation(residual[3]), self.nominal.rotation))
            world = self.taught.compose(pose)
            if any(not lo <= x <= hi for x, lo, hi in zip(world.position, self.limits.workspace_lower_m, self.limits.workspace_upper_m)):
                raise RecoverableStop("target workspace bound")
            self.request_sequence += 1
            request = TargetRequest(self.request_sequence, now, dt, residual, pose, world, self.ik_seed)
            self.last_request = request
            self.last_time, self.last_sequence, self.last_stamp = now, state.sequence, state.stamp_s
            return request
        return self._latch(advance)

    def accept_ik_candidate(self, request, q, singularity_metric):
        """External solver seeds continuously; no yaw-branch search or IK here.

        Candidate guard is necessary, not a trajectory certificate: downstream
        must also check intermediate FK, branch, tracking and SDK constraints.
        """
        def check():
            if self.stopped or request is not self.last_request:
                raise RecoverableStop(self.stopped or "obsolete IK candidate")
            if request.sequence <= self.accepted_request_sequence:
                raise RecoverableStop("IK request already consumed")
            q_checked = vector(q, 7, "IK q")
            if not math.isfinite(singularity_metric) or singularity_metric < self.limits.minimum_singularity_metric:
                raise RecoverableStop("IK singularity")
            if any(not lo < x < hi for x, lo, hi in zip(q_checked, self.limits.joint_lower_rad, self.limits.joint_upper_rad)):
                raise RecoverableStop("IK joint bounds")
            if any(abs(x-y) > min(self.limits.max_joint_step_rad, v*request.dt_s)
                   for x, y, v in zip(q_checked, request.ik_seed_q, self.limits.joint_velocity_rad_s)):
                raise RecoverableStop("IK discontinuity/rate")
            self.ik_seed = q_checked
            self.accepted_request_sequence = request.sequence
            return q_checked
        return self._latch(check)
