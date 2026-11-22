"""Synthetic numerical checks only; not physical commissioning evidence."""
import copy
from dataclasses import replace
import inspect
import json
import math
from pathlib import Path
import unittest

from residual_interface import (IDENTITY, LocalLimiter, MeasuredState, Pose,
                                RecoverableStop, ResidualInterface, WrenchBias,
                                actor_observation, wrench_at_M, yaw_rotation)


def synthetic_contract():
    return {"hardware_deployment_enabled": True, "control_contract": {
        "policy_rate_hz": 50., "maximum_action_xyz_m": [.02]*3,
        "maximum_action_yaw_rad": .2, "maximum_total_travel_xyz_m": [1.]*3,
        "maximum_total_yaw_rad": 2., "episode_timeout_s": 2.,
        "velocity_acceleration_jerk_limits": {"velocity": [.02]*4, "acceleration": [.2]*4, "jerk": [2.]*4},
        "force_torque_limits": {"force_N": 20., "torque_Nm": 2.},
        "supervisor_limits": {
            "max_state_age_s": .05, "max_wrench_age_s": .05, "max_hand_age_s": .05,
            "max_policy_age_s": .05, "max_step_s": .02, "minimum_singularity_metric": .1,
            "max_joint_step_rad": .02, "joint_lower_rad": [-2.]*7, "joint_upper_rad": [2.]*7,
            "joint_velocity_rad_s": [.5]*7, "workspace_lower_m": [-1.]*3, "workspace_upper_m": [1.]*3,
            "handoff_linear_speed_m_s": .001, "handoff_angular_speed_rad_s": .01, "handoff_joint_speed_rad_s": .01,
            "handoff_position_tolerance_m": .001, "handoff_angle_tolerance_rad": .01,
            "tracking_position_tolerance_m": .003, "tracking_angle_tolerance_rad": .02}}}


def measured(stamp=1., sequence=1, **changes):
    state = MeasuredState(sequence=sequence, stamp_s=stamp, O_T_EE=Pose((0.,)*3),
                          O_T_K=Pose((0.,)*3), twist_O_at_EE=(0.,)*6,
                          q=(0.,)*7, dq=(0.,)*7, wrench_K=(0.,)*6,
                          wrench_stamp_s=stamp, wrench_valid=True, wrench_profile_id="synthetic",
                          gripper_width_m=.014, hand_stamp_s=stamp, grasp_valid=True,
                          singularity_metric=.5)
    return replace(state, **changes)


class ResidualTests(unittest.TestCase):
    def adapter(self, contract=None):
        return ResidualInterface(contract or synthetic_contract(), Pose((0.,)*3), Pose((0.,)*3),
                                 WrenchBias((0.,)*6, "synthetic", .5, True))

    def ready(self):
        adapter = self.adapter()
        adapter.establish_handoff(measured(), 1.)
        adapter.set_action((.5, 0., 0., 0.), 1., 0, 1.)
        return adapter

    def test_absolute_action_does_not_accumulate_and_clips_total_target(self):
        adapter = self.ready()
        self.assertEqual(adapter.requested, (.01, 0., 0., 0.))
        adapter.set_action((.5, 0., 0., 0.), 1.01, 1, 1.01)
        self.assertEqual(adapter.requested, (.01, 0., 0., 0.))
        adapter.set_action((99., 0., 0., -99.), 1.02, 2, 1.02)
        self.assertEqual(adapter.requested, (.02, 0., 0., -.2))

    def test_rotated_taught_frame_and_bias_then_moment_shift(self):
        taught = Pose((0.,)*3, yaw_rotation(math.pi/2))
        state = measured(O_T_K=Pose((.2, .1, 0.)), wrench_K=(2., 0., 0., 0., 0., .7))
        bias = WrenchBias((1., 0., 0., 0., 0., .2), "synthetic", .5, True)
        transformed = wrench_at_M(state, taught, bias)
        for actual, expected in zip(transformed, (0., -1., 0., 0., 0., .4)):
            self.assertAlmostEqual(actual, expected)
        adapter = self.adapter()
        state = measured(O_T_EE=taught.compose(Pose((.01, .002, 0.))), twist_O_at_EE=(1., 0., 0., 0., 0., 1.))
        obs = actor_observation(state, 1., taught, Pose((.015, .002, 0.)), (0.,)*4, bias, adapter.limits)
        self.assertAlmostEqual(obs["tcp_position_M"][0], .01)
        self.assertAlmostEqual(obs["setpoint_position_error_M"][0], .005)
        self.assertAlmostEqual(obs["tcp_twist_M_at_EE"][1], -1.)

    def test_actor_has_no_truth_or_engagement_interface(self):
        adapter = self.adapter()
        params = inspect.signature(actor_observation).parameters
        self.assertFalse(any("brick" in key or "truth" in key or "engagement" in key for key in params))
        with self.assertRaises(TypeError):
            actor_observation(measured(), 1., adapter.taught, adapter.nominal, (0.,)*4,
                              adapter.bias, adapter.limits, true_brick_pose=Pose((0.,)*3))
        obs = actor_observation(measured(), 1., adapter.taught, adapter.nominal, (0.,)*4, adapter.bias, adapter.limits)
        self.assertEqual(set(obs), {"tcp_position_M", "tcp_rotation_M", "tcp_twist_M_at_EE",
                                   "setpoint_position_error_M", "setpoint_rotation_error_EE", "previous_action",
                                   "estimated_wrench_M_at_M", "gripper_width_m", "ages_s", "valid"})

    def test_real_null_limits_and_synthetic_enable_fail_closed(self):
        actual = json.loads(Path(__file__).with_name("real_contract.json").read_text())
        with self.assertRaises(RecoverableStop):
            self.adapter(actual)
        # Even a complete synthetic profile that says True cannot enable hardware.
        with self.assertRaisesRegex(RecoverableStop, "hardware execution unsupported"):
            self.adapter().require_hardware_execution()
        invalid = synthetic_contract()
        del invalid["control_contract"]["supervisor_limits"]["max_state_age_s"]
        with self.assertRaises(RecoverableStop):
            self.adapter(invalid)

    def test_independent_ages_invalid_flags_fault_and_singularity_stop(self):
        variants = [dict(stamp_s=.9), dict(wrench_stamp_s=.9), dict(hand_stamp_s=.9),
                    dict(stamp_s=1.1), dict(valid=False), dict(fault=True), dict(controller_ready=False),
                    dict(grasp_valid=False), dict(wrench_valid=False), dict(singularity_metric=0.),
                    dict(wrench_profile_id="different-payload"), dict(wrench_K=(float("nan"),)*6)]
        for change in variants:
            with self.subTest(change=change):
                adapter = self.adapter()
                with self.assertRaises(RecoverableStop):
                    adapter.establish_handoff(measured(**change), 1.)
                self.assertIsNotNone(adapter.stopped)

    def test_handoff_must_be_fresh_stationary_near_taught_pose(self):
        for state in (measured(O_T_EE=Pose((.002, 0., 0.))),
                      measured(twist_O_at_EE=(.01, 0., 0., 0., 0., 0.)), measured(dq=(.02,)*7)):
            with self.assertRaises(RecoverableStop):
                self.adapter().establish_handoff(state, 1.)

    def test_active_episode_cannot_reset_history_and_recovery_requires_new_state(self):
        adapter = self.ready()
        with self.assertRaisesRegex(RecoverableStop, "active residual"):
            adapter.establish_handoff(measured(1.01, 2), 1.01)
        with self.assertRaisesRegex(RecoverableStop, "new measured state"):
            adapter.establish_handoff(measured(), 1.01)
        adapter.establish_handoff(measured(1.02, 3), 1.02)
        self.assertIsNone(adapter.stopped)

    def test_policy_staleness_and_state_progress_latch_until_fresh_handoff(self):
        adapter = self.ready()
        with self.assertRaisesRegex(RecoverableStop, "policy"):
            adapter.step(measured(1.06, 2), 1.06)
        with self.assertRaises(RecoverableStop):
            adapter.set_action((0.,)*4, 1.06, 1, 1.06)
        adapter.establish_handoff(measured(1.07, 3), 1.07)
        with self.assertRaisesRegex(RecoverableStop, "nonprogressing measured"):
            adapter.step(measured(1.07, 3), 1.08)

    def test_timeout_tracking_force_joint_and_workspace_guards(self):
        tests = [(dict(wrench_K=(21., 0., 0., 0., 0., 0.)), "force/torque"),
                 (dict(q=(3.,)*7), "joint position"), (dict(dq=(1.,)*7), "joint velocity"),
                 (dict(O_T_EE=Pose((2., 0., 0.))), "workspace")]
        for changes, reason in tests:
            with self.subTest(reason=reason):
                adapter = self.ready()
                with self.assertRaisesRegex(RecoverableStop, reason):
                    adapter.step(measured(1.01, 2, **changes), 1.01)
        adapter = self.ready()
        adapter.step(measured(1.01, 2), 1.01)
        with self.assertRaisesRegex(RecoverableStop, "tracking"):
            adapter.step(measured(1.02, 3, O_T_EE=Pose((.005, 0., 0.))), 1.02)
        adapter = self.ready()
        with self.assertRaisesRegex(RecoverableStop, "episode timeout"):
            adapter.step(measured(3.1, 2), 3.1)

    def test_local_limiter_derivatives_and_cumulative_travel(self):
        limits = self.adapter().limits
        limiter = LocalLimiter((0.,)*4)
        p, v, a = limiter.position, limiter.velocity, limiter.acceleration
        for index in range(60):
            next_p = limiter.advance((.001 if index < 30 else -.001, 0., 0., 0.), .01, limits)
            discrete_v, discrete_a = [], []
            for i in range(4):
                next_v = (next_p[i]-p[i])/.01
                next_a = (next_v-v[i])/.01
                next_j = (next_a-a[i])/.01
                self.assertLessEqual(abs(next_v), limits.velocity[i]+1e-10)
                self.assertLessEqual(abs(next_a), limits.acceleration[i]+1e-8)
                self.assertLessEqual(abs(next_j), limits.jerk[i]+1e-6)
                self.assertLessEqual(abs(limiter.velocity[i]), limits.velocity[i]+1e-12)
                self.assertLessEqual(abs(limiter.acceleration[i]), limits.acceleration[i]+1e-12)
                discrete_v.append(next_v)
                discrete_a.append(next_a)
            p, v, a = limiter.position, tuple(discrete_v), tuple(discrete_a)
        limits.travel = (1e-12,)*4
        with self.assertRaisesRegex(RecoverableStop, "travel budget"):
            LocalLimiter((0.,)*4).advance((.01, 0., 0., 0.), .01, limits)

    def test_held_small_and_boundary_targets_finish_stationary(self):
        limits = self.adapter().limits
        for goal in (.001, .01, .02, -.02):
            with self.subTest(goal=goal):
                limiter = LocalLimiter((0.,)*4)
                for _ in range(220):
                    limiter.advance((goal, 0., 0., 0.), .01, limits)
                self.assertAlmostEqual(limiter.position[0], goal, places=12)
                self.assertAlmostEqual(limiter.velocity[0], 0., places=12)
                self.assertAlmostEqual(limiter.acceleration[0], 0., places=12)
                self.assertAlmostEqual(limiter.travel[0], abs(goal), places=10)

    def test_insufficient_braking_room_stops_without_position_clamp(self):
        limits = self.adapter().limits
        limiter = LocalLimiter((.019999, 0., 0., 0.))
        limiter.velocity = (.02, 0., 0., 0.)
        before = copy.deepcopy(vars(limiter))
        with self.assertRaisesRegex(RecoverableStop, "insufficient braking room"):
            limiter.advance((-.02, 0., 0., 0.), .01, limits)
        self.assertEqual(vars(limiter), before)

    def test_ik_external_branch_seed_and_candidate_checks(self):
        adapter = self.ready()
        request = adapter.step(measured(1.01, 2), 1.01)
        self.assertEqual(request.ik_seed_q, (0.,)*7)
        accepted = (.001,)*7
        adapter.accept_ik_candidate(request, accepted, .5)
        request2 = adapter.step(measured(1.02, 3), 1.02)
        self.assertEqual(request2.ik_seed_q, accepted)
        with self.assertRaisesRegex(RecoverableStop, "IK discontinuity"):
            adapter.accept_ik_candidate(request2, (.1,)*7, .5)

    def test_ik_request_cannot_be_reused_to_bypass_joint_rate_limit(self):
        adapter = self.ready()
        request = adapter.step(measured(1.01, 2), 1.01)
        adapter.accept_ik_candidate(request, (.004,)*7, .5)
        with self.assertRaisesRegex(RecoverableStop, "already consumed"):
            adapter.accept_ik_candidate(request, (.008,)*7, .5)
        self.assertEqual(adapter.ik_seed, (.004,)*7)


if __name__ == "__main__":
    unittest.main()
