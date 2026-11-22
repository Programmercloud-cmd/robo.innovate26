"""Synthetic camera/interface regressions; no physical calibration evidence."""
from dataclasses import replace
import inspect
import math
from pathlib import Path
import sys
import unittest
import numpy as np

PARENT = Path(__file__).resolve().parent.parent
if str(PARENT) not in sys.path:
    sys.path.insert(0, str(PARENT))
from residual_interface import Limits, Pose, RecoverableStop, WrenchBias
from test_residual_interface import measured, synthetic_contract
from task.observations import (ActorObservationBuilder, CameraCalibration, GoalFeatures,
                               HeldBrickEvidence, ObservationLimits, StudCandidate,
                               StudFrame, apply_detector_frame_sample, associate, project_visible_candidates,
                               sample_measured_detector_profile)


def camera():
    return CameraCalibration("wrist_sonix_c920", "synthetic-lens", (1280, 720),
                             ((800., 0., 640.), (0., 810., 360.), (0., 0., 1.)),
                             (.03, -.01, .001, -.002, .003), "brown_conrady", "synthetic-fixed-settings", True, True)


def obs_limits():
    return {"max_frame_age_s": .05, "max_held_evidence_age_s": .05,
            "max_state_camera_skew_s": .03, "minimum_confidence": .8,
            "association_gate_normalized": .02, "ambiguity_margin_normalized": .005,
            "minimum_visible_features": 4, "maximum_occluded_frames": 1, "history_length": 3}


def fixture(stamp=1., sequence=1, candidates=None):
    expected = {f"stud-{i}": ((i%4-.5)*.05, (i//4-.5)*.05) for i in range(8)}
    goal = GoalFeatures((0, 0, 0), "b_0_0_0", 0., tuple(expected), ("plate",)*8)
    lens = camera()
    if candidates is None:
        candidates = tuple(StudCandidate(lens.pixel((*xy, 1.)), .95) for xy in expected.values())
    frame = StudFrame(goal.target_id, lens.camera_id, lens.calibration_id, lens.settings_profile_id,
                      lens.image_size, sequence, stamp, stamp, "synthetic-clock", True, candidates)
    held = HeldBrickEvidence(stamp, "synthetic-grip-evidence", "independent-upper-outline", True, True, False, True, "synthetic-grasp")
    return goal, frame, expected, held


class ObservationTests(unittest.TestCase):
    def build(self, builder=None, stamp=1., sequence=1, **changes):
        builder = builder or ActorObservationBuilder(camera(), obs_limits(), "synthetic-grasp")
        goal, frame, expected, held = fixture(stamp, sequence)
        state = measured(stamp, sequence)
        return builder.build(state, stamp, Pose((0.,)*3), Pose((0.,)*3), (0.,)*4,
                             WrenchBias((0.,)*6, "synthetic", .5, True), Limits(synthetic_contract()),
                             changes.get("goal", goal), changes.get("frame", frame),
                             changes.get("expected", expected), changes.get("held", held))

    def test_brown_projection_undistortion_round_trip(self):
        lens = camera()
        for x, y, z in ((.1, -.1, .7), (-.15, .05, 1.), (.2, .1, .8)):
            recovered = lens.normalize(lens.pixel((x, y, z)))
            np.testing.assert_allclose(recovered, [x/z, y/z], atol=1e-10)
        with self.assertRaises(RecoverableStop):
            replace(lens, intrinsics_validated=False).normalize((640, 360))
        with self.assertRaises(RecoverableStop):
            replace(lens, distortion_model="inverse_brown_conrady").normalize((640, 360))
        self.assertIsNone(lens.pixel((0, 0, -.2)))

    def test_correspondence_confidence_and_observation_whitelist(self):
        result = self.build()
        packet = result["current"]
        self.assertEqual(packet["brick_index"], (0, 0, 0))
        self.assertTrue(all(packet["stud_correspondence_valid"]))
        self.assertEqual(packet["q"], (0.,)*7)
        for xy, wanted in zip(packet["stud_xy_normalized"], fixture()[2].values()):
            np.testing.assert_allclose(xy, wanted, atol=1e-10)
        parameters = inspect.signature(ActorObservationBuilder.build).parameters
        self.assertFalse(any("truth" in x or "engagement" in x or "true_pose" in x for x in parameters))
        self.assertNotIn("true_brick_pose", packet)
        self.assertNotIn("contact_state", packet)

    def test_ambiguous_repeated_studs_do_not_become_correspondences(self):
        goal, frame, expected, _ = fixture()
        slot = next(iter(expected))
        pixel = camera().pixel((*expected[slot], 1.))
        duplicate = replace(frame, candidates=(StudCandidate(pixel, .95), StudCandidate((pixel[0]+.1, pixel[1]), .95)))
        self.assertNotIn(slot, associate(duplicate, camera(), goal, expected, ObservationLimits(obs_limits())))

    def test_missing_camera_calibration_and_changed_profile_fail_closed(self):
        for lens in (replace(camera(), mounting_continuity_verified=False), replace(camera(), intrinsics_validated=False)):
            with self.assertRaises(RecoverableStop):
                self.build(ActorObservationBuilder(lens, obs_limits(), "synthetic-grasp"))
        goal, frame, _, _ = fixture()
        for changed in (replace(frame, camera_id="realsense"), replace(frame, timestamp_calibrated=False),
                        replace(frame, settings_profile_id="changed-focus"), replace(frame, captured_s=.9)):
            with self.assertRaises(RecoverableStop):
                self.build(frame=changed)
        with self.assertRaises(RecoverableStop):
            ActorObservationBuilder(camera(), {**obs_limits(), "max_frame_age_s": None})

    def test_held_brick_observability_is_independent_of_lower_studs(self):
        held = fixture()[3]
        for change in ({"pose_observable": False}, {"slip_observable": False}, {"slip_detected": True}, {"stamp_s": .8}):
            with self.assertRaisesRegex(RecoverableStop, "held-brick"):
                self.build(held=replace(held, **change))

    def test_measured_grasp_envelope_can_replace_direct_visibility(self):
        held = replace(fixture()[3], pose_observable=False, slip_observable=False,
                       transform_bound_validated=True, slip_protection_validated=True,
                       validation_evidence_ids=("synthetic-bound-validation-only",))
        packet = self.build(held=held)["current"]
        self.assertFalse(packet["held_brick_pose_observable"])
        self.assertTrue(packet["held_brick_transform_bound_validated"])
        for changed in (replace(held, slip_detected=True), replace(held, grasp_profile_id="other-grip"),
                        replace(held, validation_evidence_ids=())):
            with self.assertRaises(RecoverableStop):
                self.build(held=changed)

    def test_history_occlusion_replay_and_latched_stop(self):
        builder = ActorObservationBuilder(camera(), obs_limits(), "synthetic-grasp")
        self.build(builder)
        blank = fixture(1.01, 2, ())[1]
        result = self.build(builder, 1.01, 2, frame=blank)
        self.assertEqual(result["history_valid_count"], 2)
        self.assertTrue(result["current"]["requires_reobservation"])
        self.assertFalse(any(result["current"]["stud_visible"]))
        self.assertTrue(all(result["history"][0]["stud_visible"]))
        with self.assertRaisesRegex(RecoverableStop, "re-observation"):
            self.build(builder, 1.02, 3, frame=fixture(1.02, 3, ())[1])
        with self.assertRaises(RecoverableStop):
            self.build(builder, 1.03, 4)
        builder = ActorObservationBuilder(camera(), obs_limits(), "synthetic-grasp")
        self.build(builder)
        with self.assertRaisesRegex(RecoverableStop, "nonprogressing"):
            self.build(builder)

    def test_projection_visibility_and_empirical_profile_gate(self):
        pixels = project_visible_candidates(camera(), [(0, 0, 1), (0, 0, 1), (0, 0, -1)], [True, False, True])
        self.assertEqual(pixels[0], (640., 360.))
        self.assertEqual(pixels[1:], (None, None))
        with self.assertRaises(RecoverableStop):
            sample_measured_detector_profile({"validated": False}, np.random.default_rng(1), camera())
        # Explicit fabricated input tests plumbing; it is not saved as evidence.
        profile = {"validated": True, "provenance": "physical", "evidence_ids": ["synthetic-test-only"],
                   "camera_id": camera().camera_id, "calibration_id": camera().calibration_id,
                   "settings_profile_id": camera().settings_profile_id, "image_size": camera().image_size,
                   "samples": [{"evidence_id": "synthetic-test-only", "latency_s": .02,
                                "features": [{"detected": True, "pixel_error_xy": [.5, -.5], "confidence": .9}]*3}]}
        sample = sample_measured_detector_profile(profile, np.random.default_rng(1), camera())
        sampled = apply_detector_frame_sample(pixels, sample, camera())
        self.assertEqual(sampled[0].pixel_xy, (640.5, 359.5))
        self.assertEqual(sample.latency_s, .02)
        self.assertIsNone(sampled[1])

    def test_projected_dropout_packet_uses_same_actor_path(self):
        goal, frame, expected, _ = fixture()
        pixels = project_visible_candidates(camera(), [(*xy, 1.) for xy in expected.values()], [True]*6+[False]*2)
        profile = {"validated": True, "provenance": "physical", "evidence_ids": ["synthetic-test-only"],
                   "camera_id": camera().camera_id, "calibration_id": camera().calibration_id,
                   "settings_profile_id": camera().settings_profile_id, "image_size": camera().image_size,
                   "samples": [{"evidence_id": "synthetic-test-only", "latency_s": .02,
                                "features": [{"detected": True, "pixel_error_xy": [0., 0.], "confidence": .95}]*8}]}
        sample = sample_measured_detector_profile(profile, np.random.default_rng(1), camera())
        candidates = apply_detector_frame_sample(pixels, sample, camera())
        result = self.build(frame=replace(frame, candidates=candidates))
        self.assertEqual(result["current"]["stud_visible"], (True,)*6+(False,)*2)
        with self.assertRaisesRegex(RecoverableStop, "candidate packet"):
            self.build(frame=replace(frame, candidates=(object(),)))

    def test_detector_error_profile_is_bound_to_camera_and_evidence(self):
        lens = camera()
        profile = {"validated": True, "provenance": "physical", "evidence_ids": ["synthetic-test-only"],
                   "camera_id": lens.camera_id, "calibration_id": lens.calibration_id,
                   "settings_profile_id": lens.settings_profile_id, "image_size": lens.image_size,
                   "samples": [{"evidence_id": "synthetic-test-only", "latency_s": .02,
                                "features": [{"detected": True, "pixel_error_xy": [10000., 0.], "confidence": .95}]}]}
        with self.assertRaisesRegex(RecoverableStop, "profile camera"):
            sample_measured_detector_profile({**profile, "camera_id": "realsense"}, np.random.default_rng(1), lens)
        with self.assertRaisesRegex(RecoverableStop, "complete measured"):
            sample_measured_detector_profile({**profile, "evidence_ids": ["other"]}, np.random.default_rng(1), lens)
        sample = sample_measured_detector_profile(profile, np.random.default_rng(1), lens)
        self.assertEqual(apply_detector_frame_sample([(640., 360.)], sample, lens), (None,))
        with self.assertRaisesRegex(RecoverableStop, "sample camera"):
            apply_detector_frame_sample([(640., 360.)], sample, replace(lens, calibration_id="different-lens"))


if __name__ == "__main__":
    unittest.main()
