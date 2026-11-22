"""Synthetic pixel rings exercise software only, never physical calibration."""
import copy
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np

PARENT = Path(__file__).resolve().parent.parent
if str(PARENT) not in sys.path:
    sys.path.insert(0, str(PARENT))
from task.observations import RecoverableStop, StudFrame
from task.stud_detector import DetectorParameters, KnownStudDetector, opencv, parameter_signature
from task import test_observations as observation_fixtures

camera = observation_fixtures.camera
fixture = observation_fixtures.fixture


def synthetic_config():
    lens = camera()
    return {"schema": "known_stud_roi_detector_v1", "purpose": "synthetic_validation",
            "detector_profile_id": "synthetic-ring-detector-only", "camera_id": lens.camera_id,
            "calibration_id": lens.calibration_id, "settings_profile_id": lens.settings_profile_id,
            "image_size": lens.image_size,
            "parameters": {"center_search_radius_px": 10., "minimum_radius_px": 4., "maximum_radius_px": 14.,
                           "maximum_axis_ratio": 2.2, "canny_low": 40., "canny_high": 100.,
                           "maximum_radial_error": .12, "minimum_angular_coverage": .75,
                           "minimum_ring_contrast": 40., "duplicate_center_distance_px": 2.5,
                           "max_frame_age_s": .05},
            "confidence_calibration": {"validated": False, "provenance": "synthetic", "evidence_ids": [],
                                       "input_quality": [0., 1.], "confidence": [0., 1.]}}


def synthetic_image(offset=(3., -2.), missing=()):
    cv2 = opencv()
    image = np.full((720, 1280, 3), 210, np.uint8)
    goal, metadata, expected, _ = fixture()
    centers = {}
    for index, (slot, xy) in enumerate(expected.items()):
        nominal = np.asarray(camera().pixel((*xy, 1.)))
        center = tuple(np.round(nominal+offset).astype(int))
        centers[slot] = center
        if index not in missing:
            cv2.ellipse(image, center, (9, 6), 20, 0, 360, (20, 20, 20), 2, cv2.LINE_AA)
    return image, goal, replace(metadata, candidates=()), expected, centers


class DetectorTests(unittest.TestCase):
    def detector(self, config=None):
        return KnownStudDetector(camera(), config or synthetic_config())

    def test_observed_centers_not_nominal_and_same_actor_path(self):
        image, goal, metadata, expected, centers = synthetic_image()
        result = self.detector().detect_frame(image, goal, metadata, expected, 1.)
        self.assertFalse(result.physical_confidence_validated)
        self.assertFalse(result.diagnostics["training_ready"])
        self.assertEqual(sum(candidate is not None for candidate in result.frame.candidates), 8)
        for slot, candidate in zip(goal.support_slot_ids, result.frame.candidates):
            np.testing.assert_allclose(candidate.pixel_xy, centers[slot], atol=1.)
            self.assertGreater(np.linalg.norm(np.asarray(candidate.pixel_xy)-camera().pixel((*expected[slot], 1.))), 2.)
        # Existing shared actor normalization/association sees measured pixels.
        actor = observation_fixtures.ObservationTests().build(frame=result.frame)
        self.assertTrue(all(actor["current"]["stud_correspondence_valid"]))

    def test_no_image_evidence_and_occlusion_do_not_fabricate_studs(self):
        image, goal, metadata, expected, _ = synthetic_image(missing=(1, 3))
        result = self.detector().detect_frame(image, goal, metadata, expected, 1.)
        self.assertIsNone(result.frame.candidates[1])
        self.assertIsNone(result.frame.candidates[3])
        blank = np.full_like(image, 210)
        result = self.detector().detect_frame(blank, goal, metadata, expected, 1.)
        self.assertTrue(all(value is None for value in result.frame.candidates))
        with self.assertRaisesRegex(RecoverableStop, "stud evidence"):
            observation_fixtures.ObservationTests().build(frame=result.frame)

    def test_ambiguous_and_duplicate_pixel_rings_stay_uncertain(self):
        cv2 = opencv()
        image, goal, metadata, expected, _ = synthetic_image(missing=tuple(range(8)))
        slot = goal.support_slot_ids[0]
        point = np.round(camera().pixel((*expected[slot], 1.))).astype(int)
        for dx in (-7, 7):
            cv2.circle(image, (point[0]+dx, point[1]), 5, (20, 20, 20), 1, cv2.LINE_AA)
        result = self.detector().detect_frame(image, goal, metadata, expected, 1.)
        self.assertIsNone(result.frame.candidates[0])
        self.assertEqual(result.diagnostics["slots"][slot]["reason"], "ambiguous_rings")
        image, goal, metadata, expected, _ = synthetic_image()
        # Two nominal ROIs see the same actual circle: retain one observation.
        changed = dict(expected)
        changed[goal.support_slot_ids[1]] = (expected[slot][0]+.001, expected[slot][1])
        result = self.detector().detect_frame(image, goal, metadata, changed, 1.)
        self.assertIsNotNone(result.frame.candidates[0])
        self.assertIsNone(result.frame.candidates[1])
        self.assertEqual(result.diagnostics["slots"][goal.support_slot_ids[1]]["reason"], "duplicate_pixel_ring")

    def test_identity_calibration_timing_and_confidence_are_guards(self):
        image, goal, metadata, expected, _ = synthetic_image()
        for changed in (replace(metadata, camera_id="realsense"), replace(metadata, captured_s=.8),
                        replace(metadata, timestamp_calibrated=False), replace(metadata, settings_profile_id="other-focus")):
            with self.assertRaises(RecoverableStop):
                self.detector().detect_frame(image, goal, changed, expected, 1.)
        with self.assertRaises(RecoverableStop):
            KnownStudDetector(replace(camera(), intrinsics_validated=False), synthetic_config())
        physical = synthetic_config()
        physical["purpose"] = "physical"
        with self.assertRaisesRegex(RecoverableStop, "confidence calibration"):
            self.detector(physical)
        template = json.loads(Path(__file__).with_name("calibratedROIconfig.template.json").read_text())
        with self.assertRaises(RecoverableStop):
            self.detector(template)
        with self.assertRaisesRegex(RecoverableStop, "dtype/resolution"):
            self.detector().detect_frame(image.astype(float), goal, metadata, expected, 1.)

    def test_confidence_matrix_parameters_and_raw_artifacts_are_bound(self):
        # Fabricated profile flags exercise integrity guards only. The temporary
        # images/labels below are synthetic and are not physical evidence.
        config = synthetic_config()
        lens = camera()
        config["purpose"] = "physical"
        calibration = config["confidence_calibration"]
        calibration.update({"validated": True, "provenance": "physical", "evidence_ids": ["synthetic-test-only"],
                            "camera_id": lens.camera_id, "calibration_id": lens.calibration_id,
                            "settings_profile_id": lens.settings_profile_id, "image_size": lens.image_size,
                            "camera_matrix": lens.K, "distortion": lens.distortion,
                            "opencv_versions_validated": [opencv().__version__],
                            "detector_parameters_sha256": parameter_signature(DetectorParameters(config["parameters"]))})
        with tempfile.TemporaryDirectory() as directory:
            image_path, label_path = Path(directory)/"synthetic.png", Path(directory)/"synthetic-labels.json"
            self.assertTrue(opencv().imwrite(str(image_path), synthetic_image()[0]))
            label_path.write_text('{"synthetic": true}\n')
            calibration["evidence_files"] = [{"path": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                                               "kind": kind, "evidence_id": "synthetic-test-only"}
                                              for path, kind in [(image_path, "raw_image"), (label_path, "labels")]]
            self.detector(config)  # Structural acceptance is not physical validation.
            mismatch = copy.deepcopy(config)
            mismatch["parameters"]["canny_low"] += 1
            with self.assertRaisesRegex(RecoverableStop, "parameter binding"):
                self.detector(mismatch)
            label_path.write_text('{"synthetic": true, "changed": true}\n')
            with self.assertRaisesRegex(RecoverableStop, "hash mismatch"):
                self.detector(config)


if __name__ == "__main__":
    unittest.main()
