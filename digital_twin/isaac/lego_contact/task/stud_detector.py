"""Known-target pixel-ring detector. No camera capture, control or hidden poses.

The old factory detector's local grayscale circle evidence is useful here, but
its colored four-brick row segmentation and homography are task-specific and
are not reused. This detector searches only explicitly projected stud ROIs and
returns observed ellipse centers. Raw fit quality is not calibrated confidence.
"""
from dataclasses import dataclass, replace
import hashlib
import json
import math
from pathlib import Path
import numpy as np

from .observations import (CameraCalibration, GoalFeatures, RecoverableStop,
                           StudCandidate, StudFrame, fresh, numeric)


def opencv():
    """Lazy import: on this host /usr/bin/python3 -s uses compatible apt NumPy."""
    try:
        import cv2
    except ImportError as error:
        raise RecoverableStop("OpenCV unavailable/incompatible; use the validated image-processing interpreter") from error
    return cv2


@dataclass(frozen=True)
class RingFit:
    center_px: tuple
    radii_px: tuple
    angle_deg: float
    quality: float
    angular_coverage: float
    radial_error: float
    contrast: float


@dataclass(frozen=True)
class DetectionBatch:
    frame: StudFrame
    diagnostics: dict
    physical_confidence_validated: bool


class DetectorParameters:
    def __init__(self, config):
        try:
            for name in ("center_search_radius_px", "minimum_radius_px", "maximum_radius_px",
                         "maximum_axis_ratio", "canny_low", "canny_high", "maximum_radial_error",
                         "minimum_angular_coverage", "minimum_ring_contrast", "duplicate_center_distance_px",
                         "max_frame_age_s"):
                setattr(self, name, numeric(config[name], name))
        except (KeyError, TypeError) as error:
            raise RecoverableStop("uncommissioned known-stud ROI parameters") from error
        if any(value <= 0 for value in vars(self).values()):
            raise RecoverableStop("positive ROI detector parameters required")
        if not self.minimum_radius_px < self.maximum_radius_px or self.maximum_axis_ratio < 1:
            raise RecoverableStop("invalid observed ellipse radius/axis bounds")
        if not self.canny_low < self.canny_high <= 255 or not 0 < self.minimum_angular_coverage <= 1:
            raise RecoverableStop("invalid edge/coverage parameters")
        if self.minimum_ring_contrast > 255:
            raise RecoverableStop("ring contrast exceeds uint8 range")
        if self.maximum_radius_px > 256 or self.center_search_radius_px > 256:
            raise RecoverableStop("ROI exceeds bounded detector scope")


def parameter_signature(parameters):
    """Bind score calibration to the exact validated numeric detector settings."""
    return hashlib.sha256(json.dumps(vars(parameters), sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def verify_confidence_evidence(confidence):
    """One-time raw-image/label artifact check; no camera access or per-frame IO.

    This verifies artifact integrity only. Physical provenance, calibration
    validity and held-out score reliability remain task-level acceptance gates.
    """
    kinds = set()
    records = confidence.get("evidence_files") or []
    if not records:
        raise RecoverableStop("measured confidence raw-image/label evidence is missing")
    for record in records:
        try:
            path = Path(record["path"])
            wanted = record["sha256"]
            if record["evidence_id"] not in confidence["evidence_ids"] or record["kind"] not in ("raw_image", "labels"):
                raise RecoverableStop("confidence artifact identity/kind mismatch")
            if not isinstance(wanted, str) or len(wanted) != 64 or hashlib.sha256(path.read_bytes()).hexdigest() != wanted:
                raise RecoverableStop("confidence raw-image/label evidence hash mismatch")
            kinds.add(record["kind"])
        except (KeyError, TypeError, OSError) as error:
            raise RecoverableStop("confidence raw-image/label evidence unavailable") from error
    if kinds != {"raw_image", "labels"}:
        raise RecoverableStop("confidence calibration requires raw-image and label evidence")


def _ring_samples(gray, center, radii, angle_deg):
    """Measured radial contrast around a fitted ellipse, not expected pixels."""
    angles = np.arange(32, dtype=float)*2*math.pi/32
    direction = np.stack([radii[0]*np.cos(angles), radii[1]*np.sin(angles)])
    a = math.radians(angle_deg)
    rotation = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
    samples = []
    for rho in (.75, .9, 1., 1.1, 1.25):
        points = np.asarray(center)[:, None] + rotation @ (rho*direction)
        x, y = points
        if np.any(x < 0) or np.any(y < 0) or np.any(x >= gray.shape[1]-1) or np.any(y >= gray.shape[0]-1):
            return None  # Complete ring evidence is clipped, not fabricated.
        x0, y0 = np.floor(x).astype(int), np.floor(y).astype(int)
        dx, dy = x-x0, y-y0
        values = ((1-dx)*(1-dy)*gray[y0, x0] + dx*(1-dy)*gray[y0, x0+1]
                  + (1-dx)*dy*gray[y0+1, x0] + dx*dy*gray[y0+1, x0+1])
        samples.append(values)
    return float(np.median(np.ptp(np.asarray(samples), axis=0)))


def extract_roi_rings(gray, nominal_center_px, parameters):
    """Return distinct observed rings inside one bounded known-target ROI.

    This low-level pixel extractor supports annotation/commissioning. It does
    not produce calibrated actor confidence or establish that a ring is a stud.
    """
    cv2 = opencv()
    if gray.ndim != 2 or gray.dtype != np.uint8:
        raise RecoverableStop("uint8 grayscale ROI evidence required")
    nominal = np.asarray(nominal_center_px, dtype=float)
    if nominal.shape != (2,) or not np.isfinite(nominal).all():
        raise RecoverableStop("finite projected ROI center required")
    radius = parameters.center_search_radius_px + 1.3*parameters.maximum_radius_px + 2
    x0, y0 = np.maximum(0, np.floor(nominal-radius)).astype(int)
    x1, y1 = np.minimum([gray.shape[1], gray.shape[0]], np.ceil(nominal+radius)+1).astype(int)
    if x1-x0 < 5 or y1-y0 < 5:
        return ()
    patch = gray[y0:y1, x0:x1]
    edges = cv2.Canny(cv2.GaussianBlur(patch, (3, 3), .7), parameters.canny_low, parameters.canny_high)
    contours = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)[-2]
    fits = []
    for contour in contours:
        if len(contour) < 12:
            continue
        center, diameters, angle = cv2.fitEllipse(contour)
        radii = np.asarray(diameters, float)/2
        center = np.asarray(center, float)+[x0, y0]
        if (not np.isfinite(np.r_[center, radii, angle]).all() or np.min(radii) < parameters.minimum_radius_px
                or np.max(radii) > parameters.maximum_radius_px or np.max(radii)/np.min(radii) > parameters.maximum_axis_ratio
                or np.linalg.norm(center-nominal) > parameters.center_search_radius_px):
            continue
        a = math.radians(angle)
        inverse = np.array([[math.cos(a), math.sin(a)], [-math.sin(a), math.cos(a)]])
        points = contour.reshape(-1, 2).astype(float)+[x0, y0]
        local = (inverse @ (points-center).T).T / radii
        radial_error = float(np.sqrt(np.mean((np.linalg.norm(local, axis=1)-1)**2)))
        angular_bins = np.floor((np.arctan2(local[:, 1], local[:, 0])+math.pi)*32/(2*math.pi)).astype(int)%32
        coverage = len(np.unique(angular_bins))/32
        contrast = _ring_samples(gray, center, radii, angle)
        if (radial_error > parameters.maximum_radial_error or coverage < parameters.minimum_angular_coverage
                or contrast is None or contrast < parameters.minimum_ring_contrast):
            continue
        quality = float(np.clip(.5*coverage + .25*(1-radial_error/parameters.maximum_radial_error)
                                + .25*min(1., contrast/255), 0, 1))
        fit = RingFit(tuple(map(float, center)), tuple(map(float, radii)), float(angle), quality,
                      coverage, radial_error, contrast)
        # Inner/outer edges of one ring are concentric evidence, not two studs.
        duplicate = next((i for i, old in enumerate(fits)
                          if np.linalg.norm(np.asarray(old.center_px)-center) <= parameters.duplicate_center_distance_px), None)
        if duplicate is None:
            fits.append(fit)
        elif quality > fits[duplicate].quality:
            fits[duplicate] = fit
        if len(fits) > 8:
            return tuple(fits)  # Explicit ambiguity; no unbounded scene search.
    return tuple(sorted(fits, key=lambda value: (-value.quality, value.center_px)))


class KnownStudDetector:
    """Image→StudFrame for a known target; all pixel evidence stays explicit."""
    def __init__(self, camera, config):
        if not isinstance(camera, CameraCalibration):
            raise RecoverableStop("typed camera calibration required")
        camera.validate(require_mount=True)
        self.camera, self.config = camera, config
        if config.get("schema") != "known_stud_roi_detector_v1":
            raise RecoverableStop("unsupported known-stud detector schema")
        identity = (camera.camera_id, camera.calibration_id, camera.settings_profile_id, camera.image_size)
        if (config.get("camera_id"), config.get("calibration_id"), config.get("settings_profile_id"),
                tuple(config.get("image_size", ()))) != identity:
            raise RecoverableStop("ROI detector camera/calibration/settings identity mismatch")
        if not config.get("detector_profile_id"):
            raise RecoverableStop("named ROI detector profile required")
        self.parameters = DetectorParameters(config.get("parameters"))
        self.parameter_sha256 = parameter_signature(self.parameters)
        confidence = config.get("confidence_calibration") or {}
        self.physical = (config.get("purpose") == "physical" and confidence.get("provenance") == "physical"
                         and confidence.get("validated") is True and bool(confidence.get("evidence_ids")))
        synthetic = config.get("purpose") == "synthetic_validation" and confidence.get("provenance") == "synthetic"
        if not self.physical and not synthetic:
            raise RecoverableStop("raw ring quality lacks measured confidence calibration")
        if self.physical and (confidence.get("camera_id"), confidence.get("calibration_id"),
                              confidence.get("settings_profile_id"), tuple(confidence.get("image_size", ()))) != identity:
            raise RecoverableStop("confidence calibration camera/settings binding mismatch")
        if self.physical:
            if opencv().__version__ not in (confidence.get("opencv_versions_validated") or []):
                raise RecoverableStop("confidence calibration not validated for this OpenCV runtime")
            try:
                k, d = camera.validate(require_mount=True)
                matrices_match = (np.array_equal(np.asarray(confidence["camera_matrix"], float), k)
                                  and np.array_equal(np.asarray(confidence["distortion"], float), d))
            except (KeyError, TypeError, ValueError) as error:
                raise RecoverableStop("confidence camera matrix/distortion binding missing") from error
            if not matrices_match or confidence.get("detector_parameters_sha256") != self.parameter_sha256:
                raise RecoverableStop("confidence camera matrix/detector parameter binding mismatch")
            verify_confidence_evidence(confidence)
        self.quality_knots = np.asarray(confidence.get("input_quality"), dtype=float)
        self.confidence_knots = np.asarray(confidence.get("confidence"), dtype=float)
        if (self.quality_knots.ndim != 1 or len(self.quality_knots) < 2
                or self.quality_knots.shape != self.confidence_knots.shape
                or not np.isfinite(np.r_[self.quality_knots, self.confidence_knots]).all()
                or self.quality_knots[0] != 0 or self.quality_knots[-1] != 1 or np.any(np.diff(self.quality_knots) <= 0)
                or np.any(np.diff(self.confidence_knots) < 0) or np.any(self.confidence_knots < 0) or np.any(self.confidence_knots > 1)):
            raise RecoverableStop("bounded monotone quality→confidence calibration required")

    def detect_frame(self, image, goal, metadata, expected_normalized, now_s):
        """Use the same bounded image processing for real and rendered pixels."""
        if not isinstance(goal, GoalFeatures) or not isinstance(metadata, StudFrame):
            raise RecoverableStop("typed target/frame metadata required")
        goal.validate()
        camera = self.camera
        if metadata.target_id != goal.target_id or (metadata.camera_id, metadata.calibration_id,
                metadata.settings_profile_id, metadata.image_size) != (camera.camera_id, camera.calibration_id,
                camera.settings_profile_id, camera.image_size):
            raise RecoverableStop("ROI detector target/camera/settings metadata mismatch")
        if metadata.candidates or not metadata.timestamp_calibrated or not metadata.clock_profile_id:
            raise RecoverableStop("fresh empty candidate frame with calibrated timestamps required")
        if isinstance(metadata.sequence, bool) or not isinstance(metadata.sequence, int) or metadata.sequence < 0:
            raise RecoverableStop("nonnegative frame sequence required")
        fresh(now_s, metadata.captured_s, self.parameters.max_frame_age_s, "ROI exposure")
        if not metadata.captured_s <= metadata.received_s <= now_s:
            raise RecoverableStop("ROI exposure/receipt timing mismatch")
        if not isinstance(image, np.ndarray) or image.dtype != np.uint8 or image.shape[:2] != camera.image_size[::-1]:
            raise RecoverableStop("ROI detector image dtype/resolution mismatch")
        cv2 = opencv()
        if image.ndim == 2:
            gray = image
        elif image.ndim == 3 and image.shape[2] == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            raise RecoverableStop("uint8 grayscale or BGR image required")
        slots = [slot for slot in goal.support_slot_ids if slot is not None]
        if set(expected_normalized) != set(slots):
            raise RecoverableStop("projected nominal target stud ROIs incomplete")
        candidates, diagnostics, selected = [], {}, []
        try:
            for slot in goal.support_slot_ids:
                if slot is None:
                    candidates.append(None)
                    continue
                xy = expected_normalized[slot]
                nominal = camera.pixel((*xy, 1.))
                fits = extract_roi_rings(gray, nominal, self.parameters) if nominal is not None else ()
                if len(fits) != 1:
                    candidates.append(None)
                    diagnostics[slot] = {"observed": False, "reason": "ambiguous_rings" if len(fits) > 1 else "no_complete_ring_evidence", "ring_count": len(fits)}
                    continue
                fit = fits[0]
                duplicate = any(np.linalg.norm(np.asarray(old.center_px)-fit.center_px) <= self.parameters.duplicate_center_distance_px for old in selected)
                if duplicate:
                    candidates.append(None)
                    diagnostics[slot] = {"observed": False, "reason": "duplicate_pixel_ring"}
                    continue
                selected.append(fit)
                confidence = float(np.interp(fit.quality, self.quality_knots, self.confidence_knots))
                candidates.append(StudCandidate(fit.center_px, confidence))
                diagnostics[slot] = {"observed": True, "raw_quality": fit.quality, "confidence": confidence,
                                     "pixel_xy": fit.center_px, "radii_px": fit.radii_px, "angular_coverage": fit.angular_coverage,
                                     "radial_error": fit.radial_error, "ring_contrast": fit.contrast}
        except cv2.error as error:
            raise RecoverableStop("ROI image processing failed") from error
        return DetectionBatch(replace(metadata, candidates=tuple(candidates)),
                              {"detector_profile_id": self.config["detector_profile_id"], "purpose": self.config["purpose"],
                               "opencv_version": cv2.__version__,
                               "detector_parameters_sha256": self.parameter_sha256,
                               "confidence_evidence_ids": (self.config.get("confidence_calibration") or {}).get("evidence_ids", []),
                               "slots": diagnostics, "inventory_advance": False, "training_ready": False}, self.physical)
