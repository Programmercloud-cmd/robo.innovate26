"""Shared sparse-camera and measured-state actor interface. No hardware IO.

Pixel coordinates are distorted optical pixels; actor coordinates are undistorted
X/Z,Y/Z rays, never image-size-normalized pixels. Target association uses taught
predictions and observed candidates. True brick poses/contact labels are not actor
arguments. Geometric projection and measured-error sampling live in separate APIs.
"""
from collections import deque
from dataclasses import dataclass
import math
import numpy as np

try:
    from ..residual_interface import RecoverableStop, actor_observation, vector
except ImportError:  # Flat VM project imports task.observations.
    from residual_interface import RecoverableStop, actor_observation, vector


def numeric(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise RecoverableStop("finite " + name + " required")
    return float(value)


def fresh(now, stamp, maximum, name):
    if not 0 <= numeric(now, "now") - numeric(stamp, name) <= maximum:
        raise RecoverableStop("stale/future " + name)


@dataclass(frozen=True)
class CameraCalibration:
    camera_id: str
    calibration_id: str
    image_size: tuple
    K: tuple
    distortion: tuple
    distortion_model: str
    settings_profile_id: str
    intrinsics_validated: bool
    mounting_continuity_verified: bool

    def validate(self, require_mount=False):
        if not self.camera_id or not self.calibration_id or not self.settings_profile_id or not self.intrinsics_validated:
            raise RecoverableStop("camera intrinsics/settings calibration is unvalidated")
        if require_mount and not self.mounting_continuity_verified:
            raise RecoverableStop("camera mounting continuity is unverified")
        if len(self.image_size) != 2 or any(isinstance(x, bool) or not isinstance(x, int) or x <= 0 for x in self.image_size):
            raise RecoverableStop("positive integer camera width/height required")
        k = np.asarray(self.K, dtype=float)
        d = np.asarray(self.distortion, dtype=float)
        if k.shape != (3, 3) or not np.isfinite(k).all() or not np.allclose(k[2], [0, 0, 1], atol=1e-12):
            raise RecoverableStop("invalid optical camera matrix")
        if k[0, 0] <= 0 or k[1, 1] <= 0 or abs(k[1, 0]) > 1e-12:
            raise RecoverableStop("invalid camera focal lengths/skew convention")
        if self.distortion_model not in ("none", "brown_conrady") or d.ndim != 1 or len(d) != 5 or not np.isfinite(d).all():
            raise RecoverableStop("explicit five-coefficient Brown or zero-distortion camera required")
        if self.distortion_model == "none" and np.any(d != 0):
            raise RecoverableStop("nonzero coefficients with distortion_model=none")
        return k, d

    @staticmethod
    def _distort(xy, d):
        x, y = xy
        k1, k2, p1, p2, k3 = d
        r2 = x*x + y*y
        radial = 1 + k1*r2 + k2*r2*r2 + k3*r2*r2*r2
        return np.asarray([x*radial + 2*p1*x*y + p2*(r2+2*x*x),
                           y*radial + p1*(r2+2*y*y) + 2*p2*x*y])

    def normalize(self, pixel_xy):
        """Bounded Newton inversion of explicit Brown lens coefficients.

        Residual tolerance is numerical inversion accuracy, not calibrated
        detection/placement accuracy. Nonconvergence is recoverable uncertainty.
        """
        k, d = self.validate()
        u, v = vector(pixel_xy, 2, "pixel")
        if not 0 <= u < self.image_size[0] or not 0 <= v < self.image_size[1]:
            raise RecoverableStop("feature outside image")
        distorted = np.linalg.solve(k, [u, v, 1.0])[:2]
        estimate = distorted.copy()
        for _ in range(20):
            residual = self._distort(estimate, d) - distorted
            if np.max(np.abs(residual)) <= 1e-11:
                return tuple(float(x) for x in estimate)
            epsilon = 1e-7
            jacobian = np.column_stack([(self._distort(estimate+delta, d)-self._distort(estimate-delta, d))/(2*epsilon)
                                        for delta in (np.array([epsilon, 0]), np.array([0, epsilon]))])
            try:
                estimate -= np.linalg.solve(jacobian, residual)
            except np.linalg.LinAlgError as error:
                raise RecoverableStop("singular lens inversion") from error
            if not np.isfinite(estimate).all() or np.max(np.abs(estimate)) > 100:
                raise RecoverableStop("lens inversion diverged")
        raise RecoverableStop("lens inversion did not converge")

    def pixel(self, optical_xyz):
        """Geometric optical projection only; no detector/occlusion simulation."""
        k, d = self.validate()
        x, y, z = vector(optical_xyz, 3, "optical point")
        if z <= 0:
            return None
        projected = k @ np.r_[self._distort([x/z, y/z], d), 1.0]
        u, v = map(float, projected[:2])
        return (u, v) if 0 <= u < self.image_size[0] and 0 <= v < self.image_size[1] else None


@dataclass(frozen=True)
class StudCandidate:
    pixel_xy: tuple
    confidence: float
    visible: bool = True


@dataclass(frozen=True)
class StudFrame:
    target_id: str
    camera_id: str
    calibration_id: str
    settings_profile_id: str
    image_size: tuple
    sequence: int
    captured_s: float
    received_s: float
    clock_profile_id: str
    timestamp_calibrated: bool
    candidates: tuple


@dataclass(frozen=True)
class GoalFeatures:
    brick_index: tuple
    target_id: str
    nominal_yaw_rad: float
    support_slot_ids: tuple  # Eight underside slots; None means no lower stud.
    support_owner_ids: tuple  # Known-layout owner, e.g. plate or b_n_m_k.

    def validate(self):
        if len(self.brick_index) != 3 or any(isinstance(x, bool) or not isinstance(x, int) or x < 0 for x in self.brick_index):
            raise RecoverableStop("invalid brick index")
        n, m, _ = self.brick_index
        if n > 3 or m > 2 or self.target_id != f"b_{n}_{m}_{self.brick_index[2]}":
            raise RecoverableStop("target does not match requested-stack index")
        numeric(self.nominal_yaw_rad, "nominal yaw")
        if len(self.support_slot_ids) != 8 or len(self.support_owner_ids) != 8:
            raise RecoverableStop("explicit eight-slot support pattern required")
        slots = [slot for slot in self.support_slot_ids if slot is not None]
        if len(slots) != len(set(slots)) or any(not isinstance(slot, str) or not slot for slot in slots):
            raise RecoverableStop("support correspondence IDs must be unique")
        if any((slot is None) != (owner is None) for slot, owner in zip(self.support_slot_ids, self.support_owner_ids)):
            raise RecoverableStop("support correspondence owner mismatch")
        if any(owner is not None and (not isinstance(owner, str) or not owner) for owner in self.support_owner_ids):
            raise RecoverableStop("known-layout support owner IDs required")


@dataclass(frozen=True)
class HeldBrickEvidence:
    stamp_s: float
    evidence_id: str
    method: str
    pose_observable: bool
    slip_observable: bool
    slip_detected: bool
    valid: bool
    grasp_profile_id: str = ""
    transform_bound_validated: bool = False
    slip_protection_validated: bool = False
    validation_evidence_ids: tuple = ()


class ObservationLimits:
    def __init__(self, config):
        try:
            for name in ("max_frame_age_s", "max_held_evidence_age_s", "max_state_camera_skew_s",
                         "minimum_confidence", "association_gate_normalized", "ambiguity_margin_normalized"):
                setattr(self, name, numeric(config[name], name))
            for name in ("minimum_visible_features", "maximum_occluded_frames", "history_length"):
                value = config[name]
                if isinstance(value, bool) or not isinstance(value, int):
                    raise RecoverableStop("integer " + name + " required")
                setattr(self, name, value)
        except (KeyError, TypeError) as error:
            raise RecoverableStop("uncommissioned observation limits") from error
        if any(getattr(self, name) <= 0 for name in ("max_frame_age_s", "max_held_evidence_age_s", "max_state_camera_skew_s",
                                                    "association_gate_normalized", "ambiguity_margin_normalized")):
            raise RecoverableStop("positive observation limits required")
        if not 0 <= self.minimum_confidence <= 1 or not 1 <= self.minimum_visible_features <= 8:
            raise RecoverableStop("invalid confidence/visible-feature limits")
        if self.maximum_occluded_frames < 0 or not 1 <= self.history_length <= 16:
            raise RecoverableStop("bounded observation history required")


def associate(frame, camera, goal, expected_normalized, limits):
    """Mutually unambiguous nearest matches inside a measured local gate.

    expected_normalized comes from the *taught* target and calibrated camera,
    not true simulated brick alignment. Equal or repeated-grid alternatives
    remain unmatched; they are never silently relabeled as correct studs.
    """
    goal.validate()
    slots = [slot for slot in goal.support_slot_ids if slot is not None]
    if set(expected_normalized) != set(slots):
        raise RecoverableStop("target-associated expected support slots incomplete")
    predictions = {slot: np.asarray(vector(expected_normalized[slot], 2), float) for slot in slots}
    candidates = []
    for candidate in frame.candidates:
        if candidate is None:  # Explicit simulated or measured dropout/occlusion.
            continue
        if not isinstance(candidate, StudCandidate):
            raise RecoverableStop("invalid detector candidate packet")
        if not isinstance(candidate.visible, bool):
            raise RecoverableStop("explicit feature visibility required")
        confidence = numeric(candidate.confidence, "feature confidence")
        if not 0 <= confidence <= 1:
            raise RecoverableStop("feature confidence outside [0,1]")
        if candidate.visible and confidence >= limits.minimum_confidence:
            candidates.append((camera.normalize(candidate.pixel_xy), confidence))
    distances = {slot: sorted((float(np.linalg.norm(predictions[slot]-xy)), i)
                              for i, (xy, _) in enumerate(candidates)) for slot in slots}
    matched = {}
    for slot in slots:
        options = distances[slot]
        if not options or options[0][0] > limits.association_gate_normalized:
            continue
        distance, i = options[0]
        if len(options) > 1 and options[1][0]-distance <= limits.ambiguity_margin_normalized:
            continue
        inverse = sorted((float(np.linalg.norm(xy-predictions[other])), other)
                         for other in slots for xy in [candidates[i][0]])
        if inverse[0][1] != slot or (len(inverse) > 1 and inverse[1][0]-inverse[0][0] <= limits.ambiguity_margin_normalized):
            continue
        matched[slot] = candidates[i]
    return matched


class ActorObservationBuilder:
    """Shared real/simulation actor processing with bounded occlusion history.

    Held-brick evidence must be independent of lower-stud detections. All
    state/force/joint/fault checks in residual_interface remain outside policy.
    """
    def __init__(self, camera, observation_limits, grasp_profile_id=None):
        self.camera = camera
        self.limits = ObservationLimits(observation_limits)
        self.grasp_profile_id = grasp_profile_id
        self.history = deque(maxlen=self.limits.history_length)
        self.last_sequence = -1
        self.last_capture = -math.inf
        self.target_id = None
        self.occluded_frames = 0
        self.stopped = None

    def reset(self):
        """Supervisor calls only at a fresh stationary handoff, never mid-contact."""
        self.history.clear()
        self.last_sequence, self.last_capture = -1, -math.inf
        self.target_id, self.occluded_frames, self.stopped = None, 0, None

    def build(self, state, now, taught_O_T_M, commanded_M_T_EE, previous_action, bias, control_limits,
              goal, frame, expected_normalized, held_brick):
        if self.stopped:
            raise RecoverableStop(self.stopped)
        try:
            return self._build(state, now, taught_O_T_M, commanded_M_T_EE, previous_action, bias, control_limits,
                               goal, frame, expected_normalized, held_brick)
        except RecoverableStop as error:
            self.stopped = str(error)
            raise

    def _build(self, state, now, taught_O_T_M, commanded_M_T_EE, previous_action, bias, control_limits,
               goal, frame, expected_normalized, held_brick):
        if not isinstance(frame, StudFrame) or not isinstance(goal, GoalFeatures) or not isinstance(held_brick, HeldBrickEvidence):
            raise RecoverableStop("missing typed goal/camera/held-brick packet")
        self.camera.validate(require_mount=True)
        goal.validate()
        if frame.target_id != goal.target_id or (self.target_id is not None and self.target_id != goal.target_id):
            raise RecoverableStop("target switch requires fresh supervised handoff")
        if (frame.camera_id, frame.calibration_id, frame.settings_profile_id, frame.image_size) != (
                self.camera.camera_id, self.camera.calibration_id, self.camera.settings_profile_id, self.camera.image_size):
            raise RecoverableStop("camera identity/calibration/settings/profile mismatch")
        if not frame.timestamp_calibrated or not frame.clock_profile_id:
            raise RecoverableStop("camera timestamp/latency mapping uncalibrated")
        fresh(now, frame.captured_s, self.limits.max_frame_age_s, "camera exposure")
        if not frame.captured_s <= frame.received_s <= now or abs(state.stamp_s-frame.captured_s) > self.limits.max_state_camera_skew_s:
            raise RecoverableStop("camera/state timing mismatch")
        if isinstance(frame.sequence, bool) or not isinstance(frame.sequence, int) or frame.sequence <= self.last_sequence or frame.captured_s <= self.last_capture:
            raise RecoverableStop("replayed/nonprogressing camera frame")
        fresh(now, held_brick.stamp_s, self.limits.max_held_evidence_age_s, "held-brick evidence")
        if not self.grasp_profile_id or held_brick.grasp_profile_id != self.grasp_profile_id:
            raise RecoverableStop("held-brick grasp profile is uncommissioned/mismatched")
        bound_evidence = bool(held_brick.validation_evidence_ids) and all(isinstance(x, str) and x for x in held_brick.validation_evidence_ids)
        pose_known = held_brick.pose_observable or (held_brick.transform_bound_validated and bound_evidence)
        slip_protected = held_brick.slip_observable or (held_brick.slip_protection_validated and bound_evidence)
        if not held_brick.valid or not held_brick.evidence_id or not held_brick.method or not pose_known or not slip_protected or held_brick.slip_detected:
            raise RecoverableStop("held-brick transform/slip uncertain; lower studs are insufficient")
        base = actor_observation(state, now, taught_O_T_M, commanded_M_T_EE, previous_action, bias, control_limits)
        matched = associate(frame, self.camera, goal, expected_normalized, self.limits)
        occluded = len(matched) < self.limits.minimum_visible_features
        self.occluded_frames = self.occluded_frames+1 if occluded else 0
        if self.occluded_frames > self.limits.maximum_occluded_frames or (occluded and not self.history):
            raise RecoverableStop("target stud evidence insufficient after bounded re-observation")
        packet = {**base, "q": vector(state.q, 7), "dq": vector(state.dq, 7),
                  "brick_index": goal.brick_index, "nominal_yaw_rad": goal.nominal_yaw_rad,
                  "support_pattern": tuple(slot is not None for slot in goal.support_slot_ids),
                  "support_owner_ids": goal.support_owner_ids,
                  "stud_xy_normalized": tuple(matched.get(slot, ((0., 0.), 0.))[0] for slot in goal.support_slot_ids),
                  "stud_confidence": tuple(matched.get(slot, ((0., 0.), 0.))[1] for slot in goal.support_slot_ids),
                  "stud_visible": tuple(slot in matched for slot in goal.support_slot_ids),
                  "stud_correspondence_valid": tuple(slot in matched for slot in goal.support_slot_ids),
                  "feature_slot_ids": goal.support_slot_ids, "camera_captured_s": frame.captured_s,
                  "camera_received_s": frame.received_s, "camera_age_s": now-frame.captured_s,
                  "camera_sequence": frame.sequence, "held_brick_evidence_age_s": now-held_brick.stamp_s,
                  "held_brick_pose_observable": held_brick.pose_observable, "held_brick_slip_observable": held_brick.slip_observable,
                  "held_brick_transform_bound_validated": held_brick.transform_bound_validated,
                  "held_brick_slip_protection_validated": held_brick.slip_protection_validated,
                  "requires_reobservation": occluded}
        self.history.append(packet)
        self.last_sequence, self.last_capture, self.target_id = frame.sequence, frame.captured_s, goal.target_id
        return {"current": dict(packet), "history": tuple(dict(item) for item in self.history), "history_valid_count": len(self.history)}


def project_visible_candidates(camera, optical_points_at_capture, externally_visible):
    """Privileged SENSOR projection helper, never an actor pose observation.

    Returns geometry-only pixels. The caller must use capture-time scene state
    and independent ray/occlusion evidence. This helper supplies no detector
    confidence, noise/dropout/latency model, feature IDs or training readiness.
    """
    camera.validate(require_mount=True)
    if len(optical_points_at_capture) != len(externally_visible):
        raise RecoverableStop("explicit visibility mask required")
    return tuple(camera.pixel(point) if visible else None for point, visible in zip(optical_points_at_capture, externally_visible))


@dataclass(frozen=True)
class DetectorFrameError:
    latency_s: float
    feature_errors: tuple
    evidence_id: str
    camera_identity: tuple


def sample_measured_detector_profile(profile, rng, camera):
    """Bootstrap ONE measured frame, preserving shared latency/correlated errors.

    Sample this BEFORE rendering. The simulator must retrieve scene state at
    now-sample.latency_s, perform geometric visibility/projection at that time,
    then call apply_detector_frame_sample. No current-time pose substitution.
    """
    if profile.get("validated") is not True or profile.get("provenance") != "physical" or not profile.get("evidence_ids"):
        raise RecoverableStop("real detector error/latency evidence is missing")
    camera.validate(require_mount=True)
    identity = (camera.camera_id, camera.calibration_id, camera.settings_profile_id, camera.image_size)
    if (profile.get("camera_id"), profile.get("calibration_id"), profile.get("settings_profile_id"), tuple(profile.get("image_size", ()))) != identity:
        raise RecoverableStop("detector error profile camera/calibration/settings mismatch")
    rows = profile.get("samples", [])
    if not rows:
        raise RecoverableStop("empty measured detector profile")
    row = rows[int(rng.integers(len(rows)))]
    latency = numeric(row["latency_s"], "measured detector latency")
    if latency < 0 or row.get("evidence_id") not in profile["evidence_ids"] or not row.get("features"):
        raise RecoverableStop("invalid complete measured detector frame")
    errors = []
    for feature in row["features"]:
        confidence = numeric(feature["confidence"], "measured confidence")
        if not 0 <= confidence <= 1 or not isinstance(feature["detected"], bool):
            raise RecoverableStop("invalid measured detector feature")
        error = vector(feature["pixel_error_xy"], 2, "detector residual")
        errors.append((feature["detected"], error, confidence))
    return DetectorFrameError(latency, tuple(errors), row["evidence_id"], identity)


def apply_detector_frame_sample(geometry_pixels_at_capture, sample, camera):
    """Apply complete-frame measured errors; physical occlusion always wins."""
    camera.validate(require_mount=True)
    if sample.camera_identity != (camera.camera_id, camera.calibration_id, camera.settings_profile_id, camera.image_size):
        raise RecoverableStop("detector sample camera/calibration/settings mismatch")
    if len(geometry_pixels_at_capture) != len(sample.feature_errors):
        raise RecoverableStop("measured detector profile does not match feature layout")
    candidates = []
    for pixel, (detected, error, confidence) in zip(geometry_pixels_at_capture, sample.feature_errors):
        if pixel is None or not detected:
            candidates.append(None)
        else:
            observed = tuple(x+y for x, y in zip(vector(pixel, 2), error))
            in_frame = 0 <= observed[0] < camera.image_size[0] and 0 <= observed[1] < camera.image_size[1]
            candidates.append(StudCandidate(observed, confidence) if in_frame else None)
    return tuple(candidates)
