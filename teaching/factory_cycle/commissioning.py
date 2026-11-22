"""Separate reusable controller timing evidence from selected stationary geometry."""
import copy
import hashlib
import json
from pathlib import Path


def stationary_profile_evidence(planner, commissioned):
    """Reuse unchanged controller evidence; all new paths still need certification."""
    if planner.factory.get('conveyor_placement_mode') != 'stationary_vertical':
        return None
    archive = Path(__file__).resolve().parent.parent / 'history/placement_home_plus55_20261027T205548Z'
    try:
        old_factory = json.loads((archive / 'factory_vision_config.json').read_text())
        old_settings = json.loads((archive / 'factory_cycle_settings.json').read_text())
        original_manifest = json.loads((Path(commissioned['run']) / 'controller_manifest.json').read_text())
    except (OSError, ValueError, KeyError):
        return None
    settings = copy.deepcopy(planner.settings)
    home_offset = settings.pop('home_z_offset_m', 0.)
    expected_offset = planner.setup['placement_home_offset_world_m'] if planner.setup else [0., 0., 0.]
    if settings != old_settings or home_offset != expected_offset[2]:
        return None
    keys = ('destination_mode', 'conveyor_placement_mode', 'conveyor_target_O_T_EE')
    old_motion = {key: old_factory[key] for key in keys}
    expected_motion = copy.deepcopy(old_motion)
    for axis in range(3):
        expected_motion['conveyor_target_O_T_EE'][12+axis] += expected_offset[axis]
    placement_trim=planner.setup.get('placement_height_correction_m',0.) if planner.setup else 0.
    expected_motion['conveyor_target_O_T_EE'][14] += placement_trim
    if {key: planner.factory[key] for key in keys} != expected_motion:
        return None
    if planner.surface_offset != old_factory.get('conveyor_surface_height_offset_m', 0.):
        return None
    if planner.surface_offset:
        old_motion['conveyor_surface_height_offset_m'] = planner.surface_offset
    # Reconstruct the commissioned identity with CURRENT tool, model, source
    # poses and speed policy. Any change besides the approved translation fails.
    payload = dict(spline_schema=3, settings=old_settings, tool=planner.tool,
                   model=planner.model.model_sha256, policy=planner.model.policy,
                   geometry=planner.catalog, factory=old_motion, moving=planner.moving)
    baseline = hashlib.sha256(json.dumps(payload, sort_keys=True, separators=(',', ':')).encode()).hexdigest()
    if baseline != commissioned['planner_identity']:
        return None
    current_manifest = copy.deepcopy(planner.manifest())
    current_manifest.update(settings=old_settings, identity=baseline, commissioning_authorized=True)
    # Bounds are validated from the authorized profile. Other physical limits
    # (joint, derivatives, tool, tracking and singularity) must still match.
    if planner.setup:
        current_manifest['lower_xyz'] = original_manifest['lower_xyz']
        current_manifest['upper_xyz'] = original_manifest['upper_xyz']
    if current_manifest != original_manifest:
        return None
    return dict(translation_world_m=expected_offset, home_z_offset_delta_m=home_offset,
                measured_placement_height_correction_m=placement_trim,
                controller_timing_evidence_reused_from=commissioned['run'],
                original_planner_identity=baseline, current_planner_identity=planner.identity,
                new_geometry_physically_commissioned=False,
                full_trajectory_model_and_native_checks_required=True)
