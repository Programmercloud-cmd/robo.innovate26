"""Reuse stationary controller evidence for the operator-approved pose translation."""
import copy
import hashlib
import json
from pathlib import Path


def stationary_translation_evidence(planner, commissioned):
    """Accept only the recorded +25 mm X / +55 mm Z change, not new tuning."""
    if planner.factory.get('conveyor_placement_mode') != 'stationary_vertical':
        return None
    archive = Path(__file__).resolve().parent.parent / 'history/placement_home_plus55_20261027T205548Z'
    try:
        old_factory = json.loads((archive / 'factory_vision_config.json').read_text())
        old_settings = json.loads((archive / 'factory_cycle_settings.json').read_text())
        original_manifest = json.loads((Path(commissioned['run']) / 'controller_manifest.json').read_text())
    except (OSError, ValueError, KeyError):
        return None
    expected_settings = dict(old_settings, home_z_offset_m=old_settings.get('home_z_offset_m', 0.) + .055)
    if planner.settings != expected_settings:
        return None
    keys = ('destination_mode', 'conveyor_placement_mode', 'conveyor_target_O_T_EE')
    old_motion = {key: old_factory[key] for key in keys}
    expected_motion = copy.deepcopy(old_motion)
    expected_motion['conveyor_target_O_T_EE'][12] += .025
    expected_motion['conveyor_target_O_T_EE'][14] += .055
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
    if current_manifest != original_manifest:
        return None
    return dict(translation_world_m=[.025, 0., .055], home_z_offset_delta_m=.055,
                controller_timing_evidence_reused_from=commissioned['run'],
                original_planner_identity=baseline, current_planner_identity=planner.identity,
                new_geometry_physically_commissioned=False,
                full_trajectory_model_and_native_checks_required=True)
