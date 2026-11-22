"""Per-invocation factory geometry; never writes shared coordinates."""
import copy
import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
SETUP_NAMES = ('tabletop', 'conveyor', 'conveyor_moving')


def resolve_setup(name, settings, factory, motion):
    if name not in SETUP_NAMES:
        raise ValueError('Select factory setup: tabletop, conveyor or conveyor_moving')
    config = json.loads((HERE / 'setups.json').read_text())
    profile = config['setups'][name]
    geometry_name=profile.get('geometry_profile',name)
    if geometry_name not in ('tabletop','conveyor'):
        raise ValueError('Unknown geometry profile')
    geometry=config['setups'][geometry_name]
    delta = geometry['placement_home_offset_world_m']
    expected = [0., 0., 0.] if geometry_name == 'tabletop' else [.025, 0., .055]
    if delta != expected:
        raise ValueError(f'{name}: offsets must match the explicitly supported setup {expected}')
    if factory.get('conveyor_placement_mode') != 'stationary_vertical':
        raise ValueError('Factory setups require stationary_vertical placement; speed matching is disabled')
    settings, factory, motion = map(copy.deepcopy, (settings, factory, motion))
    for key,limit in (('joint_velocity_rad_s',1.),('joint_acceleration_rad_s2',2.),('joint_jerk_rad_s3',40.),
                      ('translation_velocity_m_s',.4),('endpoint_position_m',.001)):
        value=settings[key]
        if not isinstance(value,(int,float)) or not math.isfinite(value) or not 0<value<=limit:
            raise ValueError(f'Unsafe factory setting: {key}')
    for key in ('translation_acceleration_m_s2','translation_jerk_m_s3','endpoint_angle_rad','observation_height_m'):
        if not math.isfinite(settings[key]) or settings[key]<=0:
            raise ValueError(f'Invalid factory setting: {key}')
    gripper=motion['gripper']
    if gripper['approach_opening_m']!=.07 or gripper['close_target_m']!=.06:
        raise ValueError('Both factory setups retain the configured 70 mm open / 60 mm close widths')
    if config['tabletop_home_z_offset_m'] != 0.:
        raise ValueError('Tabletop home must retain its original lift plane without the recent offset')
    baseline = config['tabletop_placement_O_T_EE']
    if len(baseline) != 16 or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in baseline):
        raise ValueError('Invalid tabletop placement transform')
    if (baseline[3::4] != [0., 0., 0., 1.] or baseline[2] != 0 or baseline[6] != 0 or
            baseline[8:11] != [0., 0., -1.] or
            abs(baseline[0]**2 + baseline[1]**2 - 1) > 1e-6 or
            abs(baseline[4]-baseline[1]) > 1e-6 or abs(baseline[5]+baseline[0]) > 1e-6):
        raise ValueError('Placement transform must be a vertical rigid tool pose')
    factory['conveyor_target_O_T_EE'] = baseline.copy()
    for axis in range(3):
        factory['conveyor_target_O_T_EE'][12+axis] += delta[axis]
    # An operator-confirmed measured release height belongs only to this
    # surface. Keep the nominal setup translation and home lift plane intact.
    teaching = geometry.get('placement_height_teaching')
    placement_trim = 0.
    if teaching is not None:
        nominal_z = factory['conveyor_target_O_T_EE'][14] + factory.get('conveyor_surface_height_offset_m', 0.)
        measured_z = teaching['world_z_m']
        if (teaching.get('operator_confirmed') is not True or
                not isinstance(measured_z, (int, float)) or not math.isfinite(measured_z) or
                abs(teaching['previous_placement_world_z_m'] - nominal_z) > 1e-9):
            raise ValueError('Placement height teaching is invalid or belongs to different nominal geometry')
        placement_trim = measured_z - nominal_z
        if abs(placement_trim) > .002:
            raise ValueError('Measured placement height trim exceeds the bounded 2 mm local correction')
        factory['conveyor_target_O_T_EE'][14] += placement_trim
    factory['destination_mode'] = 'fixed_conveyor'
    placement_mode='speed_matched' if name=='conveyor_moving' else 'stationary_vertical'
    factory['conveyor_placement_mode']=placement_mode
    if name=='conveyor_moving':
        factory['phase_schedule']=dict(factory.get('phase_schedule',{}),enabled=False)
    settings['home_z_offset_m'] = config['tabletop_home_z_offset_m'] + delta[2]
    # Retain the source-side bounds. Extend only the placement-side X bound by
    # the authorized conveyor translation, and restore the authorized ceiling.
    bounds = motion['workspace_bounds_m']
    bounds['max'][0] += delta[0]
    bounds['max'][2] = config['authorized_workspace_ceiling_m']
    if bounds['max'][2] != .5:
        raise ValueError('Workspace ceiling must retain the operator-authorized 500 mm limit')
    if (len(bounds['min'])!=3 or len(bounds['max'])!=3 or
            any(not math.isfinite(v) for v in bounds['min'] + bounds['max']) or
            any(a >= b for a,b in zip(bounds['min'], bounds['max']))):
        raise ValueError('Invalid factory workspace bounds')
    info = dict(name=name, description=profile['description'],
                placement_home_offset_world_m=delta.copy(),
                placement_height_correction_m=placement_trim,
                placement_height_teaching=copy.deepcopy(teaching),
                workspace_bounds_m=copy.deepcopy(bounds), placement_mode=placement_mode,
                geometry_profile=geometry_name,
                moving_parameters=copy.deepcopy(profile.get('moving_parameters',{})),
                tabletop_placement_O_T_EE=baseline.copy(),
                source_poses_and_gripper_unchanged=True)
    return settings, factory, motion, info
