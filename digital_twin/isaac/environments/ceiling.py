"""Ceiling datum calculation in finger coordinates (+Z points downward)."""
import math


def thickness(spec):
    settings = spec['ceiling']
    if settings.get('mode') == 'preserve_current':
        value = settings['retained_thickness_m']
        if not isinstance(value,(int,float)) or isinstance(value,bool) or not math.isfinite(value) or value<=0:
            raise ValueError('Positive retained ceiling thickness required')
        return value
    return settings['nominal_thickness_m']+verified_clearance(spec)


def verified_clearance(spec):
    ceiling = spec['ceiling']
    clearance = ceiling.get('fingertip_to_brick_bottom_clearance_m')
    if (not isinstance(clearance,(int,float)) or isinstance(clearance,bool)
            or not math.isfinite(clearance) or clearance<0
            or ceiling.get('clearance_verified') is not True
            or not ceiling.get('clearance_evidence')):
        raise ValueError('Wide ceiling requires a verified fingertip-to-brick-bottom clearance and its evidence; estimated display-table heights are not a measurement')
    if clearance>=spec['whole_layer_size_m'][2]:
        raise ValueError('Clearance leaves no side-wall engagement with the brick body')
    return clearance


def calculate(spec, profile_top, fingertip_bottom, stud_height):
    settings = spec['ceiling']
    old_thickness = settings['fixed_top_reference_thickness_m']
    if settings.get('mode') == 'preserve_current':
        top = profile_top-old_thickness
        return {'top_surface_z_finger_m':top,'underside_z_finger_m':top+thickness(spec),
                'thickness_m':thickness(spec),'previous_underside_z_finger_m':profile_top,
                'fingertip_to_brick_bottom_clearance_m':None,'brick_bottom_z_finger_m':None,
                'stud_top_z_finger_m':None,'stud_contact_error_m':None,'clearance_evidence':None,
                'verification_scope':'Existing ceiling retained for side-spacing-only revision; stud contact not asserted'}
    clearance = verified_clearance(spec)
    nominal = settings['nominal_thickness_m']
    if not all(isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v) and v>0
               for v in (old_thickness,nominal,stud_height)):
        raise ValueError('Positive finite ceiling and stud dimensions required')
    top = profile_top-old_thickness
    total_thickness = nominal+clearance
    underside = top+total_thickness
    brick_bottom = fingertip_bottom+clearance
    stud_top = brick_bottom-spec['whole_layer_size_m'][2]-stud_height
    if abs(underside-stud_top)>1e-7:
        raise ValueError('Nominal ceiling thickness disagrees with the retained CAD profile/brick dimensions')
    return {'top_surface_z_finger_m':top, 'underside_z_finger_m':underside,
            'thickness_m':total_thickness, 'nominal_thickness_m':nominal,
            'previous_underside_z_finger_m':profile_top,
            'fingertip_to_brick_bottom_clearance_m':clearance,
            'brick_bottom_z_finger_m':brick_bottom,'stud_top_z_finger_m':stud_top,
            'stud_contact_error_m':underside-stud_top,
            'clearance_evidence':settings['clearance_evidence'],
            'verification_scope':'Geometric contact at the taught grasp using supplied verified clearance; no physical contact/lift trial'}
