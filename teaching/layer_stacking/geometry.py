"""Compact twelve-brick layers. Plate poses are simulation candidates, not TCPs."""
import json
import math
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[2]


def rz(angle):
    c,s=math.cos(angle),math.sin(angle)
    return np.array([[c,-s,0],[s,c,0],[0,0,1.]])


def transform(xyz,yaw=0.):
    result=np.eye(4);result[:3,:3]=rz(yaw);result[:3,3]=xyz
    return result


def build_plan(config):
    g=config['geometry'];catalog=json.loads((ROOT/g['source_catalog']).read_text())
    if g['alternate_quarter_turns'] is not True:raise ValueError('Layer stacking requires alternating 90-degree orientations')
    if g['footprint_columns_rows']!=[3,4]:raise ValueError('Only the fixed three-column/four-row layer is supported')
    if len(g['brick_body_size_m'])!=3 or any(not np.isfinite(v) or v<=0 for v in g['brick_body_size_m']):raise ValueError('Invalid brick body dimensions')
    if abs(g['layer_pitch_m']-g['brick_body_size_m'][2])>1e-9:raise ValueError('Layer pitch must match the configured brick body height')
    ext=json.loads((ROOT/g['external_layout']).read_text())
    plate_config=json.loads((ROOT/g['plate_config']).read_text());plate=plate_config['baseplate']
    reg=json.loads((ROOT/g['plate_registration']).read_text())
    T=np.array(reg['T_O_P_display_proxy']);pitch=g['stud_pitch_m'];center=np.array(g['target_center_stud'])
    if any(abs(v-.5-round(v-.5))>1e-8 for v in center):raise ValueError('Target center must preserve stud-grid parity')
    if config['job']['receiving_stack_initial_layers']!=0:raise ValueError('Start with a clear independently confirmed target; existing layers are not assumed')
    available={p['footprint_id']:(p['row'],p['bank']) for p in ext['footprints']}
    sources={}
    for label,(row,bank) in available.items():
        point=catalog['points'][f'point_{row}_1']['O_T_EE'];pose=np.array(point).reshape(4,4,order='F')
        # Right-bank geometry is a nominal simulation extension, never a teach.
        pose[0,3]-=bank*ext['geometry']['bank_pitch_m']
        sources[label]=dict(row=row,bank=bank,sim_O_T_grasp=pose.tolist(),
            pose_status='existing_left_center' if bank==0 else 'nominal_right_bank_simulation_only',
            physical_whole_layer_pose_verified=False)
    requested=config['job']['sources']
    if not requested:raise ValueError('Select at least one whole-layer source')
    if len(set(requested))!=len(requested) or any(s not in sources for s in requested):raise ValueError('Use distinct external_r0_c0 through external_r5_c1 footprints')
    layers=[];previous_studs={}
    for k,source in enumerate(requested):
        turn=k%2 if g['alternate_quarter_turns'] else 0;yaw=turn*math.pi/2
        origin=np.r_[center*pitch,k*g['layer_pitch_m']];pose=T@transform(origin,yaw)
        bricks=[]
        for row in range(4):
            for col in range(3):
                # Three 4-stud bricks end-to-end; four 2-stud ridges, no gaps.
                offset=np.array([(4*col-4)*pitch,(2*row-3)*pitch,0.])
                local=rz(yaw)@offset+origin
                studs=[]
                for u in range(4):
                    for v in range(2):
                        ij=center+rz(yaw)[:2,:2]@np.array([4*col-4+u-1.5,2*row-3+v-.5])
                        if any(abs(x-round(x))>1e-8 for x in ij):raise ValueError('Invalid layer stud parity')
                        if any(not g['tape_border_studs']<=ij[i]<plate['grid_shape'][i]-g['tape_border_studs'] for i in range(2)):
                            raise ValueError('Layer overlaps taped border or extends beyond plate')
                        studs.append([int(round(x)) for x in ij])
                matched=studs if k==0 else [p for p in studs if tuple(p) in previous_studs]
                if not matched:raise ValueError('Upper brick has no nominal stud support')
                c=local[:2]/pitch
                in_support=all(min(p[i] for p in matched)<=c[i]<=max(p[i] for p in matched) for i in (0,1))
                bricks.append(dict(id=f'layer_{k}_r{row}_c{col}',row=row,column=col,
                    P_T_bottom=transform(local,yaw).tolist(),O_T_bottom=(T@transform(local,yaw)).tolist(),studs=studs,
                    support=dict(potential_matched_studs=len(matched),center_inside_nominal_support=in_support,
                        lower_owners=['baseplate'] if k==0 else sorted({previous_studs[tuple(p)] for p in matched}),
                        cantilever_clutch_validation_required=not in_support,mechanical_support_verified=False)))
        layers.append(dict(index=k,source=source,yaw_rad=yaw,O_T_layer_bottom=pose.tolist(),bricks=bricks,
            potential_stud_matches=sum(b['support']['potential_matched_studs'] for b in bricks),engagement_verified=False))
        previous_studs={tuple(p):b['id'] for b in bricks for p in b['studs']}
    return dict(schema=1,task=config['task'],simulation_only=True,physical_registration_verified=False,
        target_semantics='layer bottom center, NOT robot TCP',plate_O_T_P=T.tolist(),
        target_center_stud=center.tolist(),layer_pitch_m=g['layer_pitch_m'],sources=sources,layers=layers,
        brick_body_size_m=g['brick_body_size_m'],stud_pitch_m=pitch,
        visual_stud_height_m=plate_config['visual_geometry_priors']['stud_height_m'],
        visual_stud_diameter_m=plate_config['visual_geometry_priors']['stud_diameter_m'],
        footprint_studs=[12,8],physical_contact_validated=False)
