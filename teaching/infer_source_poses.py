#!/usr/bin/python3
"""Derive 18 world-aligned factory grasp poses from the configured anchor; no robot commands."""
import argparse
import copy
from datetime import datetime,timezone
import hashlib
import json
from pathlib import Path

HERE=Path(__file__).resolve().parent
CONFIG=dict(anchor='point_5_0',anchor_indices=[5,0],rows=list(range(6)),columns=[0,1,2],
    row_step_in_base_m=[0,.0782,0],column_step_in_base_m=[-.0318,0,0],
    frame='robot_base_O',orientation='world aligned; tool Z down; tool X world +X')
ROTATION=[1,0,0,0,0,-1,0,0,0,0,-1,0]
INFERRED_SOURCES=('factory_tool_frame_inference','factory_world_frame_inference')


def source_signature(catalog):
    config=catalog['factory_source_grid']['config'];p=catalog['points'][config['anchor']]
    if p.get('operator_rejected'):raise ValueError('The configured source anchor was rejected; reteach it first')
    value=dict(config=config,anchor={key:p[key] for key in ('xyz_m','O_T_EE','tool_frames')},
        grasp_offset=catalog['default_grasp_offset_base_m'])
    if config.get('row_anchors'):
        value['row_anchors']={row:{key:catalog['points'][name][key] for key in ('xyz_m','O_T_EE','tool_frames')}
            for row,name in config['row_anchors'].items()}
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':')).encode()).hexdigest()


def infer(catalog,replace_independent=False):
    c=copy.deepcopy(catalog)
    if c.get('task_id')!='factory_columns_of_four':raise ValueError('Not the factory catalog')
    config=c['factory_source_grid']['config'];anchor_key=config['anchor']
    if config.get('frame')!='robot_base_O':raise ValueError('This generator requires the relocated world-aligned layout')
    if anchor_key not in c['points']:raise ValueError(f'Record anchor {anchor_key} first')
    if c['default_grasp_offset_base_m']!=[0,0,0]:raise ValueError('Factory grasp poses must have zero offset')
    anchor=c['points'][anchor_key]
    if anchor.get('operator_rejected'):raise ValueError('Configured anchor is rejected')
    anchor['O_T_EE']=ROTATION+anchor['xyz_m']+[1]
    anchor['grasp_reference'].update(xyz_m=anchor['xyz_m'].copy(),O_T_EE=anchor['O_T_EE'].copy(),
        offset_from_recorded_pose_base_m=[0,0,0],motion_validated=False)
    anchor['orientation_policy']=config['orientation']
    anchor['commanded_tool_z_tilt_from_base_down_deg']=0
    anchor['joint_angles_role']='Measured at the raw capture pose; IK seed only, not a solved configuration for inherited Z or normalized orientation'
    signature=source_signature(c);generated=datetime.now(timezone.utc).isoformat()
    anchor_row,anchor_column=config['anchor_indices']
    row_anchors=config.get('row_anchors',{})
    protected={anchor_key,*row_anchors.values()}
    for row in config['rows']:
        for column in config['columns']:
            label=f'{row}_{column}';key=f'point_{label}'
            if key in protected:continue
            existing=c['points'].get(key)
            if existing and existing.get('pose_source') not in INFERRED_SOURCES and not replace_independent:
                raise ValueError(f'Pose {label} was independently taught; will not overwrite it')
            base_key=row_anchors.get(str(row),anchor_key);base=c['points'][base_key]
            base_row=row if str(row) in row_anchors else anchor_row
            base_column=0 if str(row) in row_anchors else anchor_column
            offset=[(row-base_row)*config['row_step_in_base_m'][i]+(column-base_column)*config['column_step_in_base_m'][i] for i in range(3)]
            xyz=[base['xyz_m'][i]+offset[i] for i in range(3)];matrix=ROTATION+xyz+[1]
            interpolation=None
            if config.get('interpolate_row_anchors') and str(row) not in row_anchors:
                lower=max((int(r) for r in row_anchors if int(r)<row),default=None)
                upper=min((int(r) for r in row_anchors if int(r)>row),default=None)
                if lower is not None and upper is not None:
                    a,b=[c['points'][row_anchors[str(r)]]['xyz_m'] for r in (lower,upper)]
                    fraction=(row-lower)/(upper-lower)
                    xyz=[a[i]+fraction*(b[i]-a[i])+column*config['column_step_in_base_m'][i] for i in range(3)]
                    if existing:xyz[2]=existing['xyz_m'][2]
                    matrix=ROTATION+xyz+[1]
                    interpolation=dict(anchors=[row_anchors[str(lower)],row_anchors[str(upper)]],fraction=fraction)
            c['points'][key]=dict(image_point=row+1,pose_id=label,label=label,source_row=row,source_column=column,
                task_id=c['task_id'],role='source_stack_anchor',pose_kind='inferred_grasp',xyz_m=xyz,O_T_EE=matrix,q_rad=None,
                joint_configuration=None,tool_frames=copy.deepcopy(base['tool_frames']),
                grasp_reference=dict(source=f'World-frame grid from {base["pose_id"]}: measured XY, inherited grasp Z, no extra grasp offset',
                    taught_directly=False,xyz_m=xyz.copy(),O_T_EE=matrix.copy(),offset_from_recorded_pose_base_m=[0,0,0],motion_validated=False),
                pose_source='factory_world_frame_inference',derived_from=base_key,source_signature=signature,
                offset_from_anchor_base_m=offset,generated_utc=generated,
                orientation_policy=config['orientation'],motion_to_pose_validated=False)
            if interpolation:c['points'][key]['row_interpolation']=interpolation
    c['factory_source_grid'].update(source_signature=signature,generated_utc=generated,
        world_row_step_m=config['row_step_in_base_m'].copy(),world_column_step_m=config['column_step_in_base_m'].copy(),
        tool_yaw_deg=0,preserve_world_z=True,robot_commands_sent=False)
    labels=[f'{row}_{column}' for row in config['rows'] for column in config['columns']]
    c['layout'].update(source_anchor_pose_ids=[c['points'][key]['pose_id'] for key in sorted(protected)],source_pose_ids=labels,
        coordinates_status='configured row anchors measured; remaining source poses inferred; grasp Z retained',grid_coordinate_frame='robot_base_O')
    c['layout']['reference_roles'].update({label:'source_stack_anchor' for label in labels})
    return c


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--write',action='store_true');args=parser.parse_args()
    path=HERE/'positions.json';raw=path.read_text();c=infer(json.loads(raw))
    if args.write:
        history=HERE/'history';history.mkdir(exist_ok=True);stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
        (history/f'positions_before_source_inference_{stamp}.json').write_text(raw)
        temporary=HERE/'.positions.json.tmp';temporary.write_text(json.dumps(c,indent=2,allow_nan=False)+'\n');temporary.replace(path)
    for label in c['layout']['source_pose_ids']:
        print(f'Pose {label}: XYZ mm',*[round(v*1000,6) for v in c['points'][f'point_{label}']['xyz_m']])
    print('18 world-aligned source poses; inherited grasp Z; no extra offset; no robot movement.')


if __name__=='__main__':main()
