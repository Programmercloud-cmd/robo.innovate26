#!/usr/bin/python3
"""Deterministic 4x3 output stacks and color routing; never connects to hardware."""
import argparse
import copy
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re

from generate_stack import pose, source_signature as input_signature

HERE = Path(__file__).resolve().parent
STACK_ID = re.compile(r'([bcd])_(\d+)_(\d+)_(\d+)\Z')
DEFAULT_CONFIG = {
    'ridges_per_layer': 4,
    'bricks_per_ridge': 3,
    'shared_geometry_source': 'input_stack.config',
    'bottom_height_source': 'input height anchor minus (input layer count - 1) brick heights',
    'vertical_layer_anchor': 'point_11',
    'stack_anchors': {'b': 'point_11', 'c': 'point_14', 'd': 'point_15'},
    'calibration_indices': {
        'point_11': {'n': 0, 'm': 0}, 'point_09': {'n': 1, 'm': 0},
        'point_12': {'n': 0, 'm': 1}, 'point_10': {'n': 0, 'm': 0},
        'point_08': {'n': 0, 'm': 1}, 'point_13': {'n': 1, 'm': 0},
    },
    'calibration_basis': 'Indices inferred from the annotated image and measured center spacings; layer indices inferred from input-derived bottom height',
    'tool_alignment_source': 'a_0_0_9',
    'cached_layer_indices': [0, 1],
    'indexing': {'n': 'ridge 0..3', 'm': 'brick 0..2 along the ridge',
                 'k': 'layer from the bottom, starting at 0; no fixed maximum'},
    'index_direction': 'Within each parity, n/m increase away from its bottom-left corner in the annotated image',
}
DEFAULT_RULES = {
    'good_colors': {'300121': 'Red', '6117418': 'Maroon/dark red', '6487946': 'Orange-brown'},
    'reject_colors': {'6135191': 'Tan', '4211201': 'Dark brown'},
    'requested_stack': 'b', 'storage_stacks': ['c', 'd'],
    'storage_color_order': ['300121', '6117418', '6487946'],
    'reject_destination': 'point_01',
}


def source_signature(catalog, config):
    anchors = set(config['calibration_indices']) | set(config['stack_anchors'].values())
    anchors.add(catalog['input_stack']['config']['height_anchor'])
    reference = catalog['input_bricks'][config['tool_alignment_source']]
    payload = dict(model_version=1, config=config, input_config=catalog['input_stack']['config'],
                   input_calibration_signature=input_signature(catalog,catalog['input_stack']['config']),
                   grasp_offset=catalog['default_grasp_offset_base_m'],
                   points={key: {field: catalog['points'][key][field]
                                 for field in ('xyz_m', 'O_T_EE', 'tool_frames')} for key in sorted(anchors)},
                   tool_alignment={field: reference[field] for field in ('O_T_EE', 'brick_axes_world')})
    return hashlib.sha256(json.dumps(payload, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def local_xy(config, shared, vertical_parity, n, m, k):
    # Use the same center-preserving parity convention as the input generator.
    # Reversing ridge numbering after rotation retains the bottom-left start.
    length, ridge = shared['brick_center_pitch_m'], shared['ridge_center_pitch_m']
    if k % 2 == vertical_parity:
        return [n * ridge, m * length]
    center_n = (config['ridges_per_layer'] - 1) * ridge / 2
    center_m = (config['bricks_per_ridge'] - 1) * length / 2
    return [center_n + (m - (config['bricks_per_ridge'] - 1) / 2) * length,
            center_m + (n - (config['ridges_per_layer'] - 1) / 2) * ridge]


def infer(catalog, config):
    if (config['ridges_per_layer'], config['bricks_per_ridge']) != (4, 3):
        raise ValueError('The taught output layout is four ridges of three bricks')
    shared = catalog['input_stack']['config']
    if input_signature(catalog, shared) != catalog['input_stack']['report']['source_signature']:
        raise ValueError('Input calibration is stale; regenerate it before inferring output stacks')
    if catalog['default_grasp_offset_base_m'] != [0, 0, -shared['grasp_offset_m']]:
        raise ValueError('Global and input-stack grasp offsets disagree')
    points = catalog['points']; height = shared['layer_height_m']
    bottom_z = points[shared['height_anchor']]['xyz_m'][2] - (shared['layers'] - 1) * height
    layers = {}
    for name in set(config['calibration_indices']) | set(config['stack_anchors'].values()):
        measured = points[name]['xyz_m'][2]
        index = math.floor((measured - bottom_z) / height + .5)
        if index < 0: raise ValueError(f'{name} is below the inferred bottom layer')
        layers[name] = index
    parity = layers[config['vertical_layer_anchor']] % 2
    if any(layers[name] % 2 != parity for name in config['stack_anchors'].values()):
        raise ValueError('Corner anchors disagree on the shared vertical-brick parity')
    origin = points[config['stack_anchors']['b']]['xyz_m']
    dot = cross = 0.0
    for name, indices in config['calibration_indices'].items():
        x, y = local_xy(config, shared, parity, indices['n'], indices['m'], layers[name])
        actual = points[name]['xyz_m']; wx, wy = actual[0]-origin[0], actual[1]-origin[1]
        dot += x*wx + y*wy; cross += x*wy - y*wx
    if math.hypot(dot, cross) < 1e-12: raise ValueError('Cannot infer the output axes')
    angle = math.atan2(cross, dot); co, si = math.cos(angle), math.sin(angle)
    n_axis, m_axis = [co, si, 0], [-si, co, 0]
    reference = catalog['input_bricks'][config['tool_alignment_source']]
    input_axis = reference['brick_axes_world']['length']
    input_angle = math.atan2(input_axis[1], input_axis[0])
    tool_yaw = math.atan2(reference['O_T_EE'][1], reference['O_T_EE'][0])
    tool_offset = math.remainder(tool_yaw - input_angle, math.pi)
    residuals = {}; anchor_ids = {}
    for name, indices in config['calibration_indices'].items():
        n, m, k = indices['n'], indices['m'], layers[name]
        x, y = local_xy(config, shared, parity, n, m, k)
        predicted = [origin[0]+co*x-si*y, origin[1]+si*x+co*y, bottom_z+k*height]
        measured = points[name]['xyz_m']
        residuals[name] = dict(brick_id=f'b_{n}_{m}_{k}', measured_xyz_m=measured,
                               normalized_upper_xyz_m=predicted,
                               xy_residual_mm=math.dist(predicted[:2], measured[:2])*1000,
                               measured_minus_grid_z_mm=(measured[2]-predicted[2])*1000)
        anchor_ids[name] = f'b_{n}_{m}_{k}'
    for stack, name in config['stack_anchors'].items():
        anchor_ids[name] = f'{stack}_0_0_{layers[name]}'
    if max(r['xy_residual_mm'] for r in residuals.values()) > min(shared['brick_center_pitch_m'], shared['ridge_center_pitch_m'])*250:
        raise ValueError('Output anchors do not fit the inferred grid indexing')
    stacks = {}
    for stack, name in config['stack_anchors'].items():
        anchor = points[name]['xyz_m']
        stacks[stack] = dict(role='requested' if stack=='b' else 'storage', anchor=name,
            anchor_inferred_id=anchor_ids[name], anchor_measured_xyz_m=anchor,
            anchor_normalized_z_m=bottom_z+layers[name]*height,
            anchor_measured_minus_grid_z_mm=(anchor[2]-bottom_z-layers[name]*height)*1000,
            vertical_layer_origin_xy_m=anchor[:2],
            center_xy_m=[anchor[0]+co*1.5*shared['ridge_center_pitch_m']-si*shared['brick_center_pitch_m'],
                         anchor[1]+si*1.5*shared['ridge_center_pitch_m']+co*shared['brick_center_pitch_m']])
    return dict(source_signature=source_signature(catalog, config), bottom_upper_z_m=bottom_z,
        bottom_grasp_placement_z_m=bottom_z-shared['grasp_offset_m'], layer_height_m=height,
        brick_center_pitch_m=shared['brick_center_pitch_m'], ridge_center_pitch_m=shared['ridge_center_pitch_m'],
        gap_between_ridges_m=shared['ridge_center_pitch_m']-shared['brick_dimensions_m']['width'],
        vertical_layer_parity=parity, layer_zero_long_axis='parallel to image bottom' if parity else 'perpendicular to image bottom',
        image_right_axis_world=n_axis, image_up_axis_world=m_axis,
        image_right_heading_deg=math.degrees(angle), tool_to_brick_yaw_offset_rad=tool_offset,
        inferred_anchor_ids=anchor_ids, inferred_recorded_layers=layers, calibration_residuals=residuals,
        max_xy_anchor_residual_mm=max(r['xy_residual_mm'] for r in residuals.values()),
        max_abs_anchor_height_residual_mm=max(abs(points[name]['xyz_m'][2]-bottom_z-k*height)*1000 for name,k in layers.items()),
        bricks_per_layer=12, maximum_layer_index=None, stacks=stacks,
        model_scope='On-demand geometry for any nonnegative k; robot workspace and motion checks are independent and unchanged.',
        physical_validation='Derived coordinates only. Output placement, stud connection, occupancy and reachability are not established by the geometry fit.')


def make_pose(catalog, config, report, stack, n, m, k):
    if stack not in ('b','c','d') or any(type(v) is not int or v<0 for v in (n,m,k)) or n>=4 or m>=3:
        raise ValueError('Output ID must use b/c/d, n=0..3, m=0..2, k>=0')
    shared = catalog['input_stack']['config']
    x,y = local_xy(config,shared,report['vertical_layer_parity'],n,m,k)
    right,up = report['image_right_axis_world'],report['image_up_axis_world']
    origin = report['stacks'][stack]['vertical_layer_origin_xy_m']
    xyz = [origin[i]+x*right[i]+y*up[i] for i in (0,1)] + [report['bottom_upper_z_m']+k*report['layer_height_m']]
    if not all(math.isfinite(v) for v in xyz): raise ValueError('Output coordinates are not finite')
    length_axis = up if k%2==report['vertical_layer_parity'] else right
    yaw = math.atan2(length_axis[1],length_axis[0])+report['tool_to_brick_yaw_offset_rad']
    yaw = (yaw+math.pi/2)%math.pi-math.pi/2
    upper = pose(xyz,yaw); lower = upper.copy();lower[14]-=shared['grasp_offset_m']
    width_axis = [-length_axis[1],length_axis[0],0]
    return dict(stack=stack,n=n,m=m,k=k,layer_index_from_bottom=k,layer_from_bottom=k+1,
        ridge=n+1,brick_in_ridge=m+1,xyz_m=xyz,O_T_EE=upper,
        grasp_reference=dict(xyz_m=lower[12:15],O_T_EE=lower,offset_from_recorded_pose_base_m=[0,0,-shared['grasp_offset_m']],motion_validated=False),
        placement_reference=dict(xyz_m=lower[12:15],O_T_EE=lower.copy(),motion_validated=False,
                                 note='Derived matching contact-height reference; insertion/seating behavior has not been tested'),
        brick_axes_world=dict(length=length_axis.copy(),width=width_axis,up=[0,0,1]),
        layer_rotation_deg=0 if k%2==0 else 90,
        long_edge_in_image='perpendicular to bottom border' if k%2==report['vertical_layer_parity'] else 'parallel to bottom border',
        tool_frames=copy.deepcopy(catalog['points'][config['stack_anchors'][stack]]['tool_frames']),
        pose_source='deterministic_output_stack_model',joint_configuration=None,
        motion_to_pose_validated=False,grasp_physically_validated=False,placement_physically_validated=False,
        occupancy=None,note='Geometry does not assert that this slot is occupied or that lower layers have been built')


def resolve_output_pose(catalog, identifier):
    match=STACK_ID.fullmatch(str(identifier))
    if not match: raise ValueError('Use b_n_m_k, c_n_m_k or d_n_m_k for output poses')
    if 'output_stacks' not in catalog:raise ValueError('Generate the output stack calibration first')
    model=catalog['output_stacks'];config=model['config'];report=model['report']
    if source_signature(catalog,config)!=report['source_signature']:
        raise ValueError('Output anchors/configuration changed; regenerate output stacks before using these poses')
    stack,n,m,k=match.groups()
    return make_pose(catalog,config,report,stack,int(n),int(m),int(k))


def next_slot(stack, placed_count):
    if stack not in ('b','c','d') or type(placed_count) is not int or placed_count<0:
        raise ValueError('Provide a stack b/c/d and a verified nonnegative placed count')
    k,within=divmod(placed_count,12);n,m=divmod(within,3)
    return f'{stack}_{n}_{m}_{k}'


def plan_job(catalog, requested_quantities=None):
    quantities=(catalog['sorting_job']['requested_quantities'] if requested_quantities is None else requested_quantities)
    rules=catalog['sorting_rules'];good=list(rules['storage_color_order'])
    if not isinstance(quantities,dict) or not 1<=len(quantities)<=3:
        raise ValueError('Configure one to three requested good colors with positive quantities')
    if any(str(color) not in good or type(count) is not int or count<=0 for color,count in quantities.items()):
        raise ValueError('Requested quantities must be positive integers for known good colors')
    requested={str(color):count for color,count in quantities.items()}
    if len(requested)!=len(quantities):raise ValueError('Requested color IDs must be unique')
    remaining=[color for color in good if color not in requested]
    destination={color:'b' for color in requested};storage={}
    for stack,color in zip(rules['storage_stacks'],remaining):destination[color]=stack;storage[stack]=color
    destination.update({color:rules['reject_destination'] for color in rules['reject_colors']})
    return dict(requested_quantities=requested,requested_stack='b',storage_colors=storage,
        active_stacks=['b']+list(storage),color_destinations=destination,
        requested_brick_count=sum(requested.values()),requested_layers=(sum(requested.values())+11)//12,
        bricks_per_layer=12,requested_sequence=catalog['sorting_job'].get('requested_sequence'),
        unknown_color_action='reobserve_or_recoverable_stop',occupancy_initialized=False)


def destination_for(catalog,color_id,requested_quantities):
    mapping=plan_job(catalog,requested_quantities)['color_destinations']
    if str(color_id) not in mapping:raise ValueError('Unknown/uncertain color: reobserve or stop, never infer reject')
    return mapping[str(color_id)]


def render(catalog,path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon
    config=catalog['output_stacks']['config'];report=catalog['output_stacks']['report']
    shared=catalog['input_stack']['config'];dims=shared['brick_dimensions_m']
    right,up=report['image_right_axis_world'],report['image_up_axis_world']
    origin=catalog['points']['point_11']['xyz_m']
    def project(xyz):
        d=[xyz[i]-origin[i] for i in (0,1)]
        return [sum(d[i]*axis[i] for i in (0,1))*1000 for axis in (right,up)]
    fig,axes=plt.subplots(1,2,figsize=(13,8),constrained_layout=True)
    for k,ax in enumerate(axes):
        for stack,color in [('b','#bcd7f3'),('c','#cce5c8'),('d','#f4d5ad')]:
            for n in range(4):
                for m in range(3):
                    brick=make_pose(catalog,config,report,stack,n,m,k);x,y=project(brick['xyz_m'])
                    vertical=k%2==report['vertical_layer_parity'];length,width=dims['length']*1000,dims['width']*1000
                    dx,dy=(width,length) if vertical else (length,width)
                    ax.add_patch(Polygon([(x+sx*dx/2,y+sy*dy/2) for sx,sy in [(-1,-1),(1,-1),(1,1),(-1,1)]],facecolor=color,edgecolor='#44505a',linewidth=.7))
                    ax.text(x,y,f'{stack}_{n}_{m}_{k}',fontsize=6,ha='center',va='center',rotation=90 if vertical else 0)
            anchor=report['stacks'][stack]['anchor'];x,y=project(catalog['points'][anchor]['xyz_m']);ax.plot(x,y,'rx');ax.annotate('#'+str(int(anchor[-2:])),(x,y),xytext=(-20,-13),textcoords='offset points',color='#a00000')
        ax.autoscale_view();ax.set_aspect('equal');ax.grid(alpha=.25);ax.set_xlabel('Right in annotated image (mm)');ax.set_ylabel('Up in annotated image (mm)');ax.set_title(f'Layer k={k}: upper Z {(report["bottom_upper_z_m"]+k*report["layer_height_m"])*1000:.3f} mm')
    fig.suptitle('Inferred output stacks: b requested, c/d storage; red crosses are recorded corner XY')
    fig.savefig(path,dpi=180);plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--write',action='store_true');parser.add_argument('--plot',action='store_true')
    parser.add_argument('--pose',help='Print a calculated b/c/d_n_m_k pose, at any nonnegative layer')
    parser.add_argument('--request',nargs='+',help='Print routing for COLOR_ID:QUANTITY entries')
    args=parser.parse_args();raw=(HERE/'positions.json').read_text();catalog=json.loads(raw)
    if args.pose:
        print(json.dumps(resolve_output_pose(catalog,args.pose),indent=2));return
    if args.request:
        entries=[entry.split(':',1) for entry in args.request]
        if any(len(entry)!=2 for entry in entries) or len({entry[0] for entry in entries})!=len(entries):parser.error('Use unique COLOR_ID:QUANTITY entries')
        print(json.dumps(plan_job(catalog,{color:int(count) for color,count in entries}),indent=2));return
    config=copy.deepcopy(catalog.get('output_stacks',{}).get('config',DEFAULT_CONFIG));report=infer(catalog,config)
    cached={f'{stack}_{n}_{m}_{k}':make_pose(catalog,config,report,stack,n,m,k)
            for stack in ('b','c','d') for k in config['cached_layer_indices'] for n in range(4) for m in range(3)}
    catalog['output_stacks']=dict(config=config,report=report,generated_utc=datetime.now(timezone.utc).isoformat())
    catalog['output_bricks']=cached
    catalog.setdefault('sorting_rules',copy.deepcopy(DEFAULT_RULES))
    catalog.setdefault('sorting_job',dict(requested_quantities={},requested_sequence=None,
        status='Configure quantities before executing a sorting job; no occupancy assumed'))
    if args.write:
        stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
        with (HERE/'history'/f'positions_before_output_stacks_{stamp}.json').open('x') as backup:backup.write(raw)
        temporary=HERE/'.positions.json.tmp';temporary.write_text(json.dumps(catalog,indent=2,allow_nan=False)+'\n');temporary.replace(HERE/'positions.json')
    (HERE/'output_stack_report.json').write_text(json.dumps(report,indent=2)+'\n')
    if args.plot:render(catalog,HERE/'output_stack_layout.png')
    print(f'Output stacks b/c/d: 12 bricks per layer, {len(cached)} parity-preview poses cached, no model height cap')
    print(f'Bottom upper Z {report["bottom_upper_z_m"]*1000:.6f} mm; lower reference Z {report["bottom_grasp_placement_z_m"]*1000:.6f} mm')
    print('Inferred recorded anchors:',json.dumps(report['inferred_anchor_ids'],sort_keys=True))
    print(f'Maximum XY anchor residual {report["max_xy_anchor_residual_mm"]:.3f} mm; height residual {report["max_abs_anchor_height_residual_mm"]:.3f} mm')
    print('Catalog written:',args.write)


if __name__=='__main__':main()
