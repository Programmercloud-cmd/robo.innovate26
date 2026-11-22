#!/usr/bin/python3
"""Preview/execute a newly taught factory pose; grasp approaches leave the hand open."""
import argparse
import json
import math
from pathlib import Path
import subprocess
import sys
from pose_ids import normalize_pose_id, pose_key

HERE=Path(__file__).resolve().parent
TASK_ID='factory_columns_of_four'


def resolve_pose(catalog,identifier):
    if catalog.get('task_id')!=TASK_ID:raise ValueError('Not the isolated factory pose catalog')
    number=normalize_pose_id(identifier);key=pose_key(number)
    if key not in catalog['points']:raise ValueError(f'Factory pose #{number} has not been recorded')
    if catalog['points'][key].get('coordinate_status')=='requires_reteach_after_reboot':
        raise ValueError(f'Factory pose #{number} needs fresh teaching after the reported coordinate mismatch')
    if catalog['points'][key].get('operator_rejected'):
        raise ValueError(f'Factory pose #{number} was rejected by the operator; reteach it before use')
    if catalog['points'][key].get('pose_source') in ('factory_tool_frame_inference','factory_world_frame_inference'):
        from infer_source_poses import source_signature
        if catalog['points'][key]['source_signature']!=source_signature(catalog):
            raise ValueError('Inferred source poses are stale; rerun infer_source_poses.py --write after updating the configured anchor')
    return number,catalog['points'][key],False


def capture_pose():
    result=subprocess.run([str(HERE/'read_pose')],capture_output=True,text=True,timeout=10)
    if result.returncode:raise RuntimeError(result.stderr.strip() or result.stdout.strip())
    return json.loads(result.stdout)


def rotation_error(a,b):
    trace=sum(a[c*4+r]*b[c*4+r] for c in range(3) for r in range(3))
    return math.acos(max(-1,min(1,(trace-1)/2)))


def vertical_pose(pose):
    result=pose.copy()
    if math.hypot(pose[0],pose[1])<.1:raise ValueError('Tool-X heading is undefined near vertical')
    yaw=math.atan2(pose[1],pose[0]);c,s=math.cos(yaw),math.sin(yaw)
    result[:12]=[c,s,0,0,s,-c,0,0,0,0,-1,0]
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('point',help='Source n_j (n=0..5, j=0..2), or numeric conveyor/reference ID')
    parser.add_argument('--execute',action='store_true')
    parser.add_argument('--grasp',action='store_true',help='Set the configured opening and approach the taught grasp reference; does not close')
    parser.add_argument('--vertical-tool',action='store_true')
    parser.add_argument('--z-offset-mm',type=float,default=0)
    parser.add_argument('--relative-z-mm',type=float,help='Explicit stationary-start Z jog, limited to +/-2 mm')
    parser.add_argument('--time-scale',type=float,default=1)
    args=parser.parse_args()
    if not math.isfinite(args.time_scale) or not 1<=args.time_scale<=4:parser.error('--time-scale must be 1..4')
    if not math.isfinite(args.z_offset_mm) or abs(args.z_offset_mm)>20:parser.error('--z-offset-mm must be within +/-20 mm')
    if args.relative_z_mm is not None and (not math.isfinite(args.relative_z_mm) or abs(args.relative_z_mm)>2 or args.z_offset_mm):
        parser.error('--relative-z-mm must be within +/-2 mm without --z-offset-mm')
    if args.grasp and (args.relative_z_mm is not None or args.z_offset_mm or args.vertical_tool):
        parser.error('--grasp uses the saved reference without temporary pose overrides')
    raw=(HERE/'positions.json').read_bytes();catalog=json.loads(raw)
    config=json.loads((HERE/'motion_config.json').read_text())
    try:identifier,point,_=resolve_pose(catalog,args.point)
    except ValueError as error:parser.error(str(error))
    if config.get('task_id')!=TASK_ID:parser.error('Motion configuration belongs to another task')
    if not config.get('workspace_bounds_m') or not config.get('workspace_reference_distance_m'):
        parser.error('Factory workspace is unconfigured; record new poses and use configure_workspace.py first')
    if args.execute and not config.get('verified_joint_plans_enabled'):
        parser.error('Factory motion execution is disabled in motion_config.json')
    try:capture=capture_pose()
    except RuntimeError as error:print('STOP:',error,file=sys.stderr);return 1
    from arc_motion import run_arc
    if args.grasp:
        from gripper_width import command_width
        from audit_kinematics import yaw_variant
        reference=point.get('grasp_reference',{}).get('O_T_EE')
        if reference is None:parser.error('This factory pose has no grasp reference')
        upper=point['O_T_EE'].copy()
        if config.get('force_vertical_tool'):
            upper=vertical_pose(upper);reference=vertical_pose(reference)
        offset=[reference[12+i]-upper[12+i] for i in range(3)]
        if abs(offset[0])>1e-9 or abs(offset[1])>1e-9 or not -.020<=offset[2]<=0:
            parser.error('Grasp reference must be a vertical offset of at most 20 mm')
        code=command_width('approach',args.execute)
        if code:return code
        if args.execute:
            try:capture=capture_pose()
            except RuntimeError as error:print('STOP:',error,file=sys.stderr);return 1
        code=run_arc(identifier,upper,capture,raw,point,args.execute,dict(stage='factory_upper_approach',time_scale=args.time_scale))
        if code or not args.execute or all(value==0 for value in offset):return code
        try:capture=capture_pose()
        except RuntimeError as error:print('STOP before descent:',error,file=sys.stderr);return 1
        actual=capture['state']['O_T_EE'];variants=[reference.copy()]
        if config['singularity_avoidance']['allow_yaw180_for_bricks']:variants.append(yaw_variant(reference,180))
        target=min(variants,key=lambda p:rotation_error(actual,p))
        if math.dist(actual[12:15],upper[12:15])>.001 or rotation_error(actual,target)>math.pi/180:
            print('STOP: upper approach outside pose tolerance',file=sys.stderr);return 1
        return run_arc(identifier,target,capture,raw,point,True,
            dict(stage='factory_grasp_descent',target_kind='grasp_reference',direct_saved_grasp=True,
                 saved_grasp_offset_base_m=offset,time_scale=args.time_scale))
    actual=capture['state']['O_T_EE']
    target=(actual if args.relative_z_mm is not None else point['O_T_EE']).copy()
    target[14]+=(args.relative_z_mm if args.relative_z_mm is not None else args.z_offset_mm)/1000
    if args.vertical_tool or config.get('force_vertical_tool'):
        target=vertical_pose(target)
    if args.relative_z_mm is not None and rotation_error(actual,target)>math.radians(2):
        parser.error('Jog reorientation is limited to 2 degrees')
    return run_arc(identifier,target,capture,raw,point,args.execute,
        dict(z_offset_mm=args.z_offset_mm,relative_z_mm=args.relative_z_mm,
             vertical_tool=args.vertical_tool,time_scale=args.time_scale))


if __name__=='__main__':sys.exit(main())
