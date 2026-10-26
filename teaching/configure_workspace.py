#!/usr/bin/python3
"""Derive factory workspace bounds from its taught poses. No robot connection."""
import argparse
from datetime import datetime,timezone
import hashlib
import json
import math
from pathlib import Path

HERE=Path(__file__).resolve().parent


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--margin-mm',type=float,default=100)
    parser.add_argument('--ceiling-mm',type=float,default=500)
    parser.add_argument('--apply',action='store_true',help='Apply the displayed bounds after checking clearance in the new workcell')
    args=parser.parse_args()
    if not math.isfinite(args.margin_mm) or args.margin_mm<=0 or not math.isfinite(args.ceiling_mm):parser.error('Invalid margin/ceiling')
    raw=(HERE/'positions.json').read_bytes();catalog=json.loads(raw)
    if catalog.get('task_id')!='factory_columns_of_four':parser.error('Not the factory catalog')
    if not catalog['points']:parser.error('Record factory poses before configuring its workspace')
    positions=[]
    for point in catalog['points'].values():
        positions.append(point['xyz_m'])
        if point.get('grasp_reference'):positions.append(point['grasp_reference']['xyz_m'])
    if not all(len(p)==3 and all(math.isfinite(v) for v in p) for p in positions):parser.error('Invalid taught coordinates')
    ceiling=args.ceiling_mm/1000
    if max(p[2] for p in positions)>ceiling:parser.error('A taught pose exceeds the requested ceiling')
    margin=args.margin_mm/1000
    lower=[min(p[i] for p in positions)-margin for i in range(3)]
    upper=[max(p[i] for p in positions)+margin for i in range(3)];upper[2]=min(upper[2],ceiling)
    bounds=dict(min=lower,max=upper);distance=math.dist(lower,upper)
    print(json.dumps(dict(workspace_bounds_m=bounds,workspace_reference_distance_m=distance,applied=args.apply),indent=2))
    if args.apply:
        path=HERE/'motion_config.json';config=json.loads(path.read_text())
        if config.get('task_id')!='factory_columns_of_four':parser.error('Not the factory motion configuration')
        directory=HERE/'history';directory.mkdir(exist_ok=True)
        stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
        (directory/f'motion_config_before_workspace_{stamp}.json').write_text(path.read_text())
        config.update(workspace_bounds_m=bounds,workspace_reference_distance_m=distance,
            verified_joint_plans_enabled=True,clearance_basis='Caller-applied bounds derived only from newly taught factory upper/lower poses',
            clearance_recorded_utc=datetime.now(timezone.utc).isoformat(),
            workspace_catalog_sha256=hashlib.sha256(raw).hexdigest())
        temporary=HERE/'.motion_config.json.tmp';temporary.write_text(json.dumps(config,indent=2)+'\n');temporary.replace(path)


if __name__=='__main__':main()
