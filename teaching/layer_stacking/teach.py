"""Explicit read-only robot capture into this task's isolated taught poses."""
import argparse
from datetime import datetime,timezone
import json
from pathlib import Path
import sys
import numpy as np
from .geometry import ROOT


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('kind',choices=('source','seated-target'));parser.add_argument('label',nargs='?');args=parser.parse_args()
    if args.kind=='source' and args.label not in {f'external_r{r}_c{c}' for r in range(6) for c in range(2)}:parser.error('Source needs external_r0_c0 through external_r5_c1')
    sys.path.insert(0,str(ROOT/'teaching'));from move_xyz import capture_pose
    data=capture_pose();state=data['state'];pose=state['O_T_EE'];R=np.array(pose).reshape(4,4,order='F')[:3,:3]
    if np.linalg.norm(R[:,2]-[0,0,-1])>.0002:raise ValueError('Guide tool vertical before recording layer poses')
    config_path=Path(__file__).with_name('config.json');config=json.loads(config_path.read_text());h=config['hardware']
    if h['expected_F_T_EE'] is not None and not np.allclose(h['expected_F_T_EE'],state['F_T_EE'],atol=1e-6,rtol=0):raise ValueError('Tool changed; revalidate the grasp/camera profile first')
    stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ');history=config_path.parent/'teaching_history';history.mkdir(exist_ok=True)
    (history/(stamp+'_configuration_before.json')).write_bytes(config_path.read_bytes())
    (history/(stamp+'_capture.json')).write_text(json.dumps(data,indent=2)+'\n')
    h['expected_F_T_EE']=state['F_T_EE']
    if args.kind=='source':h['source_poses'][args.label]=pose
    else:h['first_seated_O_T_EE']=pose
    # Teaching never implicitly re-enables previous contact/camera approval.
    h['enabled']=False;h['commissioning_id']=None
    config_path.write_text(json.dumps(config,indent=2)+'\n')
    print('Saved',args.kind,args.label or '',pose[12:15],'; hardware remains disabled; no motion commanded')


if __name__=='__main__':main()
