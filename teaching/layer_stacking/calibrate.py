"""Fit local image servo from saved actual edge frames and measured jogs.

Input: reference edges, a known aligned reference image, and a neutral plus
six +/- local X/Y/yaw captures. Never sends motion or enables hardware.
"""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
from .vision import feature,fit_calibration


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('manifest',type=Path);parser.add_argument('--out',type=Path,required=True);args=parser.parse_args()
    data=json.loads(args.manifest.read_text());config=json.loads(Path(__file__).with_name('config.json').read_text())
    reference=data['reference'];samples=[]
    aligned=cv2.imread(str(args.manifest.parent/data['aligned_image']))
    if aligned is None:raise ValueError('Missing actual aligned reference image')
    target,_=feature(aligned,reference,config['vision'])
    for sample in data['samples']:
        if np.asarray(sample['delta_xy_yaw']).shape!=(3,) or any(v is None for v in sample['delta_xy_yaw']):raise ValueError('Fill actual measured X/Y/yaw deltas; template values are not calibration')
        image=cv2.imread(str(args.manifest.parent/sample['image']))
        if image is None:raise ValueError('Missing calibration frame '+sample['image'])
        value,_=feature(image,reference,config['vision']);samples.append(dict(delta_xy_yaw=sample['delta_xy_yaw'],feature=value.tolist()))
    fitted=fit_calibration(samples,target.tolist())
    if max(fitted['fit_rms'][:2])>1.5 or fitted['fit_rms'][2]>.002:raise ValueError('Image/pose calibration residual too large')
    J=np.asarray(fitted['image_jacobian']);condition=np.linalg.cond(np.diag([1,1,150])@J@np.diag([.001,.001,.01]))
    if condition>config['vision']['maximum_calibration_condition']:raise ValueError('Ill-conditioned local image calibration')
    args.out.write_text(json.dumps(dict(reference,**fitted,normalized_condition=float(condition)),indent=2)+'\n')
    print(args.out,'local image model saved; hardware commissioning remains disabled')


if __name__=='__main__':main()
