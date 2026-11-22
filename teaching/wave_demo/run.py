"""Independent arm-wave demo. Default: model checks only, no SDK access."""
import argparse
from datetime import datetime,timezone
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys

import numpy as np

from .plan import HERE,TEACHING,Plan


def write(path,value):path.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute',action='store_true',help='Run empty-hand demo; open gripper, within vertical column above home 5 (XY +/-50 mm)')
    parser.add_argument('--cycles',type=int);parser.add_argument('--period',type=float,help='Seconds per left/right cycle')
    args=parser.parse_args();cfg=json.loads((HERE/'config.json').read_text())
    if args.cycles is not None:cfg['cycles']=args.cycles
    if args.period is not None:cfg['period_s']=args.period
    out=HERE/'runs'/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ');out.mkdir(parents=True)
    try:
        start_q=None
        if args.execute:
            # The shared read-only helper acquires an exclusive lease and exits
            # before the dedicated controller connects. No background restart.
            raw=subprocess.run([str(TEACHING/'read_pose')],capture_output=True,text=True,timeout=10)
            if raw.returncode:raise RuntimeError(raw.stderr.strip() or raw.stdout.strip())
            capture=json.loads(raw.stdout);write(out/'start_capture.json',capture);s=capture['state']
            if s['robot_mode']!='Idle' or s['current_errors']:raise RuntimeError('Wave requires fault-free Idle')
            if not np.allclose(s['F_T_EE'],cfg['tool_F_T_EE'],atol=1e-6,rtol=0):raise RuntimeError('Tool frame changed; update isolated wave teaching first')
            start_q=s['q_d']
        plan=Plan(cfg);manifest=plan.build(start_q);write(out/'manifest.json',manifest)
        (out/'robot_model.urdf').write_bytes((TEACHING/'robot_model.urdf').read_bytes())
        phases=[dict(name=p['name'],seconds=round(p['duration_s'],3),certificate=p['certificate']) for p in manifest['paths']]
        write(out/'summary.json',dict(model_only=not args.execute,phases=phases,planned_duration_s=manifest['planned_duration_s'],
            home_5_xyz_m=cfg['home_5_O_T_EE'][12:15],upright_xyz_m=plan.pose(cfg['upright_q_rad'])[12:15],
            commanded_tilt_deg=cfg['commanded_tilt_deg'],measured_tilt_limit_deg=cfg['maximum_measured_tilt_deg']))
        for phase in phases:print(phase['name'],phase['seconds'],'s',flush=True)
        print('Home5 XYZ mm:',[round(v*1000,2) for v in cfg['home_5_O_T_EE'][12:15]],flush=True)
        print('Wave: world Y; commanded +/-',cfg['commanded_tilt_deg'],'deg; measured cap',cfg['maximum_measured_tilt_deg'],'deg',flush=True)
        print('Fingers: tool Z points world +X throughout waving; sweep measured relative to neutral forearm',flush=True)
        print('Isolated workspace m:',cfg['workspace_bounds_m'],'LOG',out,flush=True)
        binary=HERE/'controller'
        if not binary.exists():raise RuntimeError('Build first: bash teaching/wave_demo/build.sh')
        checked=subprocess.run([str(binary),str(out/'manifest.json'),'--check'],text=True,capture_output=True,timeout=30)
        (out/'native_check.log').write_text(checked.stdout+checked.stderr)
        if checked.returncode:raise RuntimeError(checked.stderr.strip() or checked.stdout.strip())
        if not args.execute:print('MODEL_CHECK_PASS; no SDK connection or robot motion');return 0
        print('EMPTY-HAND execution; gripper commands are disabled; Ctrl-C stops the demo',flush=True)
        command=['powerprofilesctl','launch','--profile','performance','--reason','Isolated arm wave demo',
            '--appid','franka-wave',str(binary),str(out/'manifest.json'),'--execute']
        with (out/'controller.ndjson').open('w') as log:
            child=subprocess.Popen(command,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,bufsize=1)
            last_phase=-1
            try:
                for line in child.stdout:
                    log.write(line);log.flush()
                    try:
                        sample=json.loads(line);phase=sample.get('phase',-1)
                        if phase!=last_phase and 0<=phase<len(phases):print('PHASE',phases[phase]['name'],flush=True);last_phase=phase
                    except ValueError:print(line.rstrip(),flush=True)
                code=child.wait()
            except KeyboardInterrupt:
                child.send_signal(2);code=child.wait(timeout=5)
        write(out/'outcome.json',dict(completed=code==0,returncode=code,factory_inventory_modified=False,
            controller_sha256=hashlib.sha256(binary.read_bytes()).hexdigest()))
        return code
    except Exception as error:
        write(out/'outcome.json',dict(completed=False,error=str(error),factory_inventory_modified=False))
        print('RECOVERABLE_STOP',error,'LOG',out,file=sys.stderr);return 1


if __name__=='__main__':sys.exit(main())
