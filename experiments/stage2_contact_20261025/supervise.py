"""Bound exactly one B0 child; reporting completion is not contact success."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[2]
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT))
from experiments.partial_disassembly_20261024.render_supervisor import finish_child


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True);parser.add_argument('--python',required=True)
    args=parser.parse_args()
    if os.environ.get('ROBO_PUEUE_JOB')!='1':raise RuntimeError('Default Pueue is mandatory')
    protocol=json.loads((HERE/'protocol.json').read_text())
    args.output.mkdir(parents=True,exist_ok=False)
    final=ROOT/'experiments/final_workcell_20261024'
    command=[args.python,str(HERE/'run_contact.py'),'--contact-case','B0_stock','--output',str(args.output),
        '--geometry',str(final/'geometry.json'),'--cameras',str(final/'cameras_realism.json'),
        '--appearance',str(final/'appearance_config.json'),'--views','all','--headless','--enable_cameras','--device','cpu']
    env=os.environ.copy();env['OSQP_EPSILON']='1e-4';env.pop('ROBO_CONTACT_OBSERVER',None)
    with (args.output/'process.log').open('w') as log:
        child=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True,env=env)
        def cancel(number,_frame):
            try:os.killpg(child.pid,signal.SIGKILL)
            except ProcessLookupError:pass
            raise SystemExit(128+number)
        signal.signal(signal.SIGTERM,cancel);signal.signal(signal.SIGINT,cancel)
        start,marked,reason=time.monotonic(),None,None
        try:
            while child.poll() is None:
                if (args.output/'failure.json').exists():reason='setup_or_capture_failure';break
                if (args.output/'contact_complete.json').exists():
                    if marked is None:marked=time.monotonic()
                    if time.monotonic()-marked>5:reason='shutdown_stall_after_capture';break
                if time.monotonic()-start>protocol['motion']['wall_limit_s']:reason='timeout';break
                time.sleep(.2)
        finally:code=finish_child(child)
    complete=(args.output/'contact_complete.json').exists() and reason not in ('timeout','setup_or_capture_failure')
    save={'case':'B0_stock','capture_completed':complete,'contact_validated':False,'independent_audit_required':True,
        'child_returncode':code,'cleanup_result':reason or 'returned','wall_seconds':time.monotonic()-start}
    (args.output/'supervisor.json').write_text(json.dumps(save,indent=2)+'\n')
    if not complete:raise RuntimeError('B0 incomplete; preserve partial geometry/failure evidence, no guessing')


if __name__=='__main__':main()
