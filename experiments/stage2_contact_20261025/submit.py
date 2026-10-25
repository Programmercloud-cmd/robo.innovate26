"""Freeze only the authorized B0 stock baseline and queue in default Pueue."""
from datetime import datetime,timezone
import hashlib
import json
from pathlib import Path
import shlex
import shutil
import subprocess

ROOT=Path(__file__).resolve().parents[2]
HERE=Path(__file__).resolve().parent
PYTHON='/home/faps/miniforge3/envs/unitree_sim_env/bin/python'


def main():
    scope=['experiments/stage2_contact_20261025','experiments/stage2_idle_20261025',
           'experiments/final_workcell_20261024','challenge2','experiments/partial_disassembly_20261024/render_supervisor.py']
    if subprocess.check_output(['git','status','--porcelain','--',*scope],cwd=ROOT):raise SystemExit('Commit reviewed B0 and independent protocol before freezing')
    protocol=json.loads((HERE/'protocol.json').read_text())
    if protocol['execution_authorized_cases']!=['B0_stock']:raise RuntimeError('Proxy/reserved execution not authorized')
    for name,digest in protocol['input_sha256'].items():
        if hashlib.sha256((ROOT/name).read_bytes()).hexdigest()!=digest:raise RuntimeError('Frozen B0 input changed: '+name)
    snapshot=ROOT/'runs'/(datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')+'-stage2-contact-B0-stock')
    snapshot.mkdir()
    for name in ('.deps','challenge2','configs'):
        shutil.copytree(ROOT/name,snapshot/name,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
    for name in ('stage2_contact_20261025','stage2_idle_20261025','final_workcell_20261024'):
        shutil.copytree(ROOT/'experiments'/name,snapshot/'experiments'/name,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
    partial=snapshot/'experiments/partial_disassembly_20261024';partial.mkdir()
    shutil.copy2(ROOT/'experiments/partial_disassembly_20261024/render_supervisor.py',partial/'render_supervisor.py')
    command=[PYTHON,str(snapshot/'experiments'/HERE.name/'supervise.py'),'--python',PYTHON,'--output',str(snapshot/'output')]
    lines=['#!/usr/bin/env bash','set -eo pipefail','source /home/faps/miniforge3/etc/profile.d/conda.sh',
        'conda activate /home/faps/miniforge3/envs/unitree_sim_env','set -u','cd '+shlex.quote(str(snapshot)),
        'export PYTHONPATH='+shlex.quote(str(snapshot/'.deps')+':'+str(snapshot)),
        'export ROBO_PUEUE_JOB=1 OSQP_EPSILON=1e-4 PYTHONDONTWRITEBYTECODE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2',
        shlex.join(command),'']
    (snapshot/'launch.sh').write_text('\n'.join(lines))
    (snapshot/'manifest.json').write_text(json.dumps({'source_commit':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        'case':'B0_stock','scope':'Stock pad geometry and bounded normal-contact reporting baseline only; no STL collision replacement/reserved/tangential/extraction execution',
        'sha256':{str(p.relative_to(snapshot)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(snapshot.rglob('*')) if p.is_file()}},indent=2)+'\n')
    result=subprocess.run(['pueue','add','--label','robo26-stage2-contact-B0-stock',shlex.join(['bash',str(snapshot/'launch.sh')])],check=True,capture_output=True,text=True)
    (snapshot/'submission.txt').write_text(result.stdout);print('Snapshot:',snapshot);print(result.stdout.strip())


if __name__=='__main__':main()
