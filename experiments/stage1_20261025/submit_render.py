"""Snapshot the approved final-workcell/R3 interface benchmark; default Pueue."""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shlex
import shutil
import subprocess

EXP = Path(__file__).resolve().parent
ROOT = EXP.parents[1]
PYTHON = '/home/faps/miniforge3/envs/unitree_sim_env/bin/python'


def main():
    scope = ['experiments/stage1_20261025', 'experiments/final_workcell_20261024',
             'experiments/partial_disassembly_20261024/render_adapter.py', 'challenge2']
    if subprocess.check_output(['git','status','--porcelain','--',*scope],cwd=ROOT):
        raise SystemExit('Commit scoped code/configuration before freezing')
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
    snap = ROOT/'runs'/(stamp+'-stage1-final-workcell-render')
    snap.mkdir()
    for name in ('.deps','challenge2','configs'):
        shutil.copytree(ROOT/name,snap/name,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
    for name in ('stage1_20261025','final_workcell_20261024','partial_disassembly_20261024',
                 'sim_views_20261024','sim_pose_20261024'):
        shutil.copytree(ROOT/'experiments'/name,snap/'experiments'/name,
                        ignore=shutil.ignore_patterns('__pycache__','*.pyc','upstream'))
    # Detector source is deliberately excluded from render job, and will be
    # frozen with the inference-only sensor package in a separate queued job.
    command = [PYTHON,str(snap/'experiments/stage1_20261025/supervise_render.py'),
               '--benchmark',str(snap/'experiments/stage1_20261025/benchmark.json'),
               '--geometry',str(snap/'experiments/final_workcell_20261024/geometry.json'),
               '--cameras',str(snap/'experiments/final_workcell_20261024/cameras_realism.json'),
               '--appearance',str(snap/'experiments/final_workcell_20261024/appearance_config.json'),
               '--output',str(snap/'render_output'),'--python',PYTHON]
    lines = ['#!/usr/bin/env bash','set -eo pipefail',
             'source /home/faps/miniforge3/etc/profile.d/conda.sh',
             'conda activate /home/faps/miniforge3/envs/unitree_sim_env','set -u',
             'cd '+shlex.quote(str(snap)),
             'export PYTHONPATH='+shlex.quote(str(snap/'.deps')+':'+str(snap)),
             'export ROBO_PUEUE_JOB=1 PYTHONDONTWRITEBYTECODE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2',
             shlex.join(command),'test -f render_output/run_complete.json','']
    (snap/'launch.sh').write_text('\n'.join(lines))
    (snap/'manifest.json').write_text(json.dumps({
        'source_commit':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        'scope':'Frozen native partial states; reserved images sealed from implementation feedback',
        'sha256':{str(p.relative_to(snap)):hashlib.sha256(p.read_bytes()).hexdigest()
                  for p in sorted(snap.rglob('*')) if p.is_file()}},indent=2)+'\n')
    result = subprocess.run(['pueue','add','--label','robo26-stage1-render',
                             shlex.join(['bash',str(snap/'launch.sh')])],check=True,capture_output=True,text=True)
    (snap/'submission.txt').write_text(result.stdout)
    print('Snapshot:',snap); print(result.stdout.strip())


if __name__ == '__main__':
    main()
