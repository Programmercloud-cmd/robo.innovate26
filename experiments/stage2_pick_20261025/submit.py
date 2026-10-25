"""Freeze one reviewed native-pick phase and submit it to default Pueue."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shlex
import shutil
import subprocess

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
PYTHON = '/home/faps/miniforge3/envs/unitree_sim_env/bin/python'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def predecessor(path, case, mode, protocol_sha):
    report = json.loads(path.read_text())
    timing = json.loads((path.parent.parent/'pipeline_supervisor.json').read_text())
    if (report.get('case') != case or report.get('mode') != mode
            or report.get('proceed_gate_passed') is not True
            or report.get('protocol_sha256') != protocol_sha
            or timing.get('case') != case or timing.get('mode') != mode
            or timing.get('proceed_gate_passed') is not True
            or timing.get('within_budget') is not True
            or timing.get('audit_sha256') != sha(path)):
        raise RuntimeError('Independent predecessor and actual timing do not match: '+str(path))
    return timing


def main():
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--case', choices=('I', 'H'), required=True)
    parser.add_argument('--mode', choices=('control', 'pick'), required=True)
    parser.add_argument('--predecessor-control', type=Path)
    parser.add_argument('--predecessor-development', type=Path)
    args = parser.parse_args()
    protocol = json.loads((HERE/'protocol.json').read_text())
    protocol_sha = sha(HERE/'protocol.json')
    if not protocol['execution_authorized']:
        raise RuntimeError('Review and freeze the implementation before execution')
    if subprocess.check_output(['git', 'status', '--porcelain', '--', str(HERE)], cwd=ROOT):
        raise RuntimeError('Commit the reviewed stage source before freezing')
    prior = 0.
    if args.mode == 'pick':
        if args.predecessor_control is None:
            parser.error('A passing matched stationary control is required')
        prior = predecessor(args.predecessor_control, args.case, 'control', protocol_sha)['cumulative_case_seconds']
    elif args.predecessor_control is not None:
        parser.error('Control mode has no same-case predecessor')
    if args.case == 'H':
        if args.predecessor_development is None:
            parser.error('Reserved H requires independent development I success')
        predecessor(args.predecessor_development, 'I', 'pick', protocol_sha)
    elif args.predecessor_development is not None:
        parser.error('Development I cannot take a reserved predecessor')

    snapshot = ROOT/'runs'/(datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
                            +f'-stage2-pick-{args.case}-{args.mode}')
    snapshot.mkdir()
    # Reuse the known working runtime/asset inventory; copy current sources,
    # not old run output or a symlink to a mutable checkout.
    known = json.loads((ROOT/'runs/20261025T064519.962873Z-explicit-tip-native-query-targeted-mass/manifest.json').read_text())
    for name in known['sha256']:
        if name == 'launch.sh' or name.startswith('experiments/compound_native_query_20261025'):
            continue
        destination = snapshot/name
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(ROOT/name, destination)
    for relative in ('experiments/stage2_pick_20261025',
                     'experiments/stage2_exposed_tip_20261025',
                     'experiments/stage1_20261025'):
        shutil.copytree(ROOT/relative, snapshot/relative,
                        ignore=shutil.ignore_patterns('__pycache__', '*.pyc', 'inputs', 'results.json'),
                        dirs_exist_ok=True)
    for relative in ('experiments/sim_pose_20261024/calibration.py',
                     'experiments/sim_views_20261024/render_views.py',
                     'experiments/partial_disassembly_20261024/render_adapter.py',
                     'experiments/partial_disassembly_20261024/benchmark.json'):
        (snapshot/relative).parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(ROOT/relative, snapshot/relative)
    inputs = snapshot/'experiments'/HERE.name/'inputs'
    inputs.mkdir()
    stock = ROOT/'runs/20261025T032638.185262Z-stage2-contact-B0-stock/output'
    files = {
        'geometry_audit.json': ROOT/'runs/20261025T080456.949852Z-exposed-tip-G/audit/evaluation.json',
        'side_authorization.json': ROOT/'experiments/stage2_exposed_tip_20261025/side_authorization.json',
        'D_contact_audit.json': ROOT/'runs/20261025T083717.764321Z-exposed-tip-D/audit/evaluation.json',
        'R_contact_audit.json': ROOT/'runs/20261025T084049.478049Z-exposed-tip-R/audit/evaluation.json',
        'baseline_metadata.json': stock/'metadata.json',
        'baseline_geometry.json': stock/'geometry.json',
        'tip.stl': ROOT/'experiments/final_workcell_20261024/end_effector/assets/franka_research3_gripper.stl',
    }
    command = [PYTHON, str(snapshot/'experiments'/HERE.name/'pipeline.py'),
               '--case', args.case, '--mode', args.mode, '--prior-wall-seconds', repr(prior)]
    for flag in ('predecessor_control', 'predecessor_development'):
        value = getattr(args, flag)
        if value is not None:
            name = flag+'.json'
            files[name] = value
            files[flag+'_pipeline_supervisor.json'] = value.parent.parent/'pipeline_supervisor.json'
            command += ['--'+flag.replace('_', '-'), str(inputs/name)]
    for name, source in files.items():
        shutil.copy2(source, inputs/name)
    lines = ['#!/usr/bin/env bash', 'set -eo pipefail',
             'source /home/faps/miniforge3/etc/profile.d/conda.sh',
             'conda activate /home/faps/miniforge3/envs/unitree_sim_env', 'set -u',
             'cd '+shlex.quote(str(snapshot)),
             'export ROBO_PUEUE_JOB=1 OSQP_EPSILON=1e-4 PYTHONDONTWRITEBYTECODE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2',
             'export PYTHONPATH='+shlex.quote(str(snapshot/'.deps')+':'+str(snapshot)),
             shlex.join(command), '']
    (snapshot/'launch.sh').write_text('\n'.join(lines))
    manifest = {
        'scope': protocol['objective'], 'case': args.case, 'mode': args.mode,
        'source_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        'protocol_sha256': protocol_sha, 'maximum_attempts_this_phase': 1,
        'prior_case_wall_seconds': prior,
        'case_capture_plus_audit_seconds': protocol['budget']['capture_plus_independent_audit_seconds_per_case'],
        'external_isaaclab_commit': subprocess.check_output(
            ['git', '-C', '/home/faps/dev/resources/IsaacLab', 'rev-parse', 'HEAD'], text=True).strip(),
        'sha256': {str(p.relative_to(snapshot)): sha(p) for p in sorted(snapshot.rglob('*')) if p.is_file()},
    }
    (snapshot/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    submitted = subprocess.run(['pueue', 'add', '--label', f'robo26-native-pick-{args.case}-{args.mode}',
                                shlex.join(['bash', str(snapshot/'launch.sh')])],
                               check=True, capture_output=True, text=True)
    (snapshot/'submission.txt').write_text(submitted.stdout)
    print(snapshot)
    print(submitted.stdout.strip())


if __name__ == '__main__':
    main()
