"""VM launcher. Only this launcher's own variant processes can be replaced."""
import argparse
import datetime
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import urllib.request

ROOT = Path(__file__).resolve().parents[2]
STATE = ROOT/'simulation_variant_session.json'


def owned_running(record, field):
    try:
        proc = Path('/proc')/str(record[field])
        fields = (proc/'stat').read_text().split()
        command = (proc/'cmdline').read_bytes().decode().replace('\0',' ')
        return (fields[2] != 'Z' and fields[21] == record.get(field+'_start')
                and str(record['output']) in command)
    except (OSError, KeyError):
        return False


def stop_owned(record):
    for field in ('isaac_pid','http_pid'):
        pid = record.get(field)
        if not pid:
            continue
        proc = Path('/proc')/str(pid)
        try:
            if (proc/'stat').read_text().split()[2] == 'Z':
                continue
            args = (proc/'cmdline').read_bytes().decode().replace('\0',' ')
            start = (proc/'stat').read_text().split()[21]
            if record.get(field+'_start') != start or str(record['output']) not in args:
                raise RuntimeError('Refusing to stop a PID not owned by this variant launcher')
            os.killpg(pid,signal.SIGTERM)
            deadline = time.monotonic()+15
            while proc.exists() and (proc/'stat').read_text().split()[2] != 'Z' and time.monotonic()<deadline:
                time.sleep(.1)
            if proc.exists() and (proc/'stat').read_text().split()[2] != 'Z':
                os.killpg(pid,signal.SIGKILL)
        except FileNotFoundError:
            pass


def launch(args):
    upstream = ROOT/'active_session.json'
    # Validate the measured clearance before stopping a working variant.
    if args.fingers == 'wide':
        from ceiling import verified_clearance
        profiles = json.loads(Path(__file__).with_name('profiles.json').read_text())
        if profiles['fingers']['wide']['ceiling'].get('mode') != 'preserve_current':
            verified_clearance(profiles['fingers']['wide'])
    if STATE.exists():
        old = json.loads(STATE.read_text())
        alive = all(owned_running(old,k) for k in ('isaac_pid','http_pid'))
        if alive and old.get('environment') == args.environment and old.get('fingers') == args.fingers and not args.rebuild:
            print(json.dumps(old)); return
        stop_owned(old)
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    output = ROOT/'simulation_variants'/f'{args.environment}-{args.fingers}-{stamp}'
    output.parent.mkdir(exist_ok=True)
    # mirror.py creates its fresh output directory; keep logs alongside it.
    log_path = output.with_suffix('.log')
    source = json.loads(upstream.read_text())
    brick_path = Path(source.get('http_root',source['output']))/'brick_observations.json'
    command = ['bash',str(ROOT/'isaac/launch.sh'),'--output',str(output),
               '--registration',str(ROOT/'registration/scene_registration.json'),
               '--environment',args.environment,'--fingers',args.fingers,
               '--follow-twin',str(upstream),'--export-viewer',
               '--brick-observations',str(brick_path)]
    with log_path.open('w') as log:
        child = subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    record = {'environment':args.environment,'fingers':args.fingers,'output':str(output),
              'isaac_pid':child.pid,'isaac_pid_start':(Path('/proc')/str(child.pid)/'stat').read_text().split()[21],
              'log':str(log_path),'upstream':str(upstream),'hardware_clients':0}
    STATE.write_text(json.dumps(record,indent=2)+'\n')
    deadline = time.monotonic()+55
    while time.monotonic()<deadline:
        if child.poll() is not None:
            raise RuntimeError('Isaac variant exited; inspect '+str(log_path))
        if all((output/name).exists() for name in ('latest.json','manifest.json','digital_twin.usdc')):
            break
        time.sleep(.2)
    else:
        stop_owned(record)
        raise RuntimeError('Bounded variant initialization timed out: '+str(log_path))
    label = f'{args.environment} · {args.fingers} fingertips'
    selector = '''<!doctype html><meta charset="utf-8"><title>FR3 simulation selector</title>
<style>body{margin:0;font:15px system-ui;background:#edf0f2}nav{height:52px;display:flex;align-items:center;gap:16px;padding:0 16px}button{padding:7px 12px;cursor:pointer}iframe{border:0;width:100%;height:calc(100vh - 52px)}</style>
<nav><b>Explicit simulation selection</b><button id="factory">Existing factory</button><button id="variant">__LABEL__</button><span>Physical finger changes do not select a scene.</span></nav>
<iframe id="scene" src="index.html" title="Selected simulation"></iframe>
<script>document.querySelector('#factory').onclick=()=>document.querySelector('#scene').src='http://127.0.0.1:18766/';document.querySelector('#variant').onclick=()=>document.querySelector('#scene').src='index.html';</script>'''.replace('__LABEL__',label)
    (output/'selector.html').write_text(selector)
    # Keep the common monitor honest about which geometry is being displayed.
    view = (output/'index.html').read_text().replace('FR3 · measured-state digital twin','FR3 · '+label)
    if args.fingers == 'wide':
        variant = json.loads((output/'environment.json').read_text())
        width_mm = variant['fingertips']['preview_native_open_width_m']*1000
        cap = variant['fingertips']
        geometry_label = f"{cap['inside_span_m']*1000:g} × {cap['inside_reach_m']*1000:g} mm inside, {cap['wall_thickness_m']*1000:g} mm walls; {cap['top_thickness_m']*1000:.3f} mm fixed-top ceiling"
        if cap.get('pocket_fit'):
            geometry_label += ' · '+cap['pocket_fit']['status']
        floor_gap = cap['geometric_fit']['wall_bottom_above_nominal_flat_layer_floor_when_roof_on_studs_m']
        floor_note = (f'<br>Geometry limit: roof-on-studs contact on a single flat layer would put the inherited walls {-floor_gap*1000:.2f} mm below its base.' if floor_gap<0 else '')
        view = view.replace('Read-only robot telemetry → Isaac PhysX → this WebGL monitor.',
                            f'Read-only robot telemetry → Isaac PhysX → this WebGL monitor.<br>Wide caps: {geometry_label}; simulated {width_mm:.3f} mm native startup width when hand telemetry is unavailable.<br>Original CAD mount retained. Plate contact physics and loaded grasp unvalidated.{floor_note}')
    (output/'index.html').write_text(view)
    with output.with_suffix('.http.log').open('w') as log:
        server = subprocess.Popen(['python3',str(Path(__file__).with_name('serve.py')),
                                   '--directory',str(output),'--upstream',str(upstream),'--port','8767'],
                                  stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    record.update(http_pid=server.pid,http_pid_start=(Path('/proc')/str(server.pid)/'stat').read_text().split()[21],
                  remote_port=8767)
    STATE.write_text(json.dumps(record,indent=2)+'\n')
    deadline = time.monotonic()+5
    while time.monotonic()<deadline:
        if server.poll() is not None:
            stop_owned(record)
            raise RuntimeError('Variant HTTP server exited; inspect '+str(output.with_suffix('.http.log')))
        try:
            with urllib.request.urlopen('http://127.0.0.1:8767/environment.json',timeout=1) as response:
                actual = json.load(response)
            if actual['environment'] == args.environment and actual['fingers'] == args.fingers:
                break
        except OSError:
            pass
        time.sleep(.1)
    else:
        stop_owned(record)
        raise RuntimeError('Variant HTTP readiness check failed')
    print(json.dumps(record))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('environment',choices=('factory','board'))
    parser.add_argument('--fingers',choices=('original','wide'),default='original')
    parser.add_argument('--rebuild',action='store_true')
    launch(parser.parse_args())
