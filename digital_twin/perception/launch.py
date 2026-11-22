"""Start the independent camera/display feed; never start a robot reader."""
import json
import fcntl
import os
from pathlib import Path
import shlex
import signal
import subprocess
import time


def main():
    here = Path(__file__).resolve().parent
    supervisor_lock = (here/'launch.lock').open('a')
    try:
        fcntl.flock(supervisor_lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:
        raise SystemExit('Perception display is already running; see digital_twin/perception/session.json.')
    root = here.parents[1]
    remote = 'robot-innovate26-place-rl3'
    remote_root = '/home/ubuntu/workspace/fr3-digital-twin-20261026'
    script = f"import json;print(json.load(open('{remote_root}/active_session.json'))['http_root'])"
    remote_output = subprocess.check_output(['ssh','-T','-o','BatchMode=yes',remote,
        'python3 -c '+shlex.quote(script)],text=True,timeout=10).strip()
    forward = ['/usr/bin/python3','-s',str(here/'forward.py'),
        '--input',str(root/'teaching/external_layout/output'),
        '--remote-script',remote_root+'/perception/ingest.py',
        '--remote-output',remote_output,'--layout',str(here/'display_layout.json')]
    capture = ['/usr/bin/python3','-s',str(root/'teaching/external_layout/run.py')]
    camera_device=Path(json.loads((root/'teaching/external_layout/config.json').read_text())['camera']['device'])
    device_was_present=camera_device.exists()
    env = dict(os.environ,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
    children = []
    camera_attempts=0
    camera_started=time.monotonic()
    camera_restart_at=None
    stopping = False
    def stop(*_):
        nonlocal stopping
        stopping = True
    signal.signal(signal.SIGTERM,stop)
    signal.signal(signal.SIGINT,stop)
    try:
        for name,command in [('forward',forward),('camera',capture)]:
            with (here/(name+'.log')).open('a') as log:
                process = subprocess.Popen(command,cwd=root,env=env,stdout=log,stderr=subprocess.STDOUT)
            children.append(process)
        (here/'session.json').write_text(json.dumps(dict(supervisor_pid=os.getpid(),
            forward_pid=children[0].pid,camera_pid=children[1].pid,remote_root=remote_root,
            http_root=remote_output),indent=2)+'\n')
        while not stopping:
            if children[0].poll() is not None:
                raise RuntimeError('Display transport exited; inspect forward.log. Robot services were not changed.')
            camera=children[1]
            device_present=camera_device.exists()
            if device_present and not device_was_present and camera.poll() is not None:
                # A physical reconnect begins one new bounded retry episode.
                camera_attempts=0
                camera_restart_at=time.monotonic()
            device_was_present=device_present
            if camera.poll() is None:
                if time.monotonic()-camera_started>30:
                    camera_attempts=0
                health=dict(state='running',camera_pid=camera.pid,restarts=camera_attempts)
            elif not device_present:
                camera_restart_at=None
                health=dict(state='waiting_for_camera_device',camera_pid=camera.pid,restarts=camera_attempts,returncode=camera.returncode)
            elif camera_attempts>=5:
                # Keep forwarding explicit stale data and measured robot state.
                # A prolonged outage requires a deliberate perception restart.
                health=dict(state='camera_unavailable_retry_budget_exhausted',camera_pid=camera.pid,restarts=camera_attempts,returncode=camera.returncode)
            else:
                if camera_restart_at is None:
                    camera_restart_at=time.monotonic()+min(8.,.5*2**camera_attempts)
                health=dict(state='camera_reconnect_backoff',restarts=camera_attempts,returncode=camera.returncode)
                if time.monotonic()>=camera_restart_at:
                    with (here/'camera.log').open('a') as log:
                        children[1]=subprocess.Popen(capture,cwd=root,env=env,stdout=log,stderr=subprocess.STDOUT)
                    camera_attempts+=1;camera_started=time.monotonic();camera_restart_at=None
                    session=json.loads((here/'session.json').read_text());session['camera_pid']=children[1].pid
                    (here/'session.json').write_text(json.dumps(session,indent=2)+'\n')
            health.update(published_unix_ns=time.time_ns(),device=str(camera_device),device_present=device_present)
            temp=here/'camera_health.tmp';temp.write_text(json.dumps(health));temp.replace(here/'camera_health.json')
            time.sleep(.2)
    finally:
        for child in children:
            if child.poll() is None:
                child.terminate()
        for child in children:
            try:child.wait(timeout=2)
            except subprocess.TimeoutExpired:child.kill();child.wait()


if __name__ == '__main__':
    main()
