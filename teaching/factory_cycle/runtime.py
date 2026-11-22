"""Resident camera and exclusive SDK-owner client for deterministic factory runs."""
import json
import os
from pathlib import Path
import subprocess
import threading
import time

import cv2
from v4l2_capture import Capture

HERE=Path(__file__).resolve().parent


class Stop(RuntimeError):
    pass


def save(path,value):
    path=Path(path);tmp=path.with_suffix(path.suffix+'.tmp')
    tmp.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n');tmp.replace(path)


class Owner:
    def __init__(self,manifest,directory,controller=None):
        self.directory=Path(directory);self.condition=threading.Condition();self.latest=None
        self.events=[];self.states=[];self.dead=False;self.closed=False;self.serial=10
        self.write_lock=threading.Lock();self.error=None
        save(self.directory/'controller_manifest.json',manifest)
        env=os.environ.copy();env['OPENBLAS_NUM_THREADS']='1'
        cpus=manifest['settings']
        command=['taskset','-c',f"{cpus['controller_cpu']},{cpus['worker_cpu']}",
                 str(Path(controller) if controller is not None else HERE/'controller'),str(self.directory/'controller_manifest.json')]
        command=['powerprofilesctl','launch','--profile','performance','--reason',
                 'Franka factory commissioning','--appid','franka-factory']+command
        self.stderr=(self.directory/'controller_stderr.log').open('w')
        self.process=subprocess.Popen(command,stdin=subprocess.PIPE,stdout=subprocess.PIPE,
            stderr=self.stderr,text=True,bufsize=1,env=env)
        self.reader=threading.Thread(target=self._read,daemon=True);self.reader.start()
        self.pinger=threading.Thread(target=self._ping,daemon=True);self.pinger.start()
        try:
            self.wait(lambda:any(e['type']=='ready' for e in self.events),5)
            self.state()
        except Exception:
            self.close();raise

    def _read(self):
        try:
            with (self.directory/'controller.ndjson').open('w',buffering=1) as log:
                for line in self.process.stdout:
                    log.write(line)
                    try:event=json.loads(line)
                    except ValueError:continue
                    event['received_monotonic']=time.monotonic()
                    with self.condition:
                        if event['type']=='state':self.latest=event;self.states.append(event)
                        else:
                            self.events.append(event)
                            if event['type'] in ('fatal','input_failed','cycle_stopped','hand_failed'):
                                self.error=event
                        self.condition.notify_all()
        finally:
            with self.condition:self.dead=True;self.condition.notify_all()

    def _ping(self):
        while not self.closed and not self.dead:
            try:self.send('ping',id=0)
            except (BrokenPipeError,ValueError,OSError):break
            time.sleep(.10)

    def next_id(self):self.serial+=1;return self.serial

    def send(self,op,**args):
        with self.write_lock:
            self.process.stdin.write(json.dumps(dict(op=op,**args),allow_nan=False)+'\n')
            self.process.stdin.flush()

    def wait(self,predicate,timeout,allow_stopped=False):
        until=time.monotonic()+timeout
        with self.condition:
            while not predicate():
                if self.error and not (allow_stopped and self.error['type']=='cycle_stopped'):raise Stop(str(self.error))
                if self.dead:raise Stop('SDK owner exited; inspect controller logs')
                remaining=until-time.monotonic()
                if remaining<=0:raise Stop('SDK owner response timed out')
                self.condition.wait(min(.05,remaining))

    def command(self,op,id=None,**args):
        id=self.next_id() if id is None else id
        mark=len(self.events)
        self.send(op,id=id,**args)
        self.wait(lambda:any(e.get('id')==id and e['type'] in ('ack','rejected') for e in self.events[mark:]),3)
        response=next(e for e in self.events[mark:] if e.get('id')==id and e['type'] in ('ack','rejected'))
        if response['type']=='rejected':raise Stop(response['detail'])
        return id

    def state(self,timeout=.4):
        self.wait(lambda:self.latest is not None and time.monotonic()-self.latest['received_monotonic']<.15,timeout)
        return self.latest

    def begin(self):
        mark=len(self.events);self.command('begin')
        # The last streamed sample may still say control=true from the previous
        # session even after cycle_finished. Wait for this session's start.
        self.wait(lambda:any(e['type']=='cycle_started' for e in self.events[mark:]),2)
        started=next(e for e in self.events[mark:] if e['type']=='cycle_started')
        # FCI refreshes q_d on entry. Do not plan the first cached leg from the
        # preceding Idle sample (or the preceding control session's command).
        self.wait(lambda:self.latest and self.latest['control'] and
                  self.latest['unix_ns']>=started['unix_ns'] and
                  self.latest['state']['robot_mode']=='Move',.5)

    def finish(self):
        before=len(self.events);self.command('finish')
        self.wait(lambda:any(e['type']=='cycle_finished' for e in self.events[before:]),2)
        ended=next(e for e in self.events[before:] if e['type']=='cycle_finished')
        self.wait(lambda:self.latest and self.latest['received_monotonic']>=ended['received_monotonic'] and
            self.latest['state']['robot_mode']=='Idle' and not self.latest['state']['current_errors'] and
            max(map(abs,self.latest['state']['dq']))<.01,.5)

    def stop_and_wait(self):
        mark=len(self.events);self.command('stop')
        self.wait(lambda:any(e['type']=='cycle_stopped' for e in self.events[mark:]),2,allow_stopped=True)
        self.wait(lambda:self.latest and self.latest['state']['robot_mode']=='Idle' and
                  not self.latest['state']['current_errors'] and max(map(abs,self.latest['state']['dq']))<.01,
                  2,allow_stopped=True)
        return self.latest

    def recover_idle(self,require_opening_done=False,opening_request_id=None):
        # A software guard may stop control while leaving the SDK owner alive.
        # Never reconnect or clear a robot Reflex here.
        until=time.monotonic()+1.5
        with self.condition:
            while True:
                s=self.latest
                if self.dead:raise Stop('SDK owner exited during bounded retreat')
                terminal=(not require_opening_done or (s and (s['hand_done']==opening_request_id if opening_request_id is not None else s['hand_done']>=1000000) and any(e.get('id')==s['hand_done'] and e['type'] in ('hand_done','hand_failed') for e in self.events)))
                if s and terminal and not s['control'] and time.monotonic()-s['received_monotonic']<.15 and time.monotonic()-s['hand_stamp']<.25 and s['state']['robot_mode']=='Idle' and not s['state']['current_errors'] and max(map(abs,s['state']['dq']))<.01 and abs(s['hand_width']-.07)<.001:break
                remaining=until-time.monotonic()
                if remaining<=0:raise Stop('Guarded retreat lacks fresh fault-free Idle/open state')
                self.condition.wait(min(.05,remaining))
        self.error=None;self.command('recover_idle')

    def load(self,path):return self.command('load',path=path)

    def go(self,id,duration):
        self.command('go',id=id)
        self.wait(lambda:self.latest and self.latest['last_done']==id,duration+1)
        return self.state()

    def servo(self,q,target):
        id=self.command('servo',q=list(q),target=target)
        self.wait(lambda:self.latest and self.latest['last_done']==id,1.2)
        return self.state()

    def hand(self,kind):
        id=self.command('hand',kind=kind)
        self.last_hand_request_id=id
        self.wait(lambda:self.latest and self.latest['hand_done']==id,2)
        s=self.state()
        if not s['hand_ok']:raise Stop('Hand action did not verify')
        return s

    def close(self):
        if self.closed:return
        try:
            if not self.dead:
                self.send('stop',id=self.next_id())
                time.sleep(.10)
                self.send('shutdown',id=self.next_id())
        except (OSError,ValueError):pass
        self.closed=True
        try:self.process.stdin.close()
        except (OSError,ValueError):pass
        try:self.process.wait(timeout=4)
        except subprocess.TimeoutExpired:
            self.process.send_signal(2)
            try:self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:raise Stop('SDK owner has not stopped; do not connect another client')
        self.reader.join(timeout=1);self.stderr.close()


class PhaseClock:
    """Resident monotonic cycle markers; late work never causes faster motion."""
    def __init__(self,owner,started,config):
        self.owner=owner;self.started=started;self.config=config;self.events=[]

    def at(self,name,offset,pose,width_min,width_max,*,position_tolerance_m=.001):
        if not 0<position_tolerance_m<=.0015:raise Stop('Invalid synchronized pose tolerance')
        deadline=self.started+offset
        ready=time.monotonic()-self.started
        if time.monotonic()>deadline+self.config['maximum_marker_lateness_s']:raise Stop('Missed synchronized '+name+' deadline')
        while True:
            if self.owner.error:raise Stop(str(self.owner.error))
            s=self.owner.state();r=s['state']
            if r['current_errors'] or max(map(abs,r['dq']))>=.01:raise Stop('Synchronized hold is not stationary/fault-free')
            if sum((a-b)**2 for a,b in zip(r['O_T_EE'][12:15],pose[12:15]))**.5>position_tolerance_m:raise Stop('Synchronized hold left its pose tolerance')
            if time.monotonic()-s['hand_stamp']>.25 or not width_min<=s['hand_width']<=width_max:raise Stop('Synchronized hold has uncertain hand telemetry')
            remaining=deadline-time.monotonic()
            if remaining<=0:break
            time.sleep(min(.005,remaining))
        elapsed=time.monotonic()-self.started
        event=dict(marker=name,target_s=offset,ready_s=ready,actual_s=elapsed,lateness_s=elapsed-offset)
        self.events.append(event);return event


class Camera:
    """Continuously drain V4L2; no stop/flush/reopen at individual stages."""
    def __init__(self,config):
        self.config=config;self.cv=threading.Condition();self.latest=None;self.sequence=0
        self.error=None;self.closed=False
        # Set/read back the camera profile before streaming. Reissuing UVC
        # controls after STREAMON intermittently failed despite correct values.
        controls=dict(focus_automatic_continuous=0,focus_absolute=config['focus'],**config.get('controls',{}))
        diagnostic=''
        for attempt in range(3):
            read=subprocess.run(['v4l2-ctl','-d',config['device'],'--get-ctrl',','.join(controls)],capture_output=True,text=True,timeout=2)
            actual={}
            for line in read.stdout.splitlines():
                try:
                    key,value=line.split(':',1);actual[key.strip()]=int(value.strip().split()[0])
                except (ValueError,IndexError):pass
            if read.returncode==0 and all(actual.get(k)==v for k,v in controls.items()):break
            changed=subprocess.run(['v4l2-ctl','-d',config['device'],'--set-ctrl',
                ','.join(f'{key}={value}' for key,value in controls.items())],capture_output=True,text=True,timeout=2)
            diagnostic=changed.stderr or read.stderr
            time.sleep(.05)
        else:raise Stop('Camera profile could not be verified before streaming: '+diagnostic)
        self.cap=Capture(config['device'],config['width'],config['height'])
        self.worker=threading.Thread(target=self._capture,daemon=True);self.worker.start()
        try:self.frame(after=time.monotonic()+.6,timeout=2)
        except Exception:self.close();raise

    def _capture(self):
        try:
            while not self.closed:
                frame=self.cap.read(timeout=2 if self.latest is None else .4)
                if frame is None:continue
                sequence,stamp,image=frame;ended=time.monotonic()
                if image.shape[:2]!=(self.config['height'],self.config['width']):raise Stop('Camera image size changed')
                with self.cv:
                    self.sequence=sequence
                    self.latest=(sequence,stamp,ended,image)
                    self.cv.notify_all()
        except Exception as error:
            with self.cv:self.error=str(error);self.cv.notify_all()

    def frame(self,after=0,sequence=0,timeout=.35,max_age=.12):
        until=time.monotonic()+timeout
        with self.cv:
            while True:
                if self.error:raise Stop(self.error)
                if self.latest:
                    seq,stamp,received,image=self.latest
                    if seq>sequence and stamp>=after and time.monotonic()-stamp<=max_age:
                        return dict(sequence=seq,captured_monotonic=stamp,received_monotonic=received,image=image.copy())
                remaining=until-time.monotonic()
                if remaining<=0:raise Stop('No fresh camera evidence within bounded observation timeout')
                self.cv.wait(min(.03,remaining))

    def close(self):
        self.closed=True;self.worker.join(timeout=1);self.cap.close()
