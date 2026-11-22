"""Persistent read-only hand subprocess, with bounded restart and owner handoff."""
import json
import math
import os
from pathlib import Path
import selectors
import subprocess
import threading
import time


def hand_fields(background=None,*,owner_sample=None,is_owner=False,now_ns=None):
    now_ns=time.time_ns() if now_ns is None else now_ns
    if is_owner:
        sample=owner_sample or {}
        width=sample.get('width_m');stamp=sample.get('width_unix_ns');source='foreground_sdk_owner'
    else:
        sample=background or {}
        width=sample.get('width');stamp=sample.get('unix_ns');source='persistent_read_only_hand'
    valid=(isinstance(width,(float,int)) and not isinstance(width,bool) and math.isfinite(width)
           and 0<=width<=.09 and type(stamp) is int and 0<=now_ns-stamp<500_000_000)
    return dict(width_m=width if valid else None,width_unix_ns=stamp if valid else None,
                hand_stale=not valid,hand_age_s=(now_ns-stamp)/1e9 if type(stamp) is int else None,
                hand_source=source if valid else 'unavailable')


class HandStream:
    def __init__(self,command,output,pause_file,foreground_busy,*,enabled=True,
                 max_retries=5,retry_initial=.5,retry_max=8.,healthy_s=5.):
        self.command=command;self.output=Path(output);self.pause_file=Path(pause_file)
        self.foreground_busy=foreground_busy;self.enabled=enabled;self.max_retries=max_retries
        self.retry_initial=retry_initial;self.retry_max=retry_max;self.healthy_s=healthy_s
        self.lock=threading.Lock();self.measurement={};self.state='starting' if enabled else 'disabled'
        self.pid=None;self.starts=0;self.failures=0;self.last_error=None

    def update(self,*,state=None,measurement=None,clear=False,**values):
        with self.lock:
            if state is not None:self.state=state
            if clear:self.measurement={}
            if measurement is not None:self.measurement=measurement
            for key,value in values.items():setattr(self,key,value)

    def sample(self):
        with self.lock:return dict(self.measurement)

    def status(self):
        with self.lock:
            fields=hand_fields(self.measurement)
            return dict(hand_reader_state=self.state,hand_reader_pid=self.pid,
                        hand_reader_starts=self.starts,hand_reader_failures=self.failures,
                        hand_reader_error=self.last_error,**fields)

    def run(self,stop):
        if not self.enabled:return
        process=None;selector=selectors.DefaultSelector();buffer=b'';next_start=0.;healthy_since=None;last_data=0.
        def retire():
            nonlocal process
            if process is None:return
            selector.unregister(process.stdout)
            if process.poll() is None:
                process.terminate()
                try:process.wait(timeout=.5)
                except subprocess.TimeoutExpired:process.kill()
            process.wait(timeout=1);process.stdout.close();process=None
            self.update(pid=None)
        with (self.output/'hand_reader_errors.log').open('a',buffering=1) as errors, \
                (self.output/'hand_reader_events.ndjson').open('a',buffering=1) as events:
            def event(kind,**details):events.write(json.dumps(dict(unix_ns=time.time_ns(),event=kind,**details))+'\n')
            try:
                while not stop.is_set():
                    now=time.monotonic();busy=self.foreground_busy();paused=self.pause_file.exists()
                    if busy or paused:
                        if process is not None:retire();event('hand_reader_yielded',reason='paused' if paused else 'foreground_owner')
                        self.update(state='paused' if paused else 'foreground_owner',clear=True)
                        next_start=now;healthy_since=None;stop.wait(.05);continue
                    if self.state=='exhausted':stop.wait(.1);continue
                    failure=None
                    if process is None and now>=next_start:
                        try:
                            process=subprocess.Popen(self.command,stdout=subprocess.PIPE,stderr=errors)
                            selector.register(process.stdout,selectors.EVENT_READ);self.starts+=1
                            self.update(state='starting',pid=process.pid,clear=True)
                            event('hand_reader_started',pid=process.pid,starts=self.starts)
                            buffer=b'';last_data=now;healthy_since=None
                        except OSError as e:failure='launch: '+str(e)
                    if process is not None:
                        if selector.select(timeout=.05):
                            data=os.read(process.stdout.fileno(),65536)
                            if not data:
                                code=process.wait(timeout=.5)
                                if code==75:
                                    retire();self.update(state='foreground_owner',clear=True)
                                    next_start=now+.25;continue
                                failure='reader exit '+str(code)
                            else:
                                buffer+=data
                                if len(buffer)>65536:failure='oversized hand packet'
                                while b'\n' in buffer and failure is None:
                                    line,buffer=buffer.split(b'\n',1)
                                    try:
                                        sample=json.loads(line);measurement=dict(width=sample['state']['width'],unix_ns=sample['unix_ns'])
                                        previous=self.sample()
                                        if hand_fields(measurement)['hand_stale'] or measurement['unix_ns']<=previous.get('unix_ns',0):raise ValueError('invalid/stale hand measurement')
                                        self.update(state='streaming',measurement=measurement,last_error=None)
                                        last_data=time.monotonic()
                                        if healthy_since is None:healthy_since=last_data
                                        if self.failures and last_data-healthy_since>=self.healthy_s:
                                            self.update(failures=0);event('hand_reader_healthy_budget_reset')
                                    except (ValueError,KeyError,TypeError) as e:failure=str(e)
                        if time.monotonic()-last_data>2:failure='hand reader watchdog'
                    else:stop.wait(.05)
                    if failure:
                        retire();self.failures+=1;self.update(clear=True,last_error=failure)
                        if self.failures>self.max_retries:
                            self.update(state='exhausted');event('hand_reader_exhausted',reason=failure)
                        else:
                            delay=min(self.retry_max,self.retry_initial*2**(self.failures-1))
                            self.update(state='backoff');next_start=time.monotonic()+delay
                            event('hand_reader_retry',reason=failure,delay_s=delay)
            finally:
                retire();selector.close()
                self.update(clear=True,state='exhausted' if self.state=='exhausted' else 'stopped')
