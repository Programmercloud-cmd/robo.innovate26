"""One external-camera owner publishes display evidence, never robot commands."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import select
import sys
import time

import cv2

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'factory_cycle'))
from v4l2_capture import Capture
from perception import Classifier,compile_layout,load_config,overlay


def atomic(path,data):
    temporary=path.with_name(path.name+'.next')
    temporary.write_bytes(data);os.replace(temporary,path)


def close_capture_safely(capture):
    """Private cleanup: STREAMOFF may fail after USB removal; still free maps/fd."""
    errors=[]
    if capture is None:return errors
    try:capture.close()
    except Exception as error:errors.append(str(error))
    finally:
        for mapping in list(getattr(capture,'maps',[])):
            try:mapping.close()
            except Exception as error:errors.append(str(error))
        capture.maps=[]
        fd=getattr(capture,'fd',-1)
        if fd>=0:
            try:os.close(fd)
            except OSError as error:errors.append(str(error))
        capture.fd=-1;capture.streaming=False
    return errors


def publish_stopped(packet,layout,output,publish_lock,shutdown_errors):
    if packet is None:
        packet={'schema':'fr3.external_layout.v1','camera_serial':'03FB123F','frame_unix_ns':0,
                'layout_hash':layout['layout_hash'],'slots':[{key:s[key] for key in ('slot_id','footprint_id','motion_pose_id')} for s in layout['slots']],
                'conveyor_slots':[{key:s[key] for key in ('slot_id','footprint_id','motion_pose_id')} for s in layout.get('conveyor_slots',[])]}
    packet['stale']=True;packet['producer_stopped_unix_ns']=time.time_ns()
    packet['capture_shutdown_errors']=shutdown_errors
    for key in ('slots','conveyor_slots'):
        packet[key]=[{**s,'status':'uncertain','color_id':None,'confidence':0.,'visible':False,'reason':'producer_stopped'} for s in packet.get(key,[])]
    fcntl.flock(publish_lock,fcntl.LOCK_EX)
    try:atomic(output/'latest.json',(json.dumps(packet,allow_nan=False)+'\n').encode())
    finally:fcntl.flock(publish_lock,fcntl.LOCK_UN)


def update_source_exclusions(previous,seen,document,now_ns,known_ids):
    """Fresh publisher snapshots include explicit controller resets upstream.

    Expired/malformed evidence retains the previous exclusions; it never clears
    departed columns merely because a producer or robot reader disappeared.
    """
    stamp=document.get('observed_unix_ns');listed=document.get('slot_ids')
    fresh=(document.get('schema')=='fr3.source_exclusions.v1' and document.get('stale') is False
           and isinstance(stamp,int) and not isinstance(stamp,bool) and 0<=now_ns-stamp<=2_000_000_000
           and isinstance(listed,list) and all(isinstance(v,str) and v in known_ids for v in listed))
    return (set(listed),True,True) if fresh else (set(previous),seen,False)


def drain_latest_frame(capture,frame,maximum_drains=6):
    """Discard a bounded ready FIFO backlog; retain real kernel frame timing."""
    latest=frame;discarded=0
    for _ in range(maximum_drains):
        if not select.select([capture.fd],[],[],0)[0]:break
        pending=capture.read(0)
        if pending is not None:latest=pending;discarded+=1
    return latest,discarded


def finalize_packet_age(packet,stamp,stale_after_s,monotonic_now=None,unix_now_ns=None):
    now=time.monotonic() if monotonic_now is None else monotonic_now
    age=now-stamp
    packet['frame_age_at_publish_s']=age
    packet['published_unix_ns']=time.time_ns() if unix_now_ns is None else unix_now_ns
    packet['stale']=packet.get('stale',False) or not 0<=age<=stale_after_s
    if packet['stale']:
        for key in ('slots','conveyor_slots'):
            packet[key]=[{**slot,'status':'uncertain','color_id':None,'confidence':0.,'visible':False,'reason':'stale_camera_frame'} for slot in packet.get(key,[])]
    return packet['stale']


def render_frame(image,layout,slots,guard,classifier,sequence,frame_ns):
    drawing=overlay(image,classifier.debug_layout or layout,slots,guard)
    for segment in classifier.last_registration.get('segments',[]):
        import numpy as np
        cv2.polylines(drawing,[np.round(segment['current_body_quad_px']).astype(np.int32)],True,(0,255,0) if segment['full_stack_anchor'] else (0,128,255),2)
    cv2.putText(drawing,classifier.last_registration['status']+' '+classifier.guard_status,(15,685),cv2.FONT_HERSHEY_SIMPLEX,.42,(0,255,255),1)
    cv2.putText(drawing,f'camera03FB123F seq {sequence} frame_unix_ns {frame_ns}',(15,705),cv2.FONT_HERSHEY_SIMPLEX,.42,(0,255,255),1)
    return drawing


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,default=Path(__file__).with_name('config.json'))
    p.add_argument('--output',type=Path,default=Path(__file__).with_name('output'))
    p.add_argument('--frames',type=int,default=0)
    args=p.parse_args()
    config=load_config(args.config);layout=compile_layout(config);args.output.mkdir(parents=True,exist_ok=True)
    reference=cv2.imread(str(args.config.parent/config['reference_image']))
    if reference is None:raise RuntimeError('Missing registered color/top-plane reference image')
    classifier=Classifier(config,reference)
    camera_lock=open('/tmp/external-logitech-03FB123F.lock','a');fcntl.flock(camera_lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    publish_lock=open(args.output/'publish.lock','a')
    atomic(args.output/'layout_static.json',(json.dumps(layout,indent=2)+'\n').encode())
    excluded_ids=set();exclusions_seen=False
    active=True
    def stop(*_):
        nonlocal active
        active=False
    signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
    camera=config['camera'];capture=None;published=0;last_publish=0.;packet=None
    try:
        capture=Capture(camera['device'],camera['width'],camera['height'])
        while active and (not args.frames or published<args.frames):
            frame=capture.read(.4)
            if frame is None:continue
            now=time.monotonic()
            if now-last_publish<1/config['maximum_fps']:continue
            frame,discarded_buffered_frames=drain_latest_frame(capture,frame)
            sequence,stamp,image=frame
            now=time.monotonic();now_ns=time.time_ns()
            last_publish=now
            frame_ns=now_ns-int((now-stamp)*1e9)
            try:guard=json.loads((args.output/'robot_guard.json').read_text())
            except (OSError,ValueError):guard={'stale':True,'robot_moving':True}
            try:exclusions=json.loads((args.output/'source_exclusions.json').read_text())
            except (OSError,ValueError):exclusions={}
            if not isinstance(exclusions,dict):exclusions={}
            excluded_ids,exclusions_seen,exclusions_fresh=update_source_exclusions(excluded_ids,exclusions_seen,exclusions,now_ns,{s['slot_id'] for s in layout['slots']})
            slots=classifier.update(image,layout,frame_ns,guard,now_ns,excluded_ids,exclusions_seen)
            stale=not 0<=now-stamp<=config['stale_after_s']
            if stale:
                slots=[{**s,'status':'uncertain','color_id':None,'confidence':0.,'visible':False,'reason':'stale_camera_frame'} for s in slots]
            source_slots=[s for s in slots if s.get('location_type')!='conveyor'];conveyor_slots=[s for s in slots if s.get('location_type')=='conveyor']
            packet={'schema':'fr3.external_layout.v1','camera_serial':camera['serial'],'frame_sequence':sequence,
                    'frame_unix_ns':frame_ns,'frame_monotonic_ns':int(stamp*1e9),'processed_unix_ns':now_ns,'stale':stale,
                    'discarded_buffered_frames':discarded_buffered_frames,'capture_age_at_processing_s':now-stamp,
                    'layout_hash':config['layout_hash'],'slots':source_slots,'conveyor_slots':conveyor_slots,'display_policy':config.get('display_policy',{}),'empty_observation_policy':'strict_positive_floor_camera_only' if classifier.strict_camera_empty else 'fresh_robot_guard','inventory_authority':False,'execution_authority':False,
                    'registration':classifier.last_registration,'conveyor_refinement':classifier.last_conveyor_refinement,'robot_guard_status':classifier.guard_status,
                    'source_exclusions':{'slot_ids':sorted(excluded_ids),'ever_observed_valid':exclusions_seen,'fresh':exclusions_fresh},
                    'confidence_definition':'heuristic color/coverage evidence score, not calibrated probability',
                    'robot_guard':guard,'color_reference_status':config['color_reference_status']}
            drawing=render_frame(image,layout,slots,guard,classifier,sequence,frame_ns)
            ok,raw=cv2.imencode('.jpg',image);ok2,debug=cv2.imencode('.jpg',drawing)
            if not ok or not ok2:raise RuntimeError('External-camera JPEG encoding failed')
            fcntl.flock(publish_lock,fcntl.LOCK_EX)
            try:
                # Encoding or a delayed reader lock can consume the freshness
                # budget. Expire both evidence collections at publication time.
                if finalize_packet_age(packet,stamp,config['stale_after_s']):
                    expired_slots=packet['slots']+packet['conveyor_slots']
                    drawing=render_frame(image,layout,expired_slots,guard,classifier,sequence,frame_ns)
                    ok2,debug=cv2.imencode('.jpg',drawing)
                    if not ok2:raise RuntimeError('Expired debug JPEG encoding failed')
                    finalize_packet_age(packet,stamp,config['stale_after_s'])
                packet['processing_duration_s']=time.monotonic()-now
                atomic(args.output/'raw.jpg',raw.tobytes());atomic(args.output/'overlay.jpg',debug.tobytes())
                atomic(args.output/'latest.json',(json.dumps(packet,allow_nan=False)+'\n').encode())
            finally:fcntl.flock(publish_lock,fcntl.LOCK_UN)
            published+=1
            if published==1:print(json.dumps({'status':'external_camera_feed_started','pid':os.getpid(),'slots':len(slots),'output':str(args.output)}),flush=True)
    finally:
        shutdown_errors=close_capture_safely(capture)
        try:publish_stopped(packet,layout,args.output,publish_lock,shutdown_errors)
        finally:camera_lock.close();publish_lock.close()


if __name__=='__main__':main()
