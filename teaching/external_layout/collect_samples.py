"""Record coherent resident feed snapshots, never open a camera or robot."""
import argparse
import collections
import fcntl
import json
from pathlib import Path
import time


def collect(source,output,count=60,timeout_s=20):
    output.mkdir(parents=True,exist_ok=False)
    found=[];seen=set();deadline=time.monotonic()+timeout_s
    lock=open(source/'publish.lock','r')
    while len(found)<count and time.monotonic()<deadline:
        fcntl.flock(lock,fcntl.LOCK_SH)
        try:
            packet=json.loads((source/'latest.json').read_text())
            key=packet['frame_unix_ns'];age=(time.time_ns()-key)*1e-9
            raw=(source/'raw.jpg').read_bytes() if key not in seen and 0<=age<.5 else None
        finally:fcntl.flock(lock,fcntl.LOCK_UN)
        if raw is not None:
            i=len(found);seen.add(key)
            (output/f'{i:03d}.jpg').write_bytes(raw)
            (output/f'{i:03d}.json').write_text(json.dumps(packet)+'\n')
            found.append(packet)
        else:time.sleep(.02)
    lock.close()
    reasons=collections.Counter(s['reason'] for p in found for s in p['slots'])
    totals={key:[sum(s['status']==key for s in p['slots']) for p in found] for key in ('occupied','empty','uncertain')}
    flips={}
    if found:
        for identity in [s['slot_id'] for s in found[0]['slots']]:
            sequence=[next((s['status'],s['color_id']) for s in p['slots'] if s['slot_id']==identity) for p in found]
            flips[identity]=sum(a!=b for a,b in zip(sequence,sequence[1:]))
    report={'samples':len(found),'requested':count,'elapsed_camera_s':(found[-1]['frame_unix_ns']-found[0]['frame_unix_ns'])*1e-9 if len(found)>1 else 0,
            'state_counts_per_frame':totals,'reason_counts':dict(reasons),'slot_state_flips':flips,
            'complete':len(found)==count,'scope':'Resident externalcamera display snapshots; no independent color/occupancy labels'}
    (output/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source',type=Path,default=Path(__file__).with_name('output'))
    p.add_argument('--output',type=Path,required=True);p.add_argument('--count',type=int,default=60);p.add_argument('--timeout',type=float,default=20)
    args=p.parse_args();report=collect(args.source,args.output,args.count,args.timeout);print(json.dumps({k:report[k] for k in ('samples','elapsed_camera_s','complete')}))
    raise SystemExit(0 if report['complete'] else 2)
