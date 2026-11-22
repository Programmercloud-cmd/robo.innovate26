"""Offline saved-frame algorithm replay; not live guard or accuracy validation."""
import argparse,collections,json
from pathlib import Path
import cv2
from perception import Classifier,compile_layout,load_config
p=argparse.ArgumentParser();p.add_argument('samples',type=Path);p.add_argument('--config',type=Path,default=Path(__file__).with_name('config.json'));p.add_argument('--output',type=Path,required=True);a=p.parse_args()
c=load_config(a.config);l=compile_layout(c);m=Classifier(c,cv2.imread(str(a.config.parent/c['reference_image'])));records=[];previous={};flips=collections.Counter()
for f in sorted(a.samples.glob('[0-9][0-9][0-9].json')):
 d=json.loads(f.read_text());stamp=d['frame_unix_ns'];g={**d['robot_guard'],'observed_unix_ns':stamp,'stale':False,'robot_moving':False}
 rows=m.update(cv2.imread(str(f.with_suffix('.jpg'))),l,stamp,g,stamp)
 for x in rows:
  value=(x['status'],x['color_id'])
  if x['slot_id'] in previous and previous[x['slot_id']]!=value:flips[x['slot_id']]+=1
  previous[x['slot_id']]=value
 records.append({'frame_sequence':d['frame_sequence'],'occupied':sum(x['status']=='occupied' for x in rows),'reasons':dict(collections.Counter(x['reason'] for x in rows)),'registration':m.last_registration})
warm=records[5:];out={'scope':'Offline cached actual images; guard timestamp/fault flags rebased only for algorithm isolation, NOT live guard validation or independent color accuracy','frames':records,'total_state_transitions':sum(flips.values()),'slot_transitions':dict(flips),'occupied_after_warmup_min_max':[min(x['occupied'] for x in warm),max(x['occupied'] for x in warm)] if warm else None};a.output.write_text(json.dumps(out,indent=2)+'\n');print(json.dumps({k:v for k,v in out.items() if k not in ('frames','slot_transitions')}))
