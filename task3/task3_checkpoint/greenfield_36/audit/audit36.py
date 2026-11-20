import gzip,json,hashlib
from pathlib import Path
p=Path('/home/vjhannox/hackathon/robo.innovate-hackathon-TUM');src=(p/'greenfield/cart_model.js').read_text();h=hashlib.sha256(src.split('/* snapshot-loader */')[0].encode()).hexdigest()[:16]
for mode in ['mixed','single']:
 f=p/'artifacts/greenfield_36'/f'{mode}-{h}.json.gz'
 if not f.exists():print(mode,'pending');continue
 m=json.load(gzip.open(f));rs={r['id']:r for r in m['records']};counts=[0,0,0]
 for l in m['layers']:
  counts[l['station']]+=1
  arrivals=[next(t['t1'] for t in rs[i]['tracks'] if t['kind'] in ['direct-row-routing','reserve-row-routing']) for i in l['ids']]
  assert all(arrivals[(r-1)*10+c]<=arrivals[r*10+c]+1e-6 for r in range(1,4) for c in range(10)), 'Arrival order'
  for kind in ['layer-buffer-zone-one','layer-buffer-zone-two']:
   spans=[[(t['t0'],t['t1']) for t in rs[i]['tracks'] if t['kind']==kind] for i in l['ids']]
   assert all(v==spans[0] for v in spans),kind
  if mode=='mixed':
   colors=[rs[i]['shade'] for i in l['ids']]
   assert all(len(set(colors[r*10+c:r*10+c+3]))>1 for r in range(4) for c in range(8))
   assert all(len({colors[r*10+c],colors[(r+1)*10+c],colors[(r+2)*10+c]})>1 for r in range(2) for c in range(10))
 assert min(counts)>0
 assert not any(t['kind']=='receiving-row-release' for r in m['records'] for t in r['tracks'])
 checks=0
 for lane in range(6):
  bs=sorted([r for r in m['records'] if r['kind']=='brick' and r.get('c')==lane],key=lambda r:r['rejectZoneEntry'])
  for a,b in zip(bs,bs[1:]):
   if a.get('ejectionCycle'):assert b['rejectZoneEntry']+1e-6>=a['ejectionCycle']['home'];checks+=1
 print(mode,{'complete':m['completeInput'],'bricks_h':m['bph'],'pallets_h':m['pph'],'layers_by_exit':counts,'safe_reject_cycles':checks})
