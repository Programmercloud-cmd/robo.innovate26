import gzip,json,collections,math,statistics,hashlib
from pathlib import Path
p=Path('/home/vjhannox/hackathon/robo.innovate-hackathon-TUM');h=hashlib.sha256(Path('/tmp/audit36-pinned/cart_model.js').read_text().split('/* snapshot-loader */')[0].encode()).hexdigest()[:16]
for mode in ['mixed','single']:
 d=json.load(gzip.open(p/'artifacts/greenfield_36'/f'{mode}-{h}.json.gz'));r={x['id']:x for x in d['records']};bs=[x for x in d['records'] if x['kind']=='brick'];accepted={b['id'] for b in bs if b['good']};bad={b['id'] for b in bs if not b['good']};out=[i for pal in d['pallets'] for i in pal['ids']];formed=[i for l in d['layers'] for i in l['ids']]
 assert len(bs)==28800 and len(r)==len(d['records'])
 assert len(out)==len(set(out)) and set(out)<=accepted
 assert len(formed)==len(set(formed)) and set(formed)<=accepted
 assert d['rejected']==len(bad) and d['inspected']==len(bs)
 assert math.isclose(d['bph'],len(out)*3600/d['total']) and math.isclose(d['pph'],len(d['pallets'])*3600/d['total'])
 layer_runs=[];across_pallet_runs=[]
 for layer in d['layers']:
  ids=layer['ids'];colors=[r[i]['shade'] for i in ids]
  for row in range(4):
   for c in range(8):
    if len(set(colors[row*10+c:row*10+c+3]))==1:layer_runs.append((layer['t'],layer['lane'],row,c))
  for c in range(10):
   for row in range(2):
    if len({colors[row*10+c],colors[(row+1)*10+c],colors[(row+2)*10+c]})==1:layer_runs.append((layer['t'],layer['lane'],row,c))
 for pal in d['pallets']:
  if mode=='single':assert len({r[i]['shade'] for i in pal['ids']})==1
  colors=[r[i]['shade'] for i in pal['ids']]
  for z in range(40):
   for height in range(12):
    if len({colors[height*40+z],colors[(height+1)*40+z],colors[(height+2)*40+z]})==1:across_pallet_runs.append((pal['t'],pal['lane'],height,z,colors[height*40+z]))
 kinds=collections.Counter(t['kind'] for b in bs for t in b['tracks'])
 print(json.dumps({'mode':mode,'total':d['total'],'accepted':len(accepted),'rejected':len(bad),'output':len(out),'pallets':len(d['pallets']),'formed':len(formed),'remaining_approved':len(accepted-set(out)),'remaining_in_layers':len(set(formed)-set(out)),'remaining_before_completed_layers':len(accepted-set(formed)),'layer_runs':len(layer_runs),'pallet_height_runs':len(across_pallet_runs),'pallet_height_run_examples':across_pallet_runs[:3],'reserved_stock_tracks':kinds['horizontal-colour-reserve'],'empty_wood_records':sum(x['kind']=='pallet' for x in d['records'])}))
