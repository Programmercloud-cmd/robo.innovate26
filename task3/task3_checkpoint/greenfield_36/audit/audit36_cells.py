import gzip,json,math
from pathlib import Path
p=Path('/home/vjhannox/hackathon/robo.innovate-hackathon-TUM/artifacts/greenfield_36');d=json.load(gzip.open(p/'mixed-468422bd7ba526f2.json.gz'));rs={r['id']:r for r in d['records']}
def cells(pt):
 x,y,z=pt[:3];return {(i,k) for i in range(math.floor((x-.245/2-.008-.04)/.08),math.floor((x+.245/2+.008-.04)/.08)+1) for k in range(math.floor((z-.067/2-.006)/.04),math.floor((z+.067/2+.006)/.04)+1)}
examples=[];count=0
for l in d['layers']:
 for row in range(4):
  for c in range(9):
   a,b=[rs[l['ids'][row*10+k]] for k in [c,c+1]]
   ta=next(t for t in a['tracks'] if t['kind']=='direct-row-routing');tb=next(t for t in b['tracks'] if t['kind']=='direct-row-routing')
   overlap=cells(ta['b'])&cells(tb['b'])
   if overlap and abs(ta['t1']-tb['t1'])>.01:
    count+=1
    if len(examples)<3:examples.append({'lane':l['lane'],'station':l['station'],'a':a['id'],'b':b['id'],'a_arrives':ta['t1'],'b_arrives':tb['t1'],'layer_leaves_after':l['t'],'a_position':ta['b'],'b_position':tb['b'],'shared_drive_cells':list(overlap)})
print(json.dumps({'stationary_cell_conflicts':count,'examples':examples},indent=2))
