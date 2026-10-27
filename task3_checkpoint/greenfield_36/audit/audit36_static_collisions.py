import json,gzip,math,collections,time
from pathlib import Path
boxes=[];grid=collections.defaultdict(set)
for b in json.load(open('/tmp/audit36-static-boxes.json')):
 m=b['m']
 if max(abs(m[i]) for i in [1,2,4,6,8,9])>1e-6:continue
 hx,hy,hz=abs(m[0])/2,abs(m[5])/2,abs(m[10])/2;x,y,z=m[12:15]
 if y+hy<.30 or y-hy>3.3:continue
 if b['color'] in [0x70b0a1,0x70cf99,0xbfc9c8]:continue
 ix=len(boxes);boxes.append((x,y,z,hx,hy,hz,b['color']))
 for xx in range(math.floor(x-hx),math.floor(x+hx)+1):
  for zz in range(math.floor(z-hz),math.floor(z+hz)+1):grid[xx,zz].add(ix)
def pose(s,u):
 if not s.get('linear'):u=u*u*(3-2*u)
 if 'path' in s:
  pts=s['path'];ds=[math.dist(a[:3],b[:3]) for a,b in zip(pts,pts[1:])];at=sum(ds)*u
  for i,d in enumerate(ds):
   if at<=d:return [pts[i][j]+(pts[i+1][j]-pts[i][j])*at/d for j in range(3)]+[s['a'][3]+(s['b'][3]-s['a'][3])*u]
   at-=d
  return pts[-1][:3]+[s['b'][3]]
 return [a+(b-a)*u for a,b in zip(s['a'],s['b'])]
def overlaps(p,b,eps=.005):
 x,y,z,rot=p;bx,by,bz,hx,hy,hz,color=b;dy=abs(y-by)
 if .0345+hy-dy<=eps:return False
 co=math.cos(rot);si=math.sin(rot);c=abs(co);ss=abs(si);dx=x-bx;dz=z-bz
 return min(hx+.1225*c+.0335*ss-abs(dx),hz+.1225*ss+.0335*c-abs(dz),.1225+hx*c+hz*ss-abs(dx*co-dz*si),.0335+hx*ss+hz*c-abs(dx*si+dz*co))>eps
m=json.load(gzip.open('/home/vjhannox/hackathon/robo.innovate-hackathon-TUM/artifacts/greenfield_36/mixed-468422bd7ba526f2.json.gz'));counts=collections.Counter();examples=[];tested=0
for r in m['records']:
 if r['kind']!='brick':continue
 for s in r['tracks']:
  p=pose(s,.5);tested+=1
  key=(math.floor(p[0]),math.floor(p[2]));near=set()
  for xx in range(key[0]-1,key[0]+2):
   for zz in range(key[1]-1,key[1]+2):near.update(grid.get((xx,zz),()))
  for i in near:
   if overlaps(p,boxes[i]):
    counts[s['kind']]+=1
    if len(examples)<30:examples.append({'id':r['id'],'time':(s['t0']+s['t1'])/2,'kind':s['kind'],'pose':p,'box':boxes[i]})
print(json.dumps({'axis_aligned_boxes':len(boxes),'midpoint_tests':tested,'overlaps_over_5mm':counts,'examples':examples},indent=2),flush=True)
