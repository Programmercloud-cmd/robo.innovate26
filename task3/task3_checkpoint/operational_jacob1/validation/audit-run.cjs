/* Independent ledger, product, temporal-resource and support audit.
 * Run on the dedicated VM or through default Pueue for a full experiment.
 * This does not qualify contact mechanics, industrial drives or truck paths.
 */
'use strict';
const fs=require('fs'),path=require('path'),crypto=require('crypto');
const EPS=1e-6;
function materialPose(r,t){
 let lo=0,hi=r.tracks.length;while(lo<hi){const m=(lo+hi)>>1;if(r.tracks[m].t0<=t)lo=m+1;else hi=m;}
 if(!lo)return r.initial.slice();const s=r.tracks[lo-1],u=Math.max(0,Math.min(1,(t-s.t0)/(s.t1-s.t0)));
 if(s.rigid&&s.rigid.from&&s.rigid.to&&s.rigid.local){const q=s.rigid,y=q.from[3]+(q.to[3]-q.from[3])*u,c=Math.cos(y),si=Math.sin(y),v=q.local;return[q.from[0]+(q.to[0]-q.from[0])*u+c*v[0]+si*v[2],q.from[1]+(q.to[1]-q.from[1])*u+v[1],q.from[2]+(q.to[2]-q.from[2])*u-si*v[0]+c*v[2],y+(q.yaw||0)];}
 if(s.path){let d=0;for(let i=1;i<s.path.length;i++)d+=Math.hypot(...s.path[i].slice(0,3).map((x,j)=>x-s.path[i-1][j]));let remaining=d*u;for(let i=1;i<s.path.length;i++){const a=s.path[i-1],b=s.path[i],len=Math.hypot(...b.slice(0,3).map((x,j)=>x-a[j]));if(remaining<=len||i===s.path.length-1){const f=len?Math.max(0,Math.min(1,remaining/len)):0;return[a[0]+(b[0]-a[0])*f,a[1]+(b[1]-a[1])*f,a[2]+(b[2]-a[2])*f,(s.a[3]||0)+((s.b[3]||0)-(s.a[3]||0))*u];}remaining-=len;}}
 return s.a.map((v,j)=>v+(s.b[j]-v)*u);
}
function audit(run,g){
 const checks=[],failureLimit=100;
 function check(name,pass,detail){if(checks.length<5000)checks.push({name,passed:!!pass,...(detail===undefined?{}:{detail})});}
 const records=Array.isArray(run.records)?run.records:[...run.records.values()];
 const byId=new Map(records.map(r=>[String(r.id),r]));
 const bricks=records.filter(r=>r.kind==='brick'),brickIds=new Set(bricks.map(r=>String(r.id)));
 const layers=run.layers||[],pallets=run.pallets||[],zones=run.zones||[];
 check('all record IDs are unique',byId.size===records.length,{records:records.length,unique:byId.size});
 check('finite input material exists',bricks.length>0,{bricks:bricks.length});
 check('all eighteen production stations are exercised',new Set(layers.map(l=>`${l.lane}:${l.station}`)).size===18);
 check('all six lanes are exercised',new Set(layers.map(l=>l.lane)).size===6);
 const layerMap=new Map(layers.map(l=>[String(l.id),l]));
 check('layer IDs are unique',layerMap.size===layers.length);
 let errors=[];
 const add=(type,detail)=>{if(errors.length<failureLimit)errors.push({type,...detail});};
 const usedLayerIds=new Set();
 for(const l of layers){
  const ids=(l.ids||[]).map(String);
  if(ids.length!==40||new Set(ids).size!==40)add('invalid layer material count',{layer:l.id,count:ids.length});
  for(const id of ids){if(!brickIds.has(id))add('unknown layer material',{layer:l.id,id});else if(byId.get(id).good===false)add('reject in layer',{layer:l.id,id});if(usedLayerIds.has(id))add('brick assigned to multiple layers',{layer:l.id,id});usedLayerIds.add(id);}
  const times=['formedAt','bufferAAt','bufferBAt','pickedAt','stackedAt'].map(k=>l[k]);
  if(times.some(t=>!Number.isFinite(t))||times.some((t,i)=>i&&t+EPS<times[i-1]))add('layer event order',{layer:l.id,times});
 }
 check('layers contain forty unique good bricks and ordered process times',errors.length===0,errors);errors=[];
 const packBricks=new Set();
 for(const p of pallets){
  const ids=(p.ids||[]).map(String),ls=(p.layers||[]).map(l=>typeof l==='object'?l:layerMap.get(String(l)));
  if(ids.length!==560||new Set(ids).size!==560)add('invalid pallet count',{pallet:p.id,count:ids.length});
  if(ls.length!==14||ls.some(l=>!l))add('invalid pallet layers',{pallet:p.id,count:ls.length});
  const flat=ls.filter(Boolean).flatMap(l=>l.ids.map(String));
  if(flat.length!==ids.length||flat.some((id,k)=>id!==ids[k]))add('pallet layer concatenation/order differs',{pallet:p.id});
  if(ls.some((l,k)=>l&&l.number!==k))add('pallet layer numbering not FIFO 0..13',{pallet:p.id,numbers:ls.map(l=>l?.number)});
  if(ls.some((l,k)=>k&&l&&ls[k-1]&&l.stackedAt+EPS<ls[k-1].stackedAt))add('pallet vertical layer order',{pallet:p.id});
  for(const id of ids){if(packBricks.has(id))add('brick in multiple pallets',{pallet:p.id,id});packBricks.add(id);if(!brickIds.has(id)||byId.get(id).good===false)add('bad pallet material',{pallet:p.id,id});}
  if(!Number.isFinite(p.completedAt)||!Number.isFinite(p.outfeedAt)||p.outfeedAt+EPS<p.completedAt||ls.some(l=>l&&l.stackedAt>p.completedAt+EPS))add('pallet completion/outfeed event order',{pallet:p.id});
  if(Number.isFinite(p.storedAt)&&p.storedAt+EPS<p.outfeedAt)add('storage predates outfeed',{pallet:p.id});
  if(p.slotId&&!g.storage.slots.some(s=>s.id===p.slotId))add('pallet uses noncanonical storage slot',{pallet:p.id,slot:p.slotId});
  const shades=ids.map(id=>byId.get(id)?.shade);
  const pure=p.recipe==='single'||typeof p.recipe==='number'||p.shade!==undefined&&p.shade!==null;
  if(pure&&new Set(shades).size>1)add('single-colour purity violation',{pallet:p.id});
  if(!pure&&ids.length===560){for(let z=0;z<14;z++)for(let r=0;r<4;r++)for(let c=0;c<10;c++){
   const i=z*40+r*10+c,s=shades[i];
   for(const [ok,di] of [[c<8,1],[r<2,10],[z<12,40]])if(ok&&s===shades[i+di]&&s===shades[i+2*di])add('mixed no-three-consecutive violation',{pallet:p.id,layer:z,row:r,column:c,stride:di});
  }}
 }
 check('completed pallets are fourteen FIFO layers / 560 unique good bricks with valid recipes',errors.length===0,errors);errors=[];
 for(const p of pallets){const wood=byId.get(String(p.wood??p.id));if(!wood||!Number.isFinite(p.completedAt)){add('missing pallet support pose',{pallet:p.id});continue;}const wp=materialPose(wood,p.completedAt),expectedOrigin=.224;if(Math.abs(wp[1]-expectedOrigin)>1e-5)add('descending well final pallet datum',{pallet:p.id,y:wp[1],expected:expectedOrigin});for(let k=0;k<p.ids.length;k++){const r=byId.get(String(p.ids[k]));if(!r)continue;const pos=materialPose(r,p.completedAt),layer=Math.floor(k/40),expected=wp[1]+.060+layer*.069+.0345;if(Math.abs(pos[1]-expected)>1e-5){add('brick layer not in contact with pallet or previous layer',{pallet:p.id,brick:r.id,layer,y:pos[1],expected});break;}}}
 check('completed pallet layers contact the indexed pallet deck and each other',errors.length===0,errors);errors=[];
 for(const p of pallets){const wood=byId.get(String(p.wood??p.id));if(!wood)continue;for(const [name,t]of [['storage',p.storedAt],['handoff',p.handoffAt]]){if(!Number.isFinite(t))continue;const wp=materialPose(wood,t);if(Math.abs(wp[1]-.060)>1e-5)add('ground pallet runner bottom is not on floor',{pallet:p.id,state:name,woodY:wp[1],runnerBottomY:wp[1]-.060});for(let k=0;k<p.ids.length;k++){const r=byId.get(String(p.ids[k]));if(!r)continue;const bp=materialPose(r,t),expected=wp[1]+.060+Math.floor(k/40)*.069+.0345;if(Math.abs(bp[1]-expected)>1e-5){add('ground pallet layer deck contact lost',{pallet:p.id,state:name,brick:r.id,y:bp[1],expected});break;}}}}
 check('storage and handoff runners contact the floor with all layers supported',errors.length===0,errors);errors=[];
 for(const l of layers){for(const [time,center]of [[l.bufferAAt,36.835],[l.bufferBAt,g.formation.layer_pickup_x_m]]){if(!Number.isFinite(time))continue;const ps=l.ids.map(id=>byId.get(String(id))).filter(Boolean).map(r=>materialPose(r,time)),mx=ps.reduce((s,p)=>s+p[0],0)/(ps.length||1);if(Math.abs(mx-center)>1e-4||ps.some(p=>p[0]-.1225<g.formation.two_layer_bed_x_min_m-EPS||p[0]+.1225>g.formation.two_layer_bed_x_max_m+EPS))add('layer buffer pose outside physical roller support',{layer:l.id,time,center:mx,expected:center});}}
 check('both layer positions are actually reached on the physical buffer bed',errors.length===0,errors);errors=[];
 const loaded=pallets.filter(p=>Number.isFinite(p.loadedAt));
 for(const incoming of loaded){const t=incoming.loadedAt+EPS,active=loaded.filter(p=>p.loadedAt<=t&&(p.shippedAt===undefined||p.shippedAt>t)),boxes=active.map(p=>({id:p.id,pos:materialPose(byId.get(String(p.wood??p.id)),t)}));for(let i=0;i<boxes.length;i++)for(let j=i+1;j<boxes.length;j++){const a=boxes[i],b=boxes[j];if(Math.abs(a.pos[0]-b.pos[0])<1.18-EPS&&Math.abs(a.pos[2]-b.pos[2])<.86-EPS&&Math.abs(a.pos[1]-b.pos[1])<.966-EPS)add('occupied truck pallet locations overlap',{time:t,first:a.id,second:b.id,positions:[a.pos,b.pos]});}}
 check('loaded truck pallets never overwrite occupied positions',errors.length===0,errors);errors=[];
 let chargedForkSegments=0;
 for(const p of pallets){const wood=byId.get(String(p.wood??p.id));if(!wood)continue;for(const s of wood.tracks||[]){const tool=byId.get(String(s.carrier));if(!tool||tool.kind!=='forklift')continue;chargedForkSegments++;const offset=tool.loadCenterOffsetX;if(!Number.isFinite(offset)){add('fork carrier lacks explicit load-center frame',{carrier:tool.id});continue;}for(const fraction of [0,.5,1]){const t=s.t0+(s.t1-s.t0)*fraction,wp=materialPose(wood,t),tp=materialPose(tool,t),wantX=tp[0]+Math.cos(tp[3]||0)*offset,wantZ=tp[2]-Math.sin(tp[3]||0)*offset;if(Math.abs(wp[0]-wantX)>1e-4||Math.abs(wp[2]-wantZ)>1e-4)add('pallet leaves the charged fork load frame',{pallet:p.id,carrier:tool.id,kind:s.kind,t,palletPose:wp,vehiclePose:tp,expectedLoadCenter:[wantX,wantZ]});}}}
 check('loaded forklift transports and insertions preserve the signed load-center frame',chargedForkSegments>0&&errors.length===0,{chargedForkSegments,errors});errors=[];
 const byZone=new Map();
 for(const z of zones){const a=z.start??z.t0,b=z.end??z.t1;if(!Number.isFinite(a)||!Number.isFinite(b)||b<a-EPS){add('invalid zone interval',{zone:z});continue;}const id=String(z.id??z.zone);if(!byZone.has(id))byZone.set(id,[]);byZone.get(id).push({...z,start:a,end:b});}
 for(const [id,rows] of byZone){rows.sort((a,b)=>a.start-b.start||a.end-b.end);let previous=null;for(const z of rows){if(previous&&z.start<previous.end-EPS&&z.owner!==previous.owner)add('occupied resource overlap',{zone:id,first:previous,second:z});if(!previous||z.end>previous.end)previous=z;}}
 check('occupied resource calendars have no conflicting owners',errors.length===0,{zoneCount:byZone.size,intervals:zones.length,errors});errors=[];
 check('occupied-zone evidence is not empty',zones.length>0&&byZone.size>0,{zoneCount:byZone.size,intervals:zones.length});
 const stationLayers=new Map();for(const l of layers){const k=`${l.lane}:${l.station}`;if(!stationLayers.has(k))stationLayers.set(k,[]);stationLayers.get(k).push(l);}
 function occupancy(rows,start,end){const points=[];for(const l of rows){if(Number.isFinite(l[start])&&Number.isFinite(l[end])&&l[end]>l[start]+EPS){points.push([l[start],1,l.id]);points.push([l[end],-1,l.id]);}}points.sort((a,b)=>a[0]-b[0]||a[1]-b[1]);let n=0,max=0;for(const p of points){n+=p[1];max=Math.max(max,n);}return max;}
 for(const [station,ls] of stationLayers){if(occupancy(ls,'bufferAAt','bufferBAt')>1)add('buffer A over capacity',{station});if(occupancy(ls,'bufferBAt','pickedAt')>1)add('buffer B over capacity',{station});const a=[...ls].sort((x,y)=>x.bufferAAt-y.bufferAAt),b=[...ls].sort((x,y)=>x.bufferBAt-y.bufferBAt),c=[...ls].sort((x,y)=>x.pickedAt-y.pickedAt);if(a.some((x,k)=>x.id!==b[k]?.id||x.id!==c[k]?.id))add('layer buffer FIFO violation',{station});}
 check('two-position buffer occupancy and FIFO reconstructed independently',errors.length===0,errors);errors=[];
 const cross=g.routing.upstream_exchange,field=g.routing.local_fanout;
 const inside=(p,r)=>p[0]>=r[0]-EPS&&p[0]<=r[1]+EPS&&p[2]>=r[2]-EPS&&p[2]<=r[3]+EPS;
 const deckRects=[[cross.x_min_m,cross.x_max_m,-15,15],...g.routing.lane_z_m.map(z=>[field.x_min_m,g.formation.alignment_field_x_max_m,z-2,z+2])];
 const pointOnRouteSupport=p=>deckRects.some(r=>inside(p,r));
 let movingTracks=0,routingTracks=0,crossTransfers=0,unsupported=0,capHits=0;
 for(const r of records){let prior=null;for(const s of r.tracks||[]){
  if(!Number.isFinite(s.t0)||!Number.isFinite(s.t1)||s.t1<=s.t0)add('invalid track interval',{record:r.id,kind:s.kind,t0:s.t0,t1:s.t1});
  if(prior&&s.t0<prior.t1-EPS)add('overlapping tracks on same record',{record:r.id,kind:s.kind});
  if(prior&&s.a&&prior.b&&s.a.some((v,k)=>Math.abs(v-prior.b[k])>1e-5))add('discontinuous material trajectory',{record:r.id,kind:s.kind});
  prior=s;movingTracks++;
  if(r.kind!=='brick'||r.good===false||!/rout|cross.transfer/i.test(s.kind||''))continue;
  routingTracks++;const points=s.path||[s.a.slice(0,3),s.b.slice(0,3)];
  if(points.some(p=>!p||p.length<3)){add('invalid route points',{id:r.id});continue;}
  for(let k=1;k<points.length;k++){const a=points[k-1],b=points[k];if(Math.abs(b[2]-a[2])>2.5){crossTransfers++;if(Math.abs(a[0]-cross.cross_transfer_x_m)>1e-4||Math.abs(b[0]-cross.cross_transfer_x_m)>1e-4)add('cross-module transfer outside supported bridge datum',{record:r.id,a,b});}
   const n=Math.max(1,Math.ceil(Math.hypot(b[0]-a[0],b[2]-a[2])/.08));for(let j=0;j<=n;j++){const u=j/n,p=a.map((v,i)=>v+(b[i]-v)*u);if(p[0]<cross.x_min_m-.123||p[0]>g.formation.alignment_field_x_max_m+.123)continue;const yaw=(s.a?.[3]||0)+((s.b?.[3]||0)-(s.a?.[3]||0))*u,hx=Math.abs(Math.cos(yaw))*.1225+Math.abs(Math.sin(yaw))*.0335,hz=Math.abs(Math.sin(yaw))*.1225+Math.abs(Math.cos(yaw))*.0335;
    // At outer entrance/exit, the continuous inspection/forming support extends
    // beyond the transfer surfaces; only interior route corners are checked.
    if(p[0]-hx<cross.x_min_m-EPS||p[0]+hx>g.formation.alignment_field_x_max_m+EPS)continue;
    for(const ch of g.reject_system.flat_bottom_channel.channels){const w=ch.upstream_retaining_wall;if(p[0]+hx>w.x_min_m+EPS&&p[0]-hx<w.x_max_m-EPS&&p[2]+hz>w.z_min_m+EPS&&p[2]-hz<w.z_max_m-EPS&&p[1]+.0345>w.y_min_m+EPS&&p[1]-.0345<w.y_max_m-EPS){capHits++;if(capHits<10)add('routing footprint intersects upstream retaining wall',{record:r.id,point:p});}}
    for(const dx of [-hx,hx])for(const dz of [-hz,hz])if(!pointOnRouteSupport([p[0]+dx,p[1],p[2]+dz])){unsupported++;if(unsupported<10)add('routing footprint over unsupported channel',{record:r.id,p:[p[0]+dx,p[1],p[2]+dz],kind:s.kind});}
   }
  }
 }}
 check('material tracks are finite, continuous and non-overlapping',!errors.some(x=>/track|trajectory/.test(x.type)),errors.filter(x=>/track|trajectory/.test(x.type)));
 check('routing swept footprint uses deck support and upstream bridge',unsupported===0&&capHits===0&&!errors.some(x=>/route|bridge|routing/.test(x.type)),{routingTracks,crossTransfers,unsupported,retainingWallIntersections:capHits,errors:errors.filter(x=>/route|bridge|routing/.test(x.type))});errors=[];
 const transactions=run.transactions||run.ledger||[];
 check('material ledger transactions are available',transactions.length>0,{transactions:transactions.length});
 const state=new Map(bricks.map(r=>[String(r.id),{state:r.initialState||'source',owner:null}]));
 let ledgerErrors=[],lastTime=-Infinity;
 for(const tx of transactions){const t=tx.t??tx.time,ids=(tx.ids||[]).map(String);if(t<lastTime-EPS)ledgerErrors.push({type:'transaction time order',t,previous:lastTime});lastTime=t;if(!ids.length)continue;if(new Set(ids).size!==ids.length)ledgerErrors.push({type:'duplicate material within transaction',t});for(const id of ids){const old=state.get(id);if(!old){if(ledgerErrors.length<failureLimit)ledgerErrors.push({type:'unknown material transaction',id,t});continue;}if(tx.from&&old.state!==tx.from){if(ledgerErrors.length<failureLimit)ledgerErrors.push({type:'material ownership mismatch',id,t,expected:old.state,from:tx.from});}if(!tx.to){if(ledgerErrors.length<failureLimit)ledgerErrors.push({type:'transaction without destination',id,t});continue;}state.set(id,{state:tx.to,owner:tx.owner??tx.toOwner??null});}}
 const finalLedger={};for(const [id,l] of state){finalLedger[l.state]=(finalLedger[l.state]||0)+1;if(/reject|skip|removed/.test(l.state)&&byId.get(id).good!==false){if(ledgerErrors.length<failureLimit)ledgerErrors.push({type:'good brick in reject ledger',id,state:l.state});}}
 check('all material ownership transitions conserve unique identities',ledgerErrors.length===0&&state.size===bricks.length,{errors:ledgerErrors.slice(0,failureLimit),finalLedger,total:state.size});
 const events=run.events||[];
 check('representative layer-transfer events exist',events.some(e=>/layer.*transfer|buffer/i.test(e.type)));
 check('representative completed-pallet events exist',events.some(e=>/pallet.*complete|complete.*pallet/i.test(e.type)));
 const exchange=events.filter(e=>/skip.*exchang|exchang.*skip/i.test(e.type));
 check('skip exchange exercised',exchange.length>0,{events:exchange.map(e=>({t:e.t,type:e.type}))});
 const batches=run.skipBatches||[],history=run.skipHistory||[],latest=(rows,t)=>{let found=null;for(const r of rows){if(r.t<=t)found=r;else break;}return found;};
 for(const b of batches){const hs=latest(history,b.headStowedAt+EPS),resume=latest(history,b.resume+EPS);if(!hs||hs.upstreamStopped!==true||hs.headLift<.5-EPS||Math.abs(hs.headYaw+Math.PI/2)>EPS)add('skip exchanged without raised west-stowed head',{batch:b.batch,head:hs});if(!resume||resume.upstreamStopped||Math.abs(resume.headLift||0)>EPS||Math.abs(resume.headYaw||0)>EPS)add('skip resumes before head returned/lowered',{batch:b.batch,head:resume});
  for(const tx of transactions)if(tx.to==='inspection'&&tx.t>b.stop+EPS&&tx.t<b.resume-EPS)add('new inspection admission during skip exchange stop',{batch:b.batch,t:tx.t});
  for(const r of bricks){const s=latest(r.states||[],b.drained+EPS);if(s?.state==='reject_transit')add('reject remains on belts at cleared confirmation',{batch:b.batch,brick:r.id});}
  const ids=(b.ids||[]).map(String);if(new Set(ids).size!==ids.length||ids.some(id=>byId.get(id)?.good!==false))add('invalid removed skip material set',{batch:b.batch});for(const id of ids){const s=latest(byId.get(id)?.states||[],b.removedAt+EPS);if(s?.state!=='removed')add('removed skip material lost from ledger',{batch:b.batch,brick:id,state:s?.state});}
 }
 check('skip exchange drains collection, stops admissions, proves stow and retains removed material',batches.length>0&&errors.length===0,{batches:batches.length,errors});errors=[];
 check('original replay is not a run input',run.replayLoaded!==true&&run.sourceReplay!==true);
 if(g.upstream_kiln_cart){
  const carts=records.filter(r=>/input.?cart|kiln.?cart/i.test(r.kind||'')||/^input-cart/.test(String(r.id)));
  let cartFaults=[];
  for(const r of carts)for(const s of r.tracks||[]){if(Math.abs(s.a[0]-s.b[0])>EPS)cartFaults.push({id:r.id,kind:s.kind,reason:'kiln cart changes longitudinal X rather than moving on its north/south track'});if(Math.abs((s.a[3]||0)-(s.b[3]||0))>EPS)cartFaults.push({id:r.id,kind:s.kind,reason:'kiln cart rotates while on its fixed track'});}
  check('north/south kiln-cart tracks retain fixed X and yaw',carts.length>0&&cartFaults.length===0,{carts:carts.length,errors:cartFaults});
  const u=g.upstream_kiln_cart,cart=carts[0],stock=bricks.filter(r=>r.source?.cart===0),expected=u.cart.bricks_per_cart;
  const supportWidth=u.cart.effective_support_width_x_m||u.cart.width_x_m;
  for(const r of stock){const p=r.initial;if(Math.abs(p[0]-u.track.center_x_m)+.1225>supportWidth/2+EPS||Math.abs(p[2]-u.track.north_loaded_entry_z_m)+.0335>u.cart.length_z_m/2+EPS)add('initial cart brick outside support tray',{id:r.id,position:p});}
  check('finite source cart has all original IDs on its support tray',stock.length===expected&&errors.length===0,{expected,actual:stock.length,errors});errors=[];
  const picks=events.filter(e=>e.type==='source-layer-pick').sort((a,b)=>a.t-b.t),stackLevels=new Map(),pickedIds=new Set();
  for(const e of picks){const ids=(e.ids||[]).map(String),rs=ids.map(id=>byId.get(id)),c=e.sourceStack?.c,j=e.sourceStack?.j,layer=e.sourceStack?.layer,key=`${c}:${j}`,next=stackLevels.get(key)??19;
   if(ids.length!==40||new Set(ids).size!==40||layer!==next||rs.some(r=>!r||r.source?.c!==c||r.source?.j!==j||r.source?.layer!==layer))add('source pick is not one complete topmost layer',{event:e.id,source:e.sourceStack,expectedLayer:next});
   stackLevels.set(key,next-1);for(const id of ids){if(pickedIds.has(id))add('source brick picked twice',{id,event:e.id});pickedIds.add(id);}
   const motions=rs.filter(Boolean).map(r=>({r,s:r.tracks.find(s=>s.kind==='source-layer-rotate-transfer')}));
   if(motions.length!==40||motions.some(v=>!v.s||!v.s.rigid||v.s.rigid.from[3]!==Math.PI||Math.abs(v.s.rigid.to[3]-Math.PI/2)>EPS))add('loaded source layer lacks rigid ninety-degree rotation',{event:e.id});
   for(const {r,s}of motions){if(!s?.rigid)continue;const q=s.rigid,a=q.from,b=q.to,co=Math.cos(a[3]),si=Math.sin(a[3]),dx=s.a[0]-a[0],dz=s.a[2]-a[2],local=[co*dx-si*dz,s.a[1]-a[1],si*dx+co*dz];if(local.some((v,i)=>Math.abs(v-q.local[i])>1e-5))add('source local grip frame inconsistent',{brick:r.id,event:e.id});
    for(const fraction of [.25,.5,.75]){const t=s.t0+(s.t1-s.t0)*fraction,pose=materialPose(r,t),yaw=a[3]+(b[3]-a[3])*fraction,c=Math.cos(yaw),sn=Math.sin(yaw),want=[a[0]+(b[0]-a[0])*fraction+c*local[0]+sn*local[2],a[1]+(b[1]-a[1])*fraction+local[1],a[2]+(b[2]-a[2])*fraction-sn*local[0]+c*local[2]];if(want.some((v,k)=>Math.abs(v-pose[k])>1e-5))add('source layer deforms during rotation',{brick:r.id,time:t});}
   }
  }
  check('source picks top-down forty-brick layers and rotates them rigidly ninety degrees',picks.length>0&&errors.length===0,{sourcePicks:picks.length,pickedBrickIds:pickedIds.size,errors});errors=[];
  const exits=cart?.tracks.filter(s=>/empty.*exit|empty.*depart/i.test(s.kind))||[];
  if(pickedIds.size<expected&&exits.length)add('partial kiln cart takes empty exit',{picked:pickedIds.size,total:expected});
  if(pickedIds.size<expected&&cart){const end=materialPose(cart,run.duration);if(Math.abs(end[2]-u.track.loading_station_z_m)>EPS)add('partial kiln cart not held at unloading station',{pose:end});}
  for(const tx of transactions.filter(t=>t.to==='inspection'))for(const id of tx.ids){const r=byId.get(String(id)),feed=r?.tracks.find(s=>s.kind==='source-supported-singulation-turn');if(!feed||feed.t1>tx.t+EPS)add('inspection precedes actual source delivery',{id,t:tx.t,sourceReady:feed?.t1});}
  check('partial cart is retained and source delivery causally precedes inspection',errors.length===0,errors);errors=[];
 }
 return {method:'Independent reconstruction of material IDs, layer grouping/recipes, resource intervals, trajectories and ledger; not engine self-validation',passed:checks.every(x=>x.passed),checkCount:checks.length,summary:{records:records.length,bricks:bricks.length,layers:layers.length,pallets:pallets.length,resourceZones:byZone.size,movingTracks,routingTracks,crossTransfers,ledger:finalLedger},checks,limitations:['Logical interlocks and authored support geometry only; not industrial drive/structural qualification.','Finite sample sweeps do not certify continuous rigid-body collision/contact or friction.','Hooklift vehicle selection, swept paths, payload, impact and dust handling remain hardware assumptions.']};
}
module.exports={audit};
if(require.main===module){
 const args=process.argv.slice(2),get=(k,f)=>{const i=args.indexOf(k);return i>=0?args[i+1]:f;};
 const raw=fs.readFileSync(get('--manifest',path.resolve(__dirname,'../geometry-manifest.json')),'utf8'),manifest=JSON.parse(raw),manifestBefore=JSON.stringify(manifest),manifestHash=crypto.createHash('sha256').update(raw).digest('hex');
 let run;const input=get('--run');
 if(input)run=JSON.parse(fs.readFileSync(input,'utf8'));else{const engine=require(path.resolve(get('--engine',path.resolve(__dirname,'../engine/operational-engine.js'))));run=engine.createRun(manifest,{manifestSha:manifestHash,...JSON.parse(get('--options','{}'))});}
 const result=audit(run,manifest),out=get('--out');result.checks.push({name:'engine leaves the shared geometry object immutable',passed:manifestBefore===JSON.stringify(manifest)},{name:'run identifies the authoritative manifest hash',passed:run.manifestSha===manifestHash,detail:{expected:manifestHash,actual:run.manifestSha}});result.passed=result.checks.every(c=>c.passed);result.checkCount=result.checks.length;if(out)fs.writeFileSync(out,JSON.stringify(result,null,2)+'\n');console.log(JSON.stringify({passed:result.passed,checkCount:result.checkCount,summary:result.summary,failed:result.checks.filter(x=>!x.passed)},null,2));process.exitCode=result.passed?0:1;
}
