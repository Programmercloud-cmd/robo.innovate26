(function(root,factory){const api=factory();if(typeof module==='object'&&module.exports)module.exports=api;else root.OperationalEngine=api;})(typeof window!=='undefined'?window:globalThis,function(){
'use strict';
// New discrete-event planning model. No frozen replay is loaded or translated.
// Retained from the frozen process specification: 4x10 bricks/layer, mixed
// (row+column+layer)%3 or pure-color recipes, two FIFO layer positions and
// fourteen indexed descending-well layers per 560-brick pallet.
const L=.245,W=.067,H=.069,B=1.25,TOP=.060,EPS=1e-7;
const sampleIndexes=new WeakMap();
const dist=(a,b)=>Math.hypot(a[0]-b[0],a[1]-b[1],a[2]-b[2]);
const pathLength=p=>p.slice(1).reduce((s,v,i)=>s+dist(p[i],v),0);
const clone=p=>p.slice();
function pathAt(p,u){let s=pathLength(p)*Math.max(0,Math.min(1,u));for(let i=1;i<p.length;i++){const d=dist(p[i-1],p[i]);if(s<=d||i===p.length-1){const f=d?s/d:0;return p[i-1].map((v,j)=>v+(p[i][j]-v)*Math.max(0,Math.min(1,f)));}s-=d;}return p[0];}
function lastAt(a,t){let lo=0,hi=a.length;while(lo<hi){const m=(lo+hi)>>1;if(a[m].t<=t)lo=m+1;else hi=m;}return lo?a[lo-1]:null;}
function pose(r,t){let lo=0,hi=r.tracks.length;while(lo<hi){const m=(lo+hi)>>1;if(r.tracks[m].t0<=t)lo=m+1;else hi=m;}if(!lo)return clone(r.initial);const s=r.tracks[lo-1],u=Math.max(0,Math.min(1,(t-s.t0)/(s.t1-s.t0)));if(s.rigid){const a=s.rigid.from,b=s.rigid.to,yaw=a[3]+(b[3]-a[3])*u,c=Math.cos(yaw),sn=Math.sin(yaw),v=s.rigid.local;return[a[0]+(b[0]-a[0])*u+c*v[0]+sn*v[2],a[1]+(b[1]-a[1])*u+v[1],a[2]+(b[2]-a[2])*u-sn*v[0]+c*v[2],yaw+s.rigid.yaw];}if(s.path){const p=pathAt(s.path,u);return[p[0],p[1],p[2],s.a[3]+(s.b[3]-s.a[3])*u];}return s.a.map((v,j)=>v+(s.b[j]-v)*u);}
function createRun(G,options={}){
 const P={seed:8,palletsPerStation:1,routeSpeed:1.8,beltSpeed:.6,layerSpeed:.45,portalSpeed:1.0,gantrySpeed:1.2,forkliftSpeed:1.0,rejectProbability:.12,skipCapacity:300,demoPortalHoldUntil:9000,...options};
 if(P.palletsPerStation!==1)throw Error('This bounded commissioning scenario implements one pallet per station');
 const run={version:'brickflow.operational.v1',manifestSha:options.manifestSha||'74a93f84df620df69fc66ba5ce95f8243554bb41fa3d8095fb3ad920b45e4249',parameters:P,records:[],layers:[],pallets:[],events:[],zones:[],transactions:[],stationHistory:{},skipHistory:[],gantryHistory:[],assumptions:[],duration:0};
 run.assumptions=[
  'New deterministic discrete-event run, not a translated frozen replay. One physical28800-brick kiln cart arrives from the north and remains at the unloading station while partially consumed.',
  'Finite demand-paced commissioning inventory contains the colors needed by the active recipes; inspection still determines quality and routing. One serialized40-brick claw and one row head per lane model the source transfer, instead of qualifying all36 simultaneous heads. Source colors are assigned to the finite inventory from the declared commissioning schedule, not presented as a random-source throughput or yield result.',
  'One admitted brick at a time per paired routing module; the full receiving slot and supported path are reserved before inspection release. Waiting occurs upstream, never in routing fields.',
  'Mixed recipes retain (row+column+layer)%3, which prevents three identical neighboring colors along every grid axis; odd-numbered human lanes are mixed, even human lanes have pure Warm/Copper/Russet formers.',
  'Speeds are assumed review kinematics; contact friction, acceleration, gripping forces and hardware dynamics are not solved.',
  'Pallet pose is a wood datum: support deck top is poseY+0.060m and runner bottom is poseY-0.060m. The well deck supports that runner bottom, finishing at0.164m roller height.',
  'A deliberate initial lane-1 portal hold demonstrates full two-position buffering and causal upstream backpressure; it is labeled as a downstream hold test, not production downtime evidence.',
  'Skip capacity is an intentionally small commissioning limit of '+P.skipCapacity+' rejected bricks, not the physical capacity of the nominal10m3 body. Dropped reject IDs become bulk inventory inside the skip; granular settling is not solved.',
  'The accepted rigid discharge head lifts0.5m before swinging west; empty-body replacement is a timed isolated exchange, not a qualified hooklift vehicle trajectory.',
  'One warehouse forklift and one dedicated side loader use mutually exclusive full-journey reservations; vehicle paths follow frozen aisles. Vehicle swept-path/turning-radius/load-chart qualification remains open.',
  'Finished packs are assumed unitized before gantry/forklift handling. One active Option-A bay is used; the second bay remains a flexible reserve. Truck securing is a timed stationary state, not a modeled restraint device.'
 ];
 const rec=new Map();let seed=P.seed>>>0,eventSerial=0,brickSerial=0,globalUntil=0,rejectTail=0,rejectReserved=0,rejectBatch=0,gantryFree=0,fleetFree=0,loaderFree=0;
 const pairFree=[0,0,0],pairCandidates=[0,0,0],pending=[];const rejectIds=[],skipBatches=[];
 const rand=()=>((seed=(Math.imul(seed,1664525)+1013904223)>>>0)/4294967296);
 function add(id,kind,initial,extra={}){const r={id,kind,initial:clone(initial),p:clone(initial),tracks:[],states:[],visibleFrom:0,...extra};if(rec.has(id))throw Error('Duplicate entity '+id);rec.set(id,r);run.records.push(r);return r;}
 function ev(t,type,label,extra={}){const e={id:'event-'+eventSerial++,t,type,label,...extra};run.events.push(e);return e;}
 function zone(id,owner,start,end,kind,extra={}){if(end<start-EPS)throw Error('Negative reservation '+id);const z={id,owner,start,end,kind,...extra};run.zones.push(z);return z;}
 function tx(t,ids,from,to,owner){run.transactions.push({t,ids:ids.slice(),from,to,owner});for(const id of ids){const r=rec.get(id);if(!r)throw Error('Unknown transaction '+id);r.states.push({t,state:to,owner});}}
 function track(r,to,t0,dt,kind,extra={}){if(dt<=0)throw Error('Nonpositive motion '+kind);const last=r.tracks[r.tracks.length-1];if(last&&last.t1>t0+EPS)throw Error('Overlapping entity motion '+r.id+' '+kind);r.tracks.push({t0,t1:t0+dt,a:clone(r.p),b:clone(to),kind,...extra});r.p=clone(to);return t0+dt;}
 function move(r,to,t,speed,kind,extra={}){return track(r,to,t,Math.max(.2,dist(r.p,to)/speed),kind,extra);}
 function path(r,points,t,speed,kind,extra={}){return track(r,[...points[points.length-1].slice(0,3),r.p[3]],t,Math.max(.2,pathLength(points)/speed),kind,{path:points.map(p=>p.slice(0,3)),...extra});}
 function shift(ids,delta,t,dt,kind,extra={}){for(const id of ids){const r=rec.get(id);track(r,r.p.map((v,j)=>v+(delta[j]||0)),t,dt,kind,extra);}return t+dt;}
 function groupedTo(ids,center,target,t,speed,kind,extra={}){const delta=target.map((v,j)=>v-center[j]),dt=Math.max(.2,Math.hypot(...delta.slice(0,3))/speed);return shift(ids,delta,t,dt,kind,extra);}
 function rigidCarry(ids,tool,target,t,speed,kind,minimum=.2){const origin=clone(tool.p),dt=Math.max(minimum,dist(origin,target)/speed,Math.abs(target[3]-origin[3])/1.0);for(const id of ids){const r=rec.get(id),dx=r.p[0]-origin[0],dz=r.p[2]-origin[2],c=Math.cos(origin[3]),s=Math.sin(origin[3]),local=[c*dx-s*dz,r.p[1]-origin[1],s*dx+c*dz],cc=Math.cos(target[3]),ss=Math.sin(target[3]),yaw=r.p[3]-origin[3];track(r,[target[0]+cc*local[0]+ss*local[2],target[1]+local[1],target[2]-ss*local[0]+cc*local[2],target[3]+yaw],t,dt,kind,{carrier:tool.id,rigid:{from:origin,to:clone(target),local,yaw}});}track(tool,target,t,dt,kind);return t+dt;}
 // Authoritative finite cart inventory, rotated on a north/south rail. The
 // source controller serializes one40-brick tool and six row tools. It never
 // invents a brick at inspection and never sends a partly loaded cart away.
 const U=G.upstream_kiln_cart;if(!U)throw Error('Operational kiln-cart manifest is required');
 const cart=add('input-cart-0','inputCart',[U.track.center_x_m,.2,U.track.north_loaded_entry_z_m,0]);
 const sourceClaw=add('source-claw','layerHead',[U.track.center_x_m,3.1945,0,Math.PI]);
 const sourceRows=Array.from({length:6},(_,lane)=>add('source-row-'+lane,'rowHead',[7,2.4945,G.routing.lane_z_m[lane],Math.PI/2],{lane}));
 const sourceStocks=Array.from({length:6},()=>[]),sourceBatch=[0,0,0,0,0,0],sourceRowFree=[0,0,0,0,0,0],sourcePlacementFree=Array.from({length:6},()=>Array(6).fill(0));
 let sourceFree=44,admissionNow=44;
 const allSource=[];
 for(let c=0;c<6;c++)for(let j=0;j<6;j++)for(let layer=0;layer<20;layer++)for(let row=0;row<4;row++)for(let col=0;col<10;col++){
  const id=(((c*6+j)*20+layer)*40+row*10+col),baseX=U.track.center_x_m+(c-2.5)*1.2,baseZ=-(j-2.5)*1.3;
  const r=add(id,'brick',[baseX-(row-1.5)*L,.32+(layer+.5)*H,baseZ-(col-4.5)*W+U.track.north_loaded_entry_z_m,Math.PI],{shade:(row+col+layer)%3,good:true,lane:c,source:{cart:0,c,j,layer,row,col},initialState:'source'});r.states.push({t:0,state:'source'});allSource.push(id);
 }
 shift(allSource,[0,0,-U.track.north_loaded_entry_z_m,0],0,44,'loaded-cart-arrival',{carrier:cart.id});track(cart,[U.track.center_x_m,.2,0,0],0,44,'loaded-cart-arrival');ev(44,'loaded-cart-arrived','Loaded kiln cart arrived from north; unloading interlock set');
 function prepareSourceBatch(lane,t){
  const n=sourceBatch[lane]++,j=n%6,layer=19-Math.floor(n/6),c=lane;if(layer<0)throw Error('Finite source inventory exhausted for lane '+lane);
  const ids=Array.from({length:40},(_,k)=>(((c*6+j)*20+layer)*40+k)),cx=U.track.center_x_m+(c-2.5)*1.2,cz=-(j-2.5)*1.3,cy=.32+(layer+.5)*H,dx=U.layer_claw_transfer.placement_x_m[j],dz=U.layer_claw_transfer.placement_z_m[c];
  let q=Math.max(t,sourceFree,sourcePlacementFree[c][j]);const start=q;
  q=move(sourceClaw,[cx,3.1945,cz,Math.PI],q,1.5,'source-claw-empty-approach');q=move(sourceClaw,[cx,cy+.0945,cz,Math.PI],q,.7,'source-claw-lower');q+=.3;tx(q,ids,'source','upstream',sourceClaw.id);ev(q,'source-layer-pick','Claw grips one complete40-brick layer',{ids:ids.slice(),sourceLane:lane,sourceStack:{c,j,layer}});
  q=rigidCarry(ids,sourceClaw,[cx,3.1945,cz,Math.PI],q,.7,'source-layer-lift');const rotateStart=q;
  q=rigidCarry(ids,sourceClaw,[dx,3.1945,dz,Math.PI/2],q,1.25,'source-layer-rotate-transfer');ev(rotateStart,'source-layer-rotation','Loaded claw rotates90° during high transfer',{ids:ids.slice(),sourceLane:lane,fromYaw:Math.PI,toYaw:Math.PI/2});
  q=rigidCarry(ids,sourceClaw,[dx,B+H/2+.0945,dz,Math.PI/2],q,.7,'source-layer-place');q+=.3;ev(q,'source-layer-placed','40-brick layer on unchanged loading station',{ids:ids.slice(),sourceLane:lane,placement:[dx,B,dz]});
  q=move(sourceClaw,[dx,3.1945,dz,Math.PI/2],q,.7,'source-claw-empty-raise');sourceFree=q;zone('source-layer-claw','source-batch-'+c+'-'+n,start,q,'source-claw');
  const mainStart=q,dt=13/.5;shift(ids,[13,0,0,0],mainStart,dt,'source-main-belt',{support:'unchanged-main-loading-belt'});q+=dt;
  const stock=[];for(let row=0;row<4;row++)for(let col=9;col>=0;col--)stock.push({id:ids[row*10+col],row,c,j,ready:q,rowIds:ids.slice(row*10,row*10+10),rowCenter:[dx+13,B+H/2,dz-(row-1.5)*L],batch:n,rowScheduled:false});sourceStocks[lane]=stock;
 }
 function supplyBrick(lane,t){if(!sourceStocks[lane].length)prepareSourceBatch(lane,t);const entry=sourceStocks[lane].shift(),r=rec.get(entry.id),rowTool=sourceRows[lane],sameRow=sourceStocks[lane].filter(e=>e.row===entry.row&&e.batch===entry.batch);
  if(!entry.rowScheduled){let q=Math.max(entry.ready,sourceRowFree[lane],t),start=q;const at=entry.rowCenter;q=move(rowTool,[at[0],2.4945,at[2],Math.PI/2],q,1.0,'source-row-empty-approach');q=move(rowTool,[at[0],at[1]+.0945,at[2],Math.PI/2],q,.7,'source-row-lower');q+=.2;q=rigidCarry(entry.rowIds,rowTool,[at[0],2.4945,at[2],Math.PI/2],q,.7,'source-row-lift');q=rigidCarry(entry.rowIds,rowTool,[at[0],2.4945,G.routing.lane_z_m[lane],Math.PI/2],q,1.0,'source-row-side-transfer');q=rigidCarry(entry.rowIds,rowTool,[at[0],B+H/2+.0945,G.routing.lane_z_m[lane],Math.PI/2],q,.7,'source-row-place');q+=.2;entry.ready=q;for(const e of sameRow){e.rowScheduled=true;e.ready=q;}q=move(rowTool,[at[0],2.6945,G.routing.lane_z_m[lane],Math.PI/2],q,.7,'source-row-raise');sourceRowFree[lane]=q;zone('source-row-'+lane,'source-batch-'+lane+'-'+entry.batch+'-row'+entry.row,start,q,'source-row');if(entry.row===3)sourcePlacementFree[lane][entry.j]=q;}
  let q=Math.max(t,entry.ready);q=move(r,[16.6,B+H/2,G.routing.lane_z_m[lane],Math.PI/2],q,.6,'source-single-brick-feed',{support:'side-feed-belt'});q=track(r,[17.0,B+H/2,G.routing.lane_z_m[lane],0],q,1.2,'source-supported-singulation-turn',{support:'wide-inspection-entry'});r.sourceReadyAt=q;return r;
 }
 const stations=[];const portals=[];
 for(let lane=0;lane<6;lane++){
  const z=G.routing.lane_z_m[lane];portals.push(add('portal-'+lane,'layerTool',[G.formation.layer_pickup_x_m,1.9445,z,0],{lane}));
  for(let s=0;s<3;s++){
   const st={id:'L'+(lane+1)+'-S'+(s+1),lane,station:s,z:z+G.formation.well_z_offsets_m[s],formerFree:0,AFree:0,BFree:0,portalFree:0,nextLayer:0,palletNumber:0,layers:[],gateIntervals:[],jobs:[]};
   stations.push(st);run.stationHistory[st.id]=[];add('gate-'+lane+'-'+s,'gate',[G.formation.retention_stop_x_m,B+.055,st.z,0],{lane,station:s});
   for(let n=0;n<P.palletsPerStation;n++){
    const id='pallet-'+lane+'-'+s+'-'+n,wood=add(id,'pallet',[39.935,.224,st.z,0],{lane,station:s,supportTopOffsetY:TOP,visibleFrom:n?Infinity:0});
    const deck=n?null:add('well-'+lane+'-'+s,'wellDeck',[G.formation.well_center_x_m,.164,st.z,0],{lane,station:s});
    const job={id,lane,station:s,stationId:st.id,number:n,recipe:lane%2===0?'mixed':['Warm','Copper','Russet'][s],shade:lane%2===0?null:s,ids:[],layers:[],wood:wood.id,deckId:'well-'+lane+'-'+s,preparedAt:0,nextLayer:0,ready:0,completedAt:null,slotId:null};st.jobs.push(job);run.pallets.push(job);
    if(n===0){let t=move(wood,[G.formation.well_center_x_m,.224,st.z,0],0,.5,'empty-pallet-feed',{support:'empty-pallet-infeed'});const dt=(1.19-.224)/.3;track(wood,[G.formation.well_center_x_m,1.19,st.z,0],t,dt,'well-raise',{support:'well-deck'});track(deck,[G.formation.well_center_x_m,1.13,st.z,0],t,dt,'well-raise');job.preparedAt=t+dt;}
   }
  }
 }
 const gantry=add('gantry','gantry',[G.gantry.pickup_x_m,4.3,0,0]);
 const forklift=add('warehouse-forklift','forklift',[G.clear_dispatch_spine.x_min_m+2.2,0,-14.2,0],{loadCenterOffsetX:1.4});
 const loader=add('side-loader','forklift',[87.8,0,-14.2,Math.PI],{loadCenterOffsetX:1.4});
 // A stationary Option-A truck is locked in the first bay throughout loading.
 const bay=G.adopted_dispatch.side_bays[0],truck=add('truck-active','truck',[bay.centre_x_m,0,0,0],{truckReference:true});
 const usedSlots=new Set();const bayCapacity=20;let truckLoaded=0;
 function stEvent(st,t,state,extra={}){run.stationHistory[st.id].push({t,state,...extra});}
 function headState(t,state,extra={}){run.skipHistory.push({t,state,headLift:0,headYaw:0,visible:true,upstreamStopped:false,...extra});}
 headState(0,'filling');
 function exchange(t){
  const batch=rejectBatch++,ids=rejectIds.splice(0);const stop=t,drained=Math.max(t,...pairFree,rejectTail)+.5;globalUntil=drained+22;
  zone('upstream-stop','skip-exchange-'+batch,stop,globalUntil,'upstream-stop');
  headState(stop,'stopping upstream',{upstreamStopped:true});ev(stop,'skip-stop','Upstream stop requested: skip reservations full',{batch});
  headState(drained,'belts clear',{upstreamStopped:true});ev(drained,'reject-belts-clear','All admitted routing/reject material has cleared',{batch});
  headState(drained+.5,'lifting head',{headLift:.5,upstreamStopped:true});
  headState(drained+2.5,'head raised',{headLift:.5,upstreamStopped:true});ev(drained+2.5,'head-raised','Discharge head raised0.5m before slew',{batch});
  headState(drained+4.5,'head stowed',{headLift:.5,headYaw:-Math.PI/2,upstreamStopped:true});ev(drained+4.5,'skip-exchange','Head stowed; isolated full-body exchange',{batch});
  const removed=drained+10.5;headState(drained+5.5,'body removed',{headLift:.5,headYaw:-Math.PI/2,visible:false,upstreamStopped:true});
  tx(removed,ids,'skip','removed','skip-batch-'+batch);skipBatches.push({batch,ids:ids.slice(),stop,drained,headStowedAt:drained+4.5,removedAt:removed,resume:globalUntil});
  headState(drained+15.5,'empty body positioned',{headLift:.5,headYaw:-Math.PI/2,upstreamStopped:true});
  headState(drained+18.5,'raised head returned',{headLift:.5,upstreamStopped:true});headState(drained+20.5,'head lowered',{upstreamStopped:true});headState(globalUntil,'filling');ev(globalUntil,'upstream-resume','Empty body verified; head returned and lowered; feed resumes',{batch});rejectReserved=0;
 }
 function newBrick(sourceLane,shade,good,t){const r=supplyBrick(sourceLane,t);brickSerial++;r.shade=shade;r.good=good;r.lane=sourceLane;tx(r.sourceReadyAt,[r.id],'upstream','inspection','inspection-'+sourceLane);admissionNow=r.sourceReadyAt;return r;}
 function inspect(r,t){t=Math.max(t,r.sourceReadyAt);const z=r.p[2];t=move(r,[G.routing.inspection_camera_x_m[0],B+H/2,z,0],t,.6,'inspection-feed',{support:'inspection-belt'});t+=.4;t=move(r,[G.routing.inspection_camera_x_m[1],B+H/2,z,0],t,.6,'inspection-dual-view',{support:'inspection-belt'});t+=.4;return move(r,[G.routing.upstream_exchange.x_min_m+.13,B+H/2,z,0],t,.6,'inspection-release',{support:'inspection-belt-and-crossfeed'});}
 function scheduleReject(pair,t,sourceLane){
  const r=newBrick(sourceLane,Math.floor(rand()*3),false,t);let q=inspect(r,t);const laneZ=G.routing.lane_z_m[sourceLane],pairZ=G.routing.pair_z_m[pair],side=Math.sign(laneZ-pairZ),x=G.routing.local_fanout.x_min_m+2.0;
  tx(q,[r.id],'inspection','reject_transit','reject-'+pair);const routeStart=q;
  q=path(r,[[r.p[0],r.p[1],laneZ],[x,B+H/2,laneZ],[x,B+H/2,pairZ+side*1.0]],q,P.routeSpeed,'reject-routing',{support:'routing-deck',pair});
  // Controlled gravity-supported chamfer: bottom follows the actual deck/chamfer.
  q=path(r,[[x,B+H/2,pairZ+side*1.0],[x,.75+H/2,pairZ+side*.4],[x,.75+H/2,pairZ]],q,.7,'reject-chamfer',{support:'accepted-flat-bottom-channel',pair});
  const pairEnd=q;
  // Conservative whole downstream collector reservation prevents merge collisions.
  q=Math.max(q,rejectTail);const beltStart=q;
  q=path(r,[[x,.75+H/2,pairZ],[46.7,.75+H/2,pairZ],[47,.4+H/2,pairZ],[47,.4+H/2,17],[47,1.8+H/2,G.reject_system.incline.z_end_m],[47,1.8+H/2,G.reject_system.swing_away_discharge.fill_nose_xyz_m[2]]],q,P.beltSpeed,'reject-collection',{support:'branch-common-incline-head',pair});
  q=path(r,[[47,1.8+H/2,r.p[2]],[47,.2+H/2,r.p[2]]],q,2.0,'reject-gravity-drop',{support:'gravity-drop-into-skip'});
  zone('reject-collector',r.id,beltStart,q,'rejectmerge');rejectTail=q;tx(q,[r.id],'reject_transit','skip','skip-'+rejectBatch);r.hiddenAt=q;rejectIds.push(r.id);rejectReserved++;
  zone('routing-pair-'+pair,r.id,t,q,'routing-pair',{sourceLane,good:false});pairFree[pair]=q+.25;
  if(rejectReserved>=P.skipCapacity)exchange(admissionNow);
  return r;
 }
 const pairJobs=[0,1,2].map(pair=>stations.filter(st=>Math.floor(st.lane/2)===pair).flatMap(st=>st.jobs).map(job=>({job,st:stations.find(s=>s.id===job.stationId),layer:0,brick:0,current:null})));
 const cursors=[0,0,0];let totalGood=0;
 function nextTask(pair){const jobs=pairJobs[pair];while(cursors[pair]<jobs.length&&jobs[cursors[pair]].layer>=14)cursors[pair]++;return jobs[cursors[pair]]||null;}
 function beginLayer(task){const {job,st}=task,number=task.layer,id='layer-'+job.id+'-'+number;const layer={id,lane:st.lane,station:st.station,stationId:st.id,number,palletId:job.id,recipe:job.recipe,ids:[],sourceLanes:[],receiving:[],formedAt:null};task.current=layer;run.layers.push(layer);job.layers.push(id);return layer;}
 function moveLayerToCenter(layer,center,t,kind,speed=P.layerSpeed){const points=layer.ids.map(id=>rec.get(id).p);const from=[points.reduce((a,p)=>a+p[0]/40,0),points.reduce((a,p)=>a+p[1]/40,0),points.reduce((a,p)=>a+p[2]/40,0),0];return groupedTo(layer.ids,from,center,t,speed,kind,{support:'continuous-layer-bed',layerId:layer.id});}
 function finishLayer(task,t){
  const {st,job,current:layer}=task;const stationId=st.id;
  // Formation closes the receiving gaps on the same supported powered field.
  const dt=1.5;for(let k=0;k<40;k++){const r=rec.get(layer.ids[k]),row=Math.floor(k/10),col=k%10;track(r,[G.formation.alignment_center_x_m+(1.5-row)*L,B+H/2,st.z+(col-4.5)*W,0],t,dt,'layer-compaction',{support:'formation-field',layerId:layer.id});}
  t+=dt;layer.formedAt=t;ev(t,'layer-formed','40-brick layer formed at '+stationId,{stationId,lane:st.lane,station:st.station,layerId:layer.id,palletId:job.id});stEvent(st,t,'formed',{layerId:layer.id});
  const aStart=Math.max(t,st.AFree),aEnd=moveLayerToCenter(layer,[36.835,B+H/2,st.z,0],aStart,'layer-to-buffer-a');layer.bufferAAt=aEnd;tx(aStart,layer.ids,'forming','bufferA',layer.id);
  if(aStart>t+.01)ev(t,'formation-blocked','Formed layer waits: buffer A occupied; further admission inhibited',{stationId,lane:st.lane,layerId:layer.id,until:aStart});
  const bStart=Math.max(aEnd,st.BFree),bEnd=moveLayerToCenter(layer,[G.formation.layer_pickup_x_m,B+H/2,st.z,0],bStart,'layer-to-buffer-b');layer.bufferBAt=bEnd;tx(bStart,layer.ids,'bufferA','bufferB',layer.id);
  zone(st.id+'-bufferA',layer.id,aStart,bEnd,'bufferA',{stationId});st.gateIntervals.push([aStart,bEnd]);st.AFree=bEnd;st.formerFree=aEnd;
  ev(aEnd,'buffer-a','Layer in first buffer position',{stationId,lane:st.lane,station:st.station,layerId:layer.id,palletId:job.id});ev(bEnd,'buffer-b','Layer ready in second buffer position',{stationId,lane:st.lane,station:st.station,layerId:layer.id,palletId:job.id});
  const tool=portals[st.lane],hold=(st.lane===0&&P.demoPortalHoldUntil)?P.demoPortalHoldUntil:0;let q=Math.max(bEnd,tool.available||0,job.preparedAt,hold);
  if(hold&&layer.number===0){zone('portal-test-hold-'+st.lane,'commissioning-hold',0,hold,'downstream-hold');ev(0,'downstream-hold','Lane1 portal held to exercise buffer interlocks',{stationId,lane:st.lane,until:hold});ev(hold,'downstream-hold-release','Commissioning portal hold released',{stationId,lane:st.lane});}
  const portalStart=q;q=move(tool,[G.formation.layer_pickup_x_m,1.9445,st.z,0],q,P.portalSpeed,'portal-empty-approach');q=move(tool,[G.formation.layer_pickup_x_m,B+H/2+.0945,st.z,0],q,.7,'portal-lower-to-layer');q+=.3;
  layer.pickedAt=q;tx(q,layer.ids,'bufferB','tool',tool.id);ev(q,'layer-transfer','Portal picks complete40-brick layer',{stationId,lane:st.lane,station:st.station,layerId:layer.id,palletId:job.id});
  const liftDt=(1.85-(B+H/2))/.7;shift(layer.ids,[0,1.85-(B+H/2),0,0],q,liftDt,'portal-layer-lift',{carrier:tool.id,layerId:layer.id});track(tool,[G.formation.layer_pickup_x_m,1.9445,st.z,0],q,liftDt,'portal-layer-lift');q+=liftDt;layer.bufferReleasedAt=q;
  zone(st.id+'-bufferB',layer.id,bStart,q,'bufferB',{stationId});st.BFree=q;
  let d=(G.formation.well_center_x_m-G.formation.layer_pickup_x_m)/P.portalSpeed;shift(layer.ids,[G.formation.well_center_x_m-G.formation.layer_pickup_x_m,0,0,0],q,d,'portal-layer-transfer',{carrier:tool.id,layerId:layer.id});track(tool,[G.formation.well_center_x_m,1.9445,st.z,0],q,d,'portal-layer-transfer');q+=d;
  d=(1.85-(B+H/2))/.7;shift(layer.ids,[0,(B+H/2)-1.85,0,0],q,d,'portal-layer-place',{carrier:tool.id,layerId:layer.id});track(tool,[G.formation.well_center_x_m,B+H/2+.0945,st.z,0],q,d,'portal-layer-place');q+=d+.3;layer.placedAt=q;tx(q,layer.ids,'tool','stacked',job.id);
  q=move(tool,[G.formation.well_center_x_m,1.9445,st.z,0],q,.7,'portal-raise-clear');job.ids.push(...layer.ids);
  const wood=rec.get(job.wood),deck=rec.get(job.deckId),lowerDt=H/.15;
  shift(job.ids,[0,-H,0,0],q,lowerDt,'well-index-lower',{carrier:deck.id,layerId:layer.id,palletId:job.id});track(wood,[wood.p[0],wood.p[1]-H,wood.p[2],0],q,lowerDt,'well-index-lower',{carrier:deck.id});track(deck,[deck.p[0],deck.p[1]-H,deck.p[2],0],q,lowerDt,'well-index-lower');q+=lowerDt;
  layer.stackedAt=q;job.nextLayer++;st.layers.push(layer.id);tool.available=q;zone('portal-lane-'+st.lane,layer.id,portalStart,q,'portal',{stationId});zone(st.id+'-former',layer.id,layer.firstAdmission,aEnd,'forming',{stationId});
  ev(q,'layer-stacked','Well indexed: '+job.nextLayer+'/14 layers',{stationId,lane:st.lane,station:st.station,layerId:layer.id,palletId:job.id,layers:job.nextLayer,bricks:job.ids.length});
  if(job.nextLayer===14)finishPallet(job,st,q);
  task.layer++;task.brick=0;task.current=null;
 }
 function carryPack(ids,tool,target,t,speed,kind){const from=clone(tool.p),delta=target.map((v,j)=>v-from[j]),dt=Math.max(.2,dist(from,target)/speed);shift(ids,delta,t,dt,kind,{carrier:tool.id});track(tool,target,t,dt,kind);return t+dt;}
 function finishPallet(job,st,t){
  job.completedAt=t;ev(t,'pallet-complete','Completed560-brick pallet at '+st.id,{stationId:st.id,lane:st.lane,station:st.station,palletId:job.id});
  const pack=job.ids.concat(job.wood);tx(t,job.ids,'stacked','outfeed',job.id);const outStart=t+.5,dt=(G.formation.finished_pallet_pickup_x_m-G.formation.well_center_x_m)/.35;shift(pack,[G.formation.finished_pallet_pickup_x_m-G.formation.well_center_x_m,0,0,0],outStart,dt,'completed-pallet-outfeed',{support:'finished-pallet-rollers'});job.outfeedAt=outStart+dt;
  ev(job.outfeedAt,'pallet-pickup','Completed pallet at gantry pickup',{stationId:st.id,lane:st.lane,station:st.station,palletId:job.id});
  let q=Math.max(job.outfeedAt,gantryFree);const gStart=q;
  const slot=G.storage.slots.filter(s=>!usedSlots.has(s.id)).sort((a,b)=>(Math.abs(a.x_m-G.gantry.pickup_x_m)+Math.abs(a.z_m-st.z))-(Math.abs(b.x_m-G.gantry.pickup_x_m)+Math.abs(b.z_m-st.z)))[0];if(!slot)throw Error('Storage full');usedSlots.add(slot.id);job.slotId=slot.id;
  q=move(gantry,[G.gantry.pickup_x_m,4.3,st.z,0],q,P.gantrySpeed,'gantry-empty-approach');q=move(gantry,[G.gantry.pickup_x_m,.224+1.1,st.z,0],q,.7,'gantry-frame-lower');q+=.4;job.gantryPickupAt=q;tx(q,job.ids,'outfeed','gantry',gantry.id);
  q=carryPack(pack,gantry,[G.gantry.pickup_x_m,4.3,st.z,0],q,.7,'gantry-supported-pallet-lift');zone(st.id+'-outfeed',job.id,outStart,q,'outfeed',{stationId:st.id});
  q=carryPack(pack,gantry,[slot.x_m,4.3,st.z,0],q,P.gantrySpeed,'gantry-traverse-x');q=carryPack(pack,gantry,[slot.x_m,4.3,slot.z_m,0],q,P.gantrySpeed,'gantry-traverse-z');q=carryPack(pack,gantry,[slot.x_m,.060+1.1,slot.z_m,0],q,.7,'gantry-storage-lower');q+=.4;job.storedAt=q;tx(q,job.ids,'gantry','stored',slot.id);ev(q,'pallet-stored','Gantry placed pallet in '+slot.id,{stationId:st.id,lane:st.lane,station:st.station,palletId:job.id,slotId:slot.id});
  const sign=slot.approach_from==='+X'?1:-1;q=move(gantry,[slot.x_m+sign*1.4,.060+1.1,slot.z_m,0],q,.35,'gantry-fork-withdraw');q=move(gantry,[slot.x_m+sign*1.4,4.3,slot.z_m,0],q,.7,'gantry-frame-raise');gantryFree=q;job.gantryClearAt=q;zone('storage-gantry',job.id,gStart,q,'gantry');
  // Allocation is released only after the warehouse forklift has cleared.
  dispatchPallet(job,slot,q+20);
 }
 function forkliftDrive(tool,points,t,kind,cargo){for(const p of points){const target=[p[0],tool.p[1],p[1],tool.p[3]],dt=Math.max(.2,dist(tool.p,target)/P.forkliftSpeed);if(cargo)shift(cargo,[target[0]-tool.p[0],0,target[2]-tool.p[2],0],t,dt,kind,{carrier:tool.id});track(tool,target,t,dt,kind);t+=dt;}return t;}
 function turnForklift(tool,yaw,t,cargo,kind){if(Math.abs(tool.p[3]-yaw)<1e-6)return t;const to=[tool.p[0],tool.p[1],tool.p[2],yaw];return cargo?rigidCarry(cargo,tool,to,t,1.0,kind,2.0):track(tool,to,t,2.0,kind);}
 function dispatchPallet(job,slot,earliest){
  const pack=job.ids.concat(job.wood),side=slot.approach_from==='+X'?1:-1,aisle=G.storage.aisles.find(a=>a.id===slot.aisle_id),aisleX=(aisle.x_min_m+aisle.x_max_m)/2,crossZ=slot.z_m<0?-14.2:14.2,stageX=85,stageZ=-9+(job.lane*3+job.station)%16*1.2;
  let q=Math.max(earliest,fleetFree,loaderFree),start=q;
  q=forkliftDrive(forklift,[[81.5,-14.2],[aisleX,-14.2],[aisleX,slot.z_m]],q,'warehouse-forklift-empty',null);q=turnForklift(forklift,side===1?Math.PI:0,q,null,'warehouse-forklift-empty-turn');q=forkliftDrive(forklift,[[slot.x_m+side*1.4,slot.z_m]],q,'warehouse-forklift-empty',null);
  job.warehousePickupAt=q;tx(q,job.ids,'stored','warehouse-forklift',forklift.id);q=shift(pack,[0,.2,0,0],q,1,'warehouse-forklift-lift',{carrier:forklift.id});
  q=forkliftDrive(forklift,[[aisleX,slot.z_m],[aisleX,crossZ],[81.5,crossZ]],q,'warehouse-forklift-loaded',pack);q=turnForklift(forklift,0,q,pack,'warehouse-loaded-turn-in-spine');q=forkliftDrive(forklift,[[81.5,stageZ],[83.6,stageZ]],q,'warehouse-forklift-loaded',pack);
  // The chassis reference is1.4m behind the load center at pickup; preserve
  // that loaded offset during every drive, then place on the two-sided handoff.
  const wood=rec.get(job.wood);if(Math.abs(wood.p[0]-stageX)>.001||Math.abs(wood.p[2]-stageZ)>.001)throw Error('Unsupported warehouse handoff offset');q=shift(pack,[0,.060-wood.p[1],0,0],q,1,'warehouse-handoff-place',{carrier:forklift.id});job.handoffAt=q;tx(q,job.ids,'warehouse-forklift','handoff','handoff-'+job.id);ev(q,'pallet-handoff','Warehouse forklift placed pack at two-sided handoff',{palletId:job.id,stationId:job.stationId,slotId:slot.id});
  const slotRelease=q;zone('storage-slot-'+slot.id,job.id,job.storedAt,slotRelease,'storage',{slotId:slot.id});
  q=forkliftDrive(forklift,[[81.5,stageZ],[81.5,-14.2]],q,'warehouse-forklift-return',null);fleetFree=q;zone('warehouse-forklift',job.id,start,q,'forklift');
  // Each exposed side places only its near row. A full fixed-offset carried
  // turn is performed in the west work lane when forks must face east.
  let d=Math.max(q,loaderFree),loadStart=d;const east=truckLoaded>=10,trailerX=bay.centre_x_m+(east?.65:-.65),trailerZ=-6.8+(truckLoaded%10)*1.2;
  d=forkliftDrive(loader,[[87.8,-14.2],[87.8,stageZ]],d,'side-loader-empty',null);d=turnForklift(loader,Math.PI,d,null,'side-loader-empty-turn');d=forkliftDrive(loader,[[86.4,stageZ]],d,'side-loader-empty',null);tx(d,job.ids,'handoff','side-loader',loader.id);d=shift(pack,[0,.2,0,0],d,1,'side-loader-lift',{carrier:loader.id});
  d=forkliftDrive(loader,[[87.8,stageZ]],d,'side-loader-loaded',pack);
  if(east)d=forkliftDrive(loader,[[87.8,-14.2],[95.975,-14.2],[95.975,trailerZ]],d,'side-loader-north-crossing',pack);
  else{d=turnForklift(loader,0,d,pack,'side-loader-loaded-turn');d=forkliftDrive(loader,[[87.8,trailerZ]],d,'side-loader-loaded',pack);}
  const cargoWood=rec.get(job.wood),truckWoodY=1.21+.060;d=shift(pack,[0,truckWoodY-cargoWood.p[1],0,0],d,2,'side-loader-height',{carrier:loader.id});
  d=forkliftDrive(loader,[[east?94.05:89.95,trailerZ]],d,'side-loader-supported-placement',pack);
  if(Math.abs(rec.get(job.wood).p[0]-trailerX)>.001||Math.abs(rec.get(job.wood).p[2]-trailerZ)>.001)throw Error('Unsupported truck placement offset');job.loadedAt=d;tx(d,job.ids,'side-loader','truck',truck.id);truckLoaded++;ev(d,'truck-loaded','Two-sided near-row truck load '+truckLoaded+'/'+Math.min(bayCapacity,run.pallets.length),{palletId:job.id,stationId:job.stationId,side:east?'east':'west'});
  d=forkliftDrive(loader,[[east?95.975:87.8,trailerZ],[east?95.975:87.8,-14.2],[87.8,-14.2]],d,'side-loader-clear',null);loaderFree=d;zone('truck-cell-active',job.id,loadStart,d,'truckcell');
 }
 // Work queues are consumed one brick at a time in chronological admission order.
 // Each station completes its requested layers in order; independent pairs and
 // downstream machines run concurrently on their own reservation calendars.
 let guard=0;
 while(true){
  let pair=-1,best=Infinity,task=null;
  for(let p=0;p<3;p++){const a=nextTask(p);if(!a)continue;const at=Math.max(pairFree[p],a.st.formerFree,globalUntil,admissionNow);if(at<best){pair=p;best=at;task=a;}}
  if(pair<0)break;if(++guard>200000)throw Error('Admission scheduler did not converge');
  if(rand()<P.rejectProbability){scheduleReject(pair,best,pair*2+(rand()<.5?0:1));continue;}
  const layer=task.current||beginLayer(task),row=Math.floor(task.brick/10),col=task.brick%10,shade=task.job.shade===null?(row+col+task.layer)%3:task.job.shade;
  const sourceLane=(task.brick+task.layer)%3===0?(task.st.lane^1):task.st.lane;
  const r=newBrick(sourceLane,shade,true,best);r.station=task.st.station;r.destinationLane=task.st.lane;r.layerId=layer.id;r.palletId=task.job.id;
  let t=inspect(r,best);tx(t,[r.id],'inspection','routing',layer.id);const routeStart=t,sourceZ=G.routing.lane_z_m[sourceLane],destZ=G.routing.lane_z_m[task.st.lane],targetZ=task.st.z+(col-4.5)*.1,receiveX=G.formation.receiving_row_center_x_m[3-row];
  const pts=[[r.p[0],B+H/2,sourceZ],[G.routing.upstream_exchange.cross_transfer_x_m,B+H/2,sourceZ]];
  if(sourceLane!==task.st.lane)pts.push([G.routing.upstream_exchange.cross_transfer_x_m,B+H/2,destZ]);
  pts.push([G.routing.local_fanout.usable_center_forward_start_x_m,B+H/2,destZ],[G.routing.local_fanout.usable_center_forward_end_x_m,B+H/2,targetZ],[receiveX,B+H/2,targetZ]);
  t=path(r,pts,t,P.routeSpeed,'supported-color-routing',{support:'upstream-crossfeed-then-own-lane-field',pair,sourceLane,destinationLane:task.st.lane,stationId:task.st.id});
  zone('routing-pair-'+pair,r.id,best,t,'routing-pair',{sourceLane,destinationLane:task.st.lane,good:true});pairFree[pair]=t+.25;tx(t,[r.id],'routing','forming',layer.id);
  if(!layer.ids.length)layer.firstAdmission=best;layer.ids.push(r.id);layer.sourceLanes.push(sourceLane);layer.receiving.push([receiveX,B+H/2,targetZ]);task.brick++;totalGood++;
  if(task.brick===40)finishLayer(task,t);
 }
 // Close remaining reject inventory in the final installed skip (no forced swap).
 run.skipBatches=skipBatches;
 // Generate each physical retention gate from merged A-position occupancy.
 for(const st of stations){const r=rec.get('gate-'+st.lane+'-'+st.station),ints=st.gateIntervals.sort((a,b)=>a[0]-b[0]),merged=[];for(const a of ints){const p=merged[merged.length-1];if(p&&a[0]<=p[1]+.6)p[1]=Math.max(p[1],a[1]);else merged.push(a.slice());}for(const [a,b]of merged){const start=Math.max(0,a-.3);track(r,[r.p[0],B-.13,r.p[2],0],start,.3,'retention-lower');track(r,[r.p[0],B+.055,r.p[2],0],b,.3,'retention-raise');}}
 // Truck departure after all modeled loads and an explicit stationary securing
 // period. Cargo is moved with the truck and remains accounted as shipped.
 let end=Math.max(loaderFree,fleetFree,gantryFree,rejectTail,globalUntil,...run.records.flatMap(r=>r.tracks.length?[r.tracks[r.tracks.length-1].t1]:[0]));
 const loaded=run.pallets.filter(p=>p.loadedAt!==undefined),loadIds=loaded.flatMap(p=>p.ids),cargo=loaded.flatMap(p=>p.ids.concat(p.wood));
 const securingStart=end;ev(securingStart,'truck-securing','All loaders clear; stationary load securing');zone('truck-securing','truck-active',securingStart,securingStart+15,'securing');end+=15;
 ev(end,'truck-departure','Secured truck released; all loaders clear');const departStart=end;shift(cargo,[0,0,27,0],departStart,27/.8,'secured-truck-departure',{carrier:truck.id});track(truck,[truck.p[0],0,27,0],departStart,27/.8,'secured-truck-departure');end+=27/.8;
 tx(end,loadIds,'truck','shipped','truck-active');for(const p of loaded){p.shippedAt=end;for(const id of p.ids.concat(p.wood))rec.get(id).hiddenAt=end;}truck.hiddenAt=end;
 for(const st of stations){const ls=run.layers.filter(l=>l.stationId===st.id);let marked=false;for(const a of ls)for(const b of ls){if(marked||a===b)continue;const start=Math.max(a.bufferAAt,b.bufferBAt),finish=Math.min(a.bufferBAt,b.bufferReleasedAt);if(finish>start+.1){ev(start,'buffers-full','Both forty-brick buffer positions occupied; downstream interlock visible',{stationId:st.id,lane:st.lane,layerId:a.id,bufferBLayer:b.id,until:finish});marked=true;}}}
 run.duration=end+5;run.events.sort((a,b)=>a.t-b.t||a.id.localeCompare(b.id));run.transactions.sort((a,b)=>a.t-b.t);run.zones.sort((a,b)=>a.start-b.start);
 for(const r of run.records){r.states.sort((a,b)=>a.t-b.t);delete r.p;}
 for(const h of Object.values(run.stationHistory))h.sort((a,b)=>a.t-b.t);run.skipHistory.sort((a,b)=>a.t-b.t);
 run.summary={inputBricks:28800,releasedBricks:brickSerial,goodBricks:totalGood,rejectedBricks:brickSerial-totalGood,layers:run.layers.length,completedPallets:run.pallets.filter(p=>p.completedAt!==null).length,storedPallets:run.pallets.filter(p=>p.storedAt!==undefined).length,loadedPallets:loaded.length,skipExchanges:skipBatches.length,neighborTransfers:run.records.filter(r=>r.kind==='brick'&&r.layerId&&r.good&&r.lane!==r.destinationLane).length,sourceBoundary:'finite north-arriving kiln cart; serialized40-layer claw and10-row extraction',durationSeconds:run.duration,throughputValidated:false,cartPartiallyLoaded:true};
 run.initialInput=28800;run.stations=stations.map(st=>({id:st.id,lane:st.lane,station:st.station,z:st.z}));return run;
}
function sample(run,time){
 let index=sampleIndexes.get(run);if(!index){index=new Map(run.records.map(r=>[r.id,r]));sampleIndexes.set(run,index);}
 const t=Math.max(0,Math.min(run.duration,time)),entities=[],counts={input:run.initialInput,unreleased:0,inProcess:0,goodStored:0,rejectInTransit:0,rejectInSkip:0,rejectRemoved:0,completedPallets:0,storedPallets:0,loadedPallets:0,shippedBricks:0,truckBricks:0};
 for(const r of run.records){const s=lastAt(r.states||[],t),state=s?s.state:(r.kind==='brick'?'source':'idle'),visible=t>=r.visibleFrom&&(r.hiddenAt===undefined||t<r.hiddenAt);if(r.kind==='brick'){if(state==='source')counts.unreleased++;else if(state==='skip')counts.rejectInSkip++;else if(state==='removed')counts.rejectRemoved++;else if(state==='reject_transit')counts.rejectInTransit++;else if(state==='stored')counts.goodStored++;else if(state==='truck')counts.truckBricks++;else if(state==='shipped')counts.shippedBricks++;else counts.inProcess++;}
  if(visible)entities.push({id:r.id,kind:r.kind,pose:pose(r,t),shade:r.shade,visible:true,state,lane:r.lane,station:r.station,supportTopOffsetY:r.supportTopOffsetY,loadCenterOffsetX:r.loadCenterOffsetX});
 }
 for(const p of run.pallets){if(p.completedAt!==null&&t>=p.completedAt)counts.completedPallets++;if(p.storedAt!==undefined&&t>=p.storedAt&&t<(p.warehousePickupAt||Infinity))counts.storedPallets++;if(p.loadedAt!==undefined&&t>=p.loadedAt)counts.loadedPallets++;}
 const stations=run.stations.map(st=>{
  const ls=run.layers.filter(l=>l.stationId===st.id),forming=ls.find(l=>t>=l.firstAdmission&&t<l.bufferAAt),a=ls.find(l=>t>=l.bufferAAt&&t<l.bufferBAt),b=ls.find(l=>t>=l.bufferBAt&&t<l.bufferReleasedAt),p=run.pallets.find(p=>p.stationId===st.id&&p.completedAt!==null&&t>=p.completedAt&&t<(p.gantryPickupAt||Infinity));
  const activeJob=run.pallets.find(p=>p.stationId===st.id&&t<(p.completedAt||Infinity))||run.pallets.filter(p=>p.stationId===st.id).slice(-1)[0],woodPose=pose(index.get(activeJob.wood),t),wellOccupied=Math.abs(woodPose[0]-42.335)<.01,stacked=wellOccupied?ls.filter(l=>l.palletId===activeJob.id&&t>=l.stackedAt).length:0;
  let n=0;if(forming)for(const id of forming.ids){const r=index.get(id),s=lastAt(r.states,t);if(s&&s.state==='forming')n++;}
  return{...st,forming:n,formingLayer:forming?.id||null,bufferA:a?.id||null,bufferB:b?.id||null,stackedLayers:stacked,stackedBricks:stacked*40,outfeed:p?.id||null,state:p?'completed pallet outfeed':b?'layer ready':a?'buffer A occupied':forming?'forming':'awaiting input',recipe:activeJob.recipe};
 });
 const skip={...(lastAt(run.skipHistory,t)||{}),fill:counts.rejectInSkip,capacity:run.parameters.skipCapacity,removed:counts.rejectRemoved};
 for(const e of entities.filter(e=>e.kind==='forklift')){const cargo=run.pallets.find(p=>{const s=lastAt(index.get(p.ids[0]).states,t);return s&&s.owner===e.id&&(s.state==='warehouse-forklift'||s.state==='side-loader');});e.cargoPalletId=cargo?.id||null;e.forkTopY=cargo?pose(index.get(cargo.wood),t)[1]+.025:.085;}
 counts.conservationBalance=counts.input-(counts.unreleased+counts.inProcess+counts.goodStored+counts.rejectInTransit+counts.rejectInSkip+counts.rejectRemoved+counts.shippedBricks+counts.truckBricks);
 const zones=run.zones.filter(z=>z.start<=t&&z.end>t);const gp=entities.find(e=>e.id==='gantry');return{time:t,entities,stations,counts,skip,gantry:{state:gp?.state||'idle',pose:gp?.pose,palletId:zones.find(z=>z.kind==='gantry')?.owner||null},zones,zoneowners:zones};
}
return{createRun,sample,pose,pathAt,constants:{brick:[L,H,W],bedY:B,palletSupportTopOffsetY:TOP}};
});
