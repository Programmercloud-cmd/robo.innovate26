'use strict';
window.BuildOperationalShell=()=>{
const T=window.THREE,C=window.PROPOSED_LAYOUT,$=id=>document.getElementById(id);
const renderer=new T.WebGLRenderer({canvas:$('canvas'),antialias:true,preserveDrawingBuffer:true});renderer.setPixelRatio(Math.min(devicePixelRatio,1.5));renderer.outputColorSpace=T.SRGBColorSpace;
const scene=new T.Scene();scene.background=new T.Color(0xd6dedf);const camera=new T.PerspectiveCamera(42,1,.1,500);scene.add(new T.HemisphereLight(0xffffff,0x6b7c80,2.1));const light=new T.DirectionalLight(0xfff6e7,2.1);light.position.set(0,80,-25);scene.add(light);
const mat=(c,transparent=false,opacity=1)=>new T.MeshLambertMaterial({color:c,transparent,opacity,side:T.DoubleSide});const M={steel:mat(0x91a1a5),dark:mat(0x283c44),green:mat(0x3c625d),blue:mat(0x437f98),yellow:mat(0xc1aa52),wood:mat(0xa7916d),brick:mat(0xb89d76),floor:mat(0xc3cdcd),white:mat(0xe3e7de),red:mat(0xac6b58),route:mat(0x78a998),glass:mat(0x53717b,true,.18)};
const rejectMesh=window.RejectMeshFactory({T,scene,M});
const unit=new T.BoxGeometry(1,1,1),cyl=new T.CylinderGeometry(.5,.5,1,10),covers=new T.Group(),overhead=new T.Group();scene.add(covers,overhead);const labels=[],instances=new Map(),proof={cameraCount:0,bins:0,rejectPushers:0,slots:0,truckCabsEast:false};
function box(x,y,z,sx,sy,sz,m=M.steel,parent=scene){const o=new T.Mesh(unit,m);o.position.set(x,y,z);o.scale.set(sx,sy,sz);parent.add(o);return o}
function rod(x,y,z,r,len,m=M.steel,axis='y',parent=scene){const o=new T.Mesh(cyl,m);o.position.set(x,y,z);o.scale.set(2*r,len,2*r);if(axis==='x')o.rotation.z=Math.PI/2;if(axis==='z')o.rotation.x=Math.PI/2;parent.add(o);return o}
function beam(a,b,w=.06,m=M.steel,parent=scene){const av=new T.Vector3(...a),bv=new T.Vector3(...b),v=bv.clone().sub(av);const o=box(...av.clone().add(bv).multiplyScalar(.5),w,v.length(),w,m,parent);o.quaternion.setFromUnitVectors(new T.Vector3(0,1,0),v.normalize());return o}
function support(x,z,y,w=.09,parent=scene){box(x,y/2,z,w,y,w,M.steel,parent);box(x,.035,z,.26,.07,.26,M.dark,parent)}
function label(text,x,y,z){const e=document.createElement('div');e.className='label';e.textContent=text;$('labels').append(e);labels.push({e,p:new T.Vector3(x,y,z)})}
function arrow(a,b,color=0x36897e){const av=new T.Vector3(...a),v=new T.Vector3(...b).sub(av);scene.add(new T.ArrowHelper(v.clone().normalize(),av,v.length(),color,Math.min(.5,v.length()*.25),Math.min(.3,v.length()*.15)))}
function belt(x0,x1,z,w=.4,y=1.25,m=M.dark,legs=true){box((x0+x1)/2,y-.10,z,x1-x0,.17,w,M.green);box((x0+x1)/2,y-.012,z,x1-x0,.024,w-.025,m);for(const side of [-1,1])box((x0+x1)/2,y+.05,z+side*w/2,x1-x0,.06,.025,M.steel);if(legs)for(let x=x0+.25;x<x1;x+=3)for(const side of [-1,1])support(x,z+side*(w/2-.04),y-.16,.07)}
function rollers(x0,x1,z,w=.86,y=1.25,legs=true){box((x0+x1)/2,y-.13,z,x1-x0,.14,w,M.green);for(let x=x0+.04;x<x1;x+=.11)rod(x,y-.027,z,.027,w-.035,M.steel,'z');for(const side of [-1,1])box((x0+x1)/2,y-.02,z+side*w/2,x1-x0,.05,.025,M.steel);if(legs)for(const x of [x0+.1,x1-.1])for(const side of [-1,1])support(x,z+side*(w/2-.06),y-.18,.07)}
function flatPath(points,w,m=M.dark,rail=.10){rejectMesh.conveyor(points,w,m,rail)}
function routingDeck(x0,x1,z,w=4){box((x0+x1)/2,1.13,z,x1-x0,.20,w,M.green);rejectMesh.face([[x0,1.25,z-w/2],[x1,1.25,z-w/2],[x1,1.25,z+w/2],[x0,1.25,z+w/2]],M.route,[0,1,0],'green-rail-free-deck');for(const x of [x0+.2,x1-.2])for(const side of [-1,1])support(x,z+side*(w/2-.08),1.05,.075)}
function addInstance(x,y,z,sx,sy,sz,m=M.brick,yaw=0){if(!instances.has(m))instances.set(m,[]);instances.get(m).push({x,y,z,sx,sy,sz,yaw})}
function brick(){} // Material inventory is supplied only by the operational engine.
function pallet(){} // No illustrative static pallets or product loads.
function cameraHead(x,z,y=2.05,inspection=false){box(x,y,z,.18,.17,.17,M.dark);rod(x,y-.13,z,.05,.10,M.dark);for(const s of [-1,1])box(x,y-.2,z+s*.25,.4,.025,.08,M.white);if(inspection)proof.cameraCount++}
box(41.5,-.10,18,177,.16,96,M.floor);const grid=new T.GridHelper(177,177,0x9eafb0,0xb3bfc0);grid.scale.z=96/177;grid.position.set(41.5,-.014,18);scene.add(grid);
// Authorized rigid translation of the complete pre-inspection upstream block.
const upstreamSceneStart=scene.children.length,upstreamOverheadStart=overhead.children.length;
// Existing upstream frozen37 layout, shown as authored schematic geometry.
// Cart bodies and material are dynamic engine actors.
const U=C.upstreamCart;
if(U){const tr=U.track;for(const dx of tr.rail_x_offsets_m)for(const side of [-1,1])box(tr.center_x_m-C.upstreamTranslation+dx+side*tr.rail_gauge_m/2,.015,(tr.z_min_m+tr.z_max_m)/2,.065,.025,tr.z_max_m-tr.z_min_m,M.steel);}
if(U){const b=U.unloading_bridge_planning_bounds;for(const z of [b.z_min_m,b.z_max_m]){for(const x of [b.x_min_m,b.x_max_m])support(x-C.upstreamTranslation,z,b.nominal_overhead_y_m,.22,overhead);box((b.x_min_m+b.x_max_m)/2-C.upstreamTranslation,b.nominal_overhead_y_m-.1,z,b.x_max_m-b.x_min_m,.24,.22,M.green,overhead);}}
// Upstream handling grippers are engine actors.
C.groupZ.forEach((z,g)=>{belt(.2,22.5,z,2.4);for(let j=0;j<6;j++){const x=14+j*1.5;for(const s of [-1,1])support(x,z+s*3.55,3.3,.13);box(x,3.3,z,1,.16,7.25,M.green);}for(let j=0;j<4;j++)for(let r=0;r<4;r++)for(let c=0;c<10;c++)brick(3+j*1.3+(c-4.5)*.067,1.25+.0345,z-(r-1.5)*.245,Math.PI/2);arrow([4,1.42,z],[9,1.42,z])});
label('N/S KILN CARTS · ROTATING LAYER CLAWS',-20,3.2,0);label('ROW EXTRACTION & INSPECTION SUPPLY',16,4.2,0);
for(const o of scene.children.slice(upstreamSceneStart))o.position.x+=C.upstreamTranslation;
for(const o of overhead.children.slice(upstreamOverheadStart))o.position.x+=C.upstreamTranslation;
for(const poses of instances.values())for(const p of poses)p.x+=C.upstreamTranslation;
for(const q of labels)q.p.x+=C.upstreamTranslation;
// Compact unified enclosure, two original inspection cameras per lane.
C.laneZ.forEach((z,lane)=>{
 belt(12.8+C.upstreamTranslation,23+C.upstreamTranslation,z,.38);belt(23+C.upstreamTranslation,26.05+C.upstreamTranslation,z,.53,1.25,M.blue);rollers(26.05+C.upstreamTranslation,28.65+C.upstreamTranslation,z,.34);
 routingDeck(C.crossfeed[0],C.crossfeed[1],z);routingDeck(C.router[0],C.router[1],z);
 // Fanout directions are static suggestions, not simulation trajectories.
 for(const dz of [-1.2,0,1.2])arrow([C.router[0]+.35,1.31,z],[C.router[1]-.40,1.31,z+dz],0x38796c);
 for(const x of C.inspectionCameras){cameraHead(x,z,1.96,true);box(x,2.12,z,.13,.13,1.12,M.steel)}
 cameraHead(C.router[0]+3,z,2.66);box(C.router[0]+3,2.83,z,.13,.12,4.15,M.green);
 const cover=new T.Group();covers.add(cover);box((C.cover[0]+C.cover[1])/2,3.16,z,C.cover[1]-C.cover[0],.08,4.2,M.green,cover);
 for(const s of [-1,1]){box((C.cover[0]+C.cover[1])/2,2.34,z+s*2.05,C.cover[1]-C.cover[0],1.58,.045,M.green,cover);for(const x of C.cover)support(x,z+s*2.09,3.12,.085)}
 // Lower edge panels stop before local fanout, leaving the reject edge exit open.
 const inner=lane%2===0?1:-1; // No lower internal panel/rail obstructs cross-transfer or reject access.
 for(const dz of [-1.2,0,1.2]){
  const zz=z+dz;belt(34.25,34.40,zz,.86);belt(34.40,36.35,zz,1.06,1.25,M.route);rollers(36.285,38.985,zz,.86);
  // Frozen37 downstream module translated coherently by -10.665m.
  const wx=C.wellX;for(const xx of [wx-.64,wx+.64])for(const s of [-1,1])support(xx,zz+s*.50,1.55,.07);
  for(const s of [-1,1])box(wx,.7,zz+s*.50,1.3,1.2,.055,mat(0x3c625d,true,.22));box(wx,.12,zz,1.4,.18,1.08,M.dark);
  rollers(39.935,41.735,zz,.86,.164,false);rollers(43.135,C.outfeedEnd,zz,.92,.164,false);pallet(wx,zz,.224,lane%2===0&&dz===0);
  // Retention stop is driven by its engine actor.
  for(let r=0;r<4;r++)for(let c=0;c<10;c++)brick(C.formationCenter+(1.5-r)*.32,1.2845,zz+(c-4.5)*.1);
 }
 for(const xx of [37.535,43.335])for(const s of [-1,1])support(xx,z+s*1.9,3.55,.15);
 for(const s of [-1,1])box(40.435,3.5,z+s*1.9,6.1,.2,.16,M.green);// Portal carriage and layer tool are dynamic.

 // Defect arrows stay downstream of the upstream cap; geometry authored once below.
 const pairZ=C.groupZ[Math.floor(lane/2)],edgeZ=z+inner*2;arrow([C.router[0]+3.1,1.33,edgeZ-inner*.30],[C.router[0]+3.1,.80,pairZ-inner*.4],0xb16750);
});
// Shared upstream crossfeed across each paired lane; clear of lower reject collection.
C.groupZ.forEach(z=>{routingDeck(C.crossfeed[0],C.crossfeed[1],z,2);box(C.crossCenterX,3.16,z,1.3,.08,2.1,M.green,covers);cameraHead(C.crossCenterX,z,2.7);arrow([C.crossCenterX,1.33,z-2.5],[C.crossCenterX,1.33,z+2.5])});
rejectMesh.channels(C.rejectChannels);
// Only full-process outer perimeter rails; all internal bridge seams remain clear.
for(const z of [-15,15])box((C.crossfeed[0]+C.router[1])/2,1.30,z,C.router[1]-C.crossfeed[0],.06,.025,M.steel);
label('03  TRANSLATED INSPECTION + SUPPORTED CROSSFEED',C.crossCenterX,4.1,-10);label('9.5625m USABLE FORWARD · NO INTERMEDIATE BUFFER',29.2,4.1,10);label('18 GUIDED PALLET WELLS',42.3,4.4,0);
// Reject belts remain in the clear middle gap between paired wells/gantry legs.
C.groupZ.forEach(z=>{rejectMesh.conveyor([[C.reject.branchStart,.75,z],[34.25,.75,z]],.8,M.red,0,true);flatPath([[34.25,.75,z],[46.7,.75,z]],.8,M.red,.10);flatPath([[46.7,.75,z],[47,.4,z]],.8,M.red,.12);arrow([36,.88,z],[40,.88,z],0x9c5645)});
flatPath([[47,.4,-10.4],[47,.4,17]],.8,M.red,.14);arrow([47,.55,-6],[47,.55,-2],0x9c5645);arrow([47,.55,5],[47,.55,9],0x9c5645);
const rejectService=window.BuildRejectService({T,scene,C,box,rod,beam,support,arrow,label,M});
// Clear-span finished-goods bridge; no posts in reject branch paths.
for(const z of C.gantry.outer_post_z_m){for(const x of C.gantry.post_x_m)support(x,z,6.3,.24,overhead);box(61.77,6.3,z,36.465,.24,.24,M.green,overhead)}for(const x of C.gantry.post_x_m)box(x,6.4,0,.24,.24,33.4,M.green,overhead);
// Gantry carriage/tool driven by engine actor.

// Slots are supplied by the shared layout manifest, with real approach direction.
for(const slot of C.warehouse.slots){const x=slot.x,z=slot.z,sign=slot.approachSide==='-X'?-1:1;box(x,.012,z,1.55,.018,1.35,M.white);for(const s of [-1,1])box(x,.022,z+s*.69,1.57,.02,.035,M.blue);if(slot.index%7===0)pallet(x,z,.17,true);else pallet(x,z,.17,false);if(slot.index%4===0)arrow([x+sign*2.4,.12,z],[x+sign*.85,.12,z],0x3e869b);proof.slots++}
// Neutral slot registry. Four broad aisles / two cross aisles stay visually open.
label('05  NEUTRAL STORAGE · '+C.warehouse.slots.length+' GROSS PLANNING SLOTS',65,3.5,0);
for(const a of [...C.warehouse.aisles,...C.warehouse.crossAisles]){box((a.x_min_m+a.x_max_m)/2,.014,(a.z_min_m+a.z_max_m)/2,a.x_max_m-a.x_min_m,.014,a.z_max_m-a.z_min_m,mat(0x79a8b0,true,.20))}
for(const st of C.staging){} // Legacy dispatch boundary; old rear-dock geometry intentionally absent.
const dispatchState=window.BuildDispatch({T,scene,C,box,rod,beam,support,arrow,label,brick,M});
// Batch repeated bricks; static means no replay/physics or old synthetic metrics.
for(const [m,poses] of instances){const mesh=new T.InstancedMesh(unit,m,poses.length),d=new T.Object3D();poses.forEach((p,i)=>{d.position.set(p.x,p.y,p.z);d.rotation.set(0,p.yaw,0);d.scale.set(p.sx,p.sy,p.sz);d.updateMatrix();mesh.setMatrixAt(i,d.matrix)});mesh.instanceMatrix.needsUpdate=true;scene.add(mesh)}

covers.visible=false;overhead.visible=false;dispatchState.canopy.visible=false;rejectService.groups.clearance.visible=false;
return {T,C,renderer,scene,camera,M,unit,cyl,covers,overhead,dispatchState,rejectService,labels,proof,box,rod,beam,support,label,pallet,brick,arrow};
};
