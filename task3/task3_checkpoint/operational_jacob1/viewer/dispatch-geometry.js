'use strict';
window.BuildDispatch=({T,scene,C,box,rod,beam,support,arrow,label,brick,M})=>{
const D=C.dispatch,canopy=new T.Group(),trucks=[],truckGates=[];scene.add(canopy);
const transparent=(color,opacity)=>new T.MeshLambertMaterial({color,opacity,transparent:true,side:T.DoubleSide,depthWrite:false});
const laneMaterial=transparent(0x739ba6,.23),gateMaterial=new T.MeshLambertMaterial({color:0xb99b4d}),roofMaterial=transparent(0x647d82,.12);
function rect(a,material=laneMaterial,y=.023){box((a.x_min_m+a.x_max_m)/2,y,(a.z_min_m+a.z_max_m)/2,a.x_max_m-a.x_min_m,.018,a.z_max_m-a.z_min_m,material)}
function line(x0,x1,z,m=M.yellow){box((x0+x1)/2,.038,z,x1-x0,.025,.055,m)}
function pack(){} // No static inventory; operational engine owns all pallet material.
function forklift(){} // Operational vehicle actors replace reference silhouettes.
// Frozen dispatch spine remains clear and connected; no OCF enters the loader cell.
const spine={x_min_m:79.3,x_max_m:83.7,z_min_m:-16.4,z_max_m:16.4};rect(spine);arrow([81.5,.08,-11],[81.5,.08,-5],0x3e869b);arrow([81.5,.08,6],[81.5,.08,11],0x3e869b);
label('FROZEN 4.4m DISPATCH SPINE',81.5,2.8,-15.8);
for(const [i,st] of D.staging_positions.entries()){
 const x=st.x_m,z=st.z_m;box(x,.025,z,1.4,.02,1.12,M.white);pack(x,z,0,i%4===0);
 // Each side reaches only its own handoff face, not the opposite vehicle lane.
 if(i%4===0){arrow([83.75,.13,z],[84.30,.13,z],0x3d8396);arrow([86.28,.13,z],[85.70,.13,z],0xa17739)}
}
forklift(82.35,-7.8,1,0,'warehouse',1.2);label('WAREHOUSE OCF-CLASS HANDOFF ONLY · WEST',81.5,3.6,-7.8);label('16 TWO-SIDED HANDOFF POSITIONS · EXCLUSIVE RESERVATIONS',85,3.8,0);
rect(D.north_forklift_cross_connection);rect(D.south_forklift_cross_connection);
for(const z of [-14.2,14.2]){arrow([87,.08,z],[94,.08,z],0x3d8396);arrow([100,.08,z],[111,.08,z],0x3d8396)}
for(const [index,b] of D.side_bays.entries()){
 const x=b.centre_x_m,z=b.centre_z_m,deck=b.trailer_deck_height_nominal_m;
 rect(b.west_forklift_work_lane);rect(b.east_forklift_work_lane);
 const truckStart=scene.children.length;
 //16.5m overall N/S silhouette,13.6m open-sided trailer reference, cab toward+Z.
 box(x,deck-.12,-1.45,2.55,.24,13.6,M.dark);box(x,deck-.025,-1.45,2.48,.05,13.50,M.wood);
 for(const dz of [-6.6,-5.4,4.4,6.9])for(const s of [-1,1])rod(x+s*1.10,.42,dz,.42,.22,M.dark,'x');
 box(x,1.60,6.8,2.50,2.25,2.9,M.green);box(x,2.12,7.65,2.53,.65,1.10,M.blue);
 // Curtain roof and end wall: both long faces OPEN. Curtains bunch near front.
 /* Trailer roof cut away for operational inspection. */box(x,2.57,-8.2,2.55,2.65,.06,M.green);
 for(const side of [-1,1]){box(x+side*1.29,2.57,4.9,.10,2.66,.75,M.blue);for(const zz of [-8.1,4.5])box(x+side*1.27,2.58,zz,.06,2.66,.06,M.steel)}
 // Only sample poses: two near-side rows, no full-capacity cargo array.
 for(const zz of [-5.8,-2.2,1.4])for(const side of [-1,1])pack(x+side*.61,zz,deck,true);
 for(const side of [-1,1])for(const zz of [-5.8,1.4])arrow([x+side*2.35,deck+.18,zz],[x+side*1.23,deck+.18,zz],0xae8139);
 const truckGroup=new T.Group();truckGroup.name='dispatch-truck-body-'+index;const truckParts=scene.children.slice(truckStart);scene.add(truckGroup);for(const p of truckParts)truckGroup.attach(p);trucks.push({group:truckGroup,referenceX:x,referenceZ:z});
 // Closed crossing gates depict the IMMOBILIZED loading reference state.
 for(const zz of [-11.5,11.5]){for(const xx of [x-1.5,x+1.5])support(xx,zz,1.1,.08);const bar=box(x,.94,zz,3.0,.10,.06,gateMaterial);truckGates.push({bar,bay:index,closedX:x});}
 arrow([x,.10,-24],[x,.10,-18],0x618378);arrow([x,.10,18],[x,.10,21],0x618378);
 label(index===0?'BAY 1 · ACTIVE':'BAY 2 · FLEX',x,4.8,5.4);
 label('BOTH SIDES OPEN · NEAR-SIDE PLACEMENT',x,4.8,-5.6);
}
// Dedicated loader east of the handoff; forks lift to a near-side trailer row.
//Mast atX90.21, blade end91.41, serving west sample centred91.39 only.
forklift(89.27,-5.8,1,1.21,'loader',1.2);label('DEDICATED LOADER CANDIDATE · VISIONNAV-CLASS PRECEDENT',88.5,4.0,-8.7);
const roof=D.canopy_reservation,y=C.dispatchCanopyUndersideY;
for(const x of [roof.x_min_m,roof.x_max_m])for(const z of [-11.7,11.7])support(x,z,y,.16,canopy);
for(const z of [-11.7,11.7])box((roof.x_min_m+roof.x_max_m)/2,y,z,roof.x_max_m-roof.x_min_m,.18,.16,M.green,canopy);
box((roof.x_min_m+roof.x_max_m)/2,y+.10,0,roof.x_max_m-roof.x_min_m,.08,24,roofMaterial,canopy);
label('COVERED LOADING CELL · 5.8m PLANNING CLEARANCE / STRUCTURE TBD',102,7,-10.5);
// Shared-manifest driver refuge; no separate load-securing destination.
const refuge=D.driver_parking_and_pedestrian_refuge_reservation;
rect(refuge,transparent(0xd1b86d,.32));label('DRIVER REFUGE / CONTROLLED ACCESS',(refuge.x_min_m+refuge.x_max_m)/2,2.0,(refuge.z_min_m+refuge.z_max_m)/2);
for(const z of [-10,1,12]){box(120,1,z,.10,2,.10,M.yellow);box(124,1,z,8,.05,.04,M.yellow)}
label('GATED STATES: TRUCK MOVE / FORKLIFT LOAD · NO SIMULTANEOUS CROSSING',101,3.0,15.8);
return{canopy,trucks,truckGates,proof:{handoffPositions:16,bays:2,assignment:'1active+1flex',referencePacksPerTrailer:0,bothSidesOpen:true,loadingHeightReferenceM:1.21,warehouseFleetEntersTruckCell:false,dispatchSpinePreserved:true}};
};
