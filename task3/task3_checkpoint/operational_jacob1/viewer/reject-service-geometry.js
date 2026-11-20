'use strict';
// Static fill/stowed alternatives only. No moving-body or control simulation.
window.BuildRejectService=({T,scene,C,box,rod,beam,support,arrow,label,M})=>{
 const R=C.rejectService,I=R.incline,H=R.swing_away_discharge,B=R.container,S=R.service;
 const groups={};for(const name of ['fixed','extended','lifted','retracted','skip','clearance']){const g=new T.Group();g.name='reject-service-'+name;scene.add(g);groups[name]=g;}
 const receipts=[];
 function named(o,name,role){o.name=name;o.userData.rejectRole=role;return o;}
 function b(x,y,z,sx,sy,sz,m,g,name){return named(box(x,y,z,sx,sy,sz,m,g),name,g.name);}
 function rr(x,y,z,r,len,m,axis,g,name){return named(rod(x,y,z,r,len,m,axis,g),name,g.name);}
 const fixedMesh=window.RejectMeshFactory({T,scene:groups.fixed,M});
 fixedMesh.conveyor([[I.x_m,I.y_start_m,I.z_start_m],[I.x_m,I.y_end_m,I.z_end_m]],I.width_m,M.red,.18);
 for(const z of I.support_z_m){const y=I.y_start_m+(z-I.z_start_m)*Math.tan(I.angle_degrees*Math.PI/180)-.12;const start=groups.fixed.children.length;support(I.x_m,z,y,.12,groups.fixed);groups.fixed.children.slice(start).forEach((o,j)=>named(o,'incline-support-'+z+'-'+j,'fixed-support'));}
 // Horizontal module rotates as one rigid unit about the fixed intake pivot.
 const zi=H.pivot_z_m,x=H.pivot_x_m,y=H.belt_top_y_m,length=H.rigid_length_m;
 // East-side mast is outside every westward swing configuration. The overhead
 // cantilever/yoke and central lift shaft are schematic engineered interfaces.
 b(x+.9,1.37,zi,.14,2.74,.14,M.steel,groups.fixed,'fixed-east-lift-mast');
 b(x+.9,.035,zi,.26,.07,.26,M.dark,groups.fixed,'fixed-east-lift-mast-base');
 b(x+.45,2.75,zi,1.04,.12,.14,M.steel,groups.fixed,'fixed-overhead-lift-yoke');
 rr(x,2.66,zi,.055,.26,M.steel,'y',groups.fixed,'fixed-concept-lift-shaft-interface');
 function head(group,name){
  group.position.set(x,0,zi);
  b(0,y-.125,length/2,H.width_m,.19,length,M.green,group,name+'-rigid-frame');
  b(0,y-.012,length/2,H.width_m-.02,.024,length,M.red,group,name+'-belt');
  for(const sign of [-1,1]){
   b(sign*.46,y-.02,length/2,.08,.20,length,M.steel,group,name+'-rigid-side-'+sign);
   b(sign*.4,y+.09,length/2,.025,.18,length,M.steel,group,name+'-containment-'+sign);
  }
  rr(0,y-.08,length-.065,.06,H.width_m,M.dark,'x',group,name+'-nose-roller');
  b(0,y-.125,length-.015,H.width_m,.19,.03,M.green,group,name+'-endplate');
 }
 head(groups.extended,'fill-head');head(groups.lifted,'lifted-head');head(groups.retracted,'stowed-head');groups.lifted.position.y=H.lift_stroke_m;groups.retracted.position.y=H.lift_stroke_m;groups.retracted.rotation.y=H.stow_rotation_y_rad;
 // Heavy-rubble body envelope, not a reverse-engineered or volume-qualified SKU.
 const cx=B.center_x_m,cz=B.center_z_m,W=B.width_x_m,L=B.length_z_m,fy=B.floor_top_y_m,top=B.external_body_rim_y_m;
 b(cx,fy-.045,cz,W,.09,L,M.blue,groups.skip,'skip-floor');
 for(const sign of [-1,1]){
  b(cx+sign*(W/2-.035),(top+fy)/2,cz,.07,top-fy,L,M.blue,groups.skip,'skip-long-wall-'+sign);
  b(cx,(top+fy)/2,cz+sign*(L/2-.035),W,top-fy,.07,M.blue,groups.skip,'skip-end-wall-'+sign);
  b(cx+sign*(W/2-.045),top-.035,cz,.09,.07,L,M.steel,groups.skip,'skip-top-rim-'+sign);
  b(cx+sign*.56,.08,cz,.14,.16,L-.3,M.dark,groups.skip,'skip-underframe-runner-'+sign);
  rr(cx+sign*.82,.105,B.z_min_m+.15,.105,.28,M.dark,'x',groups.skip,'skip-rear-roller-'+sign);
  for(let j=0;j<7;j++)b(cx+sign*(W/2-.03),.56,B.z_min_m+.3+j*.85,.06,.78,.07,M.steel,groups.skip,'skip-wall-reinforcement-'+sign+'-'+j);
 }
 // South-facing pickup mast, its entire crossbar and hook stay within body bounds.
 const mz=B.z_max_m-.19;
 for(const sign of [-1,1])b(cx+sign*.36,(B.hook_mast_top_y_m+.20)/2,mz,.09,B.hook_mast_top_y_m-.20,.12,M.steel,groups.skip,'skip-hook-mast-'+sign);
 rr(cx,B.hook_mast_top_y_m-.055,mz,.055,.80,M.steel,'x',groups.skip,'skip-hook-pickup-bar');
 // A protruding south-facing lug, still within the manifest outer footprint.
 b(cx,1.40,B.z_max_m-.08,.13,.24,.15,M.steel,groups.skip,'skip-hook-lug');
 // Reject material is rendered only from engine records.
 const clearM=new T.MeshLambertMaterial({color:0xe2b76d,transparent:true,opacity:.07,depthWrite:false,side:T.DoubleSide});
 const A=S.lift_exclusion_reservation,P=S.south_approach_reservation;
 const reservation=b((A.x_min_m+A.x_max_m)/2,(A.y_min_m+A.y_max_m)/2,(A.z_min_m+A.z_max_m)/2,A.x_max_m-A.x_min_m,A.y_max_m-A.y_min_m,A.z_max_m-A.z_min_m,clearM,groups.clearance,'planning-lift-reservation-not-swept-path');
 const edges=new T.LineSegments(new T.EdgesGeometry(new T.BoxGeometry(A.x_max_m-A.x_min_m,A.y_max_m-A.y_min_m,A.z_max_m-A.z_min_m)),new T.LineBasicMaterial({color:0xb67e31,transparent:true,opacity:.5}));edges.position.copy(reservation.position);groups.clearance.add(edges);
 b((P.x_min_m+P.x_max_m)/2,.025,(P.z_min_m+P.z_max_m)/2,P.x_max_m-P.x_min_m,.025,P.z_max_m-P.z_min_m,new T.MeshLambertMaterial({color:0xcca85d,transparent:true,opacity:.24}),groups.clearance,'south-truck-approach-reservation');
 // Ground route is a direction marker, not a model of a selected hooklift truck.
 arrow([47,.16,45],[47,.16,33],0xad7d35);
 label('RECYCLING SKIP · REMOVABLE HOOKLIFT BODY',47,2.0,27.5);
 label('SOUTH EXCHANGE APPROACH · TURNING / STAGING TBD',47,.9,42);
 label('1.8m HEAD · LIFT THEN SWING WEST BEFORE PICKUP',47,2.6,21.2);
 groups.clearance.visible=false;
 const state={groups,receipts:[...fixedMesh.receipts],state:'fill',proof:{staticStatesOnly:true,controlsImplemented:false,sweptPathValidated:false,nominalSkipClassM3:B.nominal_capacity_class_m3,bodyReference:B.reference_url,headNoses:{fill:H.fill_nose_xyz_m,lifted:H.lifted_fill_nose_xyz_m,exchange:H.stowed_nose_xyz_m},reservedPickupEnvelope:A,upstreamMustPauseDuringExchange:true},setState(name){if(!['fill','lifted','exchange'].includes(name))throw Error('Unknown static reject state');state.state=name;groups.extended.visible=name==='fill';groups.lifted.visible=name==='lifted';groups.retracted.visible=name==='exchange';return name;}};
 state.setState('fill');return state;
};
