import * as THREE from 'three';

const COLORS={'300121':0xb80709,'6117418':0x4f090e,'6487946':0x974214,'6135191':0xc4a875,'4211201':0x2e170b};

export class BrickView {
  constructor(scene,tcpMatrix,staticMeshes){
    this.scene=scene;this.tcpMatrix=tcpMatrix;this.staticMeshes=staticMeshes;
    this.slots=new Map();this.columns=new Map();this.factoryMeshes=[];this.sceneKey=null;
    this.update();
  }
  body(id,size,map){
    let value=map.get(id);if(value)return value;
    const geometry=new THREE.BoxGeometry(...size);
    const solid=new THREE.Mesh(geometry,new THREE.MeshStandardMaterial({roughness:.7}));
    const outline=new THREE.LineSegments(new THREE.EdgesGeometry(geometry),new THREE.LineBasicMaterial({color:0xc48b18}));
    const group=new THREE.Group();group.add(solid,outline);this.scene.add(group);
    value={group,solid,outline};map.set(id,value);return value;
  }
  setFactory(geometry){
    if(!geometry)return;
    const key=JSON.stringify(geometry);if(key===this.sceneKey)return;this.sceneKey=key;
    for(const mesh of this.factoryMeshes){this.scene.remove(mesh);mesh.geometry.dispose();mesh.material.dispose()}
    this.factoryMeshes=[];
    const hidden=new Set(geometry.replace_registered_names||[]);
    for(const mesh of this.staticMeshes)if(hidden.has(mesh.name.split('/').pop()))mesh.visible=false;
    for(const item of geometry.polygons||[]){
      const vertices=item.points_base_m.flat();const indices=[];
      for(let i=1;i<item.points_base_m.length-1;i++)indices.push(0,i,i+1);
      const g=new THREE.BufferGeometry();g.setAttribute('position',new THREE.Float32BufferAttribute(vertices,3));g.setIndex(indices);g.computeVertexNormals();
      const color=item.rgb?new THREE.Color().setRGB(...item.rgb):new THREE.Color(item.color||0x090909);
      const mesh=new THREE.Mesh(g,new THREE.MeshStandardMaterial({color,side:THREE.DoubleSide,roughness:.8}));
      this.scene.add(mesh);this.factoryMeshes.push(mesh);
    }
  }
  apply(p){
    const age=Date.now()-p.frame_unix_ns/1e6,stale=p.stale||age>1000||age< -500;
    const slots=[...(p.display_slots||p.slots),...(p.display_conveyor_slots||[])];
    let occupied=0,empty=0,unknown=0,retained=0,conveyorOccupied=0,conveyorCurrent=0;
    for(const s of slots){
      if(!s.display_base_xyz_m||!s.display_size_m)continue;
      const g=this.body(s.slot_id,s.display_size_m,this.slots);g.group.position.set(...s.display_base_xyz_m);
      const known=s.status==='occupied'&&COLORS[String(s.color_id)]!==undefined;
      g.solid.visible=known;
      if(known){g.solid.material.color.setHex(COLORS[String(s.color_id)]);occupied++;if(s.slot_id.startsWith('conveyor_'))conveyorOccupied++}
      else if(s.status==='empty')empty++;else unknown++;
      const current=!stale&&(s.confirmed_in_current_frame===true||(!p.display_slots&&s.visible!==false));
      if(known&&current&&s.slot_id.startsWith('conveyor_'))conveyorCurrent++;
      if(known&&!current)retained++;
      g.outline.material.color.setHex(s.provenance==='source_vacated_by_controller_pick'?0x4287bf:
        current&&s.status==='empty'?0x4b9563:current&&known?0x575757:0xc48b18);
    }
    const active=new Set();let held=0,placed=0;
    for(const c of p.display_columns||[]){
      if(['pending_grasp','departed_conveyor'].includes(c.mode)||c.render_as_payload===false)continue;
      const carrying=['held','held_unverified','releasing'].includes(c.mode);
      if(carrying)held++;else placed++;
      for(const s of c.slots){
        const id=c.label+':'+s.slot_id;active.add(id);
        const g=this.body(id,s.display_size_m,this.columns);g.group.matrixAutoUpdate=false;g.group.visible=true;
        g.relative=s.relative_to_tcp;g.carrying=carrying;
        if(!carrying&&s.placed_O_T_body){g.group.matrix.fromArray(s.placed_O_T_body);g.group.matrixWorldNeedsUpdate=true}
        const known=COLORS[String(s.color_id)]!==undefined;g.solid.visible=known;
        if(known)g.solid.material.color.setHex(COLORS[String(s.color_id)]);
        g.outline.material.color.setHex(c.mode==='conveyor_verified'?0x4b9563:0x4287bf);
      }
    }
    for(const [id,g] of this.columns)if(!active.has(id))g.group.visible=false;
    this.setFactory(p.display_registration?.factory_scene);
    const status=document.querySelector('#brick-status');status.style.whiteSpace='pre-line';
    status.textContent=(stale?'Camera STALE · last confirmed display retained':'Live C920 03FB123F · '+Math.max(0,age).toFixed(0)+' ms')+
      '\n'+occupied+' occupied ('+retained+' retained/unverified) · '+empty+' empty · '+unknown+' unknown'+
      '\nColumns: '+held+' held · '+(p.display_conveyor_slots?conveyorOccupied+' conveyor bricks displayed ('+conveyorCurrent+' seen now)':placed+' on conveyor (controller-reported)')+
      '\nImage alignment: '+(p.registration?.valid?'matched':'uncertain; retaining prior colors')+
      '\n'+(p.display_registration?.status||'Scene mapping not registered');
    const reasons=new Set(p.slots.filter(s=>s.status==='uncertain').map(s=>s.reason));
    if([...reasons].some(r=>r&&r.includes('registration_changed')))status.textContent+='\nCamera registration changed · image mapping needs review';
    if(document.querySelector('#brick-debug').open)document.querySelector('#brick-overlay').src='brick_overlay.jpg?t='+Date.now();
  }
  async update(){
    try{const response=await fetch('brick_observations.json?t='+Date.now(),{cache:'no-store'});if(!response.ok)throw Error('Brick observations unavailable');
      const p=await response.json();if(p.schema!=='fr3.external_layout.v1')throw Error('Unsupported brick feed');this.apply(p);
    }catch(e){document.querySelector('#brick-status').textContent='Camera unavailable · retaining last confirmed display\n'+e.message}
    setTimeout(()=>this.update(),200);
  }
  animate(){
    const tcp=this.tcpMatrix();if(!tcp)return;
    for(const g of this.columns.values())if(g.carrying&&g.group.visible){g.group.matrix.copy(tcp).multiply(new THREE.Matrix4().fromArray(g.relative));g.group.matrixWorldNeedsUpdate=true}
  }
}
