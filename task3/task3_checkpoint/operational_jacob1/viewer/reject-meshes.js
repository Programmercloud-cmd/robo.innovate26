'use strict';
// Pure explicit-corner geometry; no quaternion alignment or rotated-box skin.
window.RejectMeshMath=(()=>{
const sub=(a,b)=>a.map((v,i)=>v-b[i]),cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]],dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0);
function normal(v){const n=cross(sub(v[1],v[0]),sub(v[2],v[0])),l=Math.hypot(...n);if(l<1e-10)throw Error('Degenerate face');return n.map(a=>a/l)}
function orient(v,want){const n=normal(v);return dot(n,want)<0?[...v].reverse():v.map(a=>a.slice())}
function validate(v,want){const n=normal(v),planarity=Math.max(...v.map(p=>Math.abs(dot(n,sub(p,v[0])))));if(planarity>1e-8||dot(n,want)<.0001)throw Error('Invalid face planarity/winding');return{normal:n,planarity,vertices:v}}
function conveyorCorners(a,b,w){const dx=b[0]-a[0],dz=b[2]-a[2];if(Math.abs(dx)>1e-8&&Math.abs(dz)>1e-8)throw Error('Non-axis-aligned conveyor plan');const t=Math.abs(dx)>1e-8?[0,0,w/2]:[w/2,0,0];const plus=(p,s)=>p.map((v,i)=>v+s*t[i]);return orient([plus(a,-1),plus(a,1),plus(b,1),plus(b,-1)],[0,1,0])}
function boxFaces(b){const x=b.x_min_m,X=b.x_max_m,y=b.y_min_m,Y=b.y_max_m,z=b.z_min_m,Z=b.z_max_m;return[
 {v:[[x,y,z],[x,Y,z],[x,Y,Z],[x,y,Z]],n:[-1,0,0]}, {v:[[X,y,z],[X,y,Z],[X,Y,Z],[X,Y,z]],n:[1,0,0]},
 {v:[[x,Y,z],[X,Y,z],[X,Y,Z],[x,Y,Z]],n:[0,1,0]}, {v:[[x,y,z],[x,y,Z],[X,y,Z],[X,y,z]],n:[0,-1,0]},
 {v:[[x,y,z],[X,y,z],[X,Y,z],[x,Y,z]],n:[0,0,-1]}, {v:[[x,y,Z],[x,Y,Z],[X,Y,Z],[X,y,Z]],n:[0,0,1]}].map(f=>({v:orient(f.v,f.n),n:f.n}))}
return{normal,orient,validate,conveyorCorners,boxFaces};})();
window.RejectMeshFactory=({T,scene,M})=>{
const Q=window.RejectMeshMath,receipts=[],red=new T.MeshLambertMaterial({color:0xac6b58,side:T.FrontSide});
function face(vertices,material,want=[0,1,0],name='face'){
 const v=Q.orient(vertices,want),qa=Q.validate(v,want),g=new T.BufferGeometry();g.setAttribute('position',new T.Float32BufferAttribute(v.flat(),3));g.setIndex([0,1,2,0,2,3]);g.computeVertexNormals();const mesh=new T.Mesh(g,material);mesh.name=name;scene.add(mesh);receipts.push({name,...qa});return mesh;
}
function panel(vertices,name){return face(vertices,red,[0,1,0],name)}
function conveyor(points,w,material,rail=.10,omitTop=false){for(let i=1;i<points.length;i++){
 const v=Q.conveyorCorners(points[i-1],points[i],w);if(!omitTop)face(v,material,[0,1,0],'axis-belt-top');const bot=v.map(p=>[p[0],p[1]-.12,p[2]]);face(bot,M.green,[0,-1,0],'axis-belt-bottom');
 const center=v.reduce((a,p)=>a.map((n,j)=>n+p[j]/4),[0,0,0]);for(let k=0;k<4;k++){const j=(k+1)%4,edge=[v[k],v[j],bot[j],bot[k]],mid=v[k].map((n,i)=>(n+v[j][i])/2),out=[mid[0]-center[0],0,mid[2]-center[2]];face(edge,M.green,out,'axis-belt-side')}
 // Only longitudinal side rails: transverse ends remain open for transfer.
 const a=points[i-1],b=points[i],alongX=Math.abs(b[0]-a[0])>1e-8;
 if(rail>0)for(const side of [-1,1]){const offset=alongX?[0,0,side*w/2]:[side*w/2,0,0],lo=a.map((n,j)=>n+offset[j]),hi=b.map((n,j)=>n+offset[j]),out=offset;face([lo,hi,[hi[0],hi[1]+rail,hi[2]],[lo[0],lo[1]+rail,lo[2]]],M.steel,out,'axis-belt-rail')}
 }}
function channels(config){for(const ch of config.channels){panel(ch.west_z_chamfer_vertices_xyz,ch.id+'-north-chamfer');panel(ch.east_z_chamfer_vertices_xyz,ch.id+'-south-chamfer');panel(ch.bottom_belt_vertices_xyz,ch.id+'-flat-bottom');for(const f of Q.boxFaces(ch.upstream_retaining_wall))face(f.v,red,f.n,ch.id+'-upstream-wall')}}
return{face,conveyor,channels,receipts};};
