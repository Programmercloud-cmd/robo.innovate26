"""Persistent visual-only LEGO display; raw observations and controller evidence separate."""
import json
import time
from pathlib import Path

COLORS={'300121':(.72,.025,.035),'6117418':(.31,.035,.055),'6487946':(.59,.26,.08),
        '6135191':(.77,.66,.46),'4211201':(.18,.09,.045)}


def multiply(a,b):
    return [sum(a[k*4+i]*b[j*4+k] for k in range(4)) for j in range(4) for i in range(4)]


def translation(xyz):
    return [1.,0.,0.,0.,0.,1.,0.,0.,0.,0.,1.,0.,*xyz,1.]


class BrickDisplay:
    def __init__(self,stage,path):
        from pxr import UsdGeom
        self.stage,self.path=stage,Path(path)
        self.meshes={};self.packet=None;self.mtime=None;self.next_check=0.;self.scene_key=None
        self.status=dict(stale=True,visible_bricks=0,scope='perception_only')
        UsdGeom.Scope.Define(stage,'/World/PerceptionOnly')

    def cube(self,key,size,pose,color):
        from pxr import Gf,UsdGeom
        if key not in self.meshes:
            if not key.replace('_','').isalnum():return
            cube=UsdGeom.Cube.Define(self.stage,'/World/PerceptionOnly/'+key);cube.CreateSizeAttr(1.)
            self.meshes[key]=(cube,cube.AddTransformOp())
        cube,op=self.meshes[key];scaled=pose.copy()
        for axis in range(3):
            for row in range(3):scaled[axis*4+row]*=size[axis]
        op.Set(Gf.Matrix4d(*scaled));cube.CreateDisplayColorAttr([Gf.Vec3f(*color)]);cube.MakeVisible()

    def factory(self,geometry):
        from pxr import Gf,UsdGeom
        if not geometry:return
        key=json.dumps(geometry,sort_keys=True)
        if key==self.scene_key:return
        self.scene_key=key
        for name in geometry.get('replace_registered_names',[]):
            prim=self.stage.GetPrimAtPath('/World/RegisteredScene/'+name)
            if prim:UsdGeom.Imageable(prim).MakeInvisible()
        for item in geometry.get('polygons',[]):
            mesh=UsdGeom.Mesh.Define(self.stage,'/World/PerceptionOnly/Factory_'+item['name'])
            points=item['points_base_m'];mesh.CreatePointsAttr([Gf.Vec3f(*v) for v in points])
            mesh.CreateFaceVertexCountsAttr([len(points)]);mesh.CreateFaceVertexIndicesAttr(list(range(len(points))))
            mesh.CreateSubdivisionSchemeAttr('none');mesh.CreateDoubleSidedAttr(True)
            rgb=item.get('rgb',[.025,.025,.026]);mesh.CreateDisplayColorAttr([Gf.Vec3f(*rgb)])

    def update(self,tcp=None):
        if time.monotonic()<self.next_check:return
        self.next_check=time.monotonic()+.1
        try:
            stamp=self.path.stat().st_mtime_ns
            if stamp!=self.mtime:
                data=json.loads(self.path.read_text())
                if data.get('schema')!='fr3.external_layout.v1':raise ValueError('Invalid brick schema')
                self.packet,self.mtime=data,stamp
        except (OSError,ValueError):pass
        p=self.packet
        if not p:return
        age=(time.time_ns()-p['frame_unix_ns'])/1e9
        stale=p.get('stale',True) or not 0<=age<=1.
        self.status=dict(stale=stale,visible_bricks=0,retained_bricks=0,held_bricks=0,placed_bricks=0,
                         scope='display_memory_not_inventory',frame_unix_ns=p['frame_unix_ns'])
        for cube,_ in self.meshes.values():cube.MakeInvisible()
        for s in p.get('display_slots',p.get('slots',[]))+p.get('display_conveyor_slots',[]):
            color=COLORS.get(str(s.get('color_id')))
            if s.get('status')!='occupied' or color is None or not s.get('display_base_xyz_m'):continue
            if 'display_slots' not in p and (stale or s.get('visible') is not True):continue
            self.cube(s['slot_id'],s['display_size_m'],translation(s['display_base_xyz_m']),color)
            self.status['visible_bricks']+=1
            if s['slot_id'].startswith('conveyor_'):self.status['placed_bricks']+=1
            if stale or not s.get('confirmed_in_current_frame',False):self.status['retained_bricks']+=1
        for column in p.get('display_columns',[]):
            mode=column['mode']
            if mode in ('pending_grasp','departed_conveyor') or column.get('render_as_payload') is False:continue
            held=mode in ('held','held_unverified','releasing')
            for s in column['slots']:
                color=COLORS.get(str(s.get('color_id')))
                if color is None:continue
                pose=multiply(tcp,s['relative_to_tcp']) if held and tcp else s.get('placed_O_T_body')
                if pose is None:continue
                self.cube('payload_'+column['label']+'_'+s['slot_id'],s['display_size_m'],pose,color)
                self.status['held_bricks' if held else 'placed_bricks']+=1
        self.factory(p.get('display_registration',{}).get('factory_scene'))
