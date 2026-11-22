"""Isolated target-layout USD preview, not a physical seating simulation."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--session',type=Path,default=Path(__file__).resolve().parents[1]/'simulation_variant_session.json')
    args=parser.parse_args();plan=json.loads(args.plan.read_text());session=json.loads(args.session.read_text());base=Path(session['output'])
    if (session['environment'],session['fingers'])!=('board','wide'):raise ValueError('Select board/wide before exporting a layer target preview')
    args.output.mkdir(parents=True,exist_ok=False)
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':True,'enable_crashreporter':False},experience=str(Path(__file__).with_name('physics_only.kit')))
    try:
        import numpy as np
        from pxr import Gf,Usd,UsdGeom,UsdPhysics
        from environments.geometry import mesh
        from export_viewer import export
        stage=Usd.Stage.Open(str(base/'digital_twin.usdc'));root='/World/LayerStackingPlanPreview';UsdGeom.Xform.Define(stage,root)
        def body(path,size,color):
            x,y,z=size;vertices=[(-x/2,-y/2,0),(x/2,-y/2,0),(x/2,y/2,0),(-x/2,y/2,0),
                (-x/2,-y/2,z),(x/2,-y/2,z),(x/2,y/2,z),(-x/2,y/2,z)]
            faces=[0,2,1,0,3,2,4,5,6,4,6,7,0,1,5,0,5,4,1,2,6,1,6,5,2,3,7,2,7,6,3,0,4,3,4,7]
            return mesh(stage,path,vertices,[3]*12,faces,color)
        count=0
        for layer in plan['layers']:
            for brick in layer['bricks']:
                path=root+'/'+brick['id'];parent=UsdGeom.Xform.Define(stage,path)
                parent.AddTransformOp().Set(Gf.Matrix4d(*np.array(brick['O_T_bottom']).T.reshape(-1).tolist()))
                color=(.15,.46,.75) if layer['index']%2==0 else (.18,.63,.45)
                size=plan['brick_body_size_m'];body(path+'/Body',size,color)
                for u in range(4):
                    for v in range(2):
                        cx=(u-1.5)*plan['stud_pitch_m'];cy=(v-.5)*plan['stud_pitch_m'];radius=plan['visual_stud_diameter_m']/2
                        points=[(cx+radius*math.cos(s*math.pi/8),cy+radius*math.sin(s*math.pi/8),z) for z in (size[2],size[2]+plan['visual_stud_height_m']) for s in range(16)]
                        faces=[list(range(15,-1,-1)),list(range(16,32))]+[[s,(s+1)%16,(s+1)%16+16,s+16] for s in range(16)]
                        mesh(stage,path+f'/Stud_{u}_{v}',points,list(map(len,faces)),[i for f in faces for i in f],color)
                count+=1
        assert count==12*len(plan['layers'])
        assert not any(p.HasAPI(UsdPhysics.CollisionAPI) for p in Usd.PrimRange(stage.GetPrimAtPath(root)))
        stage.Export(str(args.output/'layer_stack_preview.usdc'))
        manifest=json.loads((base/'manifest.json').read_text());manifest.update(mode='layer_target_geometry_preview',physical_contact_validated=False,
            layer_stacking=dict(layers=len(plan['layers']),bricks=count,target_center_stud=plan['target_center_stud'],source_plan_sha256=hashlib.sha256(args.plan.read_bytes()).hexdigest()))
        manifest['webgl_visual_monitor']=export(stage,manifest['link_names'],'/World/envs/env_0/Robot',args.output)
        (args.output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
        latest=json.loads((base/'latest.json').read_text());latest['stale']=True;latest['source']='layer_target_preview_not_live_hardware'
        (args.output/'latest.json').write_text(json.dumps(latest)+'\n')
        page=(args.output/'index.html').read_text().replace('FR3 · measured-state digital twin','Layer targets · geometry preview')
        page=page.replace('Read-only robot telemetry → Isaac PhysX → this WebGL monitor.','Blue/green: planned alternating twelve-brick layers. Static robot pose. No contact simulation or physical interlock verification.')
        (args.output/'index.html').write_text(page)
        (args.output/'plan.json').write_text(json.dumps(plan,indent=2)+'\n')
        print(json.dumps(manifest['layer_stacking']),flush=True)
    except Exception:
        import traceback;traceback.print_exc();raise
    finally:app.close(wait_for_replicator=False)


if __name__=='__main__':main()
