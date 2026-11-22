"""Bounded generic synthetic pair; force coupling only, no native clutch/robot.

The pair collision alone is excluded. Both bodies remain dynamic and collide
with the support. Equal/opposite forces act at the same upper-COM world point;
the lower moment and power therefore match the axial coordinate derivative.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import numpy as np
from effective_axial import EffectiveAxialModel,ModelDomainError

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument("--fixture",type=Path,default=Path(__file__).with_name("synthetic_fixture.json"))
parser.add_argument("--output",type=Path,required=True)
parser.add_argument("--dt",type=float,default=.0005)
parser.add_argument("--duration",type=float,default=1.)
args=parser.parse_args()
cfg=json.loads(args.fixture.read_text())
if cfg.get("parameter_status")!="synthetic_validation_fixture" or cfg.get("real_ready") or cfg.get("training_ready"):
    raise ValueError("runner accepts explicitly synthetic validation fixtures only")
if not 0<args.dt<=.001 or not 0<args.duration<=2.:
    raise ValueError("bounded diagnostic timestep/duration required")
step_count=round(args.duration/args.dt)
if not 1<=step_count<=10000:
    raise ValueError("software diagnostic budget is 1–10000 steps; not a physical parameter")
model=EffectiveAxialModel(cfg)
args.output.mkdir(parents=True,exist_ok=False)
from isaacsim import SimulationApp
app=SimulationApp({"headless":True,"enable_crashreporter":False},experience=str(Path(__file__).with_name("physics_only.kit")))
import omni.usd
import omni.physx
import omni.physics.tensors as tensors
from pxr import Gf,UsdGeom,UsdPhysics,PhysxSchema
stage=omni.usd.get_context().get_stage()
UsdGeom.SetStageMetersPerUnit(stage,1.)
UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z)
UsdGeom.Xform.Define(stage,"/World")
scene=UsdPhysics.Scene.Define(stage,"/World/Physics")
scene.CreateGravityMagnitudeAttr(9.81)
scene.CreateGravityDirectionAttr(Gf.Vec3f(0,0,-1))
ps=PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
ps.CreateEnableGPUDynamicsAttr(False);ps.CreateSolverTypeAttr("PGS")
material=UsdPhysics.MaterialAPI.Apply(stage.DefinePrim("/World/SupportMaterial","Material"))
material.CreateStaticFrictionAttr(0.);material.CreateDynamicFrictionAttr(0.);material.CreateRestitutionAttr(0.)
from pxr import UsdShade
geometry=cfg["synthetic_geometry"];size=geometry["body_size_m"];support_size=geometry["support_size_m"]
for name,dimensions,position,dynamic in [
    ("Support",support_size,[0,0,-support_size[2]/2],False),
    ("Lower",size,[0,0,size[2]/2],True),
    ("Upper",size,[0,0,size[2]/2+cfg["contact_center_gap_m"]+geometry["initial_extra_gap_m"]],True)]:
    cube=UsdGeom.Cube.Define(stage,"/World/"+name)
    cube.CreateSizeAttr(1.)
    cube.AddTranslateOp().Set(Gf.Vec3d(*position))
    cube.AddScaleOp().Set(Gf.Vec3d(*dimensions))
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(UsdShade.Material(material.GetPrim()),materialPurpose="physics")
    PhysxSchema.PhysxCollisionAPI.Apply(cube.GetPrim()).CreateContactOffsetAttr(1e-5)
    PhysxSchema.PhysxCollisionAPI.Apply(cube.GetPrim()).CreateRestOffsetAttr(0.)
    if dynamic:
        UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
        UsdPhysics.MassAPI.Apply(cube.GetPrim()).CreateMassAttr(geometry["body_mass_kg"])
        PhysxSchema.PhysxRigidBodyAPI.Apply(cube.GetPrim()).CreateSleepThresholdAttr(0.)
UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath("/World/Lower")).CreateFilteredPairsRel().SetTargets(["/World/Upper"])
physics=omni.physx.get_physx_interface();simulation=omni.physx.get_physx_simulation_interface()
simulation.attach_stage(omni.usd.get_context().get_stage_id());physics.force_load_physics_from_usd()
view=tensors.create_simulation_view("numpy");view.set_subspace_roots("/")
lower=view.create_rigid_body_view("/World/Lower");upper=view.create_rigid_body_view("/World/Upper")
indices=np.array([0],dtype=np.uint32)
rows=[];dissipation=0.;connector_work=0.;maximum_power_error=0.;previous_energy=None;final_energy=None;guard_stop=None
for step in range(step_count):
    lp=lower.get_transforms()[0].astype(float);up=upper.get_transforms()[0].astype(float)
    lv=lower.get_velocities()[0].astype(float);uv=upper.get_velocities()[0].astype(float)
    x,y,z,w=lp[3:];axis=np.array([2*(x*z+y*w),2*(y*z-x*w),1-2*(x*x+y*y)])
    separation=up[:3]-lp[:3]
    gap=float(np.dot(axis,separation));depth=cfg["contact_center_gap_m"]-gap
    depth_rate=-float(np.dot(axis,uv[:3]-lv[:3])+np.dot(np.cross(lv[3:],axis),separation))
    xy_error=float(np.linalg.norm(separation-axis*gap))
    relative_quat=Gf.Rotation(Gf.Quatd(lp[6],*lp[3:6])).GetInverse()*Gf.Rotation(Gf.Quatd(up[6],*up[3:6]))
    rotation=Gf.Matrix3d(1.).SetRotate(relative_quat)
    tilt=math.acos(float(np.clip(rotation[2][2],-1.,1.)))
    yaw=math.atan2(float(rotation[1][0]),float(rotation[0][0]))
    try:
        model.check_admission(xy_error,yaw,tilt)
        response=model.evaluate(depth,depth_rate)
    except ModelDomainError as error:
        guard_stop={"reason":str(error),"step":step,"time_s":step*args.dt,"depth_m":depth,"xy_error_m":xy_error,"yaw_rad":yaw,"tilt_rad":tilt,"lower_pose_xyzw":lp.tolist(),"upper_pose_xyzw":up.tolist(),"no_force_applied_after_rejection":True}
        print("SYNTHETIC_PAIR_DOMAIN_GUARD",json.dumps(guard_stop),flush=True)
        break
    pair_force=axis*response["force_N"]
    lower_moment=np.cross(separation,-pair_force)
    pair_power=float(np.dot(pair_force,uv[:3])+np.dot(-pair_force,lv[:3])+np.dot(lower_moment,lv[3:]))
    maximum_power_error=max(maximum_power_error,abs(pair_power-response["connector_power_W"]))
    external_fz=sum(item["upper_fz_N"] for item in cfg["synthetic_external_force_schedule"] if item["from_s"]<=step*args.dt<item["until_s"])
    point=np.asarray([up[:3]],dtype=np.float32)
    upper.apply_forces_and_torques_at_position(np.asarray([pair_force+[0,0,external_fz]],dtype=np.float32),None,point,indices,True)
    lower.apply_forces_and_torques_at_position(np.asarray([-pair_force],dtype=np.float32),None,point,indices,True)
    connector_work+=pair_power*args.dt;dissipation+=response["dissipation_W"]*args.dt
    if previous_energy is None:previous_energy=response["stored_energy_J"]
    rows.append({"step":step,"time_s":step*args.dt,"depth_m":depth,"depth_rate_m_s":depth_rate,"response":response,"upper_external_fz_N":external_fz,"pair_force_upper_N":pair_force.tolist(),"pair_force_lower_N":(-pair_force).tolist(),"shared_world_point_m":up[:3].tolist(),"lower_moment_Nm":lower_moment.tolist(),"pair_power_W":pair_power,"lower_pose_xyzw":lp.tolist(),"upper_pose_xyzw":up.tolist(),"lower_velocity":lv.tolist(),"upper_velocity":uv.tolist(),"xy_error_m":xy_error,"tilt_rad":tilt,"yaw_rad":yaw})
    simulation.simulate(args.dt,step*args.dt);simulation.fetch_results();physics.update_transformations(False,True,True,False)
    next_lower=lower.get_transforms()[0].astype(float);next_upper=upper.get_transforms()[0].astype(float)
    x,y,z,w=next_lower[3:];next_axis=np.array([2*(x*z+y*w),2*(y*z-x*w),1-2*(x*x+y*y)])
    next_depth=cfg["contact_center_gap_m"]-float(np.dot(next_axis,next_upper[:3]-next_lower[:3]))
    try:
        final_energy=model.evaluate(next_depth,0.)["stored_energy_J"]
    except ModelDomainError as error:
        final_energy=None
        guard_stop={"reason":str(error),"phase":"post_step","step":step,"time_s":(step+1)*args.dt,"depth_m":next_depth,"no_force_applied_after_rejection":True}
    rows[-1]["depth_after_step_m"]=next_depth
    rows[-1]["potential_after_step_J"]=final_energy
    if guard_stop is not None:
        print("SYNTHETIC_PAIR_DOMAIN_GUARD",json.dumps(guard_stop),flush=True)
        break
    if step%max(1,round(.1/args.dt))==0:print("SYNTHETIC_FORCE_PAIR",step,"depth_m",depth,"force_N",response["force_N"],flush=True)
with (args.output/"trace.ndjson").open("w") as stream:
    for row in rows:stream.write(json.dumps(row)+"\n")
stage.Export(str(args.output/"scene.usdc"))
potential_change=final_energy-previous_energy if rows and final_energy is not None else None
result={"completed":guard_stop is None,"status":"completed_bounded_synthetic_diagnostic" if guard_stop is None else "domain_guard_stop","guard_stop":guard_stop,"synthetic_fixture_only":True,"real_ready":False,"training_ready":False,"dt_s":args.dt,"steps":len(rows),"depth_range_m":[min(r["depth_m"] for r in rows),max(r["depth_m"] for r in rows)] if rows else None,"max_pair_power_identity_error_W":maximum_power_error,"connector_work_J":connector_work,"dissipation_J":dissipation,"potential_change_J":potential_change,"discrete_connector_balance_residual_J":connector_work+dissipation+potential_change if potential_change is not None else None,"pair_collision_exclusion_only":True,"lower_body_dynamic":True,"joints_created":0,"post_reset_pose_writes":0,"capture_jam_yaw_tilt":"unsupported states rejected; no capture or jam physics","rate_independent_hysteresis":False,"hardware_clients":0,"source_sha256":{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [Path(__file__),Path(__file__).with_name("effective_axial.py"),args.fixture]}}
(args.output/"result.json").write_text(json.dumps(result,indent=2)+"\n")
print(json.dumps(result,indent=2),flush=True)
# Release tensor backends while their PhysX plugin is still loaded. Keeping
# these wrappers until Python GC after Kit shutdown can dereference unloaded
# native code. This changes teardown only, never the synthetic force law.
del lower,upper,view
import gc
gc.collect()
simulation.detach_stage();app.close(wait_for_replicator=False)
raise SystemExit(2 if guard_stop is not None else 0)
