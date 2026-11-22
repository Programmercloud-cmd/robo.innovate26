"""Bounded two-coupon native contact diagnostic. No robot, policy or hardware IO.

All dimensions/parameters are installed upstream defaults, not measurements.
Bottom-center positions are SI meters; quaternions are WXYZ in USD, XYZW in
PhysX tensor outputs. The lower coupon is dynamic on a static support surface.
"""
import argparse
import hashlib
import importlib
import json
import math
import sys
import types
from pathlib import Path

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--output", required=True, type=Path)
parser.add_argument("--case", choices=["no_force", "aligned_load", "misaligned_load", "too_high", "yaw_error", "assembly_disabled"], default="no_force")
parser.add_argument("--steps", type=int, default=600)
args = parser.parse_args()
args.output.mkdir(parents=True, exist_ok=False)
from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "enable_crashreporter": False},
                    experience=str(Path(__file__).with_name("physics_only.kit")))
import numpy as np
import omni.usd
import omni.physx
import omni.physics.tensors as tensors
from pxr import Gf, Usd, UsdGeom, UsdPhysics, PhysxSchema

# Load the installed native core without registering unrelated UI/RL extensions.
package = types.ModuleType("bricksim")
package.__path__ = ["/home/ubuntu/workspace/robo26-runtime-isaac51/.deps/bricksim"]
sys.modules["bricksim"] = package
core = importlib.import_module("bricksim.core")
stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageMetersPerUnit(stage, 1.)
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.Xform.Define(stage, "/World")
scene = UsdPhysics.Scene.Define(stage, "/World/Physics")
scene.CreateGravityDirectionAttr(Gf.Vec3f(0,0,-1))
scene.CreateGravityMagnitudeAttr(9.81)
PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim()).CreateEnableGPUDynamicsAttr(False)
PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim()).CreateSolverTypeAttr("PGS")
support = UsdGeom.Cube.Define(stage, "/World/Support")
support.CreateSizeAttr(1.)
support.AddTranslateOp().Set(Gf.Vec3d(0,0,-.01))
support.AddScaleOp().Set(Gf.Vec3d(.2,.2,.02))
UsdPhysics.CollisionAPI.Apply(support.GetPrim())
core.allocate_unmanaged_brick_part((4,2,3),(160,170,180),"/World/Lower")
core.allocate_unmanaged_brick_part((4,2,3),(180,70,50),"/World/Upper")
offset_x = .003 if args.case == "misaligned_load" else 0.
height = .025 if args.case == "too_high" else .0113
yaw = math.radians(10.) if args.case == "yaw_error" else 0.
for path, position, angle in [("/World/Lower",(0,0,0),0.),("/World/Upper",(offset_x,0,height),yaw)]:
    xform = UsdGeom.Xformable(stage.GetPrimAtPath(path))
    xform.ClearXformOpOrder()
    # Native graph writeback uses a double quaternion; author that exact schema
    # rather than retaining the prototype's float quaternion attribute.
    xform.GetPrim().RemoveProperty("xformOp:orient")
    xform.AddTranslateOp().Set(Gf.Vec3d(*position))
    xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(math.cos(angle/2),0,0,math.sin(angle/2)))
if args.case == "too_high":
    PhysxSchema.PhysxRigidBodyAPI.Apply(stage.GetPrimAtPath("/World/Upper")).CreateDisableGravityAttr(True)
assembly = core.get_assembly_thresholds()
if args.case == "assembly_disabled":
    assembly.enabled = False
    core.set_assembly_thresholds(assembly)
thresholds = {name:getattr(assembly,name) for name in ["enabled","distance_tolerance","max_penetration","z_angle_tolerance","required_force","yaw_tolerance","position_tolerance"]}
breakage = core.get_breakage_thresholds()
breakage_values = {name:getattr(breakage,name) for name in ["enabled","contact_regularization","clutch_axial_compliance","clutch_radial_compliance","clutch_tangential_compliance","friction_coefficient","preloaded_force","breakage_cooldown_time"]}
audit=[]
for p in Usd.PrimRange(stage.GetPseudoRoot(), Usd.TraverseInstanceProxies()):
    if p.HasAPI(UsdPhysics.CollisionAPI):
        audit.append({"path":str(p.GetPath()),"type":p.GetTypeName(),
                      "attributes":{a.GetName():str(a.Get()) for a in p.GetAttributes() if "physics" in a.GetName().lower() or "xform" in a.GetName().lower()},
                      "material_relationships":{r.GetName():list(map(str,r.GetTargets())) for r in p.GetRelationships() if "material" in r.GetName()}})
(args.output/"configuration.json").write_text(json.dumps({"case":args.case,"assembly":thresholds,"breakage":breakage_values,"colliders":audit,"parameter_provenance":"installed upstream defaults; unmeasured, not fitted","lower_fixture":"dynamic brick on static support; no lower-body weld or kinematic flag","native_binary_sha256":hashlib.sha256(Path(core.__file__).read_bytes()).hexdigest()},indent=2))
print("CONTACT_PROBE_AUTHORED",args.case,flush=True)
physics = omni.physx.get_physx_interface()
simulation = omni.physx.get_physx_simulation_interface()
simulation.attach_stage(omni.usd.get_context().get_stage_id())
physics.force_load_physics_from_usd()
view = tensors.create_simulation_view("numpy")
view.set_subspace_roots("/")
lower = view.create_rigid_body_view("/World/Lower")
upper = view.create_rigid_body_view("/World/Upper")
print("CONTACT_PROBE_PHYSX_READY",lower.count,upper.count,flush=True)
print("NATIVE_PHYSX_MAPPINGS",core.get_physx_id_mappings(),flush=True)
(args.output/"runtime_mass_kg.json").write_text(json.dumps({"lower":lower.get_masses().tolist(),"upper":upper.get_masses().tolist()}))
print("FORCE_API",[name for name in dir(upper) if "force" in name],flush=True)
dt=.001
rows=[]
for step in range(args.steps):
    # A synthetic threshold-crossing input, never a claimed physical press law.
    force = 0. if args.case in ("no_force","too_high") else (-2. if step>=100 and step<350 else 0.)
    if force:
        upper.apply_forces_and_torques_at_position(np.array([[0,0,force]],dtype=np.float32),None,None,np.array([0],dtype=np.uint32),True)
    simulation.simulate(dt,step*dt)
    simulation.fetch_results()
    physics.update_transformations(False,True,True,False)
    assembled=core.get_assembled_connections(clear=True)
    disassembled=core.get_disassembled_connections(clear=True)
    debug=core.get_assembly_debug_infos()
    row={"step":step,"t_s":(step+1)*dt,"external_upper_fz_N":force,"lower_pose_xyzw":lower.get_transforms()[0].tolist(),"upper_pose_xyzw":upper.get_transforms()[0].tolist(),"lower_velocity":lower.get_velocities()[0].tolist(),"upper_velocity":upper.get_velocities()[0].tolist(),"connected":core.are_parts_connected("/World/Lower","/World/Upper"),"assembled":len(assembled),"disassembled":len(disassembled),"assembly_debug":[{key:getattr(info,key) for key in ["accepted","relative_distance","tilt","projected_force","yaw_error","position_error"]} for info in debug]}
    rows.append(row)
    if assembled or step%100==0:print("CONTACT_PROBE_STEP",step,"connected",row["connected"],"events",len(assembled),len(disassembled),flush=True)
with (args.output/"trace.ndjson").open("w") as f:
    for row in rows:f.write(json.dumps(row)+"\n")
stage.Export(str(args.output/"scene.usdc"))
(args.output/"result.json").write_text(json.dumps({"completed":True,"case":args.case,"steps":len(rows),"assembly_events":sum(r["assembled"] for r in rows),"disassembly_events":sum(r["disassembled"] for r in rows),"assembly_debug_samples":sum(len(r["assembly_debug"]) for r in rows),"native_physx_mapping_sizes":[len(m) for m in core.get_physx_id_mappings()],"final_connected":rows[-1]["connected"],"final_relative_z_m":rows[-1]["upper_pose_xyzw"][2]-rows[-1]["lower_pose_xyzw"][2],"hardware_clients":0,"training":False,"training_readiness":False,"gripper_present":False,"external_load_scope":"2N synthetic algorithm threshold probe, not measured insertion force or proposed controller"},indent=2))
simulation.detach_stage()
app.close(wait_for_replicator=False)
