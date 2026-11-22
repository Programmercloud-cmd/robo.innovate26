"""Bounded renderer-free Kit startup probe; never connects to robot hardware."""
from pathlib import Path
from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "enable_crashreporter": False,
                     "extra_args": ["--/app/renderer/enabled=false", "--/renderer/enabled=false"]},
                    experience=str(Path(__file__).with_name("physics_only.kit")))
import omni.usd
from pxr import UsdGeom, UsdPhysics
stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageMetersPerUnit(stage, 1.)
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdPhysics.Scene.Define(stage, "/World/Physics")
for _ in range(10):
    app.update()
print("FR3_PHYSICS_KIT_STARTUP_OK", flush=True)
app.close(wait_for_replicator=False)
