Open **http://127.0.0.1:18767/**. With the wide simulation selected, the browser buttons switch between **Slim gripper · factory** and **Wide gripper · board**. The slim view is served read-only through the same endpoint, so its old separate HTTP server is not required. No physical finger change automatically selects anything.

From the repository root:

```bash
digital_twin/sim.sh slim     # factory workcell, original slim fingertips
digital_twin/sim.sh wide     # workcell + empty RL plate, supplied wide STL
```

Explicit independent scene/finger choices remain available:

```bash
digital_twin/sim.sh factory --fingers wide
digital_twin/sim.sh board --fingers original
```

The launcher prints its selection. An unchanged running selection is reused; changes to model assets or implementation trigger rebuilding automatically. `--rebuild` forces a rebuild. Only this launcher's own variant processes are replaced. Initialization is bounded, and failures identify the VM log. Switching in the browser does not restart Isaac or command hardware. The original factory assets and active-session record are preserved.

The authoritative wide asset is `assets/wide_lego_gripper.stl` (474 triangles per finger; SHA-256 `7d076beec9820dacc2788e91cfb1dad65f19a0076d0776a8b9f37bf17eedcd1c`). The mistaken `Lego_holder.stl` is ignored. The supplied export was 10× oversized and translated in CAD space: `profiles.json` records the uniform 0.1 conversion and translation into the existing fingertip CAD frame. Both original rigid mount transforms are retained. The source CAD and mount transforms remain unchanged; the ceiling is thickened with the aligned solid described below. No nonuniform scaling or pocket-width reshaping is applied.

Original CAD surface measurements after normalization: **95.8 mm pocket span, 20 mm inward reach, 19 mm cavity height, 6.55 mm ceiling, 5 mm distal side walls** with the supplied thicker proximal taper. The active wide profile now adds **13 mm of ceiling material downward**, leaving **6 mm free cavity height** and **19.55 mm total ceiling thickness**, with the external top fixed. `ceiling.target_cavity_height_m` controls this change. The added solid overlaps the original roof internally by 0.1 mm; source vertices, finger bottoms, side profiles and mounting transforms do not move. These replace the previous procedural approximation; its configuration is archived under `evidence/before_supplied_wide_stl/`. These are CAD dimensions, not physical metrology.

The wide mesh uses convex decomposition rather than a single hull across the pocket. Obsolete stock-pad collision portions are removed while native metal supports remain. Simulation-only lower finger limits preserve at least 1 mm clearance between opposing parts over the allowed 38.187–80 mm native-width stroke. Real Hand settings and taught poses are unchanged. Material, mass, loaded grasp, receiving-layer clearance and stud engagement are not physically validated; this is a measured-state display twin, not a training-ready contact model.

The plate remains the empty RL plate: 32×32 studs, first row taped on each edge, 30×30 exposed. No photographed bricks are added. Its photo-derived placement is retained; physical thickness and contact registration remain unverified.

Both views consume existing telemetry/display files without an SDK connection or competing camera owner. Source timestamps and stale status are preserved. A VM restart does not make cached samples live. When hand measurements are unavailable, the wide preview starts at an explicitly simulated 70 mm width; incompatible measured widths are marked rather than forcing cap interpenetration.

`environment.json` records the selected CAD, normalization, dimensions and fit bounds. `audit.py OUTPUT_DIRECTORY`, run with the VM Isaac Python environment, independently checks USD mesh fidelity, mounting transforms, cavity dimensions, collision setup and opposing clearance. `evidence/` contains the deployment, browser and geometry validation records.
