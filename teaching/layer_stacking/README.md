This isolated task transfers complete **twelve-brick 4×3 layers** from `external_r0_c0` … `external_r5_c1` onto the white plate, alternating 90°. It shares the factory planner, resident camera and exclusive SDK-owner implementation. Conveyor jobs, geometry and the commissioned conveyor binary remain unchanged.

```bash
./run_layers.sh --plan          # write poses.simulation.json; no hardware
./run_layers.sh --simulate      # run two-layer synthetic software rehearsal
./run_layers.sh --simulate --sources external_r0_c0 external_r0_c1
```

`config.json` is the task configuration. The provisional target is plate stud center **[23.5,23.5]**, toward the supplied photo's lower-left corner. Both orientations fit inside the taped outer stud row. These are **layer-bottom poses from the photo-derived simulation transform, not taught real TCP coordinates**. Rotating these rectangular layers creates overhanging bricks: the first layer has 96 potential baseplate stud matches; subsequent layers have 64 potential matches and require cantilever-clutch qualification. These are geometric coincidences, not verified engagements. The compact layer is 12×8 studs, using the nominal 8 mm lattice rather than summing 31.8/15.8 mm body dimensions or reusing the older gapped RL ridges. Sources on the left use saved center-column XY; right-bank centers are explicitly nominal simulation extensions. Source grasp height with the wide tool must be taught separately.

One deterministic supervisor owns source presence → approach → taught grasp descent → close → measured lift/retention → clearance transfer → fresh empty-source check → edge alignment → guarded contact/admittance → bounded light-contact XY search if stalled → unloading → opening → retreat → fresh verification → inventory commit. It never requests an assistant decision. Normal corrections are X/Y/yaw; no roll/pitch search or loaded yaw twisting is used. A release-view failure gets bounded fresh re-observation; unresolved vision, grasp, load or placement uncertainty stops without committing that layer.

The image servo uses small strips around configured edge segments, fixed contrast polarity, subpixel gradient fits and a local 3×3 image Jacobian. Receiver edges constrain X/Y/yaw; visible held-layer edges guard slip. It observes before contact and after retreat, without requiring all bricks visible throughout grasp descent. There are separate references for first-layer baseplate alignment, subsequent even/odd layers, and even/odd post-release views. `calibrate.py` fits the Jacobian from saved neutral and ±X/±Y/±yaw image/actual-pose samples; it does not command jogs or claim calibration validity.

Alignment references are captured at the configured **4 mm above seated TCP height**, not at a different contact height. Verification references are captured **100 mm above the seated TCP height**, following the corrected placement X/Y/yaw; the robot returns to nominal home after verification.

The seating controller is a slow **force-feedback admittance outer loop over the existing joint-impedance position interface**. A separate native controller variant enforces hard force/lateral-force/torque limits every callback, alongside the original motion, joint, tracking, singularity, heartbeat and robot-fault guards. The finite XY spiral first unloads to light contact; its radius is bounded around one fixed aligned origin. Native smoothing/derivative limits remain active. This is not a new high-bandwidth torque controller.

Real execution is deliberately disabled until `hardware` contains actual taught whole-layer grasp poses, the first seated TCP reference, verified workspace/tool, a fixed clear fixture, qualified grasp/retention evidence, measured whole-layer contact-width/retention windows, contact depth/force/admittance limits, qualified free-space wrench baselines for both orientations, and wrist references matching this wide STL, 6 mm cavity and camera profile. **No synthetic force value falls back into hardware.** Missing fields are reported before opening any SDK or camera connection.

Explicit teaching reads only, under the existing SDK ownership mechanism:

```bash
source /opt/ros/humble/setup.bash
/usr/bin/python3 -s -m teaching.layer_stacking.teach source external_r5_c0
/usr/bin/python3 -s -m teaching.layer_stacking.teach seated-target
```

Guide the tool vertically to the corresponding complete-layer grasp or first fully seated pressing reference before recording. Captures/configuration history are archived, and teaching disables prior hardware commissioning. No automatic 15 mm/other Z offset is applied. Physical target clearance and the previously photographed bricks must be checked at teaching; the simulation does not fabricate their occupancy.

Build the separate native executable with `teaching/layer_stacking/build.sh`. Only after physical commissioning:

```bash
./run_layers.sh --execute --confirm-empty-gripper
```

The empty-hand flag acknowledges an empty tool at startup. `--reset-confirmed` additionally acknowledges selected sources restored and the receiving target cleared after an earlier job; it does not claim an interrupted placement succeeded. A pending/completed physical job otherwise blocks a restart. The persistent physical ledger is `layer_stacking/inventory.json`, separate from factory inventory; fixture runs write only their own run ledger. Native arm/hand measurements still flow through the existing twin telemetry tap. No independent SDK reader runs over the owner.

Logs under `runs/` contain configuration, generated poses, phase timings, edge overlays/corrections, force/depth samples, pending inventory and outcome. Hardware completion means the commissioned compound seating/release criteria passed; independent mechanical-interlock checks are not fabricated. Fixture outcomes always report **zero verified physical interlocks**. The fixture generates images and an arbitrary software wrench surrogate; it is not validated LEGO physics, a real-camera benchmark or an RL training environment. Actual whole-layer engagement, force limits and camera visibility still require physical commissioning.

The isolated Isaac **target-geometry preview** is at http://127.0.0.1:18768/ (two planned alternating layers, static robot, no contact simulation). `digital_twin/isaac/layer_stack_preview.py` recreates this from a saved plan and the selected board/wide USD into a fresh output directory. It does not modify the live twin or animate a successful physical pick.
