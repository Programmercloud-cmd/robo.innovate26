# Reference-corrected C920 and supplied fingertip visuals

The user's folded-holder photograph supersedes the earlier C920 proxy. The
contact side is the holder UNDERSIDE. Its rear tail points armward to the
nominal circular wrist-face plane (hand Z=0). A compact folded clip, front
fold/lip and tape are represented against hand −X, centered laterally at Y=0.
The housing overlaps the foot instead of hanging below a long exposed strap.
D435i and its bracket remain on hand +X, with their existing joint frames.

In the reference the fascia faces the front of the folded foot. That forward
direction maps to hand +Z/fingers. Housing-only pitch toward hand +X therefore
points the decorative fascia toward the nominal grasp region, rather than
outward away from it. The illustrative 50° pitch and 94×29×24 mm housing remain
artist estimates. Neither the true mechanical stop nor optical-center position,
viewing direction, camera intrinsics or FOV was measured. No physical C920
image sensor is activated. The folded-contact placement is visual, not a fit or
clearance measurement. The C920 has no added collision, rigid-body or mass API.

The user corrected the fingertip orientation: the tapered **low CAD Z** end
points distally toward the bricks, the bosses point into the supporting metal
fingers, and the broad CAD X=0 gripping faces oppose each other across the
opening. A proper rotation, CAD Ry(180°) followed by finger Rz(±90°), reverses
CAD X/Z without reflecting the mesh.

The supplied STL remains unchanged, with nominal CAD bounds X[−11,0],
Y[−23,−8], Z[2.050005,27.600006]. STL has no units; millimeters remain assumed.
The body mounting plane at CAD X=−9.5 is placed at the old stock-pad backplane;
the boss front at X=−11 therefore extends nominally 1.5 mm into the support.
Width centering and the previous visual distal endpoint are retained as
**nominal visual anchors**. The saved-mesh seat inspection found only proximal
circles unrelated to the distal mounting feature. Recess concentricity,
clearances and physical fit have not been measured or established by this
placement. The per-render `updated_fingertips.json` records the actual transform,
axis probes, nominal boss center and depth, and these limitations.

A simple round screw head is a rigid visual child of each finger, on the
nominal boss axis. Head diameter 6 mm and outward protrusion
2 mm are configurable estimates. The head uses a nearby outer-metal visual
bound at the boss height; its full envelope is reported for later clearance
work. The through-metal connection is implicit; there are no shaft, threads,
form-fit, fastener collision or added mass.

Only the 262 old dark-pad faces are hidden; all 362 white-finger faces remain.
**Stock collision geometry and friction remain unchanged.** This visual
correction does not validate supplied-tip contact, grasping or insertion.
Closed perception/contact experiments retain their original transforms and
results; their old mounting is not silently updated by this correction.

Historical artifacts below show the **superseded fingertip mounting**. They
remain useful for the earlier camera-holder and invariance work:

- [Historical C920 fascia and fingertips](../../../runs/20261025T000918.719244Z-final-workcell-preview/output/images/diagnostic_c920_grasp_side.png)
- [Historical camera-housing profile](../../../runs/20261025T000918.719244Z-final-workcell-preview/output/images/diagnostic_c920_profile.png)
- [Historical USD](../../../runs/20261025T000918.719244Z-final-workcell-preview/output/final_workcell.usda)
- [Historical invariance checks](../../../runs/20261025T001003.870890Z-end-effector-verification/output/verification.json)
- [Superseded STL registration report](../../../runs/20261025T000918.719244Z-final-workcell-preview/output/updated_fingertips.json)

Historical default-queue jobs were 1718 (C920/native-finger audit), 1719
(native pad separation), 1720 (earlier STL visuals) and 1721 (verification).
The current visual change preserves the source's 300 bricks, 930 native
connections and 301 connected parts, robot configuration, table layout and
both camera housings. New close-up views are diagnostic virtual cameras.

Reproduce the corrected visual preview from a stable queued snapshot:

```bash
python3 experiments/final_workcell_20261024/submit_preview.py --cameras experiments/final_workcell_20261024/end_effector/mount_correction/cameras.json --appearance experiments/final_workcell_20261024/appearance_config.json
```
