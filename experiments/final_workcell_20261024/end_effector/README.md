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

The supplied `franka_research3_gripper.stl` contains one connected tip-sized
solid: CAD bounds X[-11,0], Y[-23,-8], Z[2.05,27.60]. STL stores no units;
**millimeters are assumed**. It is instantiated twice as mirrored pad visuals.
The native finger mesh combines 362 white-finger faces and 262 dark-pad faces;
only the pad faces are hidden using holeIndices. The white fingers and joints
remain intact. CAD X=0 maps to each native pad's broad backplane, CAD Y is
centered laterally, and CAD Z=27.60 aligns with the native distal end. This
nominal registration extends the replacement 7.74 mm armward versus the stock
pad. Physical attachment/orientation is not independently measured. Dark pad
finish is illustrative; no material properties are inferred from STL.

**The stock pad collision geometry/friction is retained.** The new STL visual
fit does not establish new-tip contact behavior. Any manipulation experiment
using the new geometry must explicitly validate its contact representation.
No grasping, camera visibility of an actual held brick or insertion is proved
by these static renders.

- [C920 fascia and updated fingertips](../../../runs/20261025T000918.719244Z-final-workcell-preview/output/images/diagnostic_c920_grasp_side.png)
- [Profile showing both camera housings](../../../runs/20261025T000918.719244Z-final-workcell-preview/output/images/diagnostic_c920_profile.png)
- [Updated USD](../../../runs/20261025T000918.719244Z-final-workcell-preview/output/final_workcell.usda)
- [Independent invariant checks](../../../runs/20261025T001003.870890Z-end-effector-verification/output/verification.json)
- [STL registration report](../../../runs/20261025T000918.719244Z-final-workcell-preview/output/updated_fingertips.json)

Pueue default-queue jobs: **1718** corrected C920 and initial native-finger
audit; **1719** pad face/material audit; **1720** final STL visuals; **1721**
comparison/invariance checks. All capture jobs preserve 300 bricks, 930 native
connections, source poses, arm joints, D435i transform, table layout, original
camera poses/K, lighting and exposure. No post-connection physics steps are
used. Prior results are retained. Source commits and hashes are in run manifests.

Reproduce with a fresh stable queued snapshot:

```bash
python3 experiments/final_workcell_20261024/submit_preview.py --cameras experiments/final_workcell_20261024/end_effector/cameras.json --appearance experiments/final_workcell_20261024/appearance_config.json
```
