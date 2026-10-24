# Final workcell from ten setup photographs

This is an approximate, editable reconstruction of the final workcell, using
the existing Franka Research 3 / BrickSim runtime. It does not establish a
calibrated robot or camera frame. Previous perception experiments and their
results are preserved.

The ten original images are archived unchanged under
`resources/setup_images/final_workcell_20261024`; `photo_inventory.json`
records their SHA256 hashes. All photos were inspected. Photo 9 provides the
clearest whole-layout reference; photos 8 and 3 corroborate the layout and stud
lattice, and the other views show the stack, ruler, bowl and robot mounting.
The user confirms an **iPhone 12**, with original photos **6 and 7 at 0.5×**
and the remainder at **1×**. The new reference attachments match photos 7 and
6 byte-for-byte; the primary layout reference, photo 9, is therefore 1×.
The supplied JPEGs contain no embedded focal-length or camera-model metadata;
some have been resized. Device/zoom provenance is recorded separately from
calibrated photo intrinsics and from the wrist D435i.

## Coordinate frame and evidence

The right-handed table frame has its origin at the white source plate centre,
X toward the gray destination plate, Y away from the robot toward the reject
bowl, and Z up. This frame is separate from the uncalibrated robot-base frame.

The source stack retains the confirmed 300-brick design: ten alternating
layers, six ridges of five 2×4 bricks per layer, two-stud gaps and the existing
native connection geometry. The user reports white **25×25 cm** and gray
**38×38 cm**, with black tape covering two stud rows at each taped edge.
The reused native assets have 32×32 and 48×48 studs at 8 mm pitch, giving
nominal 25.6/38.4 cm footprints. That reported/native difference is explicit;
bricks are not stretched. Hidden brick colors cannot be recovered from these
views; the existing synthetic inventory remains a placeholder.

`photo_measurements.json` records manual image anchors and their limitations.
The queued `measure_layout.py` run uses the reported white plate size to fit a
single-plane homography, then checks the separate ruler and reported gray size.
This is a layout estimate, not phone calibration, hand-eye calibration or
robot-frame registration.

The corrected audit distinguishes the visible white rectangle (25×21.8 cm)
from the hidden outer edges: two rows at 8 mm pitch imply an approximate
16 mm covered strip at each Y edge. Gray covered strips are likewise added
back before comparing outer dimensions. It measures the apparent 20 cm ruler
span as 18.68 cm and gray inferred outer spans as 40.75–40.85 cm along X and
38.42–38.99 cm along Y, versus reported 38 cm. These discrepancies remain;
the fit does not force all checks to agree. Hidden edges, manual picks, lens
distortion and raised surfaces can all matter. The two ±3-pixel
corner scenarios are sensitivity examples, not confidence intervals or bounds
on physical error. The partly occluded robot foot and elevated bowl rim have
additional projection uncertainty.

Rounded scene estimates in table coordinates are gray plate centre
(0.42, -0.06) m, robot base (-0.09, -0.40) m, and bowl centre (0.01, 0.27) m.
The robot yaw, table dimensions, bowl shape/height and plate thickness remain
explicit assumptions. Reachability and insertion clearances are not validated.

## Destinations and camera

The gray plate is one studded surface divided into three zones. The large
half away from the robot is the requested-stack zone; the two smaller zones
near the robot hold unrequested accepted colors. The bowl is the faulty-brick
destination on the far side of the input stack. At the black-taped Y edges,
two stud rows are visually covered and declared unavailable for placement;
yellow divider width is a separate estimate. Nominal zone rectangles do not
establish a collision-free, fully stud-supported output placement.

- One requested accepted color: requested color in the large zone, the other
  two accepted colors in separate small zones.
- Two requested accepted colors: both share the large zone, the third uses
  one small zone and the other small zone stays empty.
- Three requested accepted colors: all share the large zone; both small
  zones stay empty.

These are destination semantics, not a new task scheduler or classifier.
Challenge 1 still owns color/quality labels. A future mixed output can use a
simple repeating color sequence; the exact output lattice and usable stud
clearances await confirmation.

Only the existing physical wrist D435i is retained in the specification. Its
mount has not moved according to the user, but the repository did not contain
a calibrated wrist-camera transform. The earlier 6.3 cm and 2 cm measurements
describe housing offsets with unresolved axes, not an optical-frame transform.
Intrinsics and hand-to-optical transform therefore remain unset. Diagnostic
virtual inspection views are explicitly separate from physical cameras; the
two added physical cameras are omitted.

## Reproduction and scope

All photo processing and rendering uses `pueue add` in the default queue with
frozen code/configuration/input snapshots. The initial photo audit is preserved
as job **1696**. The tape-aware revision using the user's dimensions is job
**1697**, snapshot
`runs/20261024T203806.556370Z-final-workcell-photo-audit-tape`.

The scene reuses the existing runtime and native assembly/restoration checks.
No dependencies are installed, no shared environment or queue settings are
changed, and no training, manipulation, ROS or hardware control is started.
Native connections are checked at a static snapshot; connection dynamics are
not validated. Recreating native runtime links uses the scene launcher, rather
than assuming an exported USD restores all extension state by itself.

The private renderer explicitly initializes the configured nominal robot joints
with one simulation step **before** restoring the source bricks and creating
the native graph. It checks joint readback and PhysX/USD hand-pose agreement.
There are no physics steps after native preassembly. This fixes static visual
initialization; the robot joint pose is still nominal, not reconstructed from
the physical photographs.

For a new preview after committing configuration edits, run:

```bash
python3 experiments/final_workcell_20261024/submit_preview.py
```

This helper creates a fresh immutable snapshot, uses the existing environment,
submits its foreground launcher through default Pueue and prints the job ID.
It does not overwrite earlier captures.

The marker-board design is now known from the supplied specification:
**7×5 squares, DICT_4X4_50, 30 mm squares and 22 mm markers**. The nominal
pattern spans 210×150 mm, with 17 default markers and 24 inner corners. The
specified 100% print setting is an instruction, not a verified measurement
of the physical print. The A4 backing and estimated scene location remain.

Robot-base position/yaw relative to two table edges or known plate corners is
still useful. D435i intrinsics and a measured optical mount transform remain
necessary for calibrated wrist-view equivalence. Identifying the phone lenses
and ChArUco design does not supply those transforms.

## Completed preview

Final capture: **Pueue 1700**, frozen scene source `e02864e`.

- [Overhead render](../../runs/20261024T204939.156692Z-final-workcell-preview/output/images/diagnostic_table_top.png)
- [Oblique render](../../runs/20261024T204939.156692Z-final-workcell-preview/output/images/diagnostic_oblique.png)
- [USD visual scene](../../runs/20261024T204939.156692Z-final-workcell-preview/output/final_workcell.usda)
- [Native graph, scene and camera manifest](../../runs/20261024T204939.156692Z-final-workcell-preview/output/manifest.json)
- [Jobs, snapshots, artifact hashes and validation](completion.json)
- [Editable layout and destination recipes](scene_config.json)

Validation passed for all 300 source bricks, 930 native connections and 301
connected parts including the source plate. Joint readback matches requested
nominal values exactly; hand PhysX/USD positions and orientations agree within
the configured tolerances. Source placement and state are unchanged across
the two captures. One explicit preassembly initialization step is recorded;
no physics step occurs after native assembly.

Jobs 1696–1700 all ran through default Pueue; earlier audits and previews are
preserved. Kit cleanup stalled after validated capture and was bounded by the
existing owned-process supervisor, independently of capture validation.
The exported USD references the frozen snapshot's assets; use the launcher
to recreate runtime native connections. No independent graph reload or
load-bearing dynamics claim is made.
