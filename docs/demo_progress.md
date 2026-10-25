# Challenge 2 simulation demo: measured progress

Updated 2026-10-25. The active objective is an incremental end-to-end **simulation**
demo. No hardware motion, ROS deployment or physical readiness is authorized or
claimed. Existing visual and perception experiments remain preserved.

## Current end-effector model

The reference-corrected folded C920 holder and supplied fingertip STL visuals
are complete as a nominal visual update. The D435i mount, arm configuration,
layout and native 300-brick design are retained. See
[registration, renders and limitations](../experiments/final_workcell_20261024/end_effector/README.md).
STL units/attachment and both cameras' optical calibration remain unmeasured;
stock pad collisions are still used and do not validate updated-tip contact.

## Stage 1: ordinary candidate/mock interface smoke passed; coverage remains limited

Frozen R3 is reused with configured synthetic camera calibration and a translated
source ROI. Online code receives RGB, axial depth, calibration and opaque frame
IDs; actual brick poses/IDs/colors and reference metadata belong only to offline
evaluation. Candidate XY/shared ridge yaw/body-top-band assumptions are not full
poses or verified grasp access. Challenge 1 is not implemented here; the current
four exact-ID classification fixtures are explicit mocks and abstain if no
PRESENT candidate exists. An actual classification entry point is still needed.

The first integration smoke gate uses intact/interior-gap/endpoint states in
both current wide diagnostic views, requiring at least one eligible PRESENT per
state and no confident wrong/lower/absence claims. Sparse remnants and layer
transition are reserved for the frozen interface audit, not novel-stack evidence.

| Version / split | Pueue | Present matches / visible references | Decisive coverage | Supported bands | Result |
|---|---:|---:|---:|---:|---|
| Unchanged R3, development | 1723 | 0 / 171 | 0% | 0 / 6 | Fail; no candidate, mock blocked |
| Unchanged R3, reserved | 1724 | 0 / 64 | 0% | 0 / 4 | Fail; no candidate |
| R3 + fixed Z workspace, development | 1726 | 0 / 171 | 0% | 3 / 6 | Fail; height corrected, fragments rejected |
| Same R3/workspace, closer virtual views, development | 1729 | 41 / 176 | 23.3% | 6 / 6 | Ordinary integration smoke passes |
| Same closer views, reserved | 1730 | 19 / 66 | 28.8% | 2 / 4 | Sparse-state gate fails |

Zero detections give undefined precision, not 100% precision. No-match targets
are not uncertain *instances*. All-NaN controls and the fixed NaN patch produce
no false absence claims. Recovered/lost confirmed development faces: 0 / 0.

Development evidence: the wide overhead highest band selected forearm geometry
around 0.583 m rather than the source layer around 0.10 m. One reviewed fixed
work-volume prior [-0.005, 0.20] m converts excluded depth to unknown without
altering raw inputs or R3 thresholds. This finds the right overhead band but
all fragments remain too narrow (7–10 mm versus unchanged 16±4 mm). Oblique
bands remain unsupported. No threshold relaxation followed these failures.
A separately versioned fixed close-view experiment (render job 1728) subsequently
recovers confirmed candidates without changing R3 or workspace settings. Its
results are separate from the wide-view baseline.

Source and reproducibility:
[Stage 1 code/protocol](../experiments/stage1_20261025/README.md),
[development baseline](../runs/20261025T001958.593862Z-stage1-development/evaluation/evaluation.json),
[workspace comparison](../runs/20261025T002637.845861Z-stage1-development-workspace_v1/evaluation/evaluation.json).
Rendering jobs 1722 and 1728 preserve native links in each partial-state snapshot.

The close-view ordinary gate confirms 15 intact, 13 interior-gap and 13 endpoint
faces, all overhead. The complementary view confirms none. Conditional accepted
XY median is 0.50 mm and max 0.716 mm (N=41); these are ideal synthetic metric
checks, not verified grasp precision. There are 135 missed visible occurrences
(127 matched uncertain hypotheses and 8 no-match), and no confident false/lower/
merge/duplicate errors. Two lower-fragment **raw** hypotheses are unconfirmed.
The interior gap is correctly missing overhead and uncertain obliquely; a
removed endpoint remains unknown extent. NaN controls and all mock guards pass.
Of the 41 confirmed occurrences, 38 recover previously visible IDs and 3 concern
newly visible references; zero previously confirmed faces are lost. Reference
visibility changes from 171 to 176, so the two camera sets are not identical
samples. Reserved transition top confirms 19; sparse remnants confirm zero in
both views. No reserved-driven threshold or view tuning follows.

[Ordinary close-view evaluation](../runs/20261025T003437.436801Z-stage1-development-workspace_v1/evaluation/evaluation.json),
[reserved evaluation](../runs/20261025T003437.726220Z-stage1-reserved-workspace_v1/evaluation/evaluation.json),
[representative sensor overlay](../runs/20261025T003437.436801Z-stage1-development-workspace_v1/inference/f000/overlay.png).
The actual Challenge 1 service is still unavailable; passing mocked joins is
not actual classification integration. These views are virtual inspection
views, not calibrated wrist-camera or hardware reachability evidence.

## Stage 2 prerequisite: full-stack dynamics — failed baseline

Job **1725** attempts a frozen 120-step, one-second idle test with the same
300 bricks/930 native connections, nominal arm target and stock collisions.
It stops at step 7 (about 0.058 s after graph initialization): connections
836 and 837 on top-edge brick `brick_9_0_3` break; the source component changes
from 301 parts / 930 connections to 300 / 928. Maximum source drift is 0.171 mm;
robot joints hold their nominal values. Thus the visual twin's prior zero-step
captures did not establish load-bearing native stability.

No preload, friction, cooldown, timestep, breakage rule or stack size is changed
to hide this result. Diagnostic job **1727** enables only native debug output
and records utilization; it reproduces the same first failure at step 7 and
collects seven native solver dumps. Debug completion is not an idle pass. The
failure mechanism is not yet established; queued analysis must distinguish
serialized force/solver evidence from conjecture. An idle pass alone would still not prove
extraction. Bilateral contact feedback and a checked updated-tip collision
representation are further prerequisites for that claim.

[Idle protocol and commands](../experiments/stage2_idle_20261025/README.md),
[first-failure trace](../runs/20261025T002411.589299Z-stage2-full300-idle/output/idle_report.json).

## Remaining stages — not demonstrated

Selection/grasp/extraction, reject transfer, empty-baseplate insertion,
brick-on-brick stacking and repeated sorting with bounded recovery have no
passing measurements yet. Do not credit legacy truth-driven task observations,
static graph authoring, mock labels or preview renders as these stages.
Native connections may be created only for reset initialization or by native
assembly detection during measured task execution; no post-reset teleportation,
scripted attachment or manually declared connection success is allowed.


## Closed numerical diagnostic round

Native debug/analysis jobs1727/1731/1732 are followed by the explicit coupled
numerical-epsilon pair1733/1734 and read-only comparison1735. Both1e-4 and1e-6
break the same two top-edge connections at step7. The tighter value lowers the
watched relaxed-bound violation from~5.52e-5 to~5.7e-9, but positive model
relaxation remains. Numerical tightening is not a sufficient fix; underlying
contact/angular-load origin is unresolved. No physical threshold, break rule,
source geometry or task motion is changed. Independent evaluation/review agree
to stop this bounded round with the failed idle gate intact, rather than hide it.

[Machine-readable dynamics outcome](../experiments/stage2_idle_20261025/results.json),
[perception outcomes and exact snapshots](../experiments/stage1_20261025/results.json).
The six-stage goal remains incomplete. Further controller work needs a stable
native source/contact model and checked tip/contact feedback; actual Challenge1
integration also remains unavailable. No extraction, transfer or stacking
success is established by the candidate/mock-interface pass.


## Updated technical starting points

The [revisable stage protocol](demo_stage_protocol.md) now governs classification
visibility/timing, held-offset/pivot/clearance experiments, receiving stud-grid
placement and online-vs-native attachment evidence. It caps each frozen stage at
three improvement rounds before reassessment and proposes a six-brick/eight-attempt
cyclic-color demo. No classification timing or25°/5mm extraction parameter is
promoted to validated status. Native idle failure remains the contact gate.


## Optional Alex compute check

The savedSSHprofile currently reachesalex2.nhr.fau.de with strict known-host
verification and no interactive authentication or credential copying. Local
Pueue1737 submits Slurm4685752: a bounded1A40/5minute inventory probe, which
completes in10seconds with16CPU/60000M and driver610.57.04. The first submission
1736 was rejected before allocation because Alex forbids explicit --mem forGPU
jobs; the corrected payload uses automatic per-GPU resource shares.

Isaac/CUDAapplication compatibility remains NOT_TESTED: no verified matching
runtime/container was identified. The default probe interpreter isPython3.9.25;
metadata absence there is not cluster-wide absence. No remote installation,
training, simulation or scaling follows. See [probe evidence and budgets](../experiments/alex_preflight_20261025/README.md).
