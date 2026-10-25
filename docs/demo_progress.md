# Challenge 2 simulation demo: measured progress

Updated 2026-10-25. **Latest result:** the PGS workaround passes the full300
and reserved-removal 120-step native idle checks. Grasping/extraction, actual
classification, transfer, insertion and repeated sorting remain unfinished.
See the [final idle-round results](../experiments/stage2_pgs_20261025/README.md).

The active objective is an incremental end-to-end **simulation**
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
The six-stage goal remains incomplete. The source later passes the bounded PGS idle checks recorded below. Further
controller work still needs checked updated-tip collisions/contact feedback; actual Challenge1
integration also remains unavailable. No extraction, transfer or stacking
success is established by the candidate/mock-interface pass.


## Updated technical starting points

The [revisable stage protocol](demo_stage_protocol.md) now governs classification
visibility/timing, held-offset/pivot/clearance experiments, receiving stud-grid
placement and online-vs-native attachment evidence. It caps each frozen stage at
three improvement rounds before reassessment and proposes a six-brick/eight-attempt
cyclic-color demo. No classification timing or25°/5mm extraction parameter is
promoted to validated status. PGS subsequently passes the initial idle gate;
updated-tip collisions and contact feedback remain prerequisites.


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


## Bounded contact-observation diagnostic

Pueue **1738** enables only process-local contact reporting and native debug
logging for the unchanged full source, capped at eight steps/first failure.
Independent audit **1739** confirms equivalence to the default 1733 baseline:
source initial/terminal state files are byte-identical and available per-step
metrics/joints match; connections 836/837 still break at step 7. No API adaptation
or physics adjustment was made. All 42 watched body-step records report zero
normal/friction contacts, including zero aggregate normal impulses. Without a
positive reporting control, these zeros do not establish complete sensing or
zero physical loads. Native internal same-component contact suppression is
also present. Load origin remains unresolved and the idle gate still fails.

[Protocol and commands](../experiments/stage2_idle_20261025/README.md),
[independent contact audit](../runs/20261025T014929.925825Z-stage2-contact-evaluation/output/REPORT.md).
This diagnostic does not reset the stage improvement budget or validate grasping.


## Stage 2 round 2: solver-quality sensitivity did not fix idle stability

Pueue **1740**, source `1debd6e`, tests source-only velocity iterations 4→16
(position 32), retaining all physical/native break parameters and the original
120-step acceptance gate. Runtime properties on all 300 source actors verify
32/16 initially and terminally; the actual TGS scene clamps permit those values.
Audit **1741** passes all 25 experiment-integrity checks, but the same links
836/837 break at step 7. Initial states are byte-identical to both earlier
baselines and recorded per-step aggregate drift/speed metrics agree exactly.
No load/utilization comparison or executed-island iteration count is available.

This is a failed numerical sensitivity result. Default settings remain unchanged;
no reserved case or manipulation is run. Two Stage 2 improvement rounds are now
used; a third parameter guess is not justified. The source/load model needs a
concrete hypothesis before further contact work. The six-stage demo remains
incomplete.

[Round 2 protocol/results](../experiments/stage2_idle_20261025/README.md),
[independent audit](../runs/20261025T020255.062438Z-stage2-round2-evaluation/output/evaluation.json).


## Isolated bookkeeping signature and final-round hypothesis

The eccentric two-brick diagnostic **1743**, audited in **1744**, completes eight
steps/no break with all 36 integrity checks passing. Gravity-consistent linear
impulse accompanies a persistent native external angular residual, despite zero
observed angular velocity/momentum change. Independent source reviews find a
public PhysX TGS writeback expression predicting its exact sign/scale. The
loaded-binary-to-public-tag correspondence remains qualified. This does not clear
the failed full300 idle gate or justify removing BrickSim wrench transport.

[Fixture measurements, preserved setup failure and source hypothesis](../experiments/stage2_eccentric_20261025/README.md).
A single PGS workaround is being prepared as the final Stage 2 improvement round:
small-fixture gate, then full300 idle, then a reserved removal state, stopping at
any failed gate. It retains physical capacities, break rules, timestep and default
actor32/4. No native build or parameter sweep is planned.


## Final idle round passed with PGS; contact/manipulation gates remain

All three predeclared phases pass: A **1745/1746** suppresses the eccentric
angular-reporting signature; B **1747/1748** keeps all 300 bricks and 930 native
links through 120 steps; reserved C **1749/1750** keeps 299 bricks and 926 links
through 120 steps. Maximum drift is 0.208 mm in B and 0.582 mm in C, within the
unchanged 2 mm limit. No native break occurs. Actual PGS/source32/4, nominal joints,
initial poses and native physical settings are independently checked.

The sole change is TGS→PGS. Epsilon, clutch strength/friction, timestep, geometry
and stock collision assets are unchanged. A's operational artifact screen is not
force calibration; source/binary identity and broader physical validity remain
qualified. Three configuration rounds are now closed; no fourth tuning round.
New contact benchmarks must explicitly use this tested PGS setting and report
stationary controls over their own duration. The supplied fingertips still have
stock collision geometry, and no grasp, extraction, placement or integrated
sorting success is claimed.

[Commands, frozen gates, all six job IDs and independent audits](../experiments/stage2_pgs_20261025/README.md),
[machine-readable results](../experiments/stage2_pgs_20261025/results.json).
