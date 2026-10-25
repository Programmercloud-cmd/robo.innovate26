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

The original [next-contact proposal](demo_next_contact_gate.md) is preserved.
The stock baseline has now run as recorded below; the supplied-STL proxy remains
unvalidated. No fixture is claimed as a perception-driven grasp.


## Stock-pad positive contact/reopen passed

Contact benchmark B0 capture **1754**, independent audit **1755**, passes all 27
checks over 100 steps. Both stock pads exceed 0.2 N for three samples at step 49;
peaks are about 0.224 N. Force release first confirms at step 70, with controlled
return to initial opening complete at step 100. All signed force-carrying points
are pad-compatible; reciprocal/unit/buffer and source930-link guards pass. Arm
commands, source PGS32/4 and existing mass/material/actuator settings are retained.

Earlier **1751** stops before play on an unsupported Cube representation;
**1752/1753** fails the original nonnegative-scalar assumption at first small
contacts. Both failures remain preserved. Primary API examples and unmodified
signed vector checks justify the round-2 interpretation correction; no physical
force or motion threshold was tuned. Those two rounds were followed by the
final supplied-tip geometry attempt recorded below.

[Limits, exact commands and provenance](../experiments/stage2_contact_20261025/README.md),
[all outcomes](../experiments/stage2_contact_20261025/results.json).
This is a stock-pad fixture/feedback check. Actual supplied-STL collisions,
dynamic holding, extraction, insertion, classification timing and the integrated
demo remain unvalidated. Per-pad force availability on physical hardware and
shared sim/real completion semantics are not established by this diagnostic.


## Final contact round: supplied-tip geometry stopped before contact

Source `5e8ba6c`, Pueue **1756**, tests one experiment-local supplied-STL
collision decomposition in the full300 initializer. It stops after 13.373 s
before measured contact: normalized SDK hull planes fail to contain their own
vertices within the frozen 0.1 µm representation check. The raw left-tip
32-hull RESULT_VALID query is retained; the right tip and subsequent union/CAD
fidelity acceptance are untested. A representation-query failure is not proof
of invalid physical CAD. No updated-tip contact, reserved offset or grasp is
credited. The shared visual workcell and earlier stock-pad pass remain intact.

All three contact improvement rounds are closed. No recook, threshold change,
D/R trial or fourth configuration follows. Independent read-only audit **1757**
describes the retained partial geometry
without promoting G. Seventeen of 32 left hulls fail the frozen containment
check; maximum normalized residual is 48.703 µm (own-polygon residual 49.344 µm).
Maximum outer-bound difference is 0.102881 mm against the frozen 0.1 mm limit.
Capture plus audit consumes 13.437 s. Cause and full CAD/union fidelity remain
unresolved; there is no right-tip or measured-contact evidence.
The six-stage sorting demo remains incomplete, with supplied-tip collision
validation the immediate manipulation prerequisite.

[Current contact results](../experiments/stage2_contact_20261025/results.json),
[1756 failure](../runs/20261025T040508.534018Z-stage2-contact-R3-G/output/failure.json).


## Representation reassessment: observe active shapes next

Pueue **1758**, source `10177e6`, analyzes saved left-tip data only (0.109 s).
All32 indexed surfaces are closed/oriented/nondegenerate, but only16 meet the
original convexity precision. Maximum polygon nonplanarity is144.508 µm, and
fan convexity excess115.407 µm. For27 reference-free bore probes, a vertex-hull
interpretation reports27 outside, SDK halfspaces26 outside/1unknown, and
conservative indexed winding27unknown. These candidates do not establish the
actual collision shape or its suitability. OldG/bounds failure/right-tip
unknown and the closed three-round contact budget remain unchanged.

Planner, independent evaluator and reviewer agree to stop saved-data forensics.
One separately frozen, motion-free active-PhysX query diagnostic will inspect
the unchanged STL/registration/decomposition using fixed probes and controls.
It cannot promote the old benchmark or authorize D/R/contact. The full goal
remains active and incomplete.

[Reassessment evidence, versions and source contracts](../experiments/stage2_contact_20261025/reassessment/results.json).


## Active-shape observation: setup fixed, proxy discrepancies measured

Original setup1759/audit1760 stops at four empty controls; there are no target
observations. A reviewed process-local `enable_scene_query_support=True`
correction addresses the installed headless default. Corrected1761/audit1762
passes34/34 integrity checks and all250 fixed queries in15.047s including
the supervised audit command, with nominal
joints/full300/930/PGS32/4 unchanged and zero measured postassembly steps.

Each pad matches all42 CAD-predicted ray-hit cases outof96, but largest first-hit
differences are6.500mm left and1.346mm right (nongrazing/nonedge). Active pad
sphere queries overlap2/27 left and9/27 right independently CAD-free bore
locations; their clearance is~.598mm/.500mm against50µm probe radius. These
conditional local discrepancies reject the current automatic-decomposition
model for faithful updated-tip contact. Negative probes are not global void
proof. No contact/grasp result follows from the integrity pass.

The two observational attempts and old three-round contact benchmark are
closed. Planner/evaluator/reviewer recommend inspecting one explicit CAD-derived
convex compound with preserved bore/contact surfaces and unchanged registration.
Concrete parts need review before a single frozen approach attempt, with no new
three-round tuning pool. If this requires substantial geometry infrastructure,
obtain the original CAD/convex parts instead. No new proxy is implemented yet.
The full six-stage goal remains active and incomplete.

[Active-query commands, versions, raw data and outcomes](../experiments/stage2_active_query_20261025/README.md).


## Original fingertip inspection supports a concrete partition candidate

Pueue1763, source `b534b2c`, inspects only the supplied original STL in1.677s
(60s limit). The1256-triangle/630-vertex mesh is closed, consistently oriented
and nondegenerate. All35 component loops across20 fixed Xsections are closed
and unbranched, with no coplanar-face sections. Source-plane clusters identify
the boss front atX=-11mm, body front=-9.5mm, blind bore cap=-4.5mm and back=0mm.
Nominal STL volume is3366.462mm³ versus3574.971mm³ for its full vertex hull.

These measurements support a possible48-cell model-specific partition—16
sectors for each of the boss, bored-body and solid-body axial regions. It is
not yet constructed or accepted. Source-derived polygon supports, cell coverage,
original100µm fidelity/bounds and bore preservation need independent checks
before any cooking. No new framework, decomposition sweep or freshthree-round
allowance is proposed. Current CAD-to-finger registration is also unverified;
contact-face and distal-end confirmation questions are pending. CAD-frame part
construction can progress without inventing the physical mounting.

[Inspection evidence and plane/face provenance](../experiments/tip_partition_20261025/results.json),
[orthographic facet projections](../runs/20261025T050607.814521Z-original-tip-feature-inspection/output/orthographic.png),
[fixed raw cross-sections](../runs/20261025T050607.814521Z-original-tip-feature-inspection/output/sections.png).


## Explicit compound: resource guard stopped construction

Pueue1764 (source`8e1c4cd`) retains27 of48 cells and stops before cooking
at the frozen128-vertex guard. Logging-only replay1765 (source`0ab784e`)
reproduces all27 retained records exactly. Its independent audit confirms
that`front_body_bored_11` has133 extreme vertices,262 triangular facets
and3.245µm minimum vertex spacing. The124 exact plane rows are descriptive;
none of these counts establishes native cooked polygon count or cookability.
Both jobs together consume0.495s of supervised process time.

Original1764 remains failed and incomplete. One unchanged-shape CPU resource
guard reassessment is under review, with all48 cells and100µm fidelity
checks retained. There is no cooking/contact approval or fresh three-round
allowance. Mounting-face/distal-end questions remain unanswered.

[Preserved outcomes](../experiments/tip_partition_20261025/compound/results.json).


## Explicit compound passes offline sampled geometry checks

Pueue1766 (source`9d789ab`) completes the unchanged48-cell construction
with one reviewed128→255 offline vertex-guard correction. All27 earlier
records remain exactly equal. All16 independent checks pass; source-outside
maximum exact distance is59.735µm, candidate exterior sampled distance plus
its1µm bracket is60.667µm, bounds error is numerical noise and volume differs
by0.0530%. All27 reference-free50µm bore spheres remain free in the candidate.
Actual cumulative supervised time including1764/1765 is6.547s of180s.

This is sampled CAD-frame evidence, not a global Hausdorff guarantee, native
cookability, confirmed physical mounting, or contact/grasp success. Source
inside samples use a fixed100µm outward-normal endpoint bound, not nearest
Euclidean distance. Candidate filtering excludes internal interfaces;22
samples have unresolved exterior brackets. One narrow native cooking/query
gate is being reviewed before any contact is attempted. All previous failed
benchmarks remain failed; no new three-round allowance is created.

[Independent offline audit](../runs/20261025T061618.814542Z-explicit-tip-compound-255/audit/evaluation.json),
[exact27-record check](../runs/20261025T061618.814542Z-explicit-tip-compound-255/prefix_check.json),
[commands and source manifest](../runs/20261025T061618.814542Z-explicit-tip-compound-255/manifest.json).


## Native compound observation stops at physical-property preservation

Pueue1767 (source`0fbc0bf`) fails before any cooked-buffer capture or query.
Replacing collider geometry changes automatic mass/inertia/COM entries10/11;
all other body entries and all actuator arrays remain exact against1754. The
strict guard correctly rejects an unfair physical comparison. These entries
have not yet been mapped to saved historical body names. All target geometry
and query quality remain UNTESTED. Capture plus independent failed-capture audit
takes14.699s, with zero measured postassembly steps.

The team supports one explicit initialization-only preservation correction: a
same-instance scene.reset wrapper restores the declared frozen1754 mass model
before the existing explicit nominal initialization step, verifies current
finger mapping/nonfinger invariance and requires the same exact postreadback.
This is not an estimate of the real updated tip's mass. Engine-internal reset
initialization precedes this restoration. No CAD/geometry, query, tolerance or
actuator tuning is proposed. At most two native setup attempts and cumulative
180s including1767; no contact or new three-round allowance. Source review is
pending before the final attempt.

[Preserved native-test failure](../experiments/compound_native_query_20261025/results.json).


## Full-articulation property restoration fails on untouched arm links

Pueue1768 (source`cd7aca3`) completes the fixed three setters once but fails
strict readback before the explicit initialization step, cooking or queries.
Saved records show both finger property sets, all masses/COM translations and
actuators exactly restored. Only nonfinger inertia matrices/principal-axis
quaternions at indices0–7,9 differ; q/qdot and physics time are unchanged.
This is consistent with documented inertia re-diagonalization on rewritten
links; physical equivalence is not established. Both setup attempts remain
closed/failed, with29.797s cumulative process time.

A concrete source-supported reassessment targets only the two finger actors
through RigidBodyView setters, leaving arm actors unwritten. Same B0 model,
setter order, strict full-robot readback, shape, query and state guards remain.
This is under independent source review, not a fresh three-round pool or a
retroactive pass. At most one targeted-route run is proposed (aggregate three
native executions, cumulative180s including prior29.797s). Native cooked
geometry/contact and physical tool mounting remain unvalidated.
