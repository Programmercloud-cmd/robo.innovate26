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

## Stage 1: image candidates and classification handoff — not passed

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

Zero detections give undefined precision, not 100% precision. No-match targets
are not uncertain *instances*. All-NaN controls and the fixed NaN patch produce
no false absence claims. Recovered/lost confirmed development faces: 0 / 0.

Development evidence: the wide overhead highest band selected forearm geometry
around 0.583 m rather than the source layer around 0.10 m. One reviewed fixed
work-volume prior [-0.005, 0.20] m converts excluded depth to unknown without
altering raw inputs or R3 thresholds. This finds the right overhead band but
all fragments remain too narrow (7–10 mm versus unchanged 16±4 mm). Oblique
bands remain unsupported. No threshold relaxation followed these failures.
A separately versioned, fixed close-view sampling experiment is being prepared;
its results must not be substituted retroactively for the wide-view baseline.

Source and reproducibility:
[Stage 1 code/protocol](../experiments/stage1_20261025/README.md),
[development baseline](../runs/20261025T001958.593862Z-stage1-development/evaluation/evaluation.json),
[workspace comparison](../runs/20261025T002637.845861Z-stage1-development-workspace_v1/evaluation/evaluation.json).
Rendering job 1722 preserves native links in each partial-state snapshot.

## Stage 2 prerequisite: full-stack dynamics — failed baseline

Job **1725** attempts a frozen 120-step, one-second idle test with the same
300 bricks/930 native connections, nominal arm target and stock collisions.
It stops at step 7 (about 0.058 s after graph initialization): connections
836 and 837 on top-edge brick `brick_9_0_3` break; the source component changes
from 301 parts / 930 connections to 300 / 928. Maximum source drift is 0.171 mm;
robot joints hold their nominal values. Thus the visual twin's prior zero-step
captures did not establish load-bearing native stability.

No preload, friction, cooldown, timestep, breakage rule or stack size is changed
to hide this result. The next diagnostic may enable native debug output and
read utilization while keeping all physical settings unchanged. The failure
mechanism is not yet established. An idle pass alone would still not prove
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
