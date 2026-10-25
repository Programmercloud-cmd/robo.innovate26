# Position-based lift-candidate admission: one improvement

Baseline 1818 completed 30 position commands at the nominal 14 mm model gap with
bilateral contact, but never met the old 1.5 N threshold. All measured-frame and
private contact checks passed; no lift or detachment occurred. That negative
result is retained. This round changes one phase-admission condition and leaves
the gap, controller, motion, contact screen, geometry and 2 N limits unchanged.

The existing early force-confirmed branch remains. At the same 30-command
endpoint, a fallback can admit only a **GRASP_CANDIDATE** if the last three fresh
endpoint samples all satisfy:

- Both pad loads exceed the existing 1e-6 N contact-presence floor.
- Each measured finger q exceeds its last commanded q by the existing 2 um
  readback allowance.
- Actual joint-mapped opening exceeds commanded opening by 4 um, propagating
  those two allowances.
- Every current raw/contact/plane/normal/spatial/forbidden-surface/motion guard
  passes.

Joint positions and the frozen opening-axis/plane mapping determine eligibility;
the pose-derived gap remains a separate readback. No threshold is fitted to the
observed ~1 N loads or ~1.12 mm endpoint resistance. The last commanded float32
target is frozen. No extra endpoint wait, smaller gap, increased force or native
identity enters admission. Absent this evidence, the run abstains without lifting.

The candidate then receives only the unchanged 5 mm straight lift and 30-step
hold test. It is not a secure-grasp or ownership claim. Independent evaluation
must still prove selected-source BODY contact, actual native singleton detachment
after admission, target rise, held-transform retention and no collateral damage.
The unmet force-only requirements remain explicit reference outcomes; they are
not manufactured as passing. All unrelated physical guards stay authoritative.
Online held identity remains UNKNOWN and blocks transfer.

This authorizes one explicit improvement round, with no automatic retry, H,
transfer, color integration, new controller or hardware action. The 2200-second
aggregate ceiling stays unchanged: all 1844.070074964 prior seconds are retained,
and this one job is capped at 350 seconds including capture and audit. Existing
failed experiments remain immutable. The nominal model gap is not asserted
equivalent to the operator's Franka Desk 14 mm setting.

After final independent source review and a clean committed experiment:

```bash
python3 experiments/stage2_position_candidate_20261025/submit.py
```

The foreground default-Pueue job uses a stable source/configuration snapshot,
the previously reviewed control bridge, and unchanged runtime/dependencies.
