# Final fixture reassessment: fixed-target finger response

1824's verified100µm BODY-margin trial was censored after38 lift samples by
actual finger opening2.043/2.078mm/s, with targets frozen and other observed
gates passing. The2mm/s bound came from an earlier1mm/s normal-contact command
diagnostic. It was not an FR3-calibrated lift-response limit; units and indices
are correct.1824 remains FAIL/CENSORED, not numerical noise or a passed hold.

This explicitly frozen third/final fixture improvement changes only that
observation criterion. CLOSE/control retain2mm/s. LIFT/HOLD, only after validated
admission with both last commanded float32 targets bitwise frozen, use per-finger
`min(20mm/s, actual runtime asset velocity limit)`. Unknown phases, nonfinite or
missing limits, wrong joint types/units or changed targets fail closed. The
public startup max-velocity/type readback is preserved; URDF guesses are not used.
The implicit PD drive stays active.20mm/s is a declared engineering response
scale;2N/100N·s/m motivates a scale but is not a net-force bound, mechanical
guarantee, fitted2.1mm/s cutoff or hardware calibration.

Commanded closing1mm/s, nominal14mm target/30-endpoint admission,2N force limit,
actual travel/TCP/arm limits, BODY100µm margin, materials, geometry, drives,
plane/normal/WHITE/HEAD/native/raw/source/support and1mm/3° holding criteria all
remain. Actual robot qdot is never hidden or altered. Per-row ceilings and every
old2mm/s crossing are logged separately as reference evidence. Raising this
ceiling cannot itself establish success: actual≥4mm coupon rise,5mm commanded
lift,30hold and support separation remain required.

The unchanged1824matched1500-step control is reused only through exact old
protocol/audit/capture pins and live physics/q/fixture/observer/margin identity,
plus source/AST evidence that the measured control path still uses2mm/s. The
1822trial source differs only in the declared guard leaf, readonly startup
limit/type queries and evidence logging; the BODY-margin Tool class is unchanged.
This is a known centered fixture, not perception/native extraction or hardware
readiness. Earlier results and the closed margin comparison are retained.

ONE≤150s pick/audit in the SAME2800s aggregate budget, retaining2594.227219942s
(worst2744.227219942s). Max1500physics steps, capture100s/audit40s, no retry,
increasing-ceiling sequence or fourth fixture improvement. Any failure requires
reassessment; no threshold, motion, force or material tuning follows.

After final source review and commit:

```bash
python3 experiments/stage2_finger_response_20261025/submit.py
```

Same installed runtime/dependencies. All simulation and captured-array audit
runs in one foreground default-Pueue job with stable source/configuration hashes.
