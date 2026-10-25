# One corrected-material retention regression

Jobs 1827–1829 established that the brick's pre-existing declared material
values had not been bound as physics materials, and verified the corrected
values at startup. This experiment tests that correction with the unchanged
1825 free-brick fixture and position-controlled primitive. It is one correctness
regression, not another coefficient choice or a fourth fixture improvement.
The previous three improvement rounds and all failures remain closed records.

Only the isolated coupon's Body/Top bindings change. All six declared values
are copied, including Body dynamic friction decreasing from the old runtime
0.5 to the declared 0.4. The 300-brick native source, geometry, mass/inertia/COM,
drives, nominal 14 mm custom-pad gap, 3.083 mm gripping band, 100 µm BODY contact
offsets, position-only 30-command admission, phase-specific finger-response
limits, 2 N sampled force bound, motion and contact/native/holding gates stay
unchanged. This does not reproduce or calibrate the physical Desk 14 mm grasp.

The immutable 1825 trial and margin wrapper are reused. Initial and terminal
readbacks must verify the corrected coupon material multiset together with the
existing 100 µm finger-offset model, unchanged rest/finger materials and five
exact material-schema paths. Shape-row-to-collider identity remains UNKNOWN.

One job first runs and independently audits a **fresh 1500-step stationary
control** on this corrected model. Only a passing same-job, current-protocol,
content-bound control permits one conditional pick from a fresh initialization.
Older material-model controls cannot admit the pick. No benchmark reduction,
control reuse, retry or additional improvement variant is allowed.

Success requires the existing full 5 mm commanded lift, at least 4 mm actual
coupon rise, 30 hold steps, support separation and admission-relative 1 mm / 3°
retention, plus every unchanged contact, reciprocal native mapping and source
invariant. A pass establishes bounded free-coupon retention only; perception,
native tower extraction, transfer, insertion and hardware remain unproven.
Failure ends this regression and requires a different evidence-based assessment.

The prospective resource amendment retains **2698.130616356 s** already spent.
One **≤300 s** shared control/pick/audit job fits an amended **3000 s** aggregate
ceiling (worst 2998.130616356 s), up from 2800 s. It creates no new tuning pool.
The supervisor stops if the control fails or fewer than 145 s remain for the
conditional pick. All stages use the default Pueue queue and stable snapshots:

```bash
python3 experiments/declared_material_retention_20261025/submit.py
```

No shared dependencies, native source, camera layout, gripper geometry or
physical hardware are changed.
