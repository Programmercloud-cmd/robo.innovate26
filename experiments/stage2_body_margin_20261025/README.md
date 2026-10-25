# One BODY contact-margin sensitivity

Readback1823 found1.3625mm contact offsets in all returned finger/coupon slots,
with zero runtime rest offsets. Free-coupon retention fails under both original
and position-only timing. This experiment tests one contact-generation margin
hypothesis; it does not assume that margin caused those failures.

The unchanged1822 trial is imported. A small Tool subclass authors100µm contact
offset on exactly the left/right `collisions/mesh_3` BODY prims before reset.
No rest/material/geometry/mass/inertia/drive/width/controller/trajectory/admission
or threshold change. The coupon and WHITE/HEAD offsets stay unchanged. Thus the
body–coupon offset-sum descriptor becomes about1.4625mm, not100µm. Runtime
initial/terminal multisets must show exactly one100µm slot and four baseline
slots per finger; coupon slots, materials and rest offsets stay baseline. Shape
row mapping stays UNKNOWN; no index assignment by expected value is made.

PhysxCollisionAPI declaration may expose three specific unauthored defaults:
automatic restOffset (`-inf` in schema metadata,0 at runtime), torsional radius0
and minimum torsional radius0. These are permitted only as the pinned schema's
fallback metadata. Only contactOffset may gain an authored value; every other
authored attribute is unchanged. No arbitrary attribute exemption is allowed.

A NEW1500-step matched stationary control precedes a fresh conditional1500-step
position-only14mm/30-endpoint/three-fresh-window pick. Every original2N/speed/
plane/normal/WHITE/HEAD/native/raw/source/support gate remains. Holding still
requires≥4mm actual rise,5mm commanded lift,30hold and≤1mm/3° admission-relative
change with support separation. No success from disappearance of one bad point.
The full source300bricks/301parts/930edges remains, plus the isolated free coupon.
Known-fixture retention is not sensor-selected native extraction or hardware
readiness. Prior1821/1822 failures and1823 observations remain unchanged.

This is fixture improvement2of2: timing was1, contact margin is2. No hidden
counter reset, third offset, squeeze change, sweep or retry. Failure closes this
comparison for reassessment. ONE default-Pueue control/pick/audit job,≤300s,
prospectively raising the aggregate ceiling2500→2800s while retaining every
2403.718485308 prior second (worst2703.718485308s). Pick is skipped if control/
configuration fails or less than130s remains. Capture total cap220s, each audit
cap50s, all sharing the300s limit.

After independent source review and commit:

```bash
python3 experiments/stage2_body_margin_20261025/submit.py
```

Same installed unitree_sim_env/Isaac5.1/BrickSim, no new dependencies. Existing
validated reset/targeted mass restoration remains initialization, while this
new offset opinion is explicit and frozen before measurement. All physical
simulation and captured-array evaluation use the foreground default queue.
