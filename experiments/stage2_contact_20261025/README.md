# Stock-pad contact feedback benchmark

The stock-pad positive-contact and reopening diagnostic passes independent audit
**1755**, after preserving two earlier failures. This is a fixture test, not a
perception-driven pick, dynamic grasp, extraction, calibrated physical force
measurement or proof of matching real-robot feedback availability.

| Attempt | Capture / audit Pueue | Measured steps | Outcome |
|---|---|---:|---|
| Initial B0, `f9496b3` | 1751 / not submitted | 0 | Pre-play geometry exporter expected Mesh; actual stock colliders are Cubes |
| Contact round 1, `922a0d6` | 1752 / 1753 | 36 | Frozen gate failed at first ~2 mN contacts because coupon scalars were negative |
| Contact round 2, `3f0f9c9` | 1754 / 1755 | 100 | All 27 independent checks pass; both pads confirm contact and reopen |

All eight composed stock finger colliders are USD Cubes; `collisions/mesh_3` is
the pad on each finger, with separate white-finger boxes. The Cube exporter uses
actual sizes and composed transforms, without changing physical shapes. Contact
points are checked against these surfaces, not against the visible new STL.

The round-1 scalar validity assumption was unsupported: the installed API's own
example contains negative coefficients and defines vector force as signed scalar
× normal. Unmodified products in the failed trial matched pair matrices,
reciprocal forces and dt scaling. Round 2 therefore preserves signed vectors and
uses absolute magnitudes for threshold/abort and **all** force-carrying point
checks, including negative coupon reports. It does not fit gains, flip selected
forces or relax thresholds. The failed 1753 audit remains immutable; it did not
establish negative-coupon point ownership or the positive/reopen gate.

## Passed check and limits

A declared kinematic native 2×4 coupon starts near stationary pads, with 0.25 mm
clearance per side derived from actual composed stock geometry. The seven arm
command targets stay nominal. Initial finger opening changes only before play;
there is one declared coupon placement before measurement. This is test setup,
not a truth-fed online pick or attachment.

Both pads exceed 0.2 N for three fresh samples at step 49. Peak magnitudes are
0.224462 N left and 0.224318 N right. Force release below 0.01 N for three samples
first occurs at step 70; the controller finishes returning to its initial opening
at step 100. These are distinct completion observations. All substantive signed
contacts, including coupon-side negative coefficients, are uniquely compatible
with the stock pads. Reciprocal, vector/matrix, buffer, sign and unit checks pass.

The full background remains 300 bricks / 301 native parts / 930 links, with PGS
and source actor32/4. Existing 15 N / 2000 N/m / 100 N s/m hand actuator values,
material bindings, mass/inertia and native physical rules are unchanged. Actual
stock pad friction2.5/2.0 is not physically calibrated.

Command ramps are capped at 1 mm/s and travel at 1 mm per finger (+2 µm numerical
allowance). Sampled actual joint speed above 2 mm/s, normal magnitude above 2 N,
or hand drift beyond 2 mm/2° aborts. Actual speed and force are recorded; sampled
abort bounds are not guaranteed physical caps. The budget is 360 steps / 180 s.
An unsafe/incomplete-data failure halts physics; it does not claim reopening or
recovery. Successful contact explicitly reopens and verifies release.

The run logs existing SDK warnings about zero-velocity writes to a kinematic
coupon during initialization. Its pose/state checks pass; this API warning was
not hidden or changed during the signed-coefficient correction.

## Reproduction and evidence

```bash
python experiments/stage2_contact_20261025/submit.py
```

Submission freezes committed code/configuration and uses default Pueue with the
existing `unitree_sim_env`. Run array evaluation through the frozen queued audit
launcher, never directly. Earlier snapshots preserve their original contracts.

- [1754 source/input manifest](../../runs/20261025T032638.185262Z-stage2-contact-B0-stock/manifest.json)
- [1754 geometry](../../runs/20261025T032638.185262Z-stage2-contact-B0-stock/output/geometry.json)
- [1754 controller trace](../../runs/20261025T032638.185262Z-stage2-contact-B0-stock/output/trace.jsonl)
- [1755 audit launcher](../../runs/20261025T032653.179734Z-stage2-contact-B0-R2-evaluation/launch.sh)
- [1755 independent audit](../../runs/20261025T032653.179734Z-stage2-contact-B0-R2-evaluation/output/evaluation.json)
- [All outcomes, hashes and earlier failed artifacts](results.json)

The supplied STL remains visual-only. Its collision proxy requires one separately
frozen final round with actual cooked-hull fidelity checks before contact, using
the exact B0 coupon center rather than recentering its extended geometry. The
fixed +2 mm offset case remains withheld. No grasping, extraction or hardware
control was performed.
