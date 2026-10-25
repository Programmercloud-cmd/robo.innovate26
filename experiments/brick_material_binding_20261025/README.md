# Declared brick contact-material authoring

The native author creates friction/restitution attributes on BodyCollider and
TopCollider cubes, but does not provide bound physics materials. Startup
readback 1823 returned two default `[0.5, 0.5, 0]` slots for the coupon, rather
than its declared Body `[0.5, 0.4, 0.1]` and Top `[1, 0.8, 0.2]` values.

[USD's material contract](https://openusd.org/release/api/usd_physics_page_front.html)
places those properties on a material with `UsdPhysics.MaterialAPI`, bound via
`UsdShade.MaterialBindingAPI` with the physics purpose. The new helper prepares
that correction on one explicitly named brick before initialization, using
only the coefficients already declared on its two colliders.

This is opt-in source work. It does not alter the active workcell, shared
BrickSim package or previous snapshots. It does not fit coefficients or claim
a retention cure: the declared Body dynamic friction is lower than the
previous runtime default. The coefficients themselves remain physically
uncalibrated.

The single ≤10 s default-Pueue check uses the installed standalone USD library
on a tiny source-representative stage. It verifies actual material resolution,
unchanged geometry/visual/mass/unrelated properties, rejection before mutation,
and save/reopen behavior. No Kit, rendering, physics steps, grasp or effective
PhysX readback is included. Any future runtime model verification needs its own
reviewed scope and budget; the three-round retention comparison stays closed.

After source review and commit:

```bash
python3 experiments/brick_material_binding_20261025/submit.py
```

No dependencies are installed. Library paths are set only within this job.
