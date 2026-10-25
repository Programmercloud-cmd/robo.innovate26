# Startup verification of declared brick materials

This is one bounded configuration check of the opt-in authoring helper tested
in job 1827. It reuses the immutable 1822 initializer and startup-only 1823
readback. Only the isolated coupon's Body/Top physics-material bindings are
authored before initialization. The 300-brick source stack and fingers are not
changed. The original trial hook is replaced with getters; no closing, lifting
or postassembly measured physics step is authorized.

The expected coupon material values are the existing declared Body
`[0.5, 0.4, 0.1]` and Top `[1, 0.8, 0.2]`, compared as an unordered float32
multiset. Shape-row-to-collider identity remains UNKNOWN. Finger material rows
and all contact/rest offsets must match job 1823; geometry, properties, native
inventory and readback-time state must remain unchanged. Unavailable or invalid
readbacks cannot verify the correction.

This uses the pre-margin initializer for a direct comparison with 1823. It does
not adopt a new retention model or alter the closed 1825 grasp result. Inherited
reset/preassembly steps and targeted mass restoration remain disclosed. A pass
would establish effective configuration values at startup, not pair friction,
physical calibration, retention or a native extraction.

One default-Pueue job, maximum **60 s**, including capture and independent audit.
The retained prior cost is **2682.945582478 s**, within the unchanged **2800 s**
case ceiling. No automatic retry, alternative coefficient or fourth fixture
trial is permitted. Submit only after final source review and commit:

```bash
python3 experiments/brick_material_runtime_20261025/submit.py
```

The helper and new wrapper are copied into a stable snapshot; older source and
readback artifacts are pinned. Shared environments and the BrickSim source are
unchanged.
