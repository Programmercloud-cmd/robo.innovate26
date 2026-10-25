# Startup material/contact-offset readback

The closed free-coupon trials1821/1822 fail retention despite simple14mm closure.
This one readback inspects configuration before another physical hypothesis.
It reuses the exact1822 initializer and replaces only its postassembly hook.
Existing reset, one explicit preassembly physics step and targeted mass/inertia
restoration remain disclosed. The original trial loop is never called; no extra
physics steps, closure, lift, material/offset setters or task changes occur.

Public runtime getters read three exact count1 body views: left finger, right
finger and free coupon. Every material triple, contact offset and rest offset is
saved with shape/dtype/status/errors. USD collider/material/binding/combine/offset
opinions are listed separately. No public tensor-row-to-collider mapping is
available here. Rows remain UNKNOWN by component; neither traversal order nor an
expected friction value identifies the PAD row. Uniform values across all
returned slots may be described without assigning a component index.

Runtime material values are not a measured friction coefficient for the contact
pair or proof of a retention cause. Missing/invalid getters remain UNKNOWN;
diagnostic completion and valid effective arrays are separate results. No grasp,
extraction, trajectory adjustment or retry follows automatically.

ONE default-queue startup/audit job, ≤90s, retaining2388.038652171/2500s prior
cost. Capture cap75s, JSON audit cap8s. Full source/API/reference hashes are pinned;
all three bodies, source301/930 and isolated coupon1/0 are checked before/after.
Existing `unitree_sim_env`/Isaac5.1/BrickSim is reused without dependency changes.

After final independent source review and commit:

```bash
python3 experiments/stage2_contact_readback_20261025/submit.py
```
