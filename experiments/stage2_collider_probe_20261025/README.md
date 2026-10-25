# Native collider reporting probe

This is a sensor API experiment prompted by failed grasp job 1809 and saved-point
diagnostic 1810. One reported point lay 1.300 mm beyond the authored finite pad
surface. The actor-level tensor stream could not establish which shape produced
it. The old grasp remains failed.

One frozen launch holds the robot at its initial joint targets while a declared
2 mm kinematic test cube approaches three separate, fixed left-finger patches:
the pad BODY, stock metal WHITE, and outside screw HEAD. Test-fixture placements
are initialization, not a manipulation or brick transport demonstration. The
full 300-brick source and its 930 native links remain present.

The public native contact callback copies per-shape headers and contact points
before transient buffers expire. An independent evaluator compares them with
the unchanged tensor observer. Every loaded point needs a unique fresh match
in both directions under the frozen position, separation, normal and normal
impulse tolerances. No missing or ambiguous observation is accepted as zero
contact. WHITE and HEAD must have actual loaded evidence to validate forbidden
component attribution. Neither can count as a valid pad contact.

Only known robot component roles and loads may appear in online packets.
Opposing actor/collider identifiers stay in evaluation artifacts. A passing
reporting test would not validate surface-distance fidelity, change the old
point gate, establish whole-arm collision safety, or authorize a grasp retry.

## Reproduction

After independent source review and a clean commit of this directory:

```bash
python3 experiments/stage2_collider_probe_20261025/submit.py
```

The submitter copies the immutable 1809 source scene plus this experiment into
a new `runs/*-stage2-collider-probe` snapshot, then submits one foreground launch
using `pueue add` in the default queue. It refuses a second submission. The
snapshot manifest records source hashes, exact commands and the preserved prior
contact clock. The current Python environment is reused without installation.

`protocol.json` freezes the three patches, global matching parameters, 480-step
limit and 240-second capture/audit allowance. All actual elapsed compute is
charged to the existing 1800-second aggregate. Unknown/failed cases are retained;
there is no fixture relocation, parameter tuning or automatic retry.
