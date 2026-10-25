# Stage2: idle physics smoke only

The unchanged full300 workcell is initialized by the existing private renderer:
its configured nominal robot pose, source root restoration,930 genuine native
connections and strict pre-dynamics pose checks are reused. An optional callback
then advances120 physics steps at1/120s while holding the existing nominal joint
targets. It returns before the static capture path and never writes root/joint
states after initialization, resets bodies, reduces the scene or tunes settings.

`protocol.json` declares the test before execution. Every step checks all301
source-part states and robot states are finite, the original301-part/930-link
component remains exact with no breakage, and maximum position/orientation drift
stays within2mm/2degrees. Final speeds must be at most.02m/s linear and.2rad/s
angular. These are smoke-test tolerances, not production or manipulation claims.
The first failed invariant is recorded and stops the run. No recovery is tried.

`idle_trace.jsonl` records each check; NPY files preserve initial/final or failed
source states. `idle_report.json` and `idle_complete.json` refer only to idle
validation. No RGB capture or sensor equivalence is claimed. The custom supervisor
uses the existing bounded child cleanup and reports shutdown separately.

Stock collision/friction remains the contact baseline. The user-supplied updated
fingertips are visual meshes only; this test does not validate their physical
fit, force limits, contact geometry, grasping or insertion. Automatic assembly
detection stays disabled for the explicit graph, and breakage detection stays on.

After source/protocol review and freeze, the orchestrator submits the foreground
supervisor through the default Pueue queue and reports its job ID:

```sh
python experiments/stage2_idle_20261025/supervise_idle.py \
  --python /home/faps/miniforge3/envs/unitree_sim_env/bin/python --output OUTPUT
```

Only syntax and tiny numerical contract fixtures run outside the queue:

```sh
python experiments/stage2_idle_20261025/run_idle.py --self-test
```

## One unchanged-physics breakage diagnostic

The initial smoke failed at step7: two connections to brick_9_0_3 broke, reducing
930 links to928 while position drift was only.171mm. `diagnose_breakage.py` reuses
the same native initializer and runs at most8 steps, stopping at the first failed
invariant. Only `BreakageThresholds.debug_dump` changes to true; every other public
threshold is snapshotted and compared exactly. Gravity, preload, friction,
compliance, cooldown, dt and breakage enablement remain unchanged.

The diagnostic records public per-link utilization for all930 original links,
the top values, connections836/837, native ID mappings and solver debug dumps.
`TMPDIR` points to the owned output/native_debug_tmp directory before native code
loads. A missing/unmapped utilization is−1, not zero. Cooldown delays break
selection; its timing is not evidence of the failure's cause.

`diagnostic_complete.json` means observations were collected, including an
expected first failure; it explicitly does not mean idle stability passed.
Use the same frozen default-queue workflow, replacing supervise_idle.py with
supervise_diagnostic.py. No physics tuning, solver rebuild or recovery is tried.
