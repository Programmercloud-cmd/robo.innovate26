# Startup pose-reader diagnosis

User-authorized continuation after the previous control-alignment case closed.
Brev remains paused. This case diagnoses the evaluation reader only; it does
not run the task controller, gripper commands, a new trajectory or training.

Use the exact failed1837 full300 workcell initializer, including its existing
nominal robot initialization and301parts/930native links. At its startup reader
hook, compare the cached Isaac Lab property, fresh per-asset PhysX getters and
a fresh batched PhysX getter. Retain raw float32 arrays, copied WXYZ conversions,
actor paths/permutation, cache timestamps and clocks/step index. Save partial
packets before structural validation. Never invalidate caches, refresh/update
the scene, step physics or rewrite bodies to make the comparison pass.

All three pair residuals retain1nm/1e-7rad gates. A cache-only difference is an
observation, not a physics failure. Only fresh direct/batched agreement supports
using the direct getter as the reader-equivalence oracle. Even that does not
establish absolute pose accuracy or successful robot control.

Separate case ceiling:180s total scheduler wall time; one capture<=90s,
independent saved-data audit<=15s, and at most one reviewed corrected startup
check<=75s. No automatic retry or old-case credit. Full previous costs remain
255.736105687/270s, closed and incomplete. The submitter retains immutable
source/configuration manifests and uses only the default Pueue queue.

```bash
# Small invented fixtures only, no saved scene arrays or simulator.
/home/faps/miniforge3/envs/unitree_sim_env/bin/python -m unittest discover -s tests -p test_batched_source.py -v
/home/faps/miniforge3/envs/unitree_sim_env/bin/python -m unittest discover -s experiments/pose_reader_20261026 -p test_evaluation.py -v

# Commit reviewed source first; each command reports its default-queue job ID.
python3 experiments/pose_reader_20261026/submit.py --stage capture
python3 experiments/pose_reader_20261026/submit.py --stage audit --capture runs/<capture-snapshot>
# Only after independent data diagnosis and source review:
python3 experiments/pose_reader_20261026/submit.py --stage confirm --capture runs/<capture-snapshot> --audit runs/<audit-snapshot>
```

The capture runner reuses1837's unchanged `run_sim` initializer. It substitutes
only that process's startup reader entry, and exits via the initializer's
control-only completion exception before `IsaacIO`, `JointExecutor` or
`SharedStack` can be constructed. Those constructors are explicitly forbidden
in this probe. `capture_complete.json` records saved observations;
`process_complete.json` separately records normal shutdown. A timeout does not
become process-completion credit because capture data exists.

The manifest records the current diagnostic commit and exact copied historical
files. It overlays only this experiment and its reader/test changes. The
independent audit verifies manifest hashes, the historical base manifest,
raw conversions and residuals without importing the production reader.
