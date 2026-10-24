# Bounded partial-disassembly simulation benchmark

Planning/configuration is frozen before implementation in `benchmark.json`
and `evaluation/protocol.json`. The original 300-brick design remains intact
as the source geometry. Each static case removes only specified layer-9 bricks,
retaining all lower layers and genuine native connections.

Ten state/color configurations use the same fixed overhead and oblique-side
cameras. Eight configurations are development data; scattered removals and
three isolated remnants are reserved as entire geometry groups, including
both views. Paired intact and adjacent-removal palettes test color effects
without a full factorial expansion. Five development sensor-corruption controls
need no additional rendering.

The unchanged detector/configuration must run first. At most three subsequent
improvement rounds each address one observed development failure with one
minimal independently reviewed change. Each round is frozen and queued before
processing; recovered/lost faces and confidence errors are compared explicitly.
The ledger in `rounds.json` records this budget and final release.

Reserved sensor images, predictions and scores remain outside implementation
decisions until the final method selection is frozen. Baseline and retained
versions are then evaluated on reserved configurations without further tuning.

All methods are stateless and views are evaluated independently. Combined-view
and temporal results are explicitly not measured. No training, manipulation,
ROS integration, hardware control, dependency installation or scheduler changes.
All rendering and batch perception/evaluation uses `pueue add` in the default
queue with stable source/configuration/input snapshots and reported job IDs.

Static initialized native graphs are validated; load-bearing dynamics are not.
Simulator inventory, removal state, identities, colors and actual poses are
confined to rendering/evaluation. Perception receives sensor observations,
nominal calibration and allowed geometric class priors only.

## Reproduction and outputs

Use `/home/faps/miniforge3/envs/unitree_sim_env/bin/python` for queued jobs.
Submission helpers perform only lightweight freezing/copying and call
`pueue add` in the default queue. Commit scoped changes before submission.
Never run their generated `launch.sh` directly outside Pueue.

Recorded submission pattern (before reserved release; the completed ledger
deliberately prevents starting another development batch):

```bash
python3 experiments/partial_disassembly_20261024/submit_render.py
python3 experiments/partial_disassembly_20261024/freeze_dataset.py --render-run <completed-render-snapshot>
python3 experiments/partial_disassembly_20261024/submit_perception.py --version B0
python3 experiments/partial_disassembly_20261024/submit_perception.py --version R1 --previous-evaluation <B0-snapshot>/output/evaluation/evaluation.json
python3 experiments/partial_disassembly_20261024/submit_perception.py --version R2 --previous-evaluation <R1-snapshot>/output/evaluation/evaluation.json
python3 experiments/partial_disassembly_20261024/submit_perception.py --version R3 --previous-evaluation <R2-snapshot>/output/evaluation/evaluation.json
```

Reserved evaluation adds `--split reserved`, after final method selection is
committed; parent reports must come from the reserved split. The submission
helper enforces the round budget and release guard. Do not rerun development
or choose parameters after release.

Each immutable `runs/*-partial-*` snapshot contains `manifest.json`, exact
source/configuration/input hashes, `submission.txt`, and a foreground
`launch.sh`. Perception outputs retain `detections.json`, overlays and
world-XY masks for each opaque observation ID. Private observation-to-case
mapping and truth are confined to `scoring_only`; quantitative results and
evaluation-only references are in `output/evaluation`.

The reporting helper `evaluation/summarize.py` aggregates saved associations
without rescoring. `package_report.py` makes before/after panels from saved
images. Both require a queued execution. The configuration manifest, tables,
exact run/job registry and final artifact location are recorded after the
bounded experiment completes.
