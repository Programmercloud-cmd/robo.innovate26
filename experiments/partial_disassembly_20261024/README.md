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
