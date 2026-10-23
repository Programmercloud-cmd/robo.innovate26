# Iteration record

## Historical separation

- Commit: `2a3431a` — workcell source and historical pretrained experiment.
- Evidence: 18 images, two pretrained pairs, canonical Pueue job 1666; neither
  pair provides reliable individual-brick recall.
- Larger weights, generated runs and original resources remain on disk with
  locations/hashes retained. No deletion or unrelated cleanup.
- Simulation comparison was incomplete; exact original 360-brick geometry is
  still a dependency, not silently approximated.

## Geometric baseline v0

- Status: implementation and independent protocol preparation authorized.
- Scope: manual coarse global registration, supplied observed grid/plane,
  projected stud/brick geometry, bounded refinement, grayscale stud and edge
  evidence, local lower-pattern counter-evidence, three-way decisions.
- Evaluation, source commit, Pueue ID and outcomes will be recorded after the
  first frozen run. This entry is not a claim of measured performance.
