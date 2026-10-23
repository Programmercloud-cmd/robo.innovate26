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

- Status: evaluated; fails the coverage requirement and is not accepted as a
  demonstration baseline.
- Protocol checkpoint: `1dfeae8`; initial algorithm checkpoint: `7d6ae08`.
- Scope: manual coarse global registration, supplied observed grid/plane,
  projected stud/brick geometry, bounded refinement, grayscale stud and edge
  evidence, local lower-pattern counter-evidence, three-way decisions.
- Evaluation source: `2ccb3ac` (algorithm `7d6ae08`), Pueue **1667**, snapshot
  `runs/20261023T234525.001300Z-known-layout-v0-development/`.
- Fixed development batch: four images × ten cases. Nominal: 0/112 positive
  faces confirmed present; 106 uncertain and six unmatched. All six
  supplementary missing-face cells uncertain. No merges, duplicates or labeled
  lower-fragment selections; zero wrong decisions is explained by zero decisive
  coverage, not claimed reliability. All non-flat sensitivity cases also have
  zero confirmed present. Flat-image sanity: zero confirmed present.
- Projected-stud residual median 5.97 px, p90 9.10 px (32/32 manual development
  stud references), with no global registration correction. Median observability
  per image was 0.61–0.84, while accepted stud support was almost absent.
- Dominant failure: insufficient image-supported stud confirmation prevents both
  registration refinement and decisions, despite visible boundaries. The
  development registration error exceeds the initial local search radius;
  feature normalization could also suppress true studs. These explanations
  must be separated before changing the algorithm.
- Next bounded diagnostic: evaluate the unchanged feature at the 32 development
  reference stud centers, strictly in evaluation. This does not provide reference
  locations to the production perception path. Planner and reviewer both
  requested the diagnostic before proposing a single improvement.
- Evidence: `results/v0_development/`; large overlays/predictions stay in the
  frozen run. Held-out D remains reserved.
- Static validation: seven small tests passed, including the rule that a blank
  image with a supplied layout cannot confirm present or missing. This is not
  a substitute for the independent image batch.
