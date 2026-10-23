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

## v0 feature diagnostic and proposed v1

- Diagnostic: Pueue **1668**, source `94cd781`, immutable run
  `runs/20261023T234904.706461Z-geometry-v0-stud-diagnostic/`.
- Evaluation-only probes at all 32 development reference stud centers: every
  fixed-center v0 score was zero; even the existing local search reached only
  0.113 maximum, below the frozen 0.35 feature confirmation threshold.
- Observability median was 0.786. Radial-gradient mean was below the
  whole-patch mean plus two at every measured true center; the original
  concentration formula therefore clipped the response at those marked
  locations to zero. A later audit limits their interpretation as centers.
  Merely expanding registration search is not supported as the first change.
- Hypothesis: normalize radial alignment against gradient magnitude on the
  same annulus instead of comparing to the seam/shadow-contaminated whole-patch
  mean. Use the isotropic-gradient null (2/pi), angular support relative to the
  robust local median, and the existing observability gate.
- Proposed single code change: replace the stud response calculation only.
  Radii, search bounds, global registration and decision thresholds remain
  unchanged. A 0.5 gradient-unit quantization floor is explicitly documented.
- Independent reviewer endorsed this bounded test. Acceptance still requires
  useful decision coverage with controlled errors on the identical development
  protocol; nonzero feature scores alone do not establish improvement.
- Diagnostic evidence: `results/v0_stud_diagnostic/`. Implementation and
  post-change measurements are pending this checkpoint; held-out D remains
  unopened for tuning.

## v1 radial-coherence result — retained working baseline

- Pre-change evaluated checkpoint: `817dbdc`; implementation: `4106bdc`.
  Pueue **1669**, snapshot
  `runs/20261023T235320.974842Z-known-layout-v1-development/`.
- Change: the single radial-feature correction described above, selected by an
  explicit configuration field. V0's formula remains reproducible. All radius,
  search, registration and decision thresholds were unchanged; frozen
  references/protocol hashes were verified unchanged.
- Nominal development: **37/112 correct present (33.0%)**, up from 0/112;
  69 uncertain and six unmatched. Zero wrong present/missing decisions.
  Conditional present precision is 100% on 37 confirmations, not a guarantee.
  All six supplementary missing cells remain uncertain. Blank confirmations: 0.
- Correct-present sensitivity counts out of 112: dark 17, bright 47, rotations
  -5/+5 degrees 29/30, x initialization -4/+4 px 42/27, y -4/+4 px 33/25.
  No wrong confirmations were measured in these cases. Coverage is inadequate
  and lighting/alignment sensitivity remains material.
- Registration residual median 6.31 px (32 references), compared with 5.97 px in
  v0: no localization improvement is claimed. Image-derived stud peaks matched
  14/32 selected references within 8 px, median 6.25 px for those matches only.
- Decision: **retain** the small feature revision as a better working baseline
  because it produces useful correct confirmations without measured false
  confirmations. It does **not** meet the 80% coverage or missing-decision
  demonstration targets. The complexity is justified by the measured gain,
  not by the feature's mathematical plausibility alone.
- Dominant remaining failure: incomplete stud support/low mean evidence under
  the coarse geometric registration. Image 10 is particularly weak. Strong
  registration consensus sometimes results in zero correction; assess its
  capture/fit bounds before proposing one bounded registration revision.
- Evidence: `results/v1_development/`; held-out D remains reserved.

## Reference audit and proposed v2 confidence simplification

- Independent evaluator and reviewer found that some historical stud marks
  lie on cap rims/highlights, not validated centers. All labels, hashes and
  scoring rules remain unchanged. Stud distances must be reported as distances
  to historical marked points, not true center accuracy. This also limits the
  true-center interpretation of the v0 diagnostic. Face polygons/decision
  comparisons remain usable at their documented annotation uncertainty.
- Reconstructing v1's saved feature offsets disproved the suspected translation
  norm-limit failure: image 1's 117-inlier proposal and image 7's 82-inlier
  proposal were both zero. No registration-bound change is justified by that
  mechanism, and none was implemented. Audit evidence is in
  `results/reference_audit/`.
- One predeclared confidence diagnostic on the frozen v1 nominal outputs found
  11 additional positive faces passing every present-condition except the
  whole-face mean-stud threshold, with none of the six supplementary missing
  faces passing those other conditions. This is a diagnostic upper bound,
  not an evaluated post-change result or a held-out claim.
- Hypothesis: six supported studs including both end rows, edge support,
  observability and top/lower counter-evidence already provide the intended
  partial-pattern criterion. Requiring the mean of all eight scores to exceed
  a higher threshold redundantly rejects some such patterns.
- Proposed v2: remove only the mean-score decision predicate, keeping that score
  visible diagnostically. Preserve all other feature, registration and decision
  conditions and the exact evaluation protocol. The independent reviewer
  endorsed the test. Expected nominal gain is only about 48/112, still below
  demonstration acceptance. Accept only after the full sensitivity/blank batch
  confirms useful gain without false-confirmation regressions.
