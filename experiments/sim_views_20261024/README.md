# Individually selectable simulation views

**Pending contact-pattern clarification.** Job 1677 was stopped when the user
added edge/corner contact counts inconsistent with the current regular centered
layout. No completed renders are claimed. Camera presets and renderer are ready;
`geometry.json` now gates further submissions before Isaac starts. The original
attempt remains under `runs/20261024T094826.076320Z-sim-views/`, with its stopped
status in [attempts/1677.json](attempts/1677.json).

The user confirmed that the ten layers share one center and alternate 90°.
`geometry.json` therefore explicitly defines **360 bricks**, six by six in each
of ten layers, centered on the 32-stud anchored source plate. Relative origins
alternate `(0,1)` / `(1,0)`, with four-stud plate border. Historical blocked
geometry files and earlier real-image experiments remain unchanged.

This candidate already has two empty stud columns between parallel brick rows.
For full 2×4 bricks touching end-to-end at four-stud pitch, however, it yields
one distinct overlapping brick per adjacent layer for every brick. The newly
reported two edge contacts / one corner contact therefore needs clarification
(distinct bricks versus studs; one layer versus above and below together;
whether all rows contain six full end-to-end bricks). No alternative layout or
smaller brick count is substituted.

The existing FR3/BrickSim scene, pinned native dependency and Isaac runtime are
reused. Five virtual cameras provide overhead, slightly oblique and side views.
They share resolution, nominal lens settings, fixed lighting and one initialized
physics state. These are approximate virtual views, not calibrated D435i poses.
No robot trajectory or hardware motion is generated.

From the repository root, after committing the source/configuration:

```bash
# All views, each as its own RGB image, plus a comparison panel.
python3 experiments/sim_views_20261024/submit.py

# One view on its own.
python3 experiments/sim_views_20261024/submit.py --views top

# An explicitly selected pair.
python3 experiments/sim_views_20261024/submit.py --views top,oblique_front
```

Available names: `top`, `near_top_x`, `near_top_y`, `oblique_front`,
`oblique_side`. Camera offsets, up vectors, lens settings and lighting are
editable in `cameras.json`; every submission freezes a new snapshot and reports
its default Pueue job ID. No global queue or shared dependency settings change.

All views are captured with render-only calls after initialization; graph,
pose and physics-time checks verify the same state. All 360 bricks and their
native inter-brick/baseplate connections are required; no smaller fallback is
used. RGB images and their panel are separate from depth, poses, identities and
other evaluation truth. The panel is for visual comparison, not fused perception.
No detection, training, ROS integration or manipulation is started by this task.
