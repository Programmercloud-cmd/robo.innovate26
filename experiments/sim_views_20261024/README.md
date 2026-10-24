# Individually selectable simulation views

The user confirmed that the ten layers share one center and alternate 90°.
`geometry.json` therefore explicitly defines **360 bricks**, six by six in each
of ten layers, centered on the 32-stud anchored source plate. Relative origins
alternate `(0,1)` / `(1,0)`, with four-stud plate border. Historical blocked
geometry files and earlier real-image experiments remain unchanged.

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
