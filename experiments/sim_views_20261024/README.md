# Individually selectable simulation views

Completed captures: [five individual views and comparison panel](REPORT.md).
Job 1680 passed capture validation; stalled Isaac cleanup was stopped separately.

**Geometry clarified by the user's close-up.** Dots mean engaged studs, rather
than neighboring bricks. The visible one-stud stagger supersedes the earlier
rough common-center interpretation. `geometry.json` defines **360 bricks**,
six by six in each of ten alternating 90° layers, with two empty stud columns
between parallel rows. Relative origins alternate horizontal `(1,0)` / vertical
`(0,1)`, with four-stud padding on the 32-stud anchored source plate.

Adjacent layers have 121 single-stud overlap patches. Each layer's total-contact
distribution is one brick with one engaged stud, ten with two and 25 with four;
the photo's local description is not imposed as a universal all-four-corners
rule. The layer centers differ by two studs in each axis. The 25×25-stud union
fits the plate on its legal stud grid without a half-stud recentering. All native
connections are retained: 1089 interlayer plus 36 plate links, 361 connected
parts including the plate. Source-photo path and hash are in the geometry file.

Job 1677 was stopped before completed renders while the dots/contact distinction
was being resolved. Its snapshot and [attempt record](attempts/1677.json) remain.
It also emitted native solver convergence warnings during initialization; that
attempt is not a successful physics validation. The current view comparison uses
`settle_steps=0`: the complete native graph is initialized, then captured with
render-only calls. **Static initialized renders do not validate dynamics or
load-bearing clutch stability.** Earlier experiments remain unchanged.

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

Attempt 1678 stopped during native graph initialization before producing validated
images. The retry disables automatic assembly only inside the render process,
before explicitly creating all 1,125 native links. Every other threshold and
breakage check remains unchanged. Progress and failure diagnostics are saved
before application shutdown. This is a static preassembly initialization fix;
it does not establish physical stability.

Attempt 1679 produced diagnostic frames and the complete native graph, but was
rejected because reset had displaced bricks by up to 16.6 mm / 28.2 degrees.
Its [attempt record](attempts/1679.json) and run directory are preserved. Capture
now restores the configured lattice after reset and checks actual PhysX and USD
poses before/after native graph authoring and throughout capture. Oblique views
look 4 cm below the top surface to include the full stack height.
