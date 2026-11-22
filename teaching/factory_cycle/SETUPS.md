Select a setup explicitly; all three launchers accept the same arguments and share `run_rows.py` / `Factory`:

```bash
./run_rows_tabletop.sh --execute-task 5_0
./run_rows_conveyor.sh --execute-task 5_0
./run_rows_conveyor_moving.sh --execute-task 5_0
```

All three launchers also accept `--all` instead of explicit column IDs, for example `./run_rows_conveyor.sh --execute-task --all`. It expands exactly once to `0_0 0_1 0_2 1_0 ... 5_0 5_1 5_2`; explicit IDs cannot be combined with `--all`. All normal per-column checks and operator conveyor-clearing responsibilities are unchanged. Add `--validate-only` to check all paths without connecting to the robot.

`setups.json` stores the original tabletop placement transform and the two offsets. Tabletop uses zero offset. Physical conveyor adds **[25, 0, 55] mm world XYZ** to placements and homes, once, in memory. Its operator-confirmed height teaching adds a further **1.515 mm to placement Z only**, targeting the measured stopped release height of **63.588 mm**. This bounded local correction includes capture provenance and is printed before execution; it never changes home Z or accumulates on repeated launches. Shared coordinates are never rewritten when switching. Source poses, source observation/grasp heights, orientation, 70/60 mm widths and cycle sequence are identical. Both stationary setups place vertically on a stopped conveyor. The existing conveyor-only cloth correction is retained for both profiles; it never affects source heights or the home lift plane.

`conveyor_moving` inherits the physical conveyor geometry, including its measured release height. It reuses the curved descent/release/lift pipeline: smoothly accelerate in world +Y while descending, reach **0.020 m/s** before contact, maintain speed at taught height throughout opening, then lift while continuing forward and return home. Forward travel is **66 mm**; the tracking segment lasts **3.6 s**. Normal mode enforces a **0.015–0.025 m/s** measured gripper release-speed corridor. `--execute-task` ignores speed matching as a release/readiness/continuation gate, while retaining speed measurements and warnings; it never labels mismatched speed as verified. Opening confirmation/deadline, contact-height, stroke, tracking and fault checks remain enforced. The 750 ms opening window retains the existing 550 ms measured-latency bound plus command margin. These are configured limits, not a new physical benchmark. Stationary profiles and archived shared tracking settings are unchanged. Rebuild only the separate moving controller with `bash teaching/factory_cycle/build.sh --moving-conveyor`.

The original home lift plane is preserved: home Z is source grasp Z + the configured 50 mm observation height; physical conveyor adds 55 mm. This restores the actual previously working home rather than introducing another height change. For row 5, tabletop placement/home Z are 7.074/55.428 mm; physical conveyor uses 63.588/110.428 mm. Both profiles retain the authorized 500 mm ceiling. The conveyor profile extends only the placement-side X maximum by its approved 25 mm translation; the remaining source bounds stay unchanged. Bounds are limits, not obstacle detection.

Before execution the runner prints setup/offsets, validates all requested source/home/placement paths, startup and row transitions, and then acquires one resident SDK owner. Caches include resolved geometry, setup, motion limits and workspace. The native controller independently checks loaded paths and live motion. Measured arm/hand state continues through its existing twin telemetry tap.

Startup adapts its first path to the live FCI command after `begin`, including small reference shifts that consume derivative headroom. The same adapter operates in normal and task mode, automatically retiming and printing a warning when needed. It preserves the target and the 1 rad/s, 2 rad/s² and 40 rad/s³ limits. `STARTUP_BRIDGE_VALIDATION.json` records replay of the measured startup failure; no physical retest is implied.

`--execute-task` bypasses optional camera startup/calibration/alignment, source/destination visual occupancy, image-based pick/placement verification, schedule deadlines, and historical geometry commissioning records for these three explicitly supported profiles. It acknowledges an empty gripper and operator-managed conveyor clearing. `--verify-placement` is overridden in this mode. The extra 1 mm placement accuracy check becomes a printed/logged warning within the existing native **1.5 mm** endpoint/height guard; larger discrepancies still stop. Moving release logs X/Z discrepancies from its native trace. Grasp endpoints retain the strict 1 mm gate. Other commissioning workflows are unchanged.

Still enforced: valid profile/geometry; fault-free stationary Idle startup; joint/Cartesian workspace bounds; full-path IK and singularity checks; velocity/acceleration/jerk limits; native path and live tracking guards; strict grasp endpoints and bounded placement height; measured loaded contact width and retention through the full lift; measured opening; final home/open-hand checks; heartbeat/SDK-stop handling; exclusive SDK ownership. Moving placement enforces speed matching in normal mode; task mode records speed without using it as a stop condition. Both require verified opening, its deadline and measured 50–70 mm travel. An unresolved operation remains pending and never counts as a successful placement. Camera-disabled results record release evidence separately from visually verified placements.

Model-only validation, without an SDK or camera connection:

```bash
./run_rows_tabletop.sh --validate-only --execute-task 5_0
./run_rows_conveyor.sh --validate-only --execute-task 5_0
./run_rows_conveyor_moving.sh --validate-only --execute-task 5_0
```

This uses the saved teaching tool/joint seed, not fresh robot state. Live execution repeats preflight from a fresh stationary state and validates paths again in the native controller. Logs: `runs/rows_*/preflight.json`, per-column `display_geometry.json`, `commissioning_policy.json`, and `outcome.json`.

After a stop, inspect `inventory.json.active` and the physical arm/hand/source/conveyor before retrying. `--execute-task` explicitly acknowledges the hand is empty; do not use it to restart while holding bricks. Pending history is archived only after fresh Idle/open-hand checks, with no prior success inferred. Faults and user stops are not silently cleared.
