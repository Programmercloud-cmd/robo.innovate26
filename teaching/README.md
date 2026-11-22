Factory task: transfer **columns of four bricks** from the 4×3 source stacks to conveyor poses **7–12**. The annotated reference is `resources/factory_layout.jpg`: 1–6 mark source anchors; 13 is an additional, unassigned image reference. The source grid now contains 18 poses labelled `n_j` (n=0–5, j=0–2).

This branch uses a fresh factory `positions.json`; only factory taught and derived poses are present. All previous taught/input/output poses, trial logs and layouts are preserved on `task2/jacob/initial_task`. Common FR3 motion/recording tools and the robot's intrinsic URDF model are retained. The old a/b/c/d aliases and old workspace bounds are unavailable here.

After positioning the arm and holding it still, record:

```bash
source /opt/ros/humble/setup.bash
/usr/bin/python3 -s teaching/record.py 0_1
```

Recording reads only: it sends no arm, hand, homing or recovery commands. A stationary, fault-free user-stopped arm is also accepted for teaching; motion commands still require Idle and do not release the stop. Factory poses are taught **directly at grasp height**, with a default offset of **0 mm**. The saved pose and grasp reference are identical. An explicit `--above-grasp-mm` override is available only if a future pose is deliberately taught above its grasp height. `--replace` intentionally reteaches an existing factory point and archives its previous measurement. Source IDs use `n_j`; numeric conveyor/reference IDs remain available. Numeric image labels 1–6 resolve to center poses `0_1`–`5_1`, respectively.

`motion_config.json` sets **70 mm approach/open/release** and **60 mm position-controlled closing**. Gripper previews do not connect to hardware:

```bash
/usr/bin/python3 -s teaching/gripper_width.py approach
/usr/bin/python3 -s teaching/gripper_width.py close
```

The resident factory runner verifies loaded contact above 61 mm, retained within 1 mm through a measured full lift, then verifies all four bricks at placement. Empty closure is not accepted as a pick. Geometry, widths and source heights remain in configuration.

The current workcell has configured bounds and measured references; each requested move still needs preflight. For a new layout, `configure_workspace.py` displays bounds derived from those new upper/lower references with a default 100 mm margin and 500 mm ceiling. Check clearance for this layout, then use `--apply` to configure them. It never connects to the robot. Normal previews use `move_xyz.py 0_1`; add `--execute` for arm motion. `--grasp` opens to 70 mm and approaches the directly taught grasp pose without closing; it adds no descent for a zero-offset pose. Failed motion, timing, tracking or singularity checks stop execution; no fault-clear commands are sent.

Rebuild helpers, if needed, with `bash teaching/build_motion.sh`. The 1 rad/s joint cap, 20 ms callback guard, joint-position rate limiting and full-path singularity checks are retained. New poses supply all workspace coordinates and planner seeds.

The relocated source grid uses the taught `5_0` anchor and the source Z stored in `positions.json`; the conveyor cloth correction does not alter source heights. Targets are aligned to the robot world frame with tool Z down, tool X along world +X, and no added grasp offset. For `n=0..5`, `m=0..2`: **X(n,m) = X(5,0) − m·31.8 mm**, **Y(n,m) = Y(5,0) + (n−5)·78.2 mm**, and Z is constant. All geometry settings are in `positions.json → factory_source_grid.config`. `infer_source_poses.py --write` regenerates the other 17 poses and archives the prior catalog. Raw measured XYZ/orientation and joint angles are preserved at `5_0`; its measured Z is separate from the inherited commanded Z, and its joints are only an IK seed. Other poses contain no fabricated measured joints. Changing the anchor invalidates derived targets until regeneration. This geometry update sends no robot commands. Execution is disabled while teaching; software bounds have been regenerated and each later requested move still needs preflight.

The camera-guided row-0 development runner and its measured results are described in [factory_vision/README.md](factory_vision/README.md). The requested destination remains +85 mm in world X. Historical reachability results describe the earlier layout. The current resident runner validates its saved calibration, controller binary and trajectory commissioning records before execution.

Select the current stationary factory setup explicitly. With `5_0` physically restored, an empty gripper, the conveyor lane clear, and the robot ready for execution:

```bash
./run_rows_tabletop.sh --execute-task 5_0
# Or, for the physical conveyor (+25 mm world X, +55 mm world Z on placement/home):
./run_rows_conveyor.sh --execute-task 5_0
```

Add `--validate-only` for model/path validation with no SDK connection or motion. See [factory setup checks](factory_cycle/SETUPS.md) for the exact demo-mode bypasses, retained protections, geometry and recovery. Both scripts share the resident `Factory` pipeline and exclusive arm/hand SDK owner; the twin receives its measured state. Normal execution and bounded recovery need no assistant stage calls. Without `--execute-task`, optional visual and schedule gates remain enabled. The stationary-placement schedule is row at 5.0 s, release verified at 12.5 s, and stationary home at 15.0 s; demo mode bypasses those deadlines. Historical timings remain in `factory_cycle/SYNCHRONIZED_BENCHMARK.json` and `factory_cycle/BENCHMARK.json`; they are not new measurements for these profiles.

Active mode is `factory_vision/config.json → conveyor_placement_mode = stationary_vertical`. The robot places vertically at the conveyor point directly below the layer home, with no forward tracking. Opening remains position-controlled to 70 mm at 40 mm/s; closing remains 60 mm. The same file contains `phase_schedule`, the conveyor-only surface correction and the perspective-aware placement crop. Certified time stretching and checked stationary holds align the phase markers; late work never causes faster motion or skipped checks. The clock is local monotonic time, without an external PLC trigger.

The complete reusable speed-aligned configuration is saved in `moving_conveyor/profiles/speed_matched_20261027.json`, alongside retained planner/controller code. Restore its `factory_config` and `conveyor_config` objects to the corresponding configuration files to re-enable the previously validated 0.015 m/s tracking mode. That profile has a 0.020 m/s upper guard and 52 mm forward stroke. The older standalone conveyor runner remains disabled; use the resident entry above.

A transient placement-view failure gets fresh observations and at most two predefined reframe moves. If uncertainty remains, inventory does not advance. A stopped release may retreat home only with fresh open-hand telemetry, stationary Idle and no robot errors; the same SDK owner waits for the hand action before retreating. Actual Reflex/user stops are not silently cleared. Inspect the recorded `inventory.json.active` state before restarting. After physically restoring the selected source, clearing the lane and returning the robot to home/open/Idle, add `--source-reset-confirmed` to acknowledge that reset. This flag never opens an uncertain grasp or overrides robot faults. Logs and recoverable-stop reasons are under `factory_cycle/runs/`.
