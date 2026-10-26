Factory task: transfer **columns of four bricks** from the 4×3 source stacks to conveyor poses **7–12**. The annotated reference is `resources/factory_layout.jpg`: 1–6 mark source anchors; 13 is an additional, unassigned image reference. No coordinates or pitches have been inferred for this new layout.

This branch starts with an empty `positions.json`. All previous taught/input/output poses, trial logs and layouts are preserved on `task2/jacob/initial_task`. Common FR3 motion/recording tools and the robot's intrinsic URDF model are retained. The old a/b/c/d aliases and old workspace bounds are unavailable here.

After positioning the arm and holding it still, record:

```bash
source /opt/ros/humble/setup.bash
/usr/bin/python3 -s teaching/record.py 1 --label 'factory_source_1'
```

Recording reads only: it sends no arm, hand, homing or recovery commands. The inherited convention is **15 mm above the grasp reference**. Use `--above-grasp-mm 0` if teaching the actual grasp height directly, or another explicit value if needed. `--replace` intentionally reteaches an existing factory point and archives its previous measurement. Positive pose IDs are accepted, including additional points beyond the photo's labels.

`motion_config.json` sets **66 mm approach/open/release** and **60 mm position-controlled closing**. Gripper previews do not connect to hardware:

```bash
/usr/bin/python3 -s teaching/gripper_width.py approach
/usr/bin/python3 -s teaching/gripper_width.py close
```

Factory column gripping is untested. A completed finger-position command does not prove that four bricks were picked. The contact-width acceptance threshold is 65 mm (below the open target), and remains configurable.

Arm execution starts disabled because this workcell's bounds are untaught. Once poses exist, `configure_workspace.py` displays bounds derived from those new upper/lower references with a default 100 mm margin and 500 mm ceiling. Check clearance for this layout, then use `--apply` to configure them. It never connects to the robot. Normal previews use `move_xyz.py 1`; add `--execute` for arm motion. `--grasp` opens to 66 mm, approaches and descends without closing. Failed motion, timing, tracking or singularity checks stop execution; no fault-clear commands are sent.

Rebuild helpers, if needed, with `bash teaching/build_motion.sh`. The 1 rad/s joint cap, 20 ms callback guard, joint-position rate limiting and full-path singularity checks are retained. New poses supply all workspace coordinates and planner seeds.
