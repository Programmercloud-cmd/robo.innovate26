# Task 2 — physical LEGO sorting

This directory contains the two original, independent hardware workstreams. Neither has been merged into the other.

| Workstream | Entry point | Original branch |
| --- | --- | --- |
| Nitish: RGB-D perception, hand–eye calibration, Franka/MoveIt sorting | [Franka LEGO sorter](codex_stuff/franka_lego_sorter/README.md), [RealSense calibration](codex_stuff/franka_realsense_handeye_ros2/README.md) | [task2/nitish](https://github.com/Programmercloud-cmd/robo.innovate26/tree/task2/nitish) |
| Jacob: taught poses, control and camera tests | [Teaching](jacob/teaching/README.md), [Camera test](jacob/quick_test/README.md) | [task2/jacob](https://github.com/Programmercloud-cmd/robo.innovate26/tree/task2/jacob) |

Historical robot poses, calibration and configuration are retained for reference. They are not current commissioning evidence. Review the source-specific README and validate the physical setup before operating hardware.

The separate [Isaac Sim workcell](../simulation/README.md) models the LEGO task. No verified automatic physical-object-to-Isaac scene synchronization is provided.
