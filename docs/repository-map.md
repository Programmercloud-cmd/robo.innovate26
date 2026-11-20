# Repository consolidation and provenance

This repository contains independent hackathon workstreams. The organization pass moves tracked source and evidence into project directories without changing simulation, perception, robot, or rendering behavior. Run commands in the relevant project directory.

| Original branch | Source commit | Consolidated location |
| --- | --- | --- |
| [simulation/demo-progress-20261025](https://github.com/Programmercloud-cmd/robo.innovate26/tree/simulation/demo-progress-20261025) | [`5fcda264779a`](https://github.com/Programmercloud-cmd/robo.innovate26/commit/5fcda264779a675ae39bc3d7f511f6ceebc61d67) | `simulation/` |
| [task1](https://github.com/Programmercloud-cmd/robo.innovate26/tree/task1) | [`4e99c328cd21`](https://github.com/Programmercloud-cmd/robo.innovate26/commit/4e99c328cd2102a42f8358f755580d2f4934092b) | `task1/` |
| [task2/jacob](https://github.com/Programmercloud-cmd/robo.innovate26/tree/task2/jacob) | [`13f37dad1980`](https://github.com/Programmercloud-cmd/robo.innovate26/commit/13f37dad1980bcbc4491b7f27d8962602e49fea0) | `task2/jacob/` |
| [task2/nitish](https://github.com/Programmercloud-cmd/robo.innovate26/tree/task2/nitish) | [`6642362b9538`](https://github.com/Programmercloud-cmd/robo.innovate26/commit/6642362b953898a1d4fc3f91def2348dacef6ebc) | `task2/codex_stuff/` |
| [task3/jacob1](https://github.com/Programmercloud-cmd/robo.innovate26/tree/task3/jacob1) | [`22a5c6b938aa`](https://github.com/Programmercloud-cmd/robo.innovate26/commit/22a5c6b938aad47c08482352d7783580139a3408) | `task3/task3_checkpoint/operational_jacob1/` (standalone operational model); complete alternate tree stays on source branch |
| [task3/simulation-handoff](https://github.com/Programmercloud-cmd/robo.innovate26/tree/task3/simulation-handoff) | [`44bbbef5812f`](https://github.com/Programmercloud-cmd/robo.innovate26/commit/44bbbef5812f7b8b6f88af4961d81ba2aa254f5e) | `task3/` (primary adapted-layout snapshot) |

Task 3 uses the layout handoff snapshot, not a file-by-file merge of incompatible Greenfield variants. Jacob's operational package is copied as a separate checkpoint. The original branches, source histories and release assets are retained. Large recorded replays and experiment evidence are kept because they support reproducibility; generated runtime output should remain ignored as documented in each project.

## Integration status

- Task 1: camera-based industrial brick detection, tracking and good/bad classification. See its README for data/weights not included in Git.
- Task 2: physical LEGO workcell perception, calibration and robot-control workstreams. Physical commissioning and calibration are environment-dependent.
- Isaac Sim: geometry and synthetic sensor-based LEGO workcell experiments; not a live industrial factory twin synchronized with real conveyor slots.
- Task 3: industrial factory planning and simulated material flow; synthetic perception inputs and kinematic assumptions. The Unreal 30 film and adapted Greenfield 37 replay are separate versions.
- No verified automatic chain detects a new physical conveyor brick and updates its occupancy and color in Isaac Sim. This consolidation does not implement that feature.

Historical handoff documents may retain original paths, branch names and VM references. Treat them as provenance, not evidence that a service, machine or hardware configuration is still available.
