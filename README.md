Factory setup for four-brick column transfers.

The active pose set is isolated from the previous task; new poses are taught directly at grasp height with zero offset. See [teaching/README.md](teaching/README.md) to begin teaching poses against [the new annotated layout](resources/factory_layout.jpg). Gripper setpoints are 70 mm open and 60 mm closed.

The complete previous task is saved on `task2/jacob/initial_task`; this branch is `task2/jacob/factory`. Other contributors' files under `task2/` are unchanged.

Choose the stationary-placement setup explicitly:

```bash
./run_rows_tabletop.sh --execute-task 5_0
./run_rows_conveyor.sh --execute-task 5_0
```

Both use the same resident pipeline. The physical conveyor adds 25 mm world X and 55 mm world Z to placement/home only. Add `--validate-only` to either command for model-only path checks with no hardware connection. The old `run_rows.sh` exits with a setup-selection message. See [setup checks and recovery](teaching/factory_cycle/SETUPS.md).

Whole-layer stacking is isolated under [teaching/layer_stacking](teaching/layer_stacking/README.md). `./run_layers.sh --plan` saves provisional board targets; `./run_layers.sh --simulate` runs the deterministic edge-alignment/seating software fixture. Physical execution requires separately taught whole-layer poses and actual camera/contact commissioning; synthetic fixture values never enable hardware.
