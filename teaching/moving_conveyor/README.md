Conveyor placement now always means speed matching: **world +Y at0.12m/s while lowering and opening**, followed by return to the conveyor home100mm above the reference. There is no stationary fallback in the factory transfer command. This is still a stationary tabletop, with no real belt synchronization.

Current state: **loaded execution disabled** pending a corrected timing trial and operator robot recovery. The previous empty-hand test measured mean0.11936m/s and66.66mm travel. The loaded `5_1` trial measured mean0.11994m/s during its matching interval, but opening confirmation took304ms and missed the270ms deadline. It stopped with a communication fault and did not advance inventory. Four bricks were visible afterward; proper moving placement was not verified. Records: `../factory_vision/runs/20261026T201057.371864Z/conveyor_motion/`.

The SDK raises caller thread priority at robot construction. Telemetry and hand workers now explicitly switch to ordinary priority before use; the extra concurrent hand UDP observer was removed. The priority regression probe passes without connecting to the robot.

The revised profile reserves350ms for opening, uses210ms smooth ramps and commands67.2mm total +Y travel. Its local jerk limits differ from ordinary transfers; the1rad/s joint speed cap and2rad/s² acceleration bound remain. `prepare.py` certifies the complete spline and the C++ `--check` independently validates FK before any execution. `--timing-test` is an explicit empty-hand opening trial at an elevated reference, at least50mm above robot-base Z0; it does not claim a loaded placement.

```bash
source /opt/ros/humble/setup.bash
bash teaching/moving_conveyor/build.sh
/usr/bin/python3 -s teaching/moving_conveyor/run.py --layer 5
```

Factory delivery after commissioning uses `teaching/factory_vision/run.py align 5_1 --execute --close --transfer`; a replenished source additionally needs `--restored-source`. Inspect active inventory and the physical scene before resuming a failed release. Never clear progress or enable timing validation merely to bypass a failed check. No automatic robot fault recovery is sent.
