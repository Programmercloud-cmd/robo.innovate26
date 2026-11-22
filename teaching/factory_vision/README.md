Camera-guided four-brick column transfer for the factory task. Only stack **5** is currently populated: `5_0`, `5_1`, `5_2`. Geometry remains in `../positions.json`; active camera, servo and fixed conveyor settings are in `config.json`.

Every column goes to the **same fixed conveyor pose**, captured from the first successful `5_0` placement: XYZ ≈ **[698.630, 91.245, 8.184] mm**. The 85 mm offset described that first destination; it is not added to every source. The conveyor must be cleared between deliveries. Before picking, the wrist camera checks the known delivery footprint is empty; an occupied or uncertain view blocks another delivery.

```bash
source /opt/ros/humble/setup.bash
# Camera observation only.
/usr/bin/python3 -s teaching/factory_vision/run.py observe
# Align, grasp and deliver an unprocessed source to the fixed conveyor.
/usr/bin/python3 -s teaching/factory_vision/run.py align 5_2 --execute --close --transfer
# Alignment only, remaining raised and open.
/usr/bin/python3 -s teaching/factory_vision/run.py align 5_2 --execute
```

The Logitech wrist camera observes from 50 mm above grasp height. The detector fits the four-brick footprint, rectifies it, checks its 8×4 stud grid, and requires agreement across three fresh frames. A measured XY/yaw image Jacobian guides bounded corrections. Tracking can bridge a faint seam after a small correction, but fresh stud evidence is still required for every brick. Blank views remain uncertain. RealSense is not required for this baseline.

The sequence is: check conveyor clear → approach/align → preflight destination → descend to saved grasp Z → command 60 mm finger position → verify all four brick regions during a 5 mm lift → lift to clearance → deliver to the fixed pose → open to 70 mm → retreat → verify four bricks and matching per-brick appearance. Position closure alone never counts as a pickup. Measured contact widths around 65.2 mm are accepted up to 65.5 mm; the close command remains 60 mm. Existing full-path, singularity, workspace, tracking, callback and robot reflex checks remain active. Local vision moves accept at most 1.5 mm endpoint error; ordinary moves retain their 1 mm setting.

`progress.json` records completed conveyor deliveries only after release/placement verification. Uncertain holding or placement blocks a new automatic opening. `runs/` holds images, overlays, observations and outcomes; arm/hand logs remain in `teaching/moves/` and `teaching/gripper_moves/`. No automatic fault recovery or release follows a failed check. Inspect the physical state before clearing an active operation. Never reset progress just to bypass a failed check or pick an already moved column.

For a physically replenished source, add `--restored-source`: the earlier delivery is retained in history and the source is rearmed only after fresh four-brick alignment evidence. The tabletop color mask excludes the warmer shadows seen under the added light. A partly visible seam requires the opposite boundary, both outer sides, at least 24 studs with five on each brick, and the existing three-frame stability check. The saved 21 calibration images were reprocessed; the largest equivalent XY change was 0.448 mm and fit residual was 0.241 weighted pixels. These are calibration checks, not measured placement accuracy.

Recovery from the recorded misplaced `5_1` location uses `--source-placement-run teaching/factory_vision/runs/20261026T181047.056458Z`. A retained grasp can use `resume-transfer --execute` only after a successful saved four-brick verification lift and matching stationary robot/hand state.

The calibrated reference is `5_0`; calibration is tied to camera geometry, the reference pose and the detector. To recalibrate, restore/teach a physical reference and run `calibrate --execute`. Do not recalibrate over an emptied source. Actual results and remaining limitations are in `development_results.json`; strict tracking can stop on initial grip settling, and precise stud connection or placement accuracy is not established.
