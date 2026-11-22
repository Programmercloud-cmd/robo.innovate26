Independent empty-hand wave; no factory configuration, caches, inventory or simulation selection is changed.

```bash
bash teaching/wave_demo/build.sh
./run_wave.sh                       # model and native checks only; no SDK
./run_wave.sh --execute             # explicit physical execution
./run_wave.sh --cycles 2 --period 6  # same model-only preview, adjustable repetitions
```

Execution requires fault-free Idle, an already open gripper (at least 69 mm), and the configured tool frame. It sends no gripper commands and never clears robot faults. Start in the vertical column above the saved physical-conveyor home 5: ±50 mm in world X/Y, with Z between home-50 mm and the demo ceiling. The checked approach remains inside that column; the start need not be near the home height. The script moves to exact home 5, rises vertically to world Z 400 mm, straightens to an upright bent-elbow posture, waves along world Y, then reverses the route to home 5. Ctrl-C requests an SDK stop. Inspect after any stop; restarting requires returning inside the home-5 column.

`config.json` is the isolated home/tool snapshot and demo configuration. The fingers' length/tool-Z axis points world **+X** throughout waving, while the forearm swings in the Y–Z plane. Nominal sweep is ±34° relative to the upright forearm neutral; the measured orientation deviation is capped at 35° and finger-direction error at 1°. Defaults are three cycles; timing stretches automatically for joint and Cartesian limits. The new overhead workspace is explicitly separate from the factory bounds (ceiling 1.25 m). Review `runs/*/summary.json` for phase times and actual model-derived bounds. No physical wave has been demonstrated yet.

Planning and native execution check the **full 6×7 arm Jacobian**, joint limits, elbow bend, speed/acceleration/jerk, workspace and tracking. The factory's fixed-elbow 6×6 IK submatrix is not a singularity criterion for this joint-space wave. Robot collision/fault protections remain active; camera/cable clearance relies on the operator's stated clearance. One arm SDK lease and one persistent read-only hand connection forward measured state to the twin throughout execution. Logs stay under this directory.
