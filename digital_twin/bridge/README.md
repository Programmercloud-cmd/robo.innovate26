Read-only local telemetry for the Brev Isaac twin. The C++ helper calls `franka::Robot::readOnce()` only; the Python relay forwards measured joints, velocity, desired joints, measured TCP/tool frames, robot mode/errors and separately read hand width. No motion, configuration, recovery, homing or finger-position command is exposed.

```bash
source /opt/ros/humble/setup.bash
bash digital_twin/bridge/build.sh
ssh -T -N -o ExitOnForwardFailure=yes \
  -L127.0.0.1:18765:127.0.0.1:8765 \
  -L127.0.0.1:18766:127.0.0.1:8766 robot-innovate26-place-rl3
/usr/bin/python3 -s digital_twin/bridge/relay.py --output digital_twin/runs/live
```

The VM listener must already be running; see `../isaac/README.md`. Viewer: http://127.0.0.1:18766/. On reader exit, invalid/stale samples or a three-second reader stall, the relay permits five restarts with delays of 0.5, 1, 2, 4 and 8 seconds. Five seconds of continuous fresh readings resets that budget; rapid failures do not. Exhaustion exits with status 1 and records `reader_state: exhausted`. Network reconnection is independent and capped at a two-second backoff. `--duration` limits the whole relay session, including retries. Sequence numbers continue from the append-only telemetry log.

Hand telemetry uses one persistent receive-only `read_hand` process and one
`franka::Gripper` session. It yields to foreground arm/hand ownership and resumes
after release; failures have a separate bounded retry budget. Foreground hand
clients use `HandLease`, which signals demand before taking the exclusive lock.
During a factory controller session, only its timestamped hand measurements are
forwarded; background measurements never substitute for missing owner data.
`hand_stale: true` and `width_m: null` explicitly mean unavailable. The viewer
holds the last finger geometry but labels the hand stale separately from the arm.

Pause/resume **only background hand polling**, without restarting arm telemetry:

```bash
touch /tmp/fr3-sdk-$(id -u)/hand_poll.pause
rm /tmp/fr3-sdk-$(id -u)/hand_poll.pause
```

`--no-hand-polling` disables the background hand session at launch; measured
foreground-owner hand forwarding still works. The 2026-10-27 investigation is in
`../runs/hand_flicker_20261027/`: the operator confirmed that the continuous Desk
warning stopped when repeated short-lived hand sessions were paused. The separate
Desk command-service warning was not treated as that continuous flicker.

No substitute or replayed state is sent during an outage. Isaac holds its last measured joints and shows stale after 0.5 seconds until a fresh measured packet arrives. The relay also rejects samples older than 0.5 seconds. Logs, restart events and continuously updated `status.json` are in the output directory; `final_status.json` describes the last closed session. Stop with Ctrl-C; the relay reaps its reader and releases the connection. The renderer/Isaac have no command return channel to the robot.

Real motion exposed a competing-session failure: the independent reader timed out, its replacement connected during a move, and that move stopped with a communication fault. The current teaching `read_pose`, `joint_move` and gated conveyor executor therefore share exclusive SDK ownership with the background reader. Before a foreground SDK client connects, the background reader yields. The foreground owner copies actual measured states into a bounded, prefaulted, locked queue; a separate worker formats and sends them through a local Unix socket. No socket or formatting work occurs in the arm callback. This bridge only forwards those measurements. Other SDK clients must use the same ownership/tap protocol before running alongside this feed. `read_pose` now uses receive-only `readOnce`, too. `robot_time_s` converts the SDK's printed milliseconds; the unconverted value is retained as `robot_time_ms`.

The tabletop/photo registration still has about 10 mm estimated uncertainty. The RealSense identity mismatch claim was incorrect: **943222073921** is the matching camera serial; **944523026963** is its ASIC/firmware-update ID. Fresh color intrinsics and the configured tool transform match the saved calibration. Saved hand-eye remains eligible if the camera mount stayed fixed; no fresh optical reprojection validation or activated Isaac optical sensor is claimed. See `../isaac/camera_calibration_status.json`.

Regression checks without hardware: `/usr/bin/python3 -s -m unittest discover -s digital_twin/bridge -p test_relay.py -v`. Live restart and separately commanded real-motion evidence are saved under `../runs/repair_20261026T192954Z/`; the original failed motion is retained rather than counted as successful validation.

Final live checks: a stopped reader caused a frozen sequence/stale indication, held simulated joints exactly, and recovered automatically after 4.20 seconds. Real motion supplied 316 native measured packets and 61 distinct viewer samples, with zero stale observations and exact joint agreement. The final return endpoint check failed by 0.11 mm beyond its 1 mm tolerance; the arm remained fault-free Idle. Telemetry validation passed; precise motion endpoint validation did not.
