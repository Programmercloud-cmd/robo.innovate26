The VM runs a **one-way, measured-state Isaac/PhysX twin**. Its robot-base frame
is identity. Measured `q` and `dq` set the simulated articulation; `q_d` is retained
as telemetry. No hardware client or command return path exists in this code.
It does not reproduce Franka's embedded impedance controller or validate contact
dynamics. Reported gripper width drives native finger joint sum; the modified-pad
inner gap is uncalibrated. Wrist camera housings are visual approximations;
current camera optical calibration is not applied.

Correction dated 2026-10-26: fresh SDK enumeration confirms the physical camera
serial is **943222073921**, matching the saved calibration. **944523026963** is
its ASIC/firmware-update ID. Current 1280×720 color intrinsics match the saved
values, and a read-only robot sample confirms the configured tool transform is
unchanged. Saved hand-eye transforms are eligible if the camera mount has stayed
fixed; that mounting continuity has not been independently confirmed today.
The existing housing placement remains approximate. See
`camera_calibration_status.json` for evidence and what is actually applied.
Historical manifests retain their original, superseded camera-identity note.
The saved board-to-base transform is already used for the tabletop registration.
That use does not activate camera optics or establish unchanged wrist mounting.

Remote root: `/home/ubuntu/workspace/fr3-digital-twin-20261026`.
The active reviewed scene is in `live-03/`; the browser URL is
`http://127.0.0.1:18766/` through the established SSH tunnel. A self-contained
recorded-state USD snapshot is saved locally in `model/fr3_registered_workcell.usdc`.
The existing Isaac 5.1 runtime and frozen local FR3 assets are reused without
changing shared installations. `physics_only.kit` avoids the RTX startup crash
on the installed 595 driver. The optional browser monitor uses exported USD
triangles and actual PhysX link transforms; it is explicitly not an RTX render.

On the VM, use a fresh output path:

```bash
bash isaac/launch.sh --output live-new --export-viewer
# Add --registration registration.json once supplied geometry is reviewed.
python3 -m http.server 8766 --bind 127.0.0.1 --directory live-new
```

On the workstation, forward the receiver and browser monitor, then run the
read-only bridge documented in `../bridge/`:

```bash
ssh -T -N -L127.0.0.1:18765:127.0.0.1:8765 \
  -L127.0.0.1:18766:127.0.0.1:8766 robot-innovate26-place-rl3
```

Open `http://127.0.0.1:18766/`. After 0.5 s without telemetry, the twin holds the
last joint positions and sets stale status. It never extrapolates motion.
Outputs include received packets, applied samples, model/source hashes,
joint/TCP parity, `latest.json`, and a composed USD at clean shutdown.
`evidence/` preserves the original stationary measurements, synthetic kinematic
samples and synthetic disconnect/reconnect test. Later real-motion verification
is under `../runs/repair_20261026T192954Z/`: 316 measured motion packets and
61 distinct viewer motion samples, zero stale observations during that probe,
and exact measured/Isaac joint agreement. Exclusive SDK ownership and a local
measured-state tap replace competing background/controller robot connections.
The initial communication fault and the later 1.11 mm return endpoint error
(against a 1 mm controller check) remain recorded separately from the successful
telemetry validation. The arm finished Idle without robot errors.

The tabletop/tape registration is estimated from the fixed board and supplied
photo, with approximately 10 mm engineering uncertainty. The white surface
boundary is a visible crop, and the ChArUco marker side length remains nominal.
These limitations are separate from the much smaller numerical joint/FK errors.
