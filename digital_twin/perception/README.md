External Logitech display feed
=============================

Start after the existing measured-state twin is running:

```bash
/usr/bin/python3 -s digital_twin/perception/launch.py
```

Open <http://127.0.0.1:18766/> and expand **Camera debug overlay**. Ctrl+C stops
only this launcher's camera/display children. Robot/controller/hand services are
separate. Locks reject duplicate owners. Background PIDs are in `session.json`;
logs are `camera.log`, `forward.log`, `supervisor.log`.

`teaching/external_layout/config.json` owns camera regions, actual brick-TOP
landmarks, fixed occlusions, white-floor references and pigment prototypes. It
uses the existing timestamped V4L2 capture helper and opens no SDK connection.
Camera failures retry five times with .5/1/2/4/8s backoff, resetting the budget
after30 healthy seconds. A missing USB camera waits without repeated launches;
device reappearance starts a new bounded retry episode automatically.
`camera_health.json` distinguishes a missing device from exhausted retries.

Raw observations stay in `slots`: occupied/color, empty or uncertain. **Display
memory is separate**, in `display_slots` and `display_state.json`. Once a color
is confirmed it stays visible through occlusion, glare, registration uncertainty
or an outage. Its original observation timestamp remains unchanged and the
browser marks it retained/unverified. Never-observed colors are not filled in.
`display_policy` in the external-camera configuration enables manual updates:
a changed color needs five fresh consistent observations with confidence at
least0.5; an emptied slot needs three positive floor observations. Producer
temporal checks also apply. Uncertainty retains the last confirmed state.
`conveyor_slots` adds24 raw observations, `conveyor_r{row}_n{brick}`, and
`display_conveyor_slots` carries their separate persistent display memory.

Small camera shifts are corrected from segmented complete stack footprints,
with bounded translation/rotation/scale and multi-row, both-bank shape support.
Slot samples, gap checks and masks share that image correction. The debug
overlay shows the segmented anchors and moved slot regions. When robot telemetry
is unavailable, supported fresh color evidence may still update this display;
the packet explicitly marks the robot guard unknown. Empty assertions require
independently captured paper references and stricter positive material,
brightness and visibility checks. Fresh robot-motion evidence still vetoes observations. Registration
failure remains uncertain and retains earlier display colors.

The existing factory `inventory.json` is read-only input to display ownership.
Historical completions are baselined rather than replayed into a restored table.
Closure freezes the four known source colors; lift-start displays an assumed
held column, retention upgrades its controller evidence, and verified hand-open
moves it to an assumed conveyor pose pending placement verification. Completed
placement is distinguished from unverified release. During closure/holding/
release, source colors stay frozen and camera updates cannot replace the held
payload. After release, fresh bounded camera evidence may reopen each source
slot and update the conveyor, including manual removal, return or color swaps.
The ownership mask `output/source_exclusions.json` now excludes only currently
frozen payloads; stale mask data does not silently clear those exclusions.
Conveyor camera slots replace duplicate virtual placed payloads. Controller
release assumptions remain visibly unverified until camera evidence arrives.
Manual Desk grips have no automatic source identity. Execution inventory remains
read-only and is never advanced by these display observations.

Held visuals follow the existing twin's measured TCP using a rigid transform
from the taught grasp; placed visuals use the existing configured release pose
and conveyor-only cloth correction. These are presentation assumptions, not
independent retention/slip/contact measurements. No additional robot reader is
created. All144 IDs use `external_r{r}_c{bank}_m{m}_n{n}`; rows0–5 and individual
bricks n0–3 run image-top→bottom, m0–2 left→right. Bank0 aliases motion pick `r_m`;
bank1 has display IDs only. Conveyor rows use the configured release target plus
the existing conveyor row step; they do not inherit revised source-row spacing.
Positions, motion settings and execution inventory are unchanged.

`factory_reference.json` retains the refined drawing and measured242mm clear
span,65mm right tape, and162mm beyond the cross-tape's lower edge. Since drawn
15mm gaps are approximate, the visual horizontal residual is distributed over
three gaps. Existing taught left-bank XY anchors and the estimated table plane
remain the basis of `display_layout.json`; bank1/tape transforms are display-only.
The observed front-to-crossline gap is estimated12–17mm with floor/top parallax;
it is not forced to the drawing's ideal7.5mm. Run `make_display_layout.py` only
after reviewing future geometry changes. No board/camera-to-robot calibration
has been rewritten.

Source/overlay/JPEG/JSON publication uses a shared file lock, then one SSH stream.
The remote ingest expires raw observations after one second while retaining the
last confirmed display. Original exposure times survive transport; read-only
HTTP twin telemetry supplies the robot guard. Artifacts: `live_*verification.json`,
`display_memory_verification.json`, `viewer_live.png`; camera-specific evidence
is under `teaching/external_layout/captures/`.
