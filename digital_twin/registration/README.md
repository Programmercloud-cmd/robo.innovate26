The current tabletop registration is `scene_registration.json`. Its geometry
and approximately 10 mm estimated uncertainty are unchanged by the camera
identity correction dated 2026-10-26.

`calibration_identity_correction_20261026.json` records fresh SDK enumeration,
current tool-frame comparison, matched color intrinsics, mounting conditions
and which transforms are actually applied. Device serial **943222073921**
matches the saved calibration; **944523026963** is its ASIC/firmware-update ID.
The earlier camera-replacement inference is withdrawn.

The user confirms the board never moved; robot-base continuity is consistent
with this session. Wrist mounting continuity is not independently established
today. Saved hand-eye is conditionally reusable, but no camera optical model
or live RGBD registration has been activated. Historical evidence is preserved
with dated correction sidecars. The file named `photo_registration_candidate`
remains the previously rejected diagnostic candidate.
