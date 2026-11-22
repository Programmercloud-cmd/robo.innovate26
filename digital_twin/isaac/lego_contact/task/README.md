# Requested-stack seating task

Entry configuration: `config.json`. The operator confirmed the stack belongs at the plate center. The two-layer default exercises both orientations; change `stack.layer_count` for the actual job. No factory files, poses, gripper settings or SDK ownership code are changed here.

## Geometry and scene

Each layer has `b_n_m_k`, n=0..3, m=0..2, k from bottom0. Nominal stud pitch is8mm, independently of31.8mm brick body length. The footprint alternates12×14 and14×12 studs about one center. Plate-center placement resolves to stud-coordinate(15.5,15.5) on the photo-supported32×32 grid.

`stack_layout.json` lists brick-bottom poses, all8 potential stud slots, lower-owner correspondences and order dependencies. Above the baseplate,6 inner-ridge bricks have4 matches across4 lower bricks;6 outer-ridge bricks have2 collinear matches across2 lower bricks, with their centers outside the support line. These are geometric correspondences, not verified engagement or stability. Cantilever/bridge behavior must be measured separately from full8-stud seating.

`plate_registration_candidate.json` uses the supplied photograph,24 ChArUco corners and1,023 matched plate features. Board+nominal-grid rigid residual is~3.52mm; imposing the centerline/tape-edge constraints gives~7.29mm. These diagnostics do not define placement accuracy or tolerances. Candidate plate+X follows board+X(photo left); +Y follows board+Y(photo down); the far+Y edge faces the board-side tape edge. Source-camera optics, plate thickness/seating height, fixation and current robot-base registration remain unresolved. `T_O_P_display_proxy` is for display only; physical `T_O_plate` staysnull.

`scene.py` writes a NEW candidate USD over a read-only original scene. It displays the white plate, nominal CAD stud visuals and requested stack; contact/support/ground behavior remains explicitly disabled because its physical data/model are missing. It never treats a visual stud mesh or flat box as assembly physics.

## Interfaces

`control.py` adds the task's world-vertical invariant to the existing guarded absoluteXYZ/yaw interface. Residual yaw is about world vertical even if the displayed plate is tilted. Tilted nominal/task frames, measured tilt and nonvertical IK-candidate FK stop explicitly. Candidate FK must come from the trusted external model using exactly the proposed joints/tool frame. No IK solver, real actuator or impedance surrogate is supplied.

`observations.py` shares calibrated undistorted(X/Z,Y/Z) stud features, target correspondence/confidence/visibility/timestamps, goal/support data, measured TCP/joints/wrench/gripper/actions and bounded history. Hidden poses and engagement states are excluded. Projected-feature errors require a camera/settings-bound physical error profile, sampled as a whole frame; visibility/dropouts remain explicit. Independent held-brick observations or measured transform/slip bounds are required.

`stud_detector.py` produces measured ring/ellipse pixels only inside known projected stud ROIs. Blank, occluded, ambiguous or duplicate evidence cannot become a nominal position. Physical score-to-confidence calibration binds the lens/settings, detector parameters, OpenCV version and image/label evidence; `calibratedROIconfig.template.json` remains null. Synthetic detector tests are software checks only.

`camera_evidence_audit.json` records that the archived Logitech calibration was REJECTED; RealSense continuity does not validate it. `controller_replay.py` compares timestamped requested/sent/desired/measured signals and rejects ideal Cartesian control, q-overwrite replay, missing conditions and time-shifting to hide latency. Wrench state time, declared filter latency and nominal force time are retained separately; replay must match the filter timing without shifting the observations. `vm_audit.json` confirms RL software and GPU availability; missing physical evidence is the blocker.

## Identification data and independent outcomes

`owner_recorder.hpp` is a bounded recorder to include inside the future foreground SDK owner. It opens no robot/gripper connection. The callback only copies state/commands/context into locked storage; an explicitly ordinary-priority worker writes full raw state, command stages, gripper stamps, sequence, declared origin and condition profiles. Overflow, invalid input or disk failure requests a stop and invalidates the capture. `finish()` runs after leaving control. Existing SDK ownership and measured-state forwarding stay separate and unchanged. The caller must act on `offer()==false` or `stop_requested()` through its commissioned controlled-stop path. This component is not yet wired into a commissioned real seating controller or benchmarked at real callback rates.

`normalize_owner_recording.py` accepts only finalized, identity/count/byte-bound raw recordings and an explicit local wrench calibration. It preserves the declared clock origin, validates tool/stiffness frames and the calibration's condition/time/joint envelope, and applies bias then gain at K. Corrupt, unsafe or incomplete input never produces a committed normalized trace. `wrench_calibration.template.json` intentionally contains no physical bias/gain/noise/latency values. Conversion is for offline replay; do not feed already-corrected values through the live actor's raw-wrench correction again.

`contact_outcomes.py` classifies supplied independent measurements as success, partial seating, operational jam, slip or recoverable stop. Success requires seated relative geometry, safe loads, measured unloading/release, observed fixture/grasp coverage and a calibrated outward tensile retention test. TCP height, attachment flags or an unsigned push cannot certify interlock. All physical criteria remain null in `contact_outcomes.template.json`; classification does not validate the sensor calibration or contact model.

## Commands

From the repository root:

```bash
/usr/bin/python3 -s -m unittest discover -s digital_twin/isaac/lego_contact -p 'test_*.py'
python3 digital_twin/isaac/lego_contact/task/stack_layout.py digital_twin/isaac/lego_contact/task/config.json --output /tmp/requested-stack.json
/usr/bin/python3 -s digital_twin/isaac/lego_contact/task/photo_registration.py --output /tmp/plate-candidate.json
cd digital_twin/isaac/lego_contact
python3 -m task.readiness --output task/requirements_audit.json
```

The audit currently exits2 with missing evidence/implementation requirements. It has no checkbox-only path to training authorization. The system cannot honestly supply a trained checkpoint yet; do not train the installed sorting environment as a substitute.

Use `-s` locally because the installed OpenCV matches system NumPy, not the user NumPy2 install. The VM's existing Isaac Python/OpenCV runtime also passes the detector tests; no package change is needed.

The native recorder can be compiled and exercised without connecting hardware:

```bash
bash digital_twin/isaac/lego_contact/task/build_owner_recorder_synthetic.sh /tmp/owner-recorder-synthetic
/tmp/owner-recorder-synthetic /tmp/new-owner-recorder-fixture
```

These fixtures are explicitly synthetic. Raw capture normalization and outcome evaluation expose their required inputs through `--help`; missing physical calibrations return recoverable stops, not defaults. Preserve raw files and reserve complete trials before fitting.

On the Brev VM, use the existing Isaac runtime and the isolated project:

```bash
cd /home/ubuntu/workspace/lego-contact-20261026
bash task/launch_scene.sh --output /tmp/new-seating-scene --base-stage /home/ubuntu/workspace/fr3-digital-twin-20261026/model-snapshot-01/digital_twin.usdc
```

Use a fresh output directory. Scene authoring is bounded35s and checks the saved USD rather than relying on Kit's exit status. Follow the saved scene audit for the actual generated path and original-scene hashes.

## Missing physical evidence before training

Measure plate grid/datum/thickness/fixation and current board-to-base continuity; repeat Logitech intrinsic/hand-eye/settings validation; identify held-brick transform/slip, loaded joint-impedance response, payload/gravity model and wrench bias/noise/latency. Record seating/withdrawal at multiple rates, reserving complete trials and a different rate for validation. Cover8-stud baseplate seating,4-stud multi-brick bridges and2-stud cantilevers, including XY/yaw errors, partial seating, jams and independently verified retention.

Fit and validate the smallest model that predicts those cases within predeclared measurement/load tolerances, including timestep/solver sensitivity. Integrate that model with the actual deployed controller and measured camera error/latency profile. Then run the bounded aligned→handoff-error→support-pattern curriculum and compare a saved policy with a deterministic visual/contact baseline. Report failures and loads separately; if vertical-only actions fail, preserve that limitation. No roll/pitch relaxation is authorized.
