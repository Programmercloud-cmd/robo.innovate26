# Operator working-grasp reference

Data-only import of the supplied pose export and annotated workcell photo. The
operator reports **20 mm approach opening, 14 mm closing request, 15.8 mm loaded
readback, and straight upward extraction**, with the inferred input-stack poses
tested. This supplements the preserved failed simulation retention experiments;
it does not change the active model, controller, width targets or workcell.

The original Markdown/photo are retained byte for byte under `source/`, with
attachment hashes in `reference_registry.json`. The export-declared original
`positions.json` SHA is separate: that original JSON file was not supplied or
independently verified. Raw validation flags remain unchanged; the later
operator attestation is separate provenance, not instrumented test logs.

## Frame and width interpretation

The export names `robot_base_O` and `configured_end_effector_EE`, metres and
column-major transforms. The **15 mm downward base-Z offset is already applied**.
Its configured flange-to-EE transform includes −45° about flange Z and 103.4 mm
axial translation. The pinned USD flange-to-hand joint and hardware URDF verify
the nominal frame relationship. The current MODEL grip frame is hand +117.557561
mm, not a measured physical pad-contact point. Conversion is:

```
T_hand_EE = inverse(T_flange_hand) @ T_flange_EE
T_base_model_grip = T_base_EE @ inverse(T_hand_EE) @ T_hand_model_grip
```

This yields about **14.158 mm farther toward the brick** for the downward poses;
using the exported EE translation as the current grip target would be wrong.
Neither this conversion nor the photograph establishes base-to-table/plate
registration or the actual pad–brick engagement depth. The saved upper
reference's relation to the physical brick top remains unspecified.

The old model relation is `custom_gap = q_left + q_right + 2.81338 mm`.
**Conditionally**, if the 15.8 mm display readback equals SDK joint sum and the
physical mount matches this model, it predicts an 18.61338 mm model gap. That
challenges those assumptions together for a gripped 2×4 brick; it does not
identify which assumption is wrong or justify fitting a new offset. No new
width, friction or engagement-depth parameter is selected here.

The inferred export uses 31.8 mm center spacing and 9.6 mm height increments.
The native scene uses a 32 mm stud-grid pitch. Preserve both; do not deform the
native connected lattice or assume imported IDs already map to scene IDs.
Reference coordinates/identities are not online perception inputs.

## Reproduce

No dependency installation or shared runtime changes. Uses existing Python,
NumPy and SciPy. The submitter copies every pinned input, code and protocol to
an immutable `runs/…-hardware-grasp-reference/` snapshot and records its commit
and hashes. The original paths of copied USD/URDF context are provenance only.

```bash
/home/faps/miniforge3/envs/unitree_sim_env/bin/python experiments/hardware_grasp_reference_20261025/test_synthetic.py
python3 experiments/hardware_grasp_reference_20261025/submit.py
```

The first command checks only invented transforms and JSON. The second submits
the full 315-pose numerical audit through **default Pueue**, with a 10 s process
timeout plus 2 s kill grace. It starts no Isaac/Kit, rendering or physics. Audit
outputs preserve the authoritative embedded JSON and provide separate derived
MODEL-frame transforms and a comparison report. Nearest recorded/inferred pose
residuals are descriptive, not asserted physical correspondences or accuracy.

Next contact reproduction needs the upper-reference/brick-surface relation and
resolution of the physical width/mount mapping. Existing retention criteria and
closed three-round experiments remain unchanged.
