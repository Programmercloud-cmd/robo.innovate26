# Local LEGO seating foundation

Scope: one held 2x4 mating pair after a deterministic taught approach. Everything here is isolated from factory control and the live kinematic twin. No hardware client or policy training is provided. **Training and deployment readiness remain false.**

## Implemented

- `effective_axial.py`: continuous axial elastic/damping force, analytic potential and dissipation. No pose snapping, weld or attachment success. Real coefficients and contact/admission geometry remain null in `effective_fit_schema.json`. `synthetic_fixture.json` contains arbitrary generic coupon values, explicitly not a LEGO fit.
- `run_effective_pair.py` / `launch_effective.sh`: isolated Isaac force-pair experiment. The lower body is dynamic on support; pair collision is replaced only for that pair by equal/opposite forces at one shared point with the matching moment. Support collision remains. Admission/domain violations stop explicitly and preserve a partial trace. This is not a robot/gripper or identified fixture model.
- `residual_interface.py`: SDK-free absolute XYZ/yaw target interface, taught-frame observations and wrench transforms, latched guards, continuous-branch IK candidate checks, and bounded quintic target interpolation. It implements no IK solver, embedded impedance loop or actuator. Synthetic settings cannot enable hardware. Limits remain null in `real_contract.json`.
- `fit_axial.py`: fixed-knot elastic/damping fitting on complete fit trials, with rank/conditioning and potential checks. Validation trials never change coefficients. No clipping of negative damping to manufacture a fit.
- `evaluate_traces.py`: held-out force prediction error, phase peak loads and insertion/withdrawal work. It compares predictions at the same observed kinematics; it does not validate closed-loop dynamics, seating, retention or a policy. Missing tolerances, stationary relabeled traces and cross-split duplicate recordings do not pass.

The axial model rejects lateral/yaw/tilt states outside explicit admission bounds. It cannot yet reproduce capture, jamming or rate-independent clutch hysteresis. A four-DOF residual interface does not make this one-dimensional contact model a four-DOF training environment.

## Run and inspect

Locally, from the repository root:

```bash
python3 -m unittest discover -s digital_twin/isaac/lego_contact -p 'test_*.py'
python3 digital_twin/isaac/lego_contact/fit_axial.py digital_twin/isaac/lego_contact/implementation_evidence/synthetic_fit/manifest.json --output-dir /tmp/new-synthetic-fit
```

Use a fresh output directory. The saved synthetic round-trip is in `implementation_evidence/synthetic_fit/output/`; it demonstrates software plumbing only. For real data, copy `physical_trials.template.json`, identify the contact datum/depth knots, declare tolerances before fitting, and reserve whole trials for validation. It intentionally contains no fabricated measurements. The fitting CLI writes a candidate, paired traces and a replay report; exit 2 means a recoverable data/model error or failed/uncommissioned replay check.

On VM `robot-innovate26-place-rl3`, use `/home/ubuntu/workspace/lego-contact-20261026/` and a fresh output path:

```bash
timeout 40 bash launch_effective.sh --output fresh-effective --fixture synthetic_fixture.json --duration 0.7 --dt 0.001
```

See `effective_results.json` and `effective_evidence/` for actual VM results and explicit guarded stops. Continuous force response alone does not establish real contact fidelity or retained interlock.

## Physical identification next

Fix the lower fixture, mating overlap/orientation and grip. Measure held-brick pose/slip; local servo response and configured payload/impedance conditions; wrench bias/noise/latency; then repeated aligned seating/withdrawal at multiple rates and small held-out offsets. Reserve a different rate to distinguish viscous damping from rate-independent clutch friction. TCP-only traces identify a lumped grip/fixture/controller response, not independent brick deformation. Log requested, post-limiter, robot-desired and measured motion with timestamps through one SDK owner; viewer telemetry is not the identification log.

Fit only the measured operating envelope. Compare held-out force/depth curves, loads, work, jam/slip outcomes and independently verified retention, including solver/timestep sensitivity. If axial force replay fails to reproduce hysteresis or capture, extend the model from those data before pilot training. A low Z, force drop or internal engagement flag is insufficient success. Local taught-pose calibration remains necessary even with the unchanged ChArUco board and corrected RealSense identity.

## Preserved native baseline

`contact_probe.py`, `analyze.py`, `diagnostic_report.json` and `evidence/` retain the six native BrickSim tests. They exposed solid body/top-slab colliders and threshold-triggered attachment with a roughly 1.7 mm pose snap; native breakage was disabled. These remain a logic baseline, not insertion physics. The `too_high` control disables upper gravity, so it is not a free-fall test. Recheck with `python3 digital_twin/isaac/lego_contact/analyze.py --output /tmp/native-contact-report.json`.

References: [PhysX contacts](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/dev_guide/rigid_bodies_articulations/collision.html), [Isaac Lab actuator models](https://isaac-sim.github.io/IsaacLab/v2.1.0/source/api/lab/isaaclab.actuators.html).
