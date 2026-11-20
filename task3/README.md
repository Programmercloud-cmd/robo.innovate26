# Task 3 — Brickflow factory simulation and cinematic rendering

The flagship Robo.Innovate 2026 deliverable: a modeled industrial brick factory, from kiln-cart unloading through inspection, rejection, color blending, pallet formation, storage and dispatch. The project evolved from a Python/SimPy discrete-event model to a full-cart WebGL simulation and a native Unreal Engine cinematic.

## Watch the finished film

- [Continuous 1080p factory film](https://github.com/Programmercloud-cmd/robo.innovate26/releases/tag/simulation-handoff-20261027) — `brickflow-continuous-1080p.mp4`.
- [4K master and Linux Unreal build](https://github.com/Programmercloud-cmd/robo.innovate26/releases/tag/simulation-handoff-20261027) — split master, checksums, and packaged native application.
- The earlier 24-second pitch film is also in the release.

The films and build are release assets, not files to regenerate during a normal checkout. See [Unreal rendering notes](docs/unreal_continuous_film.md).

## Open the current factory layout

From this directory (`task3/`), with Python 3.10+:

```bash
python3 scripts/open_cart37_checkpoint.py
```

Open the printed HTML file in a WebGL-enabled browser. This is the recorded, self-contained Greenfield 37 adapted-layout factory replay. No GPU VM, server, or extra Python dependency is needed for playback. See [layout handoff](docs/greenfield37_layout_handoff.md) for the configuration, validation evidence, capacity assumptions and budget.

To open Jacob's separate operational layout instead:

```bash
python3 task3_checkpoint/operational_jacob1/server.py --port 8745
```

Open http://127.0.0.1:8745/viewer/; [operational model guide](task3_checkpoint/operational_jacob1/README.txt). This runs the browser-based engine, not the recorded Greenfield replay.

## Reproduce the original Python factory model

```bash
python3 -m pip install --target .deps/plant_sim -r plant_sim/requirements.txt
python3 scripts/greenfield_demo.py --config config/greenfield_dual.json --output-dir artifacts/my_task3_run
```

Open `artifacts/my_task3_run/index.html`. [Python simulation and verification guide](docs/task3_revision14_handoff.md).

## Source code map

| Directory | Purpose |
| --- | --- |
| [greenfield/model.py](greenfield/model.py), [greenfield/twin.py](greenfield/twin.py), [plant_sim/](plant_sim/) | Original Python/SimPy factory simulation |
| [greenfield/cart_model.js](greenfield/cart_model.js), [greenfield/logistics.js](greenfield/logistics.js), [greenfield/layout.js](greenfield/layout.js) | Greenfield 37 event model, logistics and adapted layout |
| [greenfield/](greenfield/) | WebGL rendering, geometry, motion and equipment |
| [scripts/](scripts/) | Python packaging, extraction, replay and audit entry points |
| [unreal/Brickflow/](unreal/Brickflow/) | Native Unreal 5.8.4 project; C++20 simulator and rendering |
| [unreal/tools/](unreal/tools/) | Geometry preparation, native build and film rendering tooling |
| [cinematic/](cinematic/) | Earlier Blender, USD and cloud rendering pipeline |
| [planning/](planning/), [integration/](integration/), [execution/](execution/) | Planning, perception handoff and orchestration |
| [config/](config/), [data/](data/) | Factory and recipe inputs |
| [task3_checkpoint/](task3_checkpoint/) | Frozen replays, source snapshots, reports and evidence |
| [tests_greenfield/](tests_greenfield/), [tests_native/](tests_native/), [tests_plant/](tests_plant/) | Regression and comparison tests |

## Version and evidence boundaries

- **Python/SimPy** is the original factory model; later Greenfield full-cart builds use a JavaScript event engine with Python packaging.
- **Unreal film:** the delivered native C++ model is pinned to Greenfield revision 30, not the later revision 37 layout. The revision 37 migration studies are preserved but do not constitute a completed native port.
- **Adapted Greenfield 37:** the primary source and frozen layout replay come from [task3/simulation-handoff](https://github.com/Programmercloud-cmd/robo.innovate26/tree/task3/simulation-handoff).
- **Jacob operational model:** the self-contained alternative is preserved from [task3/jacob1](https://github.com/Programmercloud-cmd/robo.innovate26/tree/task3/jacob1); its engine and validation are separate.
- Input brick identities, shades and quality are synthetic for the factory demos. Throughput is modelled, not independently measured industrial performance. The repository does not contain a verified live physical perception-to-Isaac factory sync.

Further reading: [original handoff](TASK3_README.md), [Unreal project guide](unreal/README.md), [native comparison](docs/unreal_native_goal.md), [layout study](docs/greenfield37_layout_handoff.md), [Unreal 37 review](docs/unreal_greenfield37_review.md).

Historical instructions referring to “repository root” refer to this Task 3 project directory. Cloud-specific paths and VM availability are historical; review them before attempting a rebuild. The original branches remain available as immutable reference points.
