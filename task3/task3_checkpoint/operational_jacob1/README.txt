JACOB OPERATIONAL SIMULATION — TASK3/JACOB1

Quick start from the repository root (Python3, WebGL browser):
  python3 task3_checkpoint/operational_jacob1/server.py --port 8745
  Open http://127.0.0.1:8745/viewer/

The browser computes the updated process in a Worker. No Node installation or
large recorded timeline is required to inspect the simulation. Three.js and its
MIT license are included. Internet access is not required after cloning.

Use Both buffers full, Layer transfer, Completed pallet, Gantry pickup,
Loaded claw rotation and Skip exchange. Playback/scrubbing and camera presets
are available. Motion assumptions explains the controlled commissioning model.

Optional audit trace generation (Node18+):
  node task3_checkpoint/operational_jacob1/integration/generate-run.cjs
  node task3_checkpoint/operational_jacob1/validation/audit-run.cjs --run task3_checkpoint/operational_jacob1/run.json --out /tmp/jacob1-audit.json
  node task3_checkpoint/operational_jacob1/validation/negative-controls.cjs --run task3_checkpoint/operational_jacob1/run.json --out /tmp/jacob1-negative-controls.json

run.json is about159MiB and is intentionally not committed. The generated JSON,
gzip copy and local server runtime files are ignored. Source, manifest, reviewed
validation results and reproducible auditors are committed.

Follow Pueue for intensive jobs on the shared workstation. Direct execution is
permitted on the dedicated Brev VM. Lightweight checks may run normally.

Current VM deployment remains http://127.0.0.1:18745/viewer/ through the existing
SSH forward. START-HERE.txt and engine/ENGINE-NOTES.txt describe the reviewed run.
