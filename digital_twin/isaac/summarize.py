"""Read-only telemetry evidence summary; never connects to hardware or simulation."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from kinematics import Model, difference

parser = argparse.ArgumentParser()
parser.add_argument("run", type=Path)
parser.add_argument("--urdf", type=Path, default=Path(__file__).resolve().parents[1]/"robot_model.urdf")
parser.add_argument("--output", type=Path, required=True)
args = parser.parse_args()
model = Model(args.urdf)
raw = (args.run/"telemetry.ndjson").read_bytes()
frames = []
for line in raw.splitlines():
    try:
        frames.append(json.loads(line))
    except ValueError:
        pass
if not frames:
    raise RuntimeError("no complete telemetry samples")
errors = []
for frame in frames:
    if frame.get("O_T_EE") is not None and frame.get("F_T_EE") is not None:
        expected = model.flange(frame["q"])@np.asarray(frame["F_T_EE"]).reshape(4,4,order="F")
        errors.append(difference(expected, np.asarray(frame["O_T_EE"]).reshape(4,4,order="F")))
q = np.asarray([frame["q"] for frame in frames])
latest = json.loads((args.run/"latest.json").read_text())
applied = []
for line in (args.run/"applied.ndjson").read_text().splitlines():
    try:
        applied.append(json.loads(line))
    except ValueError:
        pass
elapsed = (frames[-1]["unix_ns"]-frames[0]["unix_ns"])*1e-9
result = {"telemetry_file_sha256_at_read": hashlib.sha256(raw).hexdigest(),
          "complete_samples": len(frames), "duration_source_s": elapsed,
          "average_source_hz": (len(frames)-1)/elapsed if elapsed else None,
          "sources": sorted(set(frame.get("source", "unknown") for frame in frames)),
          "q_min_rad": q.min(axis=0).tolist(), "q_max_rad": q.max(axis=0).tolist(),
          "q_peak_to_peak_rad": np.ptp(q,axis=0).tolist(),
          "max_urdf_vs_hardware_tcp": {key: max((e[key] for e in errors), default=None)
                                       for key in ("position_m", "rotation_rad")},
          "max_isaac_vs_urdf_flange": {key: max((s["urdf_flange_error"][key] for s in applied),default=None)
                                       for key in ("position_m", "rotation_rad")},
          "max_isaac_vs_hardware_tcp": {key: max((s["measured_tcp_error"][key] for s in applied if s.get("measured_tcp_error")),default=None)
                                       for key in ("position_m", "rotation_rad")},
          "latest_isaac": latest, "hardware_commands": 0,
          "scope": "Passive live telemetry and kinematic agreement; physical commanded-motion following and embedded-controller equivalence not tested"}
args.output.write_text(json.dumps(result,indent=2))
print(json.dumps({key:result[key] for key in ("complete_samples","duration_source_s","average_source_hz","sources","max_urdf_vs_hardware_tcp")},indent=2))
