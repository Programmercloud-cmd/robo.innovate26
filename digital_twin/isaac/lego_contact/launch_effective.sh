#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
export OMNI_KIT_ACCEPT_EULA=YES PYTHONUNBUFFERED=1 PYTHONDONTWRITEBYTECODE=1
export OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2
export LD_LIBRARY_PATH="/home/ubuntu/workspace/robo26-scene-smoke/20261026T171524Z-brev-scene-smoke/compat_libs${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
model_python=/home/ubuntu/workspace/robo26-runtime-isaac51/venv/bin/python
set +e
"$model_python" run_effective_pair.py "$@"
child_status=$?
set -e
if [ "$child_status" -ne 0 ]; then
  exit "$child_status"
fi
# Kit fast shutdown may exit the child with zero before Python can propagate
# its domain-guard code. The public launcher waits for that child and checks
# the saved result; a guard stop must never look like successful completion.
"$model_python" - "$@" <<'PY'
import argparse
import json
from pathlib import Path
import sys
parser=argparse.ArgumentParser(add_help=False)
parser.add_argument("--output",type=Path,required=True)
args,_=parser.parse_known_args(sys.argv[1:])
result=json.loads((args.output/"result.json").read_text())
raise SystemExit(2 if result.get("status")=="domain_guard_stop" else 0 if result.get("completed") else 1)
PY
