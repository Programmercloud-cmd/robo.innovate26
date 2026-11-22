#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
runtime=/home/ubuntu/workspace/robo26-runtime-isaac51
snapshot=/home/ubuntu/workspace/robo26-scene-smoke/20261026T171524Z-brev-scene-smoke
export OMNI_KIT_ACCEPT_EULA=YES PYTHONUNBUFFERED=1 PYTHONDONTWRITEBYTECODE=1
export OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2
export LD_LIBRARY_PATH="$snapshot/compat_libs${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
exec "$runtime/venv/bin/python" isaac/mirror.py "$@"
