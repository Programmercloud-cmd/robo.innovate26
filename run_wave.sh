#!/usr/bin/env bash
set -eo pipefail
wave_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
cd "$wave_root"
exec /usr/bin/python3 -s -m teaching.wave_demo.run "$@"
