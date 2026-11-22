#!/usr/bin/env bash
# Defaults to a plan. Hardware is enabled only by --execute and commissioning.
set -eo pipefail
cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
exec /usr/bin/python3 -s -m teaching.layer_stacking.run "$@"
