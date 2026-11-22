#!/usr/bin/env bash
# All three public launchers use this same argument parser and execution pipeline.
set -eo pipefail
factory_setup=$1
shift
factory_script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
for factory_arg in "$@"; do
    if [[ $factory_arg == --setup || $factory_arg == --setup=* ]]; then
        printf '%s\n' 'Choose the setup through run_rows_tabletop.sh, run_rows_conveyor.sh or run_rows_conveyor_moving.sh.' >&2
        exit 2
    fi
done
source /opt/ros/humble/setup.bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
exec /usr/bin/python3 -s "$factory_script_dir/run_rows.py" --setup "$factory_setup" --execute "$@"
