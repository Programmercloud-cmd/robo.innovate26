#!/usr/bin/env bash
# Execute the existing factory pipeline for the supplied source columns.
set -eo pipefail

usage() {
    printf 'Usage: %s [--execute-task] [--verify-placement] [--confirm-empty-gripper] COLUMN [COLUMN ...]\n' "${0##*/}"
    printf 'Executes robot pick-and-place. Columns: 0_0 through 5_2.\n'
    printf 'Post-placement camera verification is off unless --verify-placement is supplied.\n'
    printf 'Stationary-conveyor home/free-space settling: 1.5 mm; grasp/placement accuracy: 1 mm.\n'
    printf '%s\n' '--confirm-empty-gripper acknowledges that no bricks are held and clears a pending operation after live startup checks.'
    printf '%s\n' '--execute-task includes that acknowledgement and skips camera/occupancy/visual-verification and schedule gates; operator manages conveyor clearing; motion/fault/contact protections remain active.'
    printf 'Examples: %s 5_2\n          %s 1_0 2_0 3_0\n' "${0##*/}" "${0##*/}"
}

if [[ $# -eq 1 && ( $1 == --help || $1 == -h ) ]]; then
    usage
    exit 0
fi
if [[ $# -eq 0 ]]; then
    usage >&2
    exit 2
fi
factory_columns=()
factory_options=()
for column in "$@"; do
    if [[ $column == --verify-placement || $column == --confirm-empty-gripper || $column == --execute-task ]]; then
        factory_options+=("$column")
        continue
    fi
    if [[ ! $column =~ ^[0-5]_[0-2]$ ]]; then
        printf 'Invalid column: %s (expected 0_0 through 5_2)\n' "$column" >&2
        exit 2
    fi
    factory_columns+=("$column")
done
if [[ ${#factory_columns[@]} -eq 0 ]]; then
    usage >&2
    exit 2
fi

factory_repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
exec /usr/bin/python3 -s "$factory_repo_dir/teaching/factory_cycle/run_rows.py" "${factory_columns[@]}" "${factory_options[@]}" --execute
