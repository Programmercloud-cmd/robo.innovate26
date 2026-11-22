#!/usr/bin/env bash
set -eo pipefail
exec "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/teaching/factory_cycle/launch_rows.sh" conveyor_moving "$@"
