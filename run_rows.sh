#!/usr/bin/env bash
# Fail closed: the old command must never silently select a setup.
printf '%s\n' 'Select a setup explicitly:' \
    '  ./run_rows_tabletop.sh [OPTIONS] COLUMN ...' \
    '  ./run_rows_conveyor.sh [OPTIONS] COLUMN ...' \
    '  ./run_rows_conveyor_moving.sh [OPTIONS] COLUMN ...' >&2
exit 2
