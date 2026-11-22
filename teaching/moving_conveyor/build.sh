#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/../.."
source /opt/ros/humble/setup.bash
franka_diagnostics_dir=${FRANKA_DIAGNOSTICS_DIR:-/home/nshenoy/franka_diagnostics}
g++ -std=c++17 -O2 -Wall -Wextra -Wno-misleading-indentation -Wno-maybe-uninitialized \
  -I/usr/include/eigen3 -I"$franka_diagnostics_dir/libfranka/include" \
  -I"$franka_diagnostics_dir/libfranka/common/include" teaching/moving_conveyor/execute.cpp \
  -L"$franka_diagnostics_dir/build" -Wl,-rpath,"$franka_diagnostics_dir/build" \
  -lfranka -pthread -o teaching/moving_conveyor/execute
