#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/../.."
source /opt/ros/humble/setup.bash
layer_franka_root=${FRANKA_DIAGNOSTICS_DIR:-/home/nshenoy/franka_diagnostics}
g++ -std=c++17 -O2 -Wall -Wextra -Wno-misleading-indentation -Wno-maybe-uninitialized \
  -I/usr/include/eigen3 -I"$layer_franka_root/libfranka/include" \
  -I"$layer_franka_root/libfranka/common/include" \
  teaching/layer_stacking/controller.cpp -L"$layer_franka_root/build" \
  -Wl,-rpath,"$layer_franka_root/build" -lfranka -pthread \
  -o teaching/layer_stacking/controller
