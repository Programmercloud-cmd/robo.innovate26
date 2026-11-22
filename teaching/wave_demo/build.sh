#!/usr/bin/env bash
set -eo pipefail
wave_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
wave_franka_root=${FRANKA_DIAGNOSTICS_DIR:-/home/nshenoy/franka_diagnostics}
g++ -std=c++17 -O2 -Wall -Wextra -Wno-misleading-indentation -Wno-maybe-uninitialized -DBOOST_BIND_GLOBAL_PLACEHOLDERS \
  -I/usr/include/eigen3 -I"$wave_franka_root/libfranka/include" -I"$wave_franka_root/libfranka/common/include" \
  "$wave_dir/controller.cpp" -L"$wave_franka_root/build" -Wl,-rpath,"$wave_franka_root/build" -lfranka -pthread -o "$wave_dir/controller"
