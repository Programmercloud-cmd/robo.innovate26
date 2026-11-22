#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/../.."
source /opt/ros/humble/setup.bash
franka_factory_root=${FRANKA_DIAGNOSTICS_DIR:-/home/nshenoy/franka_diagnostics}
factory_compile_flags=()
factory_binary=teaching/factory_cycle/controller
if [[ ${1:-} == --moving-conveyor && $# == 1 ]]; then
  factory_compile_flags=(-DFACTORY_MOVING_CONVEYOR_20MM)
  factory_binary=teaching/factory_cycle/controller_conveyor_moving
elif [[ $# != 0 ]]; then
  printf '%s\n' 'Usage: build.sh [--moving-conveyor]' >&2
  exit 2
fi
g++ -std=c++17 -O2 -Wall -Wextra -Wno-misleading-indentation -DBOOST_BIND_GLOBAL_PLACEHOLDERS \
  "${factory_compile_flags[@]}" \
  -I/usr/include/eigen3 -I"$franka_factory_root/libfranka/include" \
  -I"$franka_factory_root/libfranka/common/include" teaching/factory_cycle/controller.cpp \
  -L"$franka_factory_root/build" -Wl,-rpath,"$franka_factory_root/build" -lfranka -pthread \
  -o "$factory_binary"
