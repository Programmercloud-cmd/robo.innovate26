#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash
franka_recorder_root=${FRANKA_DIAGNOSTICS_DIR:-/home/nshenoy/franka_diagnostics}
recorder_binary=${1:-assets/owner_recorder_synthetic}
mkdir -p "$(dirname "$recorder_binary")"
g++ -std=c++17 -O2 -Wall -Wextra -Werror -pthread \
  -I"$franka_recorder_root/libfranka/include" owner_recorder_synthetic.cpp \
  -L"$franka_recorder_root/build" -Wl,-rpath,"$franka_recorder_root/build" \
  -lfranka -o "$recorder_binary"
