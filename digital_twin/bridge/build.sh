#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/../.."
source /opt/ros/humble/setup.bash
franka_diagnostics_dir=${FRANKA_DIAGNOSTICS_DIR:-/home/nshenoy/franka_diagnostics}
g++ -std=c++17 -O2 -Wall -Wextra \
  -I"$franka_diagnostics_dir/libfranka/include" -I"$franka_diagnostics_dir/libfranka/common/include" \
  digital_twin/bridge/read_state.cpp -L"$franka_diagnostics_dir/build" \
  -Wl,-rpath,"$franka_diagnostics_dir/build" -lfranka -pthread -o digital_twin/bridge/read_state.next
mv digital_twin/bridge/read_state.next digital_twin/bridge/read_state
g++ -std=c++17 -O2 -Wall -Wextra \
  -I"$franka_diagnostics_dir/libfranka/include" -I"$franka_diagnostics_dir/libfranka/common/include" \
  digital_twin/bridge/read_hand.cpp -L"$franka_diagnostics_dir/build" \
  -Wl,-rpath,"$franka_diagnostics_dir/build" -lfranka -pthread -o digital_twin/bridge/read_hand.next
mv digital_twin/bridge/read_hand.next digital_twin/bridge/read_hand
