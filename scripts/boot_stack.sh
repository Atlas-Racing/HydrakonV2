#!/usr/bin/env bash
set -eo pipefail

REPO=/home/atlasracing/HydrakonV2

source /opt/ros/humble/setup.bash
source /home/atlasracing/zed_ws/install/local_setup.bash
source "$REPO/install/local_setup.bash"

cd "$REPO"

pids=()
cleanup() {
    trap - SIGINT SIGTERM EXIT
    kill "${pids[@]}" 2>/dev/null || true
    wait "${pids[@]}" 2>/dev/null || true
}
trap cleanup SIGINT SIGTERM EXIT

echo "[boot_stack] launching hydrakon_bringup..."
ros2 launch hydrakon_bringup hydrakon_bringup.py &
pids+=($!)

sleep 5

echo "[boot_stack] launching hydrakon_can..."
ros2 launch hydrakon_can hydrakon_can_launch.py &
pids+=($!)

sleep 2

echo "[boot_stack] launching hydrakon_planner (corridor_controller_2d)..."
ros2 launch hydrakon_bringup corridor_controller_launch.py controller_mode:=2d &
pids+=($!)

wait -n "${pids[@]}"
