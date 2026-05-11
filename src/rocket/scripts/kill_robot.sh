#!/bin/bash
# Force-stop the slam.launch.py stack when Ctrl+C didn't fully clean up.
# Sends SIGTERM first; anything still alive after 1s gets SIGKILL.
set +e

NAMES=(
    "ros2 launch articubot_one slam"
    "rplidar_node"
    "scan_to_scan_filter"
    "component_container"
    "async_slam_toolbox"
    "static_transform_publisher"
    "robot_state_publisher"
    "rviz2"
    "odrive_can_test.py"
    "odom_publisher.py"
)

echo "[kill_robot] SIGTERM..."
for name in "${NAMES[@]}"; do
    pkill -f "$name" 2>/dev/null
done

sleep 1

echo "[kill_robot] SIGKILL stragglers..."
for name in "${NAMES[@]}"; do
    pkill -9 -f "$name" 2>/dev/null
done

echo "[kill_robot] done"
