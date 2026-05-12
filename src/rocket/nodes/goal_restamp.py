#!/usr/bin/env python3
"""Goal-pose re-stamp shim.

Subscribes to /goal_pose_raw (where RViz / CLI publishes the goal),
zero-stamps it, and republishes to /goal_pose for nav2.

Why: nav2's planner uses the goal's header.stamp when looking up TF for
the start pose. RViz stamps the goal with the current time at click. After
~10s of replanning, that stamp falls outside tf2's internal cache and
every replan fails with "extrapolation into the past". A stamp of Time(0)
tells tf2 to use the LATEST available transform on every lookup, so the
goal never goes stale.

Usage:
  - In RViz, set the "2D Goal Pose" tool's Topic to "/goal_pose_raw".
  - From CLI:
      ros2 topic pub --once /goal_pose_raw geometry_msgs/msg/PoseStamped \
        '{header: {frame_id: "map"}, pose: {position: {x: 1.0}, orientation: {w: 1.0}}}'
  - Inspect:
      ros2 topic echo /goal_pose --once     # should show stamp sec=0 nanosec=0
"""
import os
import signal
import threading

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class GoalRestamp(Node):
    def __init__(self):
        super().__init__('goal_restamp')
        self.sub = self.create_subscription(PoseStamped, '/goal_pose_raw', self.cb, 10)
        self.pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.get_logger().info('Listening on /goal_pose_raw, republishing to /goal_pose with stamp=0')

    def cb(self, msg: PoseStamped):
        msg.header.stamp = Time()  # sec=0, nanosec=0 → "use latest"
        # Preserve the frame_id (typically 'map'); only the stamp changes.
        self.pub.publish(msg)
        self.get_logger().info(
            f'Restamped goal in frame "{msg.header.frame_id}" '
            f'at ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})'
        )


def main():
    rclpy.init()
    node = GoalRestamp()

    # Same shutdown pattern as the other nodes — watchdog on signals so this
    # never blocks a relaunch.
    def _on_signal(signum, frame):
        threading.Timer(2.0, lambda: os._exit(0)).start()
        raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM, _on_signal)
    signal.signal(signal.SIGINT, _on_signal)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
