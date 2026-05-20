#!/usr/bin/env python3
"""Subscribes to ODrive encoder CAN messages and publishes /odom.

Fill in the forward-kinematics math in compute_odometry_velocity() — everything
else (CAN decode, TF, message construction) is done for you.
"""
import math
import os
import signal
import struct
import threading
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
import can

# Same wheel layout as odrive_can_test.py
NODE_IDS = {
    'RR': 59,
    'RL': 58,
    'FL': 61,
    'FR': 60,
}
INVERT = {
    'RR': -1,
    'RL': 1,
    'FL': 1,
    'FR': -1,
}

CAN_INTERFACE = 'can0'
CMD_GET_ENCODER_ESTIMATES = 0x09  # ODrive sends pos (float, turns) and vel (float, turns/sec)

WHEEL_RADIUS = 0.0635       # meters
WHEELBASE = 0.295            # meters — front-to-rear axle distance
TRACK_WIDTH = 0.4038          # meters — left-to-right wheel distance
GEAR_RATIO = 20.0           # motor turns per wheel turn

ODOM_RATE_HZ = 50.0         # how often to publish /odom and integrate pose
ODOM_FRAME = 'odom'
BASE_FRAME = 'base_link'


def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')
        self.bus = can.interface.Bus(channel=CAN_INTERFACE, interface='socketcan')
        self.reader = can.Notifier(self.bus, [self._on_can_msg])

        # Latest wheel angular velocity (rad/s) at the *wheel* (not the motor).
        # Sign already corrected for INVERT — positive means the robot's
        # convention "forward" for that wheel.
        self.wheel_omega = {name: 0.0 for name in NODE_IDS}

        # Pose state (integrated)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(1.0 / ODOM_RATE_HZ, self.publish_odom)

        self.get_logger().info(f'odom_publisher listening on {CAN_INTERFACE} for nodes {list(NODE_IDS.values())}')

    # -- CAN intake --------------------------------------------------------

    def _on_can_msg(self, msg):
        cmd_id = msg.arbitration_id & 0x1F
        node_id = msg.arbitration_id >> 5

        #reject every message except for command ids for velocity and position. a requirement is also that the message is 8 bytes
        if cmd_id != CMD_GET_ENCODER_ESTIMATES:
            return
        if len(msg.data) < 8:
            return
        
        # Find which wheel this is by matching the node id by the NODE_IDS dict. ignore if not from these node ids
        name = next((n for n, nid in NODE_IDS.items() if nid == node_id), None)
        if name is None:
            return
        # bytes 0-3: position (turns at the motor shaft)
        # bytes 4-7: velocity (turns/sec at the motor shaft)
        # multiplied by two because the ODrive config uses a 4 pole paired motor meanwhile the motor is actually a 2 pole paired.
        motor_turns_per_sec = struct.unpack('<f', bytes(msg.data[4:8]))[0] * 2.0

        # Undo INVERT to get the wheel's natural-convention spin direction,
        # then convert motor turns/sec → wheel rad/s.
        wheel_turns_per_sec = motor_turns_per_sec * INVERT[name] / GEAR_RATIO
        self.wheel_omega[name] = wheel_turns_per_sec * 2.0 * math.pi  # rad/s

    # -- Forward kinematics (FILL IN) -------------------------------------

    def compute_odometry_velocity(self, w):
        """
        Given the four wheel angular velocities (rad/s), return the robot's
        body-frame velocity as (vx, vy, wz) in m/s and rad/s.

        w is a dict: {'FL': rad/s, 'FR': rad/s, 'RL': rad/s, 'RR': rad/s}

        Mecanum forward kinematics — derive from your inverse kinematics
        and substitute your wheel geometry.
        """
        # TODO: replace these placeholders with your own equations.
        L = WHEELBASE / 2.0
        W = TRACK_WIDTH / 2.0
        r = WHEEL_RADIUS
        FL = w['FL']
        FR = w['FR']
        RL = w['RL']
        RR = w['RR']

        vx = (FL + FR + RL + RR) * r / 4.0
        vy = (-FL + FR + RL - RR) * r / 4.0
        wz = (-FL + FR - RL + RR) * r / (4.0 * (L + W))
        return vx, vy, wz

    # -- Pose integration + publishing ------------------------------------

    def publish_odom(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        if dt <= 0.0:
            return

        # calls for the function which calculates the robots velocities
        vx, vy, wz = self.compute_odometry_velocity(self.wheel_omega)

        # Integrate body-frame velocity into the world frame.
        cos_t = math.cos(self.theta)
        sin_t = math.sin(self.theta)
        self.x += (vx * cos_t - vy * sin_t) * dt
        self.y += (vx * sin_t + vy * cos_t) * dt
        self.theta += wz * dt
        
        # Wrap theta into (-pi, pi)
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        q = yaw_to_quaternion(self.theta)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = ODOM_FRAME
        odom.child_frame_id = BASE_FRAME
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = ODOM_FRAME
        t.child_frame_id = BASE_FRAME
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)

    def destroy_node(self):
        # bus.shutdown() can occasionally block on socketcan close. Run it in
        # a daemon thread with a 1s deadline so this never hangs forever.
        t = threading.Thread(target=lambda: self._safe_bus_shutdown(), daemon=True)
        t.start()
        t.join(timeout=1.0)
        super().destroy_node()

    def _safe_bus_shutdown(self):
        try:
            self.bus.shutdown()
        except Exception:
            pass


def main():
    rclpy.init()
    node = OdomPublisher()

    # Watchdog: under SIGINT or SIGTERM, arm a hard force-exit 2s later. If
    # destroy_node() hangs (CAN close, etc.), the process still dies promptly
    # so the next launch isn't blocked by a zombie holding the CAN socket.
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
