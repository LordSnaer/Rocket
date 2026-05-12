#!/usr/bin/env python3
import math
import os
import signal
import struct
import subprocess
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import can

ESP32_PORT = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
DOCK_CMD = f"stty -F {ESP32_PORT} 115200 raw -echo && printf 'dock\\n' > {ESP32_PORT}"
UNDOCK_CMD = f"stty -F {ESP32_PORT} 115200 raw -echo && printf 'undock\\n' > {ESP32_PORT}"

NODE_IDS = {
    'RR': 59,  # Rear Right
    'RL': 58,  # Rear Left
    'FL': 61,  # Front Left
    'FR': 60,  # Front Right
}
CAN_INTERFACE = 'can0' # Number on Can Network

WHEEL_RADIUS = 0.0635      # meters — TODO: set to your wheel radius
WHEELBASE = 0.25          # meters — front-axle to rear-axle distance

Beta0 = math.pi/2          # meters — angle between forward direction and the normal to the wheel front left
Beta1 = -math.pi/2         # meters — angle between forward direction and the normal to the wheel front right
Beta2 = math.pi/2          # meters — angle between forward direction and the normal to the wheel rear left
Beta3 = -math.pi/2         # meters — angle between forward direction and the normal to the wheel rear right

Gamma0 = math.pi/4          # radians — Helix angle of the wheel front left
Gamma1 = -math.pi/4         # radians — Helix angle of the wheel front right
Gamma2 = math.pi/4          # radians — Helix angle of the wheel rear left
Gamma3 = -math.pi/4         # radians — Helix angle of the wheel rear right

Alpha0 = 0.939              # angle betwwen wheel placement vector and forward direction vector
Alpha1 = -0.939             # angle betwwen wheel placement vector and forward direction vector
Alpha2 = math.pi - 0.939  # angle betwwen wheel placement vector and forward direction vector
Alpha3 = math.pi + 0.939  # angle betwwen wheel placement vector and forward direction vector
GEAR_RATIO = 20.0          # motor turns per wheel turn
MAX_TURNS_PER_SEC = 15.0   # well below vel_limit=30, gives margin for hall noise
MAX_LINEAR_SPEED = 0.2     # m/s, caps vx/vy so wz always has wheel headroom for turning
MAX_ANGULAR_SPEED = 1.5      # rad/s, clamps yaw rate from /cmd_vel
CMD_RATE_HZ = 20.0         # max rate to push setpoints onto CAN, regardless of /cmd_vel rate
CMD_VEL_TIMEOUT = 0.5      # seconds with no /cmd_vel before watchdog stops the wheels
INVERT = {                # flip sign per wheel if motor spins opposite
    'RR': -1,
    'RL': 1,
    'FL': 1,
    'FR': -1,
}

CMD_HEARTBEAT = 0x01
CMD_SET_AXIS_STATE = 0x07
CMD_SET_INPUT_VEL = 0x0D
CMD_CLEAR_ERRORS = 0x18
AXIS_STATE_CLOSED_LOOP_CONTROL = 8
AXIS_STATE_IDLE = 1

ODRIVE_ERRORS = {
    0x00000001: 'INITIALIZING',
    0x00000002: 'SYSTEM_LEVEL',
    0x00000004: 'TIMING_ERROR',
    0x00000008: 'MISSING_ESTIMATE',
    0x00000010: 'BAD_CONFIG',
    0x00000020: 'DRV_FAULT',
    0x00000040: 'MISSING_INPUT',
    0x00000100: 'DC_BUS_OVER_VOLTAGE',
    0x00000200: 'DC_BUS_UNDER_VOLTAGE',
    0x00000400: 'DC_BUS_OVER_CURRENT',
    0x00000800: 'DC_BUS_OVER_REGEN_CURRENT',
    0x00001000: 'CURRENT_LIMIT_VIOLATION',
    0x00002000: 'MOTOR_OVER_TEMP',
    0x00004000: 'INVERTER_OVER_TEMP',
    0x00008000: 'VELOCITY_LIMIT_VIOLATION',
    0x00010000: 'POSITION_LIMIT_VIOLATION',
    0x01000000: 'WATCHDOG_TIMER_EXPIRED',
    0x02000000: 'ESTOP_REQUESTED',
    0x04000000: 'SPINOUT_DETECTED',
    0x08000000: 'BRAKE_RESISTOR_DISARMED',
    0x10000000: 'THERMISTOR_DISCONNECTED',
    0x40000000: 'CALIBRATION_ERROR',
}


def decode_errors(err):
    if err == 0:
        return 'no error'
    names = [name for bit, name in ODRIVE_ERRORS.items() if err & bit]
    return ', '.join(names) if names else f'0x{err:08X}'


class ODriveCanTest(Node):
    def __init__(self):
        super().__init__('odrive_can_test')
        self.bus = can.interface.Bus(channel=CAN_INTERFACE, interface='socketcan')
        self._last_rearm = {nid: 0.0 for nid in NODE_IDS.values()}
        self._last_cmd_send = 0.0
        self._prev_motor_cmd = {name: 0.0 for name in NODE_IDS}
        self.arm_all()
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_cb, 10)
        self._prev_buttons = []
        self.reader = can.Notifier(self.bus, [self._on_can_msg])
        # Watchdog: nav2 stops publishing /cmd_vel when a goal is reached;
        # without this, the last setpoint stays latched and the robot keeps
        # moving. If no /cmd_vel arrives in CMD_VEL_TIMEOUT seconds, force
        # all wheels to 0.
        self._last_cmd_vel_time = self.get_clock().now()
        self._wd_stopped = True  # don't spam zero-set on first tick
        self._watchdog = self.create_timer(0.1, self._watchdog_cb)
        self.get_logger().info(f'Listening on /cmd_vel, controlling nodes {list(NODE_IDS.values())} on {CAN_INTERFACE}')
        self.get_logger().info(f'==BUILD MARKER v8 (RR vy flipped)==  INVERT={INVERT}')

    def arm_all(self):
        for node_id in NODE_IDS.values():
            self.clear_errors(node_id)
            self.set_velocity(node_id, 0.0)
            self.set_axis_state(node_id, AXIS_STATE_CLOSED_LOOP_CONTROL)
            self.set_velocity(node_id, 0.0)

    def _on_can_msg(self, msg):
        # Heartbeat frame: byte 4 = axis_state. If any ODrive drops out of
        # closed loop (e.g. disarmed on error), clear + re-arm it.
        cmd_id = msg.arbitration_id & 0x1F
        node_id = msg.arbitration_id >> 5
        if cmd_id != CMD_HEARTBEAT or node_id not in NODE_IDS.values():
            return
        if len(msg.data) < 5:
            return
        axis_error = struct.unpack('<I', bytes(msg.data[0:4]))[0]
        axis_state = msg.data[4]
        if axis_state == AXIS_STATE_IDLE:
            # Don't auto-rearm if there's an active error — that loops forever
            # on velocity limit violations. Log once and stay disarmed.
            if axis_error != 0:
                now = self.get_clock().now().nanoseconds / 1e9
                if now - self._last_rearm[node_id] > 5.0:
                    self._last_rearm[node_id] = now
                    self.get_logger().warn(
                        f'Node {node_id} disarmed with error ({decode_errors(axis_error)}) — NOT re-arming. Clear manually.'
                    )
                return
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self._last_rearm[node_id] < 2.0:
                return
            self._last_rearm[node_id] = now
            self.get_logger().warn(f'Node {node_id} disarmed (no error) — re-arming')
            self.set_axis_state(node_id, AXIS_STATE_CLOSED_LOOP_CONTROL)

    def send(self, node_id, cmd_id, data):
        arb_id = (node_id << 5) | cmd_id
        msg = can.Message(arbitration_id=arb_id, data=data, is_extended_id=False)
        self.bus.send(msg)

    def set_axis_state(self, node_id, state):
        data = struct.pack('<I', state) + b'\x00\x00\x00\x00'
        self.send(node_id, CMD_SET_AXIS_STATE, data)

    def set_velocity(self, node_id, vel, torque_ff=0.0):
        data = struct.pack('<ff', vel, torque_ff)
        self.send(node_id, CMD_SET_INPUT_VEL, data)

    def clear_errors(self, node_id):
        self.send(node_id, CMD_CLEAR_ERRORS, b'')

    def joy_cb(self, msg):
        buttons = list(msg.buttons)
        prev = self._prev_buttons
        self._prev_buttons = buttons
        if not prev or len(prev) != len(buttons):
            return
        # Rising-edge: was 0, now 1.
        if len(buttons) > 0 and buttons[0] == 1 and prev[0] == 0:
            self.get_logger().info('Button 0 pressed -> undock')
            subprocess.run(UNDOCK_CMD, shell=True, check=False)
        if len(buttons) > 3 and buttons[3] == 1 and prev[3] == 0:
            self.get_logger().info('Button 3 pressed -> dock')
            subprocess.run(DOCK_CMD, shell=True, check=False)

    def cmd_vel_cb(self, msg):
        # Watchdog timestamp updates on EVERY message — independent of the
        # rate-limit below — otherwise upstream rates >CMD_RATE_HZ would
        # have most messages dropped before refreshing the watchdog.
        self._last_cmd_vel_time = self.get_clock().now()
        self._wd_stopped = False
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._last_cmd_send < 1.0 / CMD_RATE_HZ:
            return
        self._last_cmd_send = now

        vx = max(-MAX_LINEAR_SPEED, min(MAX_LINEAR_SPEED, msg.linear.x))
        vy = max(-MAX_LINEAR_SPEED, min(MAX_LINEAR_SPEED, msg.linear.y))
        wz = max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, msg.angular.z))

        L = (0.295/2) + (0.4038/2)
        k = 0


        wheel_linear = {
            'FL': (1/WHEEL_RADIUS) * (vx - vy - L*wz),
            'FR': (1/WHEEL_RADIUS) * (vx + vy + L*wz),
            'RL': (1/WHEEL_RADIUS) * (vx + vy - L*wz),
            'RR': (1/WHEEL_RADIUS) * (vx - vy + L*wz),
        }

        for name, v in wheel_linear.items():
            # v is wheel angular velocity (rad/s) — paper formula includes the 1/r.
            wheel_rev_per_sec = v / (2.0 * math.pi)
            motor_turns_per_sec = wheel_rev_per_sec * GEAR_RATIO * INVERT[name]
            motor_turns_per_sec = max(-MAX_TURNS_PER_SEC, min(MAX_TURNS_PER_SEC, motor_turns_per_sec))
            # Deadzone — joystick noise below this would just thrash the
            # hall encoder around zero and trigger spurious direction flips.
            if abs(motor_turns_per_sec) < 0.5:
                motor_turns_per_sec = 0.0
            self._prev_motor_cmd[name] = motor_turns_per_sec
            self.set_velocity(NODE_IDS[name], motor_turns_per_sec)

    def _watchdog_cb(self):
        # If no /cmd_vel for CMD_VEL_TIMEOUT, stop all wheels. We only push
        # the zero command once per stale period (tracked by _wd_stopped) so
        # we don't hammer the CAN bus with redundant frames while idle.
        age = (self.get_clock().now() - self._last_cmd_vel_time).nanoseconds / 1e9
        if age > CMD_VEL_TIMEOUT and not self._wd_stopped:
            for nid in NODE_IDS.values():
                self.set_velocity(nid, 0.0)
            self._wd_stopped = True

    def destroy_node(self):
        try:
            for node_id in NODE_IDS.values():
                self.set_velocity(node_id, 0.0)
        except Exception:
            pass
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
    node = ODriveCanTest()

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
