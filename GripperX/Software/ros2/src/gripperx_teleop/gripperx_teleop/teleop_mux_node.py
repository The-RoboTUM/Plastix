#!/usr/bin/env python3
"""
Teleop multiplexer for GripperX.

Forwards the active cmd_vel input to /cmd_vel.
Switch mode: publish to topic /teleop/set_mode (std_msgs/String).
Valid modes: keyboard | controller | autonomous

In keyboard mode (default, parameter keyboard_pass_angular_z=false → real behavior
unchanged):
  - Steering → directly via /teleop/direct_steer (steer_servo_node, no swerve)
  - Drive    → /teleop/keyboard/cmd_vel (linear.x only) → /cmd_vel → swerve_cmd_node

DT-4/M2 digital twin: in the sim no steer_servo_node runs, so /teleop/direct_steer
goes unused. Parameter keyboard_pass_angular_z=true additionally passes angular.z
from /teleop/keyboard/cmd_vel through in keyboard mode, so A/D steer via the
swerve path (swerve_cmd_node IK) -- helper steering for the sim only, see DT-10.

The active mode is published under /teleop/active_mode.
Safety timeout: if the active source goes silent, zero is sent.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

VALID_MODES = ('keyboard', 'controller', 'autonomous')


class TeleopMuxNode(Node):

    def __init__(self):
        super().__init__('teleop_mux')

        self.declare_parameter('initial_mode',    'keyboard')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('cmd_timeout_sec', 0.5)
        # DT-4/M2: default false -> real behavior (steering only via direct_steer,
        # angular.z in keyboard mode stays 0) remains byte-identical. Only the
        # sim sets this to true (no steer_servo_node present). See DT-10
        # for the planned real servo steering path in the sim.
        self.declare_parameter('keyboard_pass_angular_z', False)

        self._mode    = str(self.get_parameter('initial_mode').value)
        rate          = float(self.get_parameter('publish_rate_hz').value)
        self._timeout = float(self.get_parameter('cmd_timeout_sec').value)
        self._pass_angular_z = bool(self.get_parameter('keyboard_pass_angular_z').value)

        # latest[mode] = (Twist, timestamp_sec)
        self._latest: dict = {}

        self._cmd_pub  = self.create_publisher(Twist,  '/cmd_vel',            10)
        self._mode_pub = self.create_publisher(String, '/teleop/active_mode', 10)

        for mode in VALID_MODES:
            self.create_subscription(
                Twist,
                f'/teleop/{mode}/cmd_vel',
                lambda msg, m=mode: self._on_cmd(m, msg),
                10,
            )

        self.create_subscription(String, '/teleop/set_mode', self._on_set_mode, 10)
        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f'teleop_mux ready — mode: {self._mode} '
            f'(switch: ros2 topic pub /teleop/set_mode std_msgs/String \'{{data: "keyboard"}}\')'
        )

    def _on_cmd(self, mode: str, msg: Twist):
        self._latest[mode] = (msg, self.get_clock().now().nanoseconds * 1e-9)

    def _on_set_mode(self, msg: String):
        if msg.data in VALID_MODES:
            prev = self._mode
            self._mode = msg.data
            if self._mode == 'keyboard' and prev != 'keyboard':
                self._cmd_pub.publish(Twist())
            self.get_logger().info(f'Teleop mode → {self._mode}')
        else:
            self.get_logger().warn(
                f'Unknown mode "{msg.data}" — valid: {VALID_MODES}'
            )

    def _tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        out = Twist()

        if self._mode == 'keyboard':
            # Steering: steer_servo_node via /teleop/direct_steer (no cmd_vel needed)
            # Drive: keyboard/cmd_vel → forward linear.x only, angular.z=0
            kbd = self._latest.get('keyboard')
            if kbd and (now - kbd[1]) < self._timeout:
                out.linear.x = kbd[0].linear.x
                if self._pass_angular_z:
                    # Sim helper steering (DT-4/M2, see DT-10): no
                    # steer_servo_node present -> A/D must run via /cmd_vel,
                    # instead of (real) via /teleop/direct_steer.
                    out.angular.z = kbd[0].angular.z
                # otherwise (default) deliberately 0 — steering via direct_steer
        else:
            entry = self._latest.get(self._mode)
            if entry and (now - entry[1]) < self._timeout:
                out = entry[0]

        self._cmd_pub.publish(out)

        mode_msg = String()
        mode_msg.data = self._mode
        self._mode_pub.publish(mode_msg)


def main():
    rclpy.init()
    node = TeleopMuxNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
