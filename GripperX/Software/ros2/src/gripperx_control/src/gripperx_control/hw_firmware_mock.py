"""Temporary firmware stand-in for bench testing gripperx_hardware_interface on the Pi."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

NUM_STEER = 4
NUM_WHEEL = 4
NUM_JOINTS = NUM_STEER + NUM_WHEEL


class HwFirmwareMock(Node):
    def __init__(self) -> None:
        super().__init__("hw_firmware_mock")

        self.declare_parameter("joint_commands_topic", "/hw/joint_commands")
        self.declare_parameter("joint_states_topic", "/hw/joint_states")
        self.declare_parameter("publish_rate_hz", 100.0)
        self.declare_parameter("steer_time_constant_sec", 0.15)

        self.joint_commands_topic = str(self.get_parameter("joint_commands_topic").value)
        self.joint_states_topic = str(self.get_parameter("joint_states_topic").value)
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.steer_time_constant_sec = float(self.get_parameter("steer_time_constant_sec").value)

        self.latest_command = [0.0] * NUM_JOINTS
        self.steer_positions = [0.0] * NUM_STEER
        self.wheel_velocities = [0.0] * NUM_WHEEL
        self.last_update_time = self.get_clock().now()

        self.create_subscription(
            Float64MultiArray,
            self.joint_commands_topic,
            self.command_callback,
            10,
        )
        self.states_publisher = self.create_publisher(
            Float64MultiArray,
            self.joint_states_topic,
            10,
        )
        self.create_timer(1.0 / publish_rate_hz, self.publish_states)

        self.get_logger().info(
            "Mock firmware ready. %s -> %s"
            % (self.joint_commands_topic, self.joint_states_topic)
        )

    def command_callback(self, message: Float64MultiArray) -> None:
        if len(message.data) < NUM_JOINTS:
            self.get_logger().warning(
                "Expected %d command values, got %d." % (NUM_JOINTS, len(message.data))
            )
            return
        self.latest_command = [float(value) for value in message.data[:NUM_JOINTS]]

    def publish_states(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds * 1e-9
        self.last_update_time = now

        if dt <= 0.0:
            dt = 1.0 / 100.0

        alpha = min(1.0, dt / max(self.steer_time_constant_sec, 1e-3))
        for index in range(NUM_STEER):
            target = self.latest_command[index]
            self.steer_positions[index] += alpha * (target - self.steer_positions[index])

        for index in range(NUM_WHEEL):
            self.wheel_velocities[index] = self.latest_command[NUM_STEER + index]

        message = Float64MultiArray()
        message.data = self.steer_positions + self.wheel_velocities
        self.states_publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HwFirmwareMock()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
