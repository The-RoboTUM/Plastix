"""Temporary firmware stand-in for bench testing gripperx_hardware_interface on the Pi."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

NUM_STEER = 4
NUM_WHEEL = 4
NUM_JOINTS = NUM_STEER + NUM_WHEEL
# hw/joint_states also carries 4 integrated wheel positions in rad at indices 8..11,
# matching the encoder firmware (HWR-10). Publishing them here keeps mock and real
# firmware on the same contract, so gripperx_hardware_interface::read() takes the same
# code path in both cases instead of only ever seeing the short message in bench runs.
# ... and 4 provenance codes at indices 12..15, one per wheel (FR-11 items 5/6).
NUM_STATE_VALUES = NUM_JOINTS + NUM_WHEEL + NUM_WHEEL

# Provenance code this mock reports for every wheel: NO_ENCODER, matching
# EncoderStatus::NoEncoder in the ESP32 firmware's motor_controller.hpp.
#
# THE MOCK MUST DECLARE ITS OWN ECHO. wheel_velocities[i] is literally
# latest_command[NUM_STEER + i] below — the command handed straight back, with no
# encoder, no dynamics and no error anywhere in it. Reporting anything else here would
# make the mock claim a measurement it structurally cannot make, and that is exactly the
# class of defect (a mock better than the hardware) that the steering comment above
# records. It also makes this file the standing NEGATIVE TEST for the provenance path:
# against the mock, /hw/wheel_feedback_valid must read NO_ENCODER on all four wheels.
PROVENANCE_NO_ENCODER = 0.0

# Indices 0..3 (steering) are published as a constant 0.0 — ON PURPOSE (FR-10 item 5).
# The real ESP32 zeroes the whole state array and then fills only 4..7 and 8..11
# (main.cpp): the steering servos hang off the Pi's USB bus, the ESP32 has no steering
# sensor and no steering input of any kind, so it structurally cannot report an angle.
# This mock used to model the steering with a first-order lag toward the command, which
# made it BETTER than the hardware and hid exactly that: code reading the steering
# position out of /hw/joint_states worked on the bench and read constant zero on the
# machine. A mock that is better than the hardware hides the bug it exists to expose.
# The real steering measurement is /hw/steer_states from steer_servo_node, which
# gripperx_hardware_interface merges into the steering position state interfaces.


class HwFirmwareMock(Node):
    def __init__(self) -> None:
        super().__init__("hw_firmware_mock")

        self.declare_parameter("joint_commands_topic", "/hw/joint_commands")
        self.declare_parameter("joint_states_topic", "/hw/joint_states")
        # 30.0 Hz, deliberately MATCHING the real firmware rather than being fast.
        # The firmware publishes /hw/joint_states every STATES_PUBLISH_US = 33333 us
        # (Software/microros/firmware/src/main.cpp) -> 30 Hz, measured 29.999 Hz on
        # hardware 2026-08-20. This mock is the ONLY stand-in for the firmware that
        # exists anywhere - the twin replaces the whole hardware interface with
        # gz_ros2_control and has no /hw/* topics at all - so anything characterised
        # against it sees this cadence. It was 100.0 (3.3x the real rate), which made
        # any timing conclusion drawn here wrong by construction.
        # KEEP THIS IN STEP WITH STATES_PUBLISH_US. Raise it per-launch if you want a
        # deliberate over-rate stress test; do not raise the default.
        self.declare_parameter("publish_rate_hz", 30.0)

        self.joint_commands_topic = str(self.get_parameter("joint_commands_topic").value)
        self.joint_states_topic = str(self.get_parameter("joint_states_topic").value)
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        self.latest_command = [0.0] * NUM_JOINTS
        # Reserved steering slots, never written — see the module comment.
        self.steer_positions = [0.0] * NUM_STEER
        self.wheel_velocities = [0.0] * NUM_WHEEL
        self.wheel_positions = [0.0] * NUM_WHEEL
        self.wheel_provenance = [PROVENANCE_NO_ENCODER] * NUM_WHEEL
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
            "Mock firmware ready. %s (%d values) -> %s (%d values)"
            % (
                self.joint_commands_topic,
                NUM_JOINTS,
                self.joint_states_topic,
                NUM_STATE_VALUES,
            )
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

        for index in range(NUM_WHEEL):
            self.wheel_velocities[index] = self.latest_command[NUM_STEER + index]
            # Integrate the commanded velocity so the position advances monotonically,
            # the same way the encoder firmware accumulates PCNT counts.
            self.wheel_positions[index] += self.wheel_velocities[index] * dt

        message = Float64MultiArray()
        message.data = (
            self.steer_positions
            + self.wheel_velocities
            + self.wheel_positions
            + self.wheel_provenance
        )
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
