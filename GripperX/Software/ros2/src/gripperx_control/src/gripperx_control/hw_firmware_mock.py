"""Temporary firmware stand-in for bench testing gripperx_hardware_interface on the Pi."""

from __future__ import annotations

import math

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

# Provenance codes, matching EncoderStatus in the ESP32 firmware's motor_controller.hpp.
PROVENANCE_NO_ENCODER = 0.0
PROVENANCE_INIT_FAILED = 1.0
PROVENANCE_LIVE_UNCONFIRMED = 2.0
PROVENANCE_LIVE = 3.0

# SIMULATED ENCODERS (user decision 2026-10-16). Until then this mock reported
# NO_ENCODER on every wheel and echoed the command as velocity -- which, since the
# whole-drive fault exists, trips it after 2 s of desk driving. Now each wheel has a
# simulated encoder, selected per wheel by `simulated_encoders`:
#   'ok'           position = integral of the commanded velocity, velocity = command;
#                  LIVE_UNCONFIRMED until the simulated wheel has moved by one count,
#                  then LIVE -- the real firmware's promotion on its first count change.
#   'unconfirmed'  the 2026-10-13 shape: registered, never counts. LIVE_UNCONFIRMED
#                  forever, position frozen, velocity 0.0 (a measured zero).
#   'none'         NO_ENCODER: position frozen, velocity = the command (the firmware's
#                  getRPM() fallback echoes the target when the encoder is not inited).
#   'init_failed'  INIT_FAILED: as 'none', with code 1.
# WHAT 'ok' IS NOT: a measurement. It is the command integrated, so it can never show a
# stall, slip or a wrong sign; a closed-loop regulator against it sees zero error. The
# desk can exercise the encoder-missing drive fault with it, not the stall detector.
ENCODER_MODES = {
    'ok': PROVENANCE_LIVE,
    'unconfirmed': PROVENANCE_LIVE_UNCONFIRMED,
    'none': PROVENANCE_NO_ENCODER,
    'init_failed': PROVENANCE_INIT_FAILED,
}
# One count of the real encoder: 2*pi / COUNTS_PER_OUTPUT_REV (3200, firmware
# include/motor_controller.hpp). Only used for the LIVE promotion.
COUNT_RAD = 2.0 * math.pi / 3200.0


class SimulatedEncoder:
    """One wheel's simulated encoder. Pure logic, no ROS (unit-tested)."""

    def __init__(self, mode: str) -> None:
        if mode not in ENCODER_MODES:
            raise ValueError(
                f'simulated encoder mode {mode!r} is not one of {sorted(ENCODER_MODES)}')
        self.mode = mode
        self.position = 0.0
        self.velocity = 0.0
        self._travel = 0.0
        self.provenance = (
            PROVENANCE_LIVE_UNCONFIRMED if mode == 'ok' else ENCODER_MODES[mode])

    def step(self, command: float, dt: float) -> None:
        if self.mode == 'ok':
            self.velocity = command
            self.position += command * dt
            self._travel += abs(command * dt)
            if self._travel >= COUNT_RAD:
                self.provenance = PROVENANCE_LIVE
        elif self.mode == 'unconfirmed':
            self.velocity = 0.0
        else:
            self.velocity = command

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
        # 30.0 Hz, deliberately MATCHING the real firmware rather than being
        # fast: the firmware publishes /hw/joint_states every
        # STATES_PUBLISH_US = 33333 us (Software/microros/firmware/src/main.cpp)
        # -> 30 Hz. This mock is the ONLY stand-in for the firmware that
        # exists anywhere - the twin replaces the whole hardware interface
        # with gz_ros2_control and has no /hw/* topics at all - so anything
        # characterised against it sees this cadence.
        # KEEP THIS IN STEP WITH STATES_PUBLISH_US. Raise it per-launch for a
        # deliberate over-rate stress test; do not raise the default.
        self.declare_parameter("publish_rate_hz", 30.0)
        # Per wheel, joint order FL, FR, BL, BR. See SimulatedEncoder. Default: all
        # four working. The 2026-10-13 shape is ['ok', 'ok', 'ok', 'unconfirmed'].
        self.declare_parameter("simulated_encoders", ["ok", "ok", "ok", "ok"])
        # REAL-ROBOT GUARD. The mock is selected only by real_robot.launch.py's
        # use_mock_firmware -- whose launch DEFAULT is true; only the systemd
        # bringup script passes false. A mock running next to the real ESP32 would
        # be a second writer on /hw/joint_states feeding simulated-healthy encoders
        # into the drive-fault and stall detection. So before publishing anything
        # the mock watches the graph for this long, and it refuses to publish (now
        # and later) while any other publisher exists on that topic.
        self.declare_parameter("startup_check_sec", 2.0)

        self.joint_commands_topic = str(self.get_parameter("joint_commands_topic").value)
        self.joint_states_topic = str(self.get_parameter("joint_states_topic").value)
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        self.latest_command = [0.0] * NUM_JOINTS
        # Reserved steering slots, never written — see the module comment.
        self.steer_positions = [0.0] * NUM_STEER
        self.wheel_velocities = [0.0] * NUM_WHEEL
        self.wheel_positions = [0.0] * NUM_WHEEL
        modes = [str(m) for m in self.get_parameter("simulated_encoders").value]
        if len(modes) != NUM_WHEEL:
            raise ValueError(f"simulated_encoders needs {NUM_WHEEL} entries, got {modes}")
        self.encoders = [SimulatedEncoder(mode) for mode in modes]
        self.wheel_provenance = [enc.provenance for enc in self.encoders]
        self.last_update_time = self.get_clock().now()
        self.started_at = self.get_clock().now()
        self.startup_check_sec = float(self.get_parameter("startup_check_sec").value)
        self.refused = False

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
        self.get_logger().warning(
            "SIMULATED encoders (FL, FR, BL, BR): %s -- 'ok' is the command integrated, "
            "not a measurement. Publishing starts after a %.1f s single-writer check."
            % (modes, self.startup_check_sec))

    def other_writers(self) -> int:
        """Publishers on the states topic other than this node."""
        return max(0, self.count_publishers(self.joint_states_topic) - 1)

    def check_single_writer(self) -> None:
        if self.refused:
            return
        others = self.other_writers()
        if others > 0:
            self.refused = True
            self.get_logger().fatal(
                "%d other publisher(s) on %s -- this is NOT a desk setup (the real "
                "firmware is probably running). The mock STOPS publishing for good."
                % (others, self.joint_states_topic))

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

        if (now - self.started_at).nanoseconds * 1e-9 < self.startup_check_sec:
            return
        # Checked on EVERY cycle, before anything goes out: a local graph-cache
        # query, cheap at 30 Hz, and no timer ordering can let one message slip.
        self.check_single_writer()
        if self.refused:
            return

        if dt <= 0.0:
            dt = 1.0 / 100.0

        for index, encoder in enumerate(self.encoders):
            encoder.step(self.latest_command[NUM_STEER + index], dt)
            self.wheel_velocities[index] = encoder.velocity
            self.wheel_positions[index] = encoder.position
            self.wheel_provenance[index] = encoder.provenance

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
