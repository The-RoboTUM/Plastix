"""Drive 4 Feetech STS/ST steering servos on the Raspberry Pi."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float64MultiArray, String

from gripperx_control.sts_servo_bus import (
    StsServoBus,
    calibrated_angle_to_counts_asym,
    calibrated_counts_bounds,
    calibrated_counts_to_rad_asym,
)

STEER_JOINT_COUNT = 4
STEER_JOINT_NAMES = (
    "f_left_steer",
    "f_right_steer",
    "b_leftsteer",
    "b_rightsteer",
)
# Indices of the front servos in STEER_JOINT_NAMES/servo_ids/... (Task #22c).
FRONT_JOINT_INDICES = (0, 1)

# --- Steering sign convention (2026-08-13) -----------------------------------
#
# Definition in use: a wheel steers OUTWARD when the tyre's front swings AWAY
# from the vehicle body laterally, INWARD when it swings towards it. The
# mechanical range is 100 deg outward (user measurement 2026-08-13) / 35 deg
# inward on every wheel (raised from 30 deg 2026-08-17, user estimate,
# TO-VERIFY — see steer_servo.yaml for the honesty caveat on this number).
#
# Joint-angle sign, verified to be identical for ALL FOUR steering joints:
#   * gripperx_description/urdf/gripperx_v1.core.xacro, macro `steer_joint`:
#     every steering joint is `<axis xyz="0 0 1"/>` with `rpy="0 0 0"`, all four
#     have `parent="chassis_link"`, and chassis_link sits on base_link with
#     `rpy="0 0 0"`. So every joint turns about base_link's +Z — REP-103
#     counterclockwise seen from above. There is no mirroring in the model.
#   * swerve_kinematic_model.py `inverse_kinematics()`: delta_i = atan2(vy_i, vx_i)
#     for every wheel, i.e. the angle of that wheel's velocity direction measured
#     CCW from +x in the robot frame.
#   * swerve_cmd_node.py only REORDERS between model order (FL, BL, BR, FR) and
#     joint order (FL, FR, BL, BR); there is no per-wheel sign factor anywhere in
#     the steering path (`wheel_command_multipliers` exists for the DRIVE path
#     only, and the FL/BL drive mirroring lives in the ESP32 firmware —
#     unrelated to steering).
#   => +angle points a wheel towards the robot's LEFT (+y), -angle towards the
#      RIGHT (-y). Same for all four wheels.
#
# From that alone one would conclude outward = +y on the left wheels and -y on
# the right, i.e. (+1, -1, +1, -1). THAT CONCLUSION IS WRONG — it was tested on
# the machine and refuted (2026-08-13):
#
#   steering_outward_sign = (-1, +1, +1, -1) for (FL, FR, BL, BR)   # MEASURED
#
# How it was settled: all four servos were driven 15 deg in the measured outward
# tick direction and the resulting pose was inspected. The wheels lined up
# tangentially for an in-place spin — which is exactly the sign pattern the
# kinematics produces for pure rotation (FL -58.6, FR +58.6, BL +58.6,
# BR -58.6 after normalising each wheel line mod 180; the magnitude read 50.7
# here until 2026-08-21 and was stale — measured 58.57 in the twin). Outward and the spin pose
# therefore share the pattern (-, +, +, -), so a spin turns every wheel OUTWARD.
#
# Why the URDF reading misleads: the wheel hangs on a purely lateral lever arm
# off the king pin (*_wheel_offset_xyz, y = +-0.072 m), so turning the joint
# swings the wheel fore/aft around the pin rather than in/out. "Outward" is
# therefore about where the wheel body ends up relative to the chassis, not
# about toe. The two readings agree on the rear pair and contradict each other
# on the front pair, which is precisely where the derivation went wrong.
#
# Consequence worth keeping: with (-1, +1, +1, -1) the in-place spin pose needs
# 58.57 deg OUTWARD on every wheel, comfortably inside the 100 deg outward range.
# Under the refuted sign it would have needed 58.57 deg inward on three wheels,
# violating the 35 deg inward limit — i.e. spin would have been impossible,
# which is a good sanity check to re-run if these numbers are ever touched.
#
# The reachable joint range is thus -100/+35 deg on FL and BR, +100/-35 deg on
# FR and BL — asymmetric AND per-wheel, which is why limits are resolved per
# joint below.
#
# Measured on the machine 2026-08-13 (each wheel turned outward by hand, torque
# off, watching which id moved and in which tick direction — no commanded
# motion): outward makes the raw count DECREASE on FL and BR and INCREASE on FR
# and BL (diagonal, not per-side: the corner servos are mounted mirrored both
# left/right and front/rear). This is ground truth, not derivable from the code;
# it is carried as `steering_outward_tick_direction` and only used to check a
# pasted calibration for plausibility.
#
# Consequence of the two together: the count direction of a POSITIVE joint angle
# is negative for FL/FR and positive for BL/BR, so after calibration FL and FR
# will have `counts_at_pos_limit < center` (mirrored mount). That case is handled
# explicitly in `calibrated_counts_to_rad_asym()`.
#
# The same measurement showed the committed servo_ids to be wrong: in joint order
# FL, FR, BL, BR the ids are [13, 14, 11, 12]. See steer_servo.yaml — the stale
# count arrays are index-aligned with the OLD id order, so the id list must not
# be corrected on its own.
DEFAULT_OUTWARD_SIGN = (-1, 1, 1, -1)   # MEASURED 2026-08-13, see block above
DEFAULT_OUTWARD_TICK_DIRECTION = (-1, 1, 1, -1)


@dataclass
class SteerChannel:
    name: str
    port: str
    servo_id: int
    joint_index: int
    bus: StsServoBus


class SteerServoNode(Node):
    def __init__(self) -> None:
        super().__init__("steer_servo_node")

        self.declare_parameter("use_single_bus", True)
        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter(
            "serial_ports",
            ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2", "/dev/ttyACM3"],
        )
        self.declare_parameter("baud_rate", 1_000_000)
        self.declare_parameter("protocol_end", 1)
        # Joint order: f_left, f_right, b_left, b_right
        self.declare_parameter("servo_ids", [11, 14, 12, 13])
        self.declare_parameter("center_counts", [1778, 1173, 1206, 1551])
        # Legacy symmetric calibration (misleading names: NOT the counts at +-90 deg,
        # but the counts recorded at +-steering_angle_limit_deg). Only used when the
        # per-direction keys below are absent.
        self.declare_parameter("counts_plus_90", [2792, 2196, 2163, 2599])
        self.declare_parameter("counts_minus_90", [697, 152, 123, 626])
        # Per-direction calibration (2026-08-13): raw counts recorded while the
        # wheel was held at the outward / inward mechanical limit, joint order
        # FL, FR, BL, BR. Declared without a default on purpose — absent means
        # "fall back to the symmetric legacy model above".
        self.declare_parameter("counts_outward_limit", Parameter.Type.INTEGER_ARRAY)
        self.declare_parameter("counts_inward_limit", Parameter.Type.INTEGER_ARRAY)
        self.declare_parameter("steering_outward_limit_deg", Parameter.Type.DOUBLE)
        self.declare_parameter("steering_inward_limit_deg", Parameter.Type.DOUBLE)
        # Which joint-angle sign is physically outward, per joint (URDF +Z on all
        # four joints, so outward = +y left / -y right). See the top of this file.
        self.declare_parameter("steering_outward_sign", list(DEFAULT_OUTWARD_SIGN))
        # Measured raw-count direction of an outward turn, per joint. Plausibility
        # check only: it catches a calibration pasted with the wheels/ids swapped.
        self.declare_parameter(
            "steering_outward_tick_direction", list(DEFAULT_OUTWARD_TICK_DIRECTION)
        )
        self.declare_parameter("move_time_ms", 400)
        self.declare_parameter("move_acc", 20)
        self.declare_parameter("control_rate_hz", 50.0)
        self.declare_parameter("command_timeout_sec", 0.5)
        # 60deg (2026-07-16, user decision). The physical self-collision limit is
        # still enforced by the calibration end-stops (counts_plus_90/minus_90):
        # a 60deg command maps to those old ~45deg end-stop counts, so real servos
        # cannot exceed ~45deg until recalibration at rework. See steer_servo.yaml
        # and gripperx_control/docs/STEERING_LIMITS.md.
        self.declare_parameter("steering_angle_limit_deg", 60.0)
        # Task #22c: above the base limit, only the FRONT servos may
        # steer further (rear stays at steering_angle_limit_deg).
        # Default off -> behavior-neutral. Details: gripperx_control/docs/STEERING_LIMITS.md
        self.declare_parameter("enable_front_extended_steering", False)
        self.declare_parameter("front_extended_steering_limit_deg", 60.0)
        self.declare_parameter("ping_retries", 3)
        self.declare_parameter("require_all_servos", True)
        # OP-20 Option B / SR-12: at startup AND at every respawn the node adopts
        # the position the servos report instead of driving them to center_counts.
        # Centring is opt-in because a start is not a command (OP-24 / S1).
        self.declare_parameter("center_on_startup", False)
        self.declare_parameter("joint_commands_topic", "/hw/joint_commands")
        self.declare_parameter("steer_states_topic", "/hw/steer_states")
        # Direct steering override (keyboard teleop bypasses swerve chain)
        self.declare_parameter("direct_steer_topic", "/teleop/direct_steer")
        self.declare_parameter("direct_timeout_sec", 0.5)

        use_single_bus = bool(self.get_parameter("use_single_bus").value)
        serial_port = str(self.get_parameter("serial_port").value)
        serial_ports = [str(v) for v in self.get_parameter("serial_ports").value]
        baud_rate = int(self.get_parameter("baud_rate").value)
        protocol_end = int(self.get_parameter("protocol_end").value)
        servo_ids = [int(v) for v in self.get_parameter("servo_ids").value]
        self._center_counts = [int(v) for v in self.get_parameter("center_counts").value]
        self._counts_plus_90 = [int(v) for v in self.get_parameter("counts_plus_90").value]
        self._counts_minus_90 = [int(v) for v in self.get_parameter("counts_minus_90").value]
        self.move_time_ms = int(self.get_parameter("move_time_ms").value)
        self.move_acc = int(self.get_parameter("move_acc").value)
        self.control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.command_timeout_sec = float(self.get_parameter("command_timeout_sec").value)
        self.steering_angle_limit_rad = math.radians(
            float(self.get_parameter("steering_angle_limit_deg").value)
        )
        self.enable_front_extended_steering = bool(
            self.get_parameter("enable_front_extended_steering").value
        )
        self.front_extended_steering_limit_rad = math.radians(
            float(self.get_parameter("front_extended_steering_limit_deg").value)
        )
        if (
            self.enable_front_extended_steering
            and self.front_extended_steering_limit_rad < self.steering_angle_limit_rad
        ):
            self.get_logger().warning(
                "front_extended_steering_limit_deg < steering_angle_limit_deg -- "
                "results in no extension, front_extended_steering is ignored."
            )
            self.enable_front_extended_steering = False
        self.ping_retries = int(self.get_parameter("ping_retries").value)
        self.require_all_servos = bool(self.get_parameter("require_all_servos").value)
        self.center_on_startup = bool(self.get_parameter("center_on_startup").value)
        self.joint_commands_topic = str(self.get_parameter("joint_commands_topic").value)
        self.steer_states_topic = str(self.get_parameter("steer_states_topic").value)
        direct_topic = str(self.get_parameter("direct_steer_topic").value)
        self._direct_timeout = float(self.get_parameter("direct_timeout_sec").value)
        self._use_single_bus = use_single_bus
        self._shared_bus: Optional[StsServoBus] = None

        for name, values in (
            ("servo_ids", servo_ids),
            ("center_counts", self._center_counts),
            ("counts_plus_90", self._counts_plus_90),
            ("counts_minus_90", self._counts_minus_90),
        ):
            if len(values) != STEER_JOINT_COUNT:
                raise ValueError(f"{name} must contain exactly 4 values")
        if not use_single_bus and len(serial_ports) != STEER_JOINT_COUNT:
            raise ValueError("serial_ports must contain exactly 4 values in multi_usb mode")

        self._resolve_calibration_model()

        self._target_angles = [0.0] * STEER_JOINT_COUNT
        self._last_cmd_time = None
        self._direct_angles: Optional[List[float]] = None
        self._direct_last_time = None
        self._teleop_mode: str = "keyboard"
        self._channels: List[SteerChannel] = []
        self._ready = False

        if use_single_bus:
            self._shared_bus = StsServoBus(serial_port, baud_rate, protocol_end)
            self._shared_bus.open()
            self._scan_online_ids(self._shared_bus, serial_port)
            for index in range(STEER_JOINT_COUNT):
                self._init_channel(index, serial_port, servo_ids[index], self._shared_bus)
            if not self._channels:
                raise RuntimeError("No steering servo available")
            self.get_logger().info(f"Single-bus mode on {serial_port}")
        else:
            for index in range(STEER_JOINT_COUNT):
                bus = StsServoBus(serial_ports[index], baud_rate, protocol_end)
                bus.open()
                self._init_channel(index, serial_ports[index], servo_ids[index], bus)
            self.get_logger().info("4 USB drivers mode")

        self._init_startup_pose()

        self.create_subscription(Float64MultiArray, self.joint_commands_topic, self._on_commands, 10)
        self.create_subscription(Float64MultiArray, direct_topic, self._on_direct_steer, 10)
        self.create_subscription(String, '/teleop/active_mode', lambda m: setattr(self, '_teleop_mode', m.data), 10)
        self._state_pub = self.create_publisher(Float64MultiArray, self.steer_states_topic, 10)
        self.create_timer(1.0 / self.control_rate_hz, self._on_timer)
        self._ready = True

        self.get_logger().info(
            f"Listening on {self.joint_commands_topic} | direct override: {direct_topic}"
        )

    def _ping_servo(self, bus: StsServoBus, servo_id: int) -> bool:
        for attempt in range(max(1, self.ping_retries)):
            if bus.ping(servo_id):
                return True
            time.sleep(0.05)
        return False

    def _init_channel(
        self,
        index: int,
        port: str,
        servo_id: int,
        bus: StsServoBus,
    ) -> bool:
        if not self._ping_servo(bus, servo_id):
            msg = (
                f"{STEER_JOINT_NAMES[index]}: servo id={servo_id} not responding on {port}"
            )
            if self.require_all_servos:
                raise RuntimeError(msg)
            self.get_logger().warning(f"{msg} — skipped")
            return False

        bus.enable_torque(servo_id, True)
        self._channels.append(
            SteerChannel(
                name=STEER_JOINT_NAMES[index],
                port=port,
                servo_id=servo_id,
                joint_index=index,
                bus=bus,
            )
        )
        pos_limit, neg_limit = self._limits_rad_for(index)
        low, high = calibrated_counts_bounds(
            self._counts_at_pos[index], self._counts_at_neg[index]
        )
        self.get_logger().info(
            f"{STEER_JOINT_NAMES[index]} (id={servo_id}): "
            f"center={self._center_counts[index]} "
            f"+{math.degrees(pos_limit):.1f}deg={self._counts_at_pos[index]} "
            f"-{math.degrees(neg_limit):.1f}deg={self._counts_at_neg[index]} "
            f"window=[{low}, {high}]"
        )
        return True

    def _scan_online_ids(self, bus: StsServoBus, port: str) -> List[int]:
        online = []
        for sid in range(0, 21):
            if self._ping_servo(bus, sid):
                online.append(sid)
        if online:
            self.get_logger().info(f"IDs detected on {port}: {online}")
        else:
            self.get_logger().warning(f"No servo responded on {port}")
        return online

    def _optional_int_list(self, name: str) -> Optional[List[int]]:
        """Value of a declared-but-not-necessarily-set integer array parameter."""
        param = self.get_parameter_or(name)
        if param.type_ == Parameter.Type.NOT_SET or param.value is None:
            return None
        return [int(v) for v in param.value]

    def _optional_float(self, name: str) -> Optional[float]:
        param = self.get_parameter_or(name)
        if param.type_ == Parameter.Type.NOT_SET or param.value is None:
            return None
        return float(param.value)

    def _resolve_calibration_model(self) -> None:
        """Build the per-joint conversion table (limits + endpoint counts).

        Two schemas are accepted:
          * per-direction (preferred): counts_outward_limit/counts_inward_limit at
            steering_outward_limit_deg/steering_inward_limit_deg, mapped onto the
            joint-angle signs via steering_outward_sign.
          * symmetric legacy: counts_plus_90/counts_minus_90 at
            +-steering_angle_limit_deg (identical behaviour to before 2026-08-13).
        Everything downstream only ever sees the resolved per-joint table, so the
        conversion itself never has to know about outward/inward.
        """
        counts_outward = self._optional_int_list("counts_outward_limit")
        counts_inward = self._optional_int_list("counts_inward_limit")
        outward_deg = self._optional_float("steering_outward_limit_deg")
        inward_deg = self._optional_float("steering_inward_limit_deg")
        provided = [v is not None for v in (counts_outward, counts_inward, outward_deg, inward_deg)]

        if not any(provided):
            self._asymmetric = False
            self._counts_at_pos = list(self._counts_plus_90)
            self._counts_at_neg = list(self._counts_minus_90)
            self._pos_limit_rad = [
                self._limit_rad_for(index) for index in range(STEER_JOINT_COUNT)
            ]
            self._neg_limit_rad = list(self._pos_limit_rad)
            self.get_logger().warning(
                "No per-direction steering calibration -- using the symmetric legacy model "
                f"(counts_plus_90/counts_minus_90 at +-"
                f"{math.degrees(self.steering_angle_limit_rad):.1f} deg)."
            )
            return

        if not all(provided):
            raise ValueError(
                "Incomplete per-direction steering calibration: counts_outward_limit, "
                "counts_inward_limit, steering_outward_limit_deg and "
                "steering_inward_limit_deg must be set together. Half of the schema would "
                "silently mis-scale the recorded end stops (e.g. a 30 deg command driving "
                "to the 100 deg end position)."
            )

        for name, values in (
            ("counts_outward_limit", counts_outward),
            ("counts_inward_limit", counts_inward),
        ):
            if len(values) != STEER_JOINT_COUNT:
                raise ValueError(f"{name} must contain exactly 4 values")
        if outward_deg <= 0.0 or inward_deg <= 0.0:
            raise ValueError(
                "steering_outward_limit_deg and steering_inward_limit_deg must be > 0 "
                "(both are magnitudes, the direction comes from steering_outward_sign)"
            )

        signs = [int(v) for v in self.get_parameter("steering_outward_sign").value]
        if len(signs) != STEER_JOINT_COUNT:
            raise ValueError("steering_outward_sign must contain exactly 4 values")
        if any(s not in (-1, 1) for s in signs):
            raise ValueError("steering_outward_sign values must be +1 or -1")

        outward_rad = math.radians(outward_deg)
        inward_rad = math.radians(inward_deg)

        self._asymmetric = True
        self._counts_at_pos = [0] * STEER_JOINT_COUNT
        self._counts_at_neg = [0] * STEER_JOINT_COUNT
        self._pos_limit_rad = [0.0] * STEER_JOINT_COUNT
        self._neg_limit_rad = [0.0] * STEER_JOINT_COUNT

        for index in range(STEER_JOINT_COUNT):
            joint_outward_rad = outward_rad
            # Task #22c composed with the per-direction model: the front extension
            # can only ever ENLARGE, and only in the outward direction -- inward is
            # the self-collision-critical side (SR-6), it is never extended.
            if self.enable_front_extended_steering and index in FRONT_JOINT_INDICES:
                joint_outward_rad = max(outward_rad, self.front_extended_steering_limit_rad)

            if signs[index] > 0:
                self._pos_limit_rad[index] = joint_outward_rad
                self._counts_at_pos[index] = counts_outward[index]
                self._neg_limit_rad[index] = inward_rad
                self._counts_at_neg[index] = counts_inward[index]
            else:
                self._pos_limit_rad[index] = inward_rad
                self._counts_at_pos[index] = counts_inward[index]
                self._neg_limit_rad[index] = joint_outward_rad
                self._counts_at_neg[index] = counts_outward[index]

            self._check_joint_calibration(
                index, counts_outward, counts_inward, outward_deg, inward_deg
            )

        if self.enable_front_extended_steering:
            self.get_logger().warning(
                "enable_front_extended_steering is on: with the per-direction model it only "
                "raises the OUTWARD limit of FL/FR "
                f"({math.degrees(self._front_outward_limit_rad()):.1f} deg). It is largely "
                "redundant -- the measured mechanical range is the same on all four wheels, "
                "and without recalibrated FL/FR outward counts the extra range only "
                "under-scales the outward direction."
            )

        self.get_logger().info(
            f"Per-direction steering calibration: outward {outward_deg:.1f} deg / "
            f"inward {inward_deg:.1f} deg, outward_sign={signs} (TO-VERIFY on the machine)"
        )

    def _front_outward_limit_rad(self) -> float:
        index = FRONT_JOINT_INDICES[0]
        return max(self._pos_limit_rad[index], self._neg_limit_rad[index])

    def _check_joint_calibration(
        self,
        index: int,
        counts_outward: List[int],
        counts_inward: List[int],
        outward_deg: float,
        inward_deg: float,
    ) -> None:
        """Plausibility of one wheel's recorded calibration (warnings only)."""
        name = STEER_JOINT_NAMES[index]
        center = self._center_counts[index]
        out_delta = counts_outward[index] - center
        in_delta = counts_inward[index] - center

        tick_dirs = [int(v) for v in self.get_parameter("steering_outward_tick_direction").value]
        if len(tick_dirs) == STEER_JOINT_COUNT and tick_dirs[index] != 0 and out_delta != 0:
            measured = 1 if out_delta > 0 else -1
            if measured != tick_dirs[index]:
                self.get_logger().warning(
                    f"{name}: an outward turn was measured to make the count "
                    f"{'rise' if tick_dirs[index] > 0 else 'fall'}, but the calibrated "
                    f"outward end ({counts_outward[index]}) lies on the other side of "
                    f"center ({center}). Most likely this wheel's row belongs to a "
                    "different servo — check servo_ids and the joint order FL, FR, BL, BR."
                )

        if out_delta * in_delta >= 0:
            self.get_logger().warning(
                f"{name}: outward ({counts_outward[index]}) and inward ({counts_inward[index]}) "
                f"end stop are on the SAME side of center ({center}) -- the calibration is "
                "inconsistent, check the recorded values."
            )
        # 4096 counts = one servo revolution. A |delta| beyond half a turn means the
        # recorded raw position wrapped across the 0/4095 boundary; the linear
        # interpolation below cannot represent that and would drive to a random spot.
        for label, delta in (("outward", out_delta), ("inward", in_delta)):
            if abs(delta) > 2048:
                self.get_logger().error(
                    f"{name}: {label} end stop is {abs(delta)} counts from center -- more than "
                    "half a revolution, the raw position most likely wrapped at 0/4095. "
                    "Re-center the servo horn and recalibrate."
                )
        if abs(out_delta) > 0 and abs(in_delta) > 0:
            out_per_deg = abs(out_delta) / outward_deg
            in_per_deg = abs(in_delta) / inward_deg
            if max(out_per_deg, in_per_deg) > 1.25 * min(out_per_deg, in_per_deg):
                self.get_logger().warning(
                    f"{name}: counts per degree differ between the directions "
                    f"(outward {out_per_deg:.1f}, inward {in_per_deg:.1f}) -- with a rigid 1:1 "
                    "linkage they should match; one of the two endpoints may not correspond "
                    "to the angle it is labelled with."
                )

    def _limit_rad_for(self, joint_index: int) -> float:
        """Symmetric limit for one joint (Task #22c: front may steer further).

        Only used to build the symmetric legacy fallback table; the runtime paths
        go through `_limits_rad_for()`.
        """
        if self.enable_front_extended_steering and joint_index in FRONT_JOINT_INDICES:
            return self.front_extended_steering_limit_rad
        return self.steering_angle_limit_rad

    def _limits_rad_for(self, joint_index: int) -> tuple[float, float]:
        """(positive-side, negative-side) angle limit magnitude for one joint."""
        return self._pos_limit_rad[joint_index], self._neg_limit_rad[joint_index]

    def _clamp_angle(self, angle_rad: float, joint_index: int) -> float:
        pos_limit, neg_limit = self._limits_rad_for(joint_index)
        return max(-neg_limit, min(pos_limit, angle_rad))

    def _angle_to_counts(self, angle_rad: float, joint_index: int) -> int:
        pos_limit, neg_limit = self._limits_rad_for(joint_index)
        return calibrated_angle_to_counts_asym(
            max(-neg_limit, min(pos_limit, angle_rad)),
            self._center_counts[joint_index],
            self._counts_at_pos[joint_index],
            self._counts_at_neg[joint_index],
            pos_limit,
            neg_limit,
        )

    def _counts_to_angle(self, counts: int, joint_index: int) -> float:
        pos_limit, neg_limit = self._limits_rad_for(joint_index)
        return calibrated_counts_to_rad_asym(
            counts,
            self._center_counts[joint_index],
            self._counts_at_pos[joint_index],
            self._counts_at_neg[joint_index],
            pos_limit,
            neg_limit,
        )

    def _init_startup_pose(self) -> None:
        """Decide what the steering does at startup and at every respawn.

        OP-20 Option B / SR-12 / OP-24-S1: a process start is not a command, so
        the default is to adopt whatever pose the servos are in. `real_robot.launch.py`
        runs this node with `respawn=True`, so the old unconditional centring fired
        on every crash recovery as well, with nobody at the keyboard.
        """
        if self.center_on_startup:
            self.get_logger().warning(
                "center_on_startup=true: driving the steering to center_counts. That is "
                "movement nobody commanded, on every start AND every respawn (SR-12, OP-20)."
            )
            self._hold_all_servos()
            return
        self._adopt_measured_position()

    def _read_position_retry(self, channel: SteerChannel) -> Optional[int]:
        """Raw position of one servo, retried like `_ping_servo` does."""
        last_error: Optional[Exception] = None
        for _ in range(max(1, self.ping_retries)):
            try:
                return channel.bus.read_position(channel.servo_id)
            except RuntimeError as exc:
                last_error = exc
                time.sleep(0.05)
        self.get_logger().error(
            f"{channel.name} (id={channel.servo_id}): position read failed at startup "
            f"after {max(1, self.ping_retries)} attempts: {last_error}"
        )
        return None

    def _adopt_measured_position(self) -> None:
        """Seed the targets from the servos' measured position — no position write.

        Read-only on the bus, exactly like `_publish_state_only()`. Torque is
        enabled separately in `_init_channel()`, so skipping the centring write
        does NOT leave the servos limp: they keep holding where they stand.

        A servo whose position cannot be read is NOT centred as a fallback —
        centring it is precisely the unrequested motion this path removes, and a
        safe fallback that moves is not safe (SR-14 (4)). With
        `require_all_servos` the node refuses to start and says so (SR-13); the
        launch file's respawn then retries in 5 s, the same self-healing path that
        already covers a servo that fails to answer a ping.
        """
        if not self._channels:
            return

        adopted = []
        unreadable = []
        for channel in self._channels:
            counts = self._read_position_retry(channel)
            if counts is None:
                unreadable.append(channel)
                continue
            angle = self._counts_to_angle(counts, channel.joint_index)
            # Stored unclamped: this is a measurement, not a command. Everything
            # that writes it to the bus clamps on the way out (_on_timer,
            # _angle_to_counts), so the SR-6 window is not widened by it.
            self._target_angles[channel.joint_index] = angle
            adopted.append(f"{channel.name}={counts}cts/{math.degrees(angle):.2f}deg")
            pos_limit, neg_limit = self._limits_rad_for(channel.joint_index)
            if angle > pos_limit or angle < -neg_limit:
                self.get_logger().warning(
                    f"{channel.name}: measured startup angle {math.degrees(angle):.2f}deg is "
                    f"outside its window [{-math.degrees(neg_limit):.1f}, "
                    f"{math.degrees(pos_limit):.1f}]deg. Adopted as-is (not moved); the first "
                    "command will clamp it back into the window."
                )

        if adopted:
            self.get_logger().info(
                "Startup pose adopted from the servos, nothing commanded: " + ", ".join(adopted)
            )

        if unreadable:
            names = ", ".join(f"{ch.name} (id={ch.servo_id})" for ch in unreadable)
            if self.require_all_servos:
                message = (
                    f"Startup position unreadable on: {names}. Refusing to start — centring "
                    "as a fallback would be exactly the unrequested movement this node no "
                    "longer performs (SR-12). Set center_on_startup=true only if a centring "
                    "move at startup is intended and approved."
                )
                self.get_logger().error(message)
                raise RuntimeError(message)
            self.get_logger().error(
                f"Startup position unreadable on: {names}. Those channels are DROPPED — they "
                "keep torque and hold where they stand, they are never written to, and their "
                "/hw/steer_states entries stay 0.0 (require_all_servos=false)."
            )
            dropped = set(id(ch) for ch in unreadable)
            self._channels = [ch for ch in self._channels if id(ch) not in dropped]

    def _hold_all_servos(self) -> None:
        if not self._channels:
            return

        positions = [self._center_counts[ch.joint_index] for ch in self._channels]
        active_ids = [channel.servo_id for channel in self._channels]
        try:
            if self._use_single_bus and self._shared_bus is not None:
                self._shared_bus.sync_write_positions_timed(
                    active_ids,
                    positions,
                    self.move_time_ms,
                    self.move_acc,
                )
            else:
                for channel, position in zip(self._channels, positions):
                    channel.bus.write_position_timed(
                        channel.servo_id,
                        position,
                        self.move_time_ms,
                        self.move_acc,
                    )
            self.get_logger().info("Servos at calibrated zero position (center)")
        except RuntimeError as exc:
            self.get_logger().error(f"Could not move to calibrated center: {exc}")

    def _on_commands(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < STEER_JOINT_COUNT:
            return
        for index in range(STEER_JOINT_COUNT):
            self._target_angles[index] = self._clamp_angle(float(msg.data[index]), index)
        self._last_cmd_time = self.get_clock().now()

    def _on_direct_steer(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < STEER_JOINT_COUNT:
            return
        self._direct_angles = [
            self._clamp_angle(float(msg.data[i]), i) for i in range(STEER_JOINT_COUNT)
        ]
        self._direct_last_time = self.get_clock().now()

    def _write_angles(self, angles: List[float]) -> None:
        positions = [
            self._angle_to_counts(angles[index], index)
            for index in range(STEER_JOINT_COUNT)
        ]

        active_ids = [channel.servo_id for channel in self._channels]
        active_positions = [positions[ch.joint_index] for ch in self._channels]

        state_msg = Float64MultiArray()
        state_msg.data = [0.0] * STEER_JOINT_COUNT

        try:
            if self._use_single_bus and self._shared_bus is not None:
                self._shared_bus.sync_write_positions_timed(
                    active_ids,
                    active_positions,
                    self.move_time_ms,
                    self.move_acc,
                )
            else:
                for channel in self._channels:
                    channel.bus.write_position_timed(
                        channel.servo_id,
                        positions[channel.joint_index],
                        self.move_time_ms,
                        self.move_acc,
                    )
        except RuntimeError as exc:
            self.get_logger().error(f"Servo write failed: {exc}")
            return

        for channel in self._channels:
            try:
                counts = channel.bus.read_position(channel.servo_id)
                angle = self._counts_to_angle(counts, channel.joint_index)
            except RuntimeError:
                angle = angles[channel.joint_index]
            state_msg.data[channel.joint_index] = angle

        self._state_pub.publish(state_msg)

    def _publish_state_only(self) -> None:
        """Read back current servo positions without commanding a move.

        Before the first /hw/joint_commands is received, _on_timer would
        otherwise just `return` and never publish state_msg. swerve_cmd_node,
        however, waits exactly for /hw/steer_states before it computes its
        first command at all -- without this path both sides block each
        other (chicken-and-egg) as soon as no command is present.
        Pure read access to the bus, no position command.
        """
        state_msg = Float64MultiArray()
        state_msg.data = [0.0] * STEER_JOINT_COUNT
        for channel in self._channels:
            try:
                counts = channel.bus.read_position(channel.servo_id)
                angle = self._counts_to_angle(counts, channel.joint_index)
            except RuntimeError:
                angle = self._target_angles[channel.joint_index]
            state_msg.data[channel.joint_index] = angle
        self._state_pub.publish(state_msg)

    def _on_timer(self) -> None:
        if not self._ready:
            return

        # Direct steer override: only in keyboard/controller mode (not in autonomous/nav2)
        if self._teleop_mode != 'autonomous' and self._direct_last_time is not None:
            age = (self.get_clock().now() - self._direct_last_time).nanoseconds * 1e-9
            if age < self._direct_timeout:
                self._write_angles(list(self._direct_angles))
                return

        # Normal path: /hw/joint_commands from GripperXInterface
        #
        # OP-24 / S1, stage 4 of four: on a stale /hw/joint_commands the steering HOLDS its
        # last commanded angle. This used to zero-fill (angles = [0.0] * 4), which is not a
        # stop but a CENTRE command with the servos' full torque behind it — so every loss
        # of the command chain became a motion event, in exactly the situation where nobody
        # is in control (SR-12 (1)). Holding is free: the servos are already at that angle.
        # There is deliberately nothing to do here now; _target_angles already holds the
        # last commanded value and the clamp above is unchanged. The stale case is kept as
        # an explicit branch rather than deleted so the decision stays visible at the point
        # it applies. Only the wheels go to zero, and that happens upstream in
        # GripperXInterface::publish_stop_commands() — this node never drives wheels.
        # Operator-requested centring is a COMMAND, not a timeout, and is untouched: it
        # arrives on /teleop/direct_steer (spacebar E-stop) and is handled above.
        angles = [self._clamp_angle(a, i) for i, a in enumerate(self._target_angles)]
        if self._last_cmd_time is not None:
            age = (self.get_clock().now() - self._last_cmd_time).nanoseconds * 1e-9
            if age > self.command_timeout_sec:
                # `angles` already carries the last commanded angles — holding is the
                # absence of an action here. Logged (throttled) rather than silent: a
                # command chain that has stopped delivering must announce itself, it must
                # not merely stop producing motion (SR-13 item 2).
                self.get_logger().warn(
                    f"/hw/joint_commands stale ({age:.2f}s > {self.command_timeout_sec:.2f}s) "
                    "— steering HOLDS its last commanded angle (OP-24/S1).",
                    throttle_duration_sec=5.0,
                )
        else:
            self._publish_state_only()
            return

        self._write_angles(angles)

    def destroy_node(self) -> bool:
        try:
            seen_buses = set()
            for channel in self._channels:
                channel.bus.enable_torque(channel.servo_id, False)
                bus_id = id(channel.bus)
                if bus_id not in seen_buses:
                    channel.bus.close()
                    seen_buses.add(bus_id)
        except Exception:
            pass
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = SteerServoNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
