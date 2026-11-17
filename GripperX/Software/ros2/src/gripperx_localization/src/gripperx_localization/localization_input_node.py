import bisect
from collections import deque
from copy import deepcopy
import json
import math
import statistics
import time
from typing import Dict, Iterable, Sequence

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from gripperx_control.swerve_kinematic_model import FourWIS4WIDKinematicModel


def covariance_is_unset(values: Iterable[float]) -> bool:
    return all(abs(value) < 1e-12 for value in values)


def yaw_to_quaternion(yaw: float) -> Quaternion:
    half_yaw = yaw * 0.5
    quat = Quaternion()
    quat.x = 0.0
    quat.y = 0.0
    quat.z = math.sin(half_yaw)
    quat.w = math.cos(half_yaw)
    return quat


GYRO_BIAS_STATE_TOPIC = "/localization/gyro_bias_state"


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class GyroBiasEstimator:
    """Yaw-gyro bias from standstill samples, accumulated across standstills (2.3).

    Pure bookkeeping on explicit times, no ROS, so it is unit-testable on synthetic
    streams. The node feeds it three things: motion evidence (mark_motion), stillness
    evidence from /joint_states (mark_still), and raw gyro samples (add_sample).

    A sample enters the estimate only when it lies at least settle_sec AFTER the last
    motion and at least settle_sec BEFORE the next one. The second half is enforced by
    holding samples back until stillness has been observed up to settle_sec past them:
    motion is only seen once a wheel or the steering has moved, i.e. after the body
    may already be turning, and a sample taken then must not reach the estimate.

    The estimate is the mean of the most recent window_sec of accepted samples
    (accumulated over as many standstills as needed) with the trim_count largest and
    trim_count smallest samples left out, so a few corrupted samples inside the
    plausibility limits cannot shift it. Not a median: on the quantised BNO085 rate
    the median (and any trim of 25 %) is exactly 0 in every recorded standstill. A
    symmetric trim of a few samples keeps the bias information, which sits in the
    count asymmetry of the +-1 step samples, as long as each tail holds more than
    trim_count of them. It is published only once
    min_sec of standstill has been collected; before that bias() returns None and the
    caller passes the gyro through uncorrected. No deadband and no rounding anywhere:
    the BNO085 rate is quantised to 1/512 rad/s (0.112 deg/s) while the bias measured
    on the desk is ~1/150 of one step, so the bias exists only as an average over many
    samples that mostly read exactly 0.
    """

    def __init__(self, settle_sec: float, window_sec: float, min_sec: float,
                 max_sample_dt: float, trim_count: int = 0, band: float = 0.0,
                 cold_band: float = 0.0) -> None:
        if not (settle_sec > 0.0 and window_sec > 0.0 and 0.0 < min_sec <= window_sec
                and max_sample_dt > 0.0 and trim_count >= 0):
            raise ValueError("gyro bias: need settle > 0, 0 < min <= window, "
                             "max_sample_dt > 0, trim_count >= 0")
        self.trim_count = int(trim_count)
        # Bias protection (0 = off). A standstill sample enters only if it lies within
        # band of the current estimate, or within cold_band of zero while no estimate
        # exists yet. A gyro stuck at standstill then cannot drag the estimate.
        self.band = float(band)
        self.cold_band = float(cold_band)
        self.rejected_band = 0
        self.rejected_frozen = 0
        # Standstill time (s) over which standstill samples were rejected without an
        # accepted one in between; cleared by the next accepted sample. Only the
        # spacing between consecutive REJECTED samples counts (capped at
        # max_sample_dt), so driving time adds nothing, and a lock-out after a real
        # bias drift keeps growing across short standstills (restart is the only reset).
        self.reject_run_sec = 0.0
        self.reject_last_t = None
        # Window rates kept sorted as well: insertion/removal is a memmove of at most
        # ~6000 floats (120 s at 50 Hz), no per-sample sort.
        self.sorted_rates = []
        self.settle_sec = settle_sec
        self.window_sec = window_sec
        self.min_sec = min_sec
        self.max_sample_dt = max_sample_dt
        self.last_motion = None     # time of the latest motion evidence
        self.still_until = None     # stillness observed (and no motion) up to this time
        self.command_active = False
        self.previous_sample_time = None
        self.pending = deque()      # (time, rate, weight), not yet confirmed
        self.window = deque()       # (rate, weight), confirmed standstill samples
        self.window_sum = 0.0
        self.window_time = 0.0

    def set_command(self, active: bool, now: float | None) -> None:
        """A non-zero motion command is active from now until a zero command arrives.

        now is on the sample time base; None (no sample seen yet) only records the
        state, which mark_still and add_sample then treat as motion.
        """
        self.command_active = active
        if active and now is not None:
            self.mark_motion(now)

    def reset(self) -> None:
        """Back to the state after construction; only the command state is kept.

        For a time base that went backwards: every stored time is then in the
        future, and max() bookkeeping would freeze the estimator for good.
        """
        command_active = self.command_active
        self.__init__(self.settle_sec, self.window_sec, self.min_sec, self.max_sample_dt,
                      self.trim_count, self.band, self.cold_band)
        self.command_active = command_active

    def mark_motion(self, t: float, since: float | None = None) -> None:
        """Motion at t, or possibly anywhere in [since, t] when nothing was observed."""
        self.last_motion = t if self.last_motion is None else max(self.last_motion, t)
        # Samples that fall within settle_sec before this motion are not standstill.
        start = t if since is None else min(since, t)
        while self.pending and self.pending[-1][0] >= start - self.settle_sec:
            self.pending.pop()

    def mark_still(self, t: float) -> None:
        if self.last_motion is None:
            # Nothing is known about the time before the first observation: treat it
            # as the end of a motion, so the first standstill also waits settle_sec.
            self.last_motion = t
        if self.command_active:
            self.mark_motion(t)
            return
        self.still_until = t if self.still_until is None else max(self.still_until, t)
        self._commit()

    def discard_pending_since(self, t: float) -> int:
        """Drop not yet confirmed samples taken at or after t (a frozen run's start)."""
        n = 0
        while self.pending and self.pending[-1][0] >= t:
            self.pending.pop()
            n += 1
        self.rejected_frozen += n
        return n

    def add_sample(self, t: float, rate: float, frozen: bool = False) -> None:
        """frozen: the sample is a repeat of a stuck sensor; it keeps the time
        bookkeeping going but never becomes a standstill sample."""
        weight = 0.0
        if self.previous_sample_time is not None:
            weight = min(max(t - self.previous_sample_time, 0.0), self.max_sample_dt)
        self.previous_sample_time = t
        if self.command_active:
            self.mark_motion(t)
        if self.last_motion is not None and t - self.last_motion >= self.settle_sec:
            if frozen:
                self.rejected_frozen += 1
                self._reject_run(t)
            elif self._outside_band(rate):
                self.rejected_band += 1
                self._reject_run(t)
            else:
                self.reject_run_sec = 0.0
                self.reject_last_t = None
                self.pending.append((t, rate, weight))
        self._commit()
        # Still unconfirmed 2 * settle_sec after it was taken: the stillness evidence
        # (/joint_states) is lagging or gone, so the sample cannot be vouched for.
        while self.pending and self.pending[0][0] < t - 2.0 * self.settle_sec:
            self.pending.popleft()

    def _reject_run(self, t: float) -> None:
        if self.reject_last_t is not None:
            self.reject_run_sec += min(max(t - self.reject_last_t, 0.0), self.max_sample_dt)
        self.reject_last_t = t

    def _outside_band(self, rate: float) -> bool:
        estimate = self.bias()
        if estimate is not None:
            return self.band > 0.0 and abs(rate - estimate) > self.band
        return self.cold_band > 0.0 and abs(rate) > self.cold_band

    def _commit(self) -> None:
        if self.still_until is None:
            return
        while self.pending and self.pending[0][0] + self.settle_sec <= self.still_until:
            _, rate, weight = self.pending.popleft()
            self.window.append((rate, weight))
            bisect.insort(self.sorted_rates, rate)
            self.window_sum += rate
            self.window_time += weight
            while self.window and self.window_time - self.window[0][1] >= self.window_sec:
                old_rate, old_weight = self.window.popleft()
                del self.sorted_rates[bisect.bisect_left(self.sorted_rates, old_rate)]
                self.window_sum -= old_rate
                self.window_time -= old_weight

    def collected_sec(self) -> float:
        return self.window_time

    def bias(self) -> float | None:
        k = self.trim_count
        if not self.window or self.window_time < self.min_sec or len(self.window) <= 2 * k:
            return None
        if k == 0:
            return self.window_sum / len(self.window)
        trimmed = (self.window_sum - math.fsum(self.sorted_rates[:k])
                   - math.fsum(self.sorted_rates[-k:]))
        return trimmed / (len(self.window) - 2 * k)


class LocalizationInputNode(Node):
    def __init__(self) -> None:
        super().__init__("localization_input_node")

        self.declare_parameter("imu_input_topic", "/imu/bno085/data")
        self.declare_parameter("imu_output_topic", "/imu/data/filtered")
        self.declare_parameter("imu_frame_id", "imu_link")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("wheel_odom_topic", "/wheel/odom")
        self.declare_parameter(
            "drive_joint_names",
            ["f_leftwheel", "b_leftwheel", "b_rightwheel", "f_rightwheel"],
        )
        self.declare_parameter(
            "steer_joint_names",
            ["f_left_steer", "b_leftsteer", "b_rightsteer", "f_right_steer"],
        )
        # Defaults mirror config/localization.yaml, which is the source of truth
        # and carries the full rationale (CAD origin, and why these are wheel
        # CONTACT POINTS rather than king-pin positions) — kept in step so that
        # running this node without a params file does not silently fall back to a
        # different robot.
        self.declare_parameter(
            "wheel_positions",
            Parameter.Type.DOUBLE_ARRAY,
        )
        self.declare_parameter("effective_wheel_radius", Parameter.Type.DOUBLE)
        self.declare_parameter("drive_joint_multipliers", [1.0, 1.0, 1.0, 1.0])
        # WHICH QUANTITY THE WHEEL RATE COMES FROM. Default is "position", and the
        # reason is a failure mode, not a preference:
        #
        # /joint_states velocity for a wheel is index 4-7 of the firmware's
        # /hw/joint_states, and gripperx_hardware_interface/INTERFACE.md states that
        # index is EITHER a real encoder measurement OR the commanded velocity handed
        # straight back -- MotorController::getRPM() falls back to the target when the
        # encoder is not running -- and nothing in the value itself distinguishes the
        # two. The only discriminator is the provenance array (indices 12-15,
        # /hw/wheel_feedback_valid), which THIS NODE DOES NOT READ.
        #
        # So on a wheel whose encoder is unplugged, the velocity path makes this node
        # integrate the command as though it were motion: odometry then reports
        # confident, plausible numbers with no relation to the ground, and nothing
        # upstream objects. The accumulated wheel POSITION cannot echo a command: with
        # no encoder the firmware publishes 8 values instead of 12, read() leaves the
        # position state untouched, and the derived rate is 0 -- odometry goes flat and
        # STAYS flat, which is wrong in the way that gets noticed. Fail loudly rather
        # than quietly.
        #
        # The cost is real: a position difference here is noisier than the firmware's
        # smoothed velocity (quantisation ~0.06 rad/s at 30 Hz vs. 2*pi/3200 per encoder
        # tick). This trades noise for honesty. If the noise proves to matter, the right
        # fix is to subscribe to the provenance topic and gate the velocity path on it,
        # NOT to go back to trusting an unverifiable quantity.
        #
        #   position          derive the rate from the accumulated wheel position (default)
        #   velocity          use the reported velocity, unconditionally
        #   velocity_if_valid use the reported velocity when it is present and finite,
        #                     else fall back to position -- the previous default behaviour
        self.declare_parameter("wheel_rate_source", "position")
        # Stale-read compensation for the "position" source; the rationale is in
        # config/localization.yaml next to these keys.
        self.declare_parameter("stale_sample_check", True)
        self.declare_parameter("stale_sample_min_wheel_rate", 0.3)
        self.declare_parameter("stale_sample_step_tolerance", 0.35)
        # Wheel-position reset handling for the "position" source; rationale in
        # config/localization.yaml next to these keys.
        self.declare_parameter("reset_detection", True)
        self.declare_parameter("reset_gap_sec", 2.0)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("publish_tf", False)
        # Gyro bias at standstill (2.3); rationale and decision record in
        # config/localization.yaml next to these keys.
        self.declare_parameter("gyro_bias_estimation", True)
        self.declare_parameter("gyro_bias_settle_sec", 1.5)
        self.declare_parameter("gyro_bias_window_sec", 120.0)
        self.declare_parameter("gyro_bias_min_sec", 30.0)
        self.declare_parameter("gyro_bias_max_sample_dt", 0.1)
        self.declare_parameter("gyro_bias_max_wheel_rate", 0.3)
        self.declare_parameter("gyro_bias_max_steer_rate", 0.1)
        self.declare_parameter("gyro_bias_cmd_topic", "/cmd_vel")
        self.declare_parameter("gyro_bias_trim_count", 0)
        self.declare_parameter("gyro_bias_band", 0.0)
        self.declare_parameter("gyro_bias_cold_band", 0.0)
        self.declare_parameter("gyro_bias_frozen_samples", 0)
        self.declare_parameter("gyro_bias_diag_reject_sec", 10.0)
        self.declare_parameter("gyro_bias_diag_no_estimate_sec", 180.0)
        self.declare_parameter("gyro_bias_diag_period_sec", 1.0)
        self.declare_parameter("gyro_bias_diag_imu_silent_sec", 2.0)
        # Measurement variances handed to the EKF (2.4); rationale and evidence in
        # config/localization.yaml next to these keys.
        self.declare_parameter("imu_yaw_rate_variance", 0.0)
        self.declare_parameter("wheel_odom_yaw_rate_variance", 0.08)
        # Gyro plausibility check (2.4); rationale and evidence in
        # config/localization.yaml next to these keys. 0 disables a limit.
        self.declare_parameter("imu_max_abs_yaw_rate", 0.0)
        self.declare_parameter("imu_max_yaw_rate_step", 0.0)
        self.declare_parameter("imu_max_yaw_rate_slope", 0.0)

        self.imu_input_topic = str(self.get_parameter("imu_input_topic").value)
        self.imu_output_topic = str(self.get_parameter("imu_output_topic").value)
        self.imu_frame_id = str(self.get_parameter("imu_frame_id").value)
        self.joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        self.wheel_odom_topic = str(self.get_parameter("wheel_odom_topic").value)
        self.drive_joint_names = list(self.get_parameter("drive_joint_names").value)
        self.steer_joint_names = list(self.get_parameter("steer_joint_names").value)
        self.wheel_positions = [float(value) for value in self.get_parameter("wheel_positions").value]
        self.effective_wheel_radius = float(self.get_parameter("effective_wheel_radius").value)
        self.wheel_rate_source = str(self.get_parameter("wheel_rate_source").value)
        if self.wheel_rate_source not in ("position", "velocity", "velocity_if_valid"):
            raise ValueError(
                "wheel_rate_source must be one of position | velocity | velocity_if_valid, "
                f"got {self.wheel_rate_source!r}"
            )
        self.drive_joint_multipliers = [
            float(value) for value in self.get_parameter("drive_joint_multipliers").value
        ]
        self.stale_sample_check = bool(self.get_parameter("stale_sample_check").value)
        self.stale_sample_min_wheel_rate = float(
            self.get_parameter("stale_sample_min_wheel_rate").value
        )
        self.stale_sample_step_tolerance = float(
            self.get_parameter("stale_sample_step_tolerance").value
        )
        self.reset_detection = bool(self.get_parameter("reset_detection").value)
        self.reset_gap_sec = float(self.get_parameter("reset_gap_sec").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.gyro_bias_max_wheel_rate = float(self.get_parameter("gyro_bias_max_wheel_rate").value)
        self.gyro_bias_max_steer_rate = float(self.get_parameter("gyro_bias_max_steer_rate").value)
        self.gyro_bias_cmd_topic = str(self.get_parameter("gyro_bias_cmd_topic").value)
        self.imu_yaw_rate_variance = float(self.get_parameter("imu_yaw_rate_variance").value)
        self.wheel_odom_yaw_rate_variance = float(
            self.get_parameter("wheel_odom_yaw_rate_variance").value
        )
        if not (math.isfinite(self.imu_yaw_rate_variance) and self.imu_yaw_rate_variance >= 0.0):
            raise ValueError("imu_yaw_rate_variance must be >= 0 (0 = keep the driver's value)")
        if not (math.isfinite(self.wheel_odom_yaw_rate_variance)
                and self.wheel_odom_yaw_rate_variance > 0.0):
            raise ValueError("wheel_odom_yaw_rate_variance must be > 0")
        self.imu_max_abs_yaw_rate = float(self.get_parameter("imu_max_abs_yaw_rate").value)
        self.imu_max_yaw_rate_step = float(self.get_parameter("imu_max_yaw_rate_step").value)
        self.imu_max_yaw_rate_slope = float(self.get_parameter("imu_max_yaw_rate_slope").value)
        for name in ("imu_max_abs_yaw_rate", "imu_max_yaw_rate_step", "imu_max_yaw_rate_slope"):
            value = getattr(self, name)
            if not (math.isfinite(value) and value >= 0.0):
                raise ValueError(f"{name} must be >= 0 (0 = check off)")
        # Output ordering of imu_output_topic: a step back shorter than this drops the
        # sample, a longer one starts a new time base. gyro_bias_settle_sec DOUBLES as
        # this backward-clock-step threshold (as it already does inside the bias
        # estimator since 2.3, see localization.yaml) and is read even with the
        # estimator off, so the output ordering and the estimator's own reset can
        # never disagree about what counts as a new time base.
        self.imu_backward_drop_sec = float(self.get_parameter("gyro_bias_settle_sec").value)
        self.imu_last_published_time = None
        # Last sample that passed the plausibility check: (time, raw yaw rate).
        self.imu_plausible_reference = None
        self.imu_implausible_count = 0
        self.gyro_bias = None
        if bool(self.get_parameter("gyro_bias_estimation").value):
            self.gyro_bias = GyroBiasEstimator(
                settle_sec=float(self.get_parameter("gyro_bias_settle_sec").value),
                window_sec=float(self.get_parameter("gyro_bias_window_sec").value),
                min_sec=float(self.get_parameter("gyro_bias_min_sec").value),
                max_sample_dt=float(self.get_parameter("gyro_bias_max_sample_dt").value),
                trim_count=int(self.get_parameter("gyro_bias_trim_count").value),
                band=float(self.get_parameter("gyro_bias_band").value),
                cold_band=float(self.get_parameter("gyro_bias_cold_band").value),
            )
        self.gyro_bias_frozen_limit = int(self.get_parameter("gyro_bias_frozen_samples").value)
        self.imu_frozen_signature = None
        self.imu_frozen_run = 0          # consecutive bit-identical samples (6 axes)
        self.imu_frozen_start = None
        self.imu_frozen_total = 0        # samples kept out of the estimator as frozen
        self.gyro_bias_announced = False
        self.gyro_bias_diag_reject_sec = float(
            self.get_parameter("gyro_bias_diag_reject_sec").value)
        self.gyro_bias_diag_no_estimate_sec = float(
            self.get_parameter("gyro_bias_diag_no_estimate_sec").value)
        self.gyro_bias_diag_imu_silent_sec = float(
            self.get_parameter("gyro_bias_diag_imu_silent_sec").value)
        self.gyro_bias_last_imu_wall = self._wall_now()
        self.gyro_bias_first_time = None   # first IMU sample on the current time base
        self.standstill_previous = None   # (time, drive positions, steering angles)
        self.gyro_bias_previous_imu_time = None
        # Latest /joint_states or IMU header time: the time base /cmd_vel is stamped on.
        self.gyro_bias_sample_time = None

        self._validate_parameters()
        self.geometry_a, self.geometry_b = self._compute_paper_geometry(self.wheel_positions)
        self.kinematic_model = FourWIS4WIDKinematicModel(
            a=self.geometry_a,
            b=self.geometry_b,
            wheel_radius=self.effective_wheel_radius,
        )

        self.previous_drive_positions: Dict[str, float] = {}
        self.previous_stamp = None
        self.held_stale_reads = 0
        self.feedback_step_debt = 0
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_yaw = 0.0
        self.missing_joint_warning_sent = False

        self.imu_publisher = self.create_publisher(Imu, self.imu_output_topic, 20)
        self.wheel_odom_publisher = self.create_publisher(Odometry, self.wheel_odom_topic, 20)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        # Guard state for operators/tools, latched like /localization/yaw_alarm.
        self.gyro_bias_state_publisher = None
        if self.gyro_bias is not None:
            self.gyro_bias_state_publisher = self.create_publisher(
                String, GYRO_BIAS_STATE_TOPIC,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                           reliability=ReliabilityPolicy.RELIABLE))
            period = float(self.get_parameter("gyro_bias_diag_period_sec").value)
            # Wall clock: the state must keep coming when use_sim_time is on but /clock
            # is stalled or absent, which is when it matters.
            self.create_timer(period, self.publish_gyro_bias_state,
                              clock=Clock(clock_type=ClockType.SYSTEM_TIME))

        self.imu_subscription = self.create_subscription(
            Imu,
            self.imu_input_topic,
            self.imu_callback,
            50,
        )
        self.joint_state_subscription = self.create_subscription(
            JointState,
            self.joint_state_topic,
            self.joint_state_callback,
            50,
        )
        if self.gyro_bias is not None:
            self.cmd_subscription = self.create_subscription(
                Twist, self.gyro_bias_cmd_topic, self.cmd_callback, 10
            )

        self.get_logger().info(
            (
                "Localization input node ready. IMU: %s -> %s, wheel odom: %s from %s. "
                "Paper geometry a=%.6f m, b=%.6f m, wheel_radius=%.6f m"
            )
            % (
                self.imu_input_topic,
                self.imu_output_topic,
                self.wheel_odom_topic,
                self.joint_state_topic,
                self.geometry_a,
                self.geometry_b,
                self.effective_wheel_radius,
            )
        )

    def _validate_parameters(self) -> None:
        if len(self.drive_joint_names) != 4 or len(self.steer_joint_names) != 4:
            raise ValueError("Expected exactly 4 drive joints and 4 steer joints")
        if len(self.wheel_positions) != 8:
            raise ValueError("wheel_positions must contain 8 values: x1 y1 x2 y2 x3 y3 x4 y4")
        if len(self.drive_joint_multipliers) != 4:
            raise ValueError("drive_joint_multipliers must contain exactly 4 values")
        if self.effective_wheel_radius <= 0.0:
            raise ValueError("effective_wheel_radius must be positive")

    def _compute_paper_geometry(self, wheel_positions: Sequence[float]) -> tuple[float, float]:
        x_values = [wheel_positions[index] for index in range(0, len(wheel_positions), 2)]
        y_values = [wheel_positions[index] for index in range(1, len(wheel_positions), 2)]
        a = sum(abs(value) for value in x_values) / len(x_values)
        b = sum(abs(value) for value in y_values) / len(y_values)
        if a <= 0.0 or b <= 0.0:
            raise ValueError("Paper geometry derived from wheel_positions must be positive")
        return a, b

    @staticmethod
    def _seconds(stamp) -> float:
        return stamp.sec + stamp.nanosec * 1e-9

    def _stamp_or_now(self, stamp):
        if stamp.sec == 0 and stamp.nanosec == 0:
            return self.get_clock().now().to_msg()
        return stamp

    def _gyro_bias_time_went_back(self, stream: str, t: float, previous: float) -> None:
        self.get_logger().warning(
            "Gyro bias: %s time went backwards (%.3f s -> %.3f s; bag replay/loop, "
            "simulation reset or clock step). Estimate reset; the gyro passes through "
            "uncorrected until enough standstill is collected again." % (stream, previous, t)
        )
        self.gyro_bias.reset()
        self.gyro_bias_announced = False
        self.standstill_previous = None
        self.gyro_bias_previous_imu_time = None
        self.gyro_bias_sample_time = None
        self.gyro_bias_first_time = None

    def gyro_bias_state(self) -> dict:
        """Guard state as a dict. level 1 (WARN) = samples rejected continuously for
        gyro_bias_diag_reject_sec (lock-out after real bias drift, or a frozen sensor),
        or no estimate gyro_bias_diag_no_estimate_sec after the first IMU sample.
        Durations are on the sample time base."""
        est = self.gyro_bias
        now = self.gyro_bias_sample_time
        bias = est.bias()
        run = est.reject_run_sec
        since_first = 0.0
        if self.gyro_bias_first_time is not None and now is not None:
            since_first = max(now - self.gyro_bias_first_time, 0.0)
        frozen = self.gyro_bias_frozen_limit > 0 and self.imu_frozen_run > self.gyro_bias_frozen_limit
        # Wall-clock age of the last IMU callback (start of the node before the first).
        age = self._wall_now() - self.gyro_bias_last_imu_wall
        reasons = []
        if 0.0 < self.gyro_bias_diag_imu_silent_sec <= age:
            reasons.append("IMU silent for %.0f s" % age)
        if run >= self.gyro_bias_diag_reject_sec > 0.0:
            reasons.append("standstill samples rejected continuously for %.0f s "
                           "(%d band, %d frozen); a restart is the only reset"
                           % (run, est.rejected_band, est.rejected_frozen))
        if bias is None and since_first >= self.gyro_bias_diag_no_estimate_sec > 0.0:
            reasons.append("no bias estimate %.0f s after the first IMU sample" % since_first)
        if frozen:
            reasons.append("IMU frozen (%d identical samples)" % self.imu_frozen_run)
        return {
            "level": 1 if reasons else 0,
            "message": "; ".join(reasons) if reasons else "ok",
            "estimate_present": bias is not None,
            "bias": bias,
            "collected_sec": est.collected_sec(),
            "rejected_band": est.rejected_band,
            "rejected_frozen": est.rejected_frozen,
            "reject_run_sec": run,
            "since_first_sample_sec": since_first,
            "imu_age_sec": age,
            "frozen": frozen,
            "frozen_total": self.imu_frozen_total,
        }

    @staticmethod
    def _wall_now() -> float:
        return time.monotonic()

    def publish_gyro_bias_state(self) -> None:
        # Plain JSON of the CURRENT state (field names differ from yaw_alarm on purpose:
        # yaw_alarm is a latched alarm, this is a live status that recovers).
        if self.gyro_bias_state_publisher is not None:
            self.gyro_bias_state_publisher.publish(
                String(data=json.dumps(self.gyro_bias_state())))

    def _gyro_bias_drop_backward(self, stream: str, t: float, previous: float) -> bool:
        """Handle a sample older than its stream's previous one; True = drop it.

        A step back shorter than settle_sec (a reordered pair, a small clock step) only
        drops the sample; anything longer resets the estimator, which then takes the
        sample as its first.
        """
        if previous - t < self.gyro_bias.settle_sec:
            self.get_logger().debug(
                "Gyro bias: %s sample %.3f s older than the previous one, dropped"
                % (stream, previous - t),
                throttle_duration_sec=10.0,
            )
            return True
        self._gyro_bias_time_went_back(stream, t, previous)
        return False

    def _gyro_bias_saw_time(self, t: float) -> None:
        if self.gyro_bias_sample_time is None or t > self.gyro_bias_sample_time:
            self.gyro_bias_sample_time = t

    def cmd_callback(self, message: Twist) -> None:
        active = any(
            value != 0.0
            for value in (
                message.linear.x, message.linear.y, message.linear.z,
                message.angular.x, message.angular.y, message.angular.z,
            )
        )
        # Stamped on the sample time base, never the node clock: on a bag replay
        # without use_sim_time the two differ by hours, and a node-clock motion time
        # ahead of every sample froze the estimator for good.
        self.gyro_bias.set_command(active, self.gyro_bias_sample_time)

    def _update_standstill(
        self,
        message: JointState,
        joint_indices: Dict[str, int],
        drive_positions: Dict[str, float],
        steering_angles: Sequence[float],
        stamp,
    ) -> None:
        """Feed the gyro bias estimator one /joint_states observation: moving or still.

        Still = every drive wheel AND every steering joint below its rate threshold.
        Rates come from position differences; a reported velocity counts as well when
        it is larger (it may be a command echo, which is motion intent all the same).
        A gap longer than reset_gap_sec is unknown territory and counts as motion.
        """
        t = self._seconds(stamp)
        previous = self.standstill_previous
        if previous is not None and t < previous[0]:
            if self._gyro_bias_drop_backward("/joint_states", t, previous[0]):
                return
            previous = None
        if previous is not None and t == previous[0]:
            return
        self._gyro_bias_saw_time(t)
        self.standstill_previous = (t, dict(drive_positions), list(steering_angles))
        if previous is None:
            self.gyro_bias.mark_still(t)
            return
        dt = t - previous[0]
        if dt > self.reset_gap_sec:
            # Unobserved stretch: the robot may have moved anywhere inside it.
            self.gyro_bias.mark_motion(t, since=previous[0])
            return

        velocities = message.velocity if len(message.velocity) >= len(message.name) else []

        def reported(name: str) -> float:
            if not velocities:
                return 0.0
            value = float(velocities[joint_indices[name]])
            return abs(value) if math.isfinite(value) else 0.0

        moving = any(
            max(abs(drive_positions[name] - previous[1][name]) / dt, reported(name))
            > self.gyro_bias_max_wheel_rate
            for name in self.drive_joint_names
        ) or any(
            max(abs(normalize_angle(angle - previous_angle)) / dt, reported(name))
            > self.gyro_bias_max_steer_rate
            for name, angle, previous_angle in zip(
                self.steer_joint_names, steering_angles, previous[2]
            )
        )
        if moving:
            self.gyro_bias.mark_motion(t)
        else:
            self.gyro_bias.mark_still(t)

    def _imu_sample_admitted(self, t: float) -> bool:
        """Output ordering of imu_output_topic; False = the sample is not published.

        robot_localization fuses imu0 since 2.4. A sample older than the last one
        published is out of order for the EKF, and one the gyro bias estimator has
        just discarded as reordered must not reach it either. A step back of at least
        imu_backward_drop_sec is a new time base (bag loop, simulation reset, clock
        step): publishing resumes from it, or the IMU would stay silent for good.
        """
        last = self.imu_last_published_time
        if last is None or t >= last:
            return True
        if last - t < self.imu_backward_drop_sec:
            self.get_logger().debug(
                "IMU sample %.3f s older than the last published one, dropped" % (last - t),
                throttle_duration_sec=10.0,
            )
            return False
        if self.gyro_bias is None:
            # With the estimator on, its own reset path logs this step once.
            self.get_logger().warning(
                "IMU time went backwards (%.3f s -> %.3f s); output resumes on the new "
                "time base" % (last, t)
            )
        # Forget the old time base. Needed when the first sample on the new one is
        # then rejected as implausible: the next sample must not see the old
        # watermark and log the same step back a second time.
        self.imu_last_published_time = None
        self.imu_plausible_reference = None
        return True

    def _imu_sample_plausible(self, t: float, rate: float) -> bool:
        """Gyro plausibility; False = the sample reaches neither bias nor EKF.

        Absolute: |rate| <= imu_max_abs_yaw_rate. Step: the change against the last
        PLAUSIBLE sample stays within imu_max_yaw_rate_step, widened by
        imu_max_yaw_rate_slope per second elapsed since that sample. The widening is
        what stops a rejection chain: after a rejected spike during a fast spin
        start, the next real sample is compared with an older reference and has had
        twice the time to change. A rejected sample never becomes the reference.
        """
        reason = None
        if self.imu_max_abs_yaw_rate > 0.0 and abs(rate) > self.imu_max_abs_yaw_rate:
            reason = "|z| %.3f rad/s above %.3f" % (abs(rate), self.imu_max_abs_yaw_rate)
        elif self.imu_max_yaw_rate_step > 0.0 and self.imu_plausible_reference is not None:
            reference_time, reference_rate = self.imu_plausible_reference
            allowed = max(
                self.imu_max_yaw_rate_step,
                self.imu_max_yaw_rate_slope * max(t - reference_time, 0.0),
            )
            if abs(rate - reference_rate) > allowed:
                reason = "step %.3f rad/s in %.3f s, allowed %.3f" % (
                    rate - reference_rate, t - reference_time, allowed)
        if reason is None:
            self.imu_plausible_reference = (t, rate)
            return True
        self.imu_implausible_count += 1
        self.get_logger().warning(
            "Implausible IMU yaw rate dropped (%s); %d dropped since start"
            % (reason, self.imu_implausible_count),
            throttle_duration_sec=5.0,
        )
        return False

    def _imu_frozen(self, message: Imu, t: float) -> bool:
        """True while gyro AND accelerometer repeat bit-identically beyond the limit.

        Gyro z alone is not usable: healthy data holds long identical runs of it.
        All six axes identical for more than gyro_bias_frozen_samples samples is a
        stuck sensor. Reported by a throttled warning; nothing else is acted on.
        """
        w, a = message.angular_velocity, message.linear_acceleration
        signature = (w.x, w.y, w.z, a.x, a.y, a.z)
        if signature == self.imu_frozen_signature:
            self.imu_frozen_run += 1
        else:
            self.imu_frozen_signature = signature
            self.imu_frozen_run = 1
            self.imu_frozen_start = t
        limit = self.gyro_bias_frozen_limit
        if limit <= 0 or self.imu_frozen_run <= limit:
            return False
        if self.imu_frozen_run == limit + 1 and self.gyro_bias is not None:
            self.gyro_bias.discard_pending_since(self.imu_frozen_start)
        self.imu_frozen_total += 1
        self.get_logger().warning(
            "IMU frozen: %d bit-identical samples on all six axes; kept out of the "
            "gyro bias estimate (%d samples since start)"
            % (self.imu_frozen_run, self.imu_frozen_total),
            throttle_duration_sec=5.0,
        )
        return True

    def imu_callback(self, message: Imu) -> None:
        raw_rate = float(message.angular_velocity.z)
        if not math.isfinite(raw_rate):
            self.get_logger().warning(
                "IMU sample with non-finite yaw rate dropped", throttle_duration_sec=10.0
            )
            return
        t = self._seconds(self._stamp_or_now(message.header.stamp))
        self.gyro_bias_last_imu_wall = self._wall_now()
        frozen = self._imu_frozen(message, t)
        if not self._imu_sample_admitted(t):
            return
        if not self._imu_sample_plausible(t, raw_rate):
            return
        processed = deepcopy(message)
        processed.header.frame_id = self.imu_frame_id

        if self.gyro_bias is not None:
            previous = self.gyro_bias_previous_imu_time
            if previous is not None and t < previous and self._gyro_bias_drop_backward(
                "IMU", t, previous
            ):
                return
            self.gyro_bias_previous_imu_time = t
            if self.gyro_bias_first_time is None:
                self.gyro_bias_first_time = t
            self._gyro_bias_saw_time(t)
            rejected_before = self.gyro_bias.rejected_band
            self.gyro_bias.add_sample(t, raw_rate, frozen=frozen)
            if self.gyro_bias.rejected_band != rejected_before:
                self.get_logger().warning(
                    "Gyro bias guard: standstill sample %.5f rad/s outside the band, kept "
                    "out of the estimate; %d rejected since start"
                    % (raw_rate, self.gyro_bias.rejected_band),
                    throttle_duration_sec=10.0,
                )
            bias = self.gyro_bias.bias()
            if bias is not None:
                processed.angular_velocity.z = raw_rate - bias
                if not self.gyro_bias_announced:
                    self.gyro_bias_announced = True
                    self.get_logger().info(
                        "Gyro bias correction active after %.1f s of standstill: "
                        "z bias %.6f rad/s (%.5f deg/s)"
                        % (self.gyro_bias.collected_sec(), bias, math.degrees(bias))
                    )

        if covariance_is_unset(processed.orientation_covariance):
            processed.orientation_covariance = [
                0.02,
                0.0,
                0.0,
                0.0,
                0.02,
                0.0,
                0.0,
                0.0,
                0.04,
            ]

        if covariance_is_unset(processed.angular_velocity_covariance):
            processed.angular_velocity_covariance = [
                0.001,
                0.0,
                0.0,
                0.0,
                0.001,
                0.0,
                0.0,
                0.0,
                0.002,
            ]

        if covariance_is_unset(processed.linear_acceleration_covariance):
            processed.linear_acceleration_covariance = [
                0.04,
                0.0,
                0.0,
                0.0,
                0.04,
                0.0,
                0.0,
                0.0,
                0.06,
            ]

        if self.imu_yaw_rate_variance > 0.0:
            # Only z-z: imu0 fuses the yaw rate alone, so robot_localization reads
            # nothing else of this matrix. The driver's value (bno085.yaml) is the
            # datasheet's and stays untouched for every other consumer.
            covariance = list(processed.angular_velocity_covariance)
            covariance[8] = self.imu_yaw_rate_variance
            processed.angular_velocity_covariance = covariance

        self.imu_last_published_time = t
        self.imu_publisher.publish(processed)

    def joint_state_callback(self, message: JointState) -> None:
        joint_indices = {name: index for index, name in enumerate(message.name)}
        required_joints = self.drive_joint_names + self.steer_joint_names
        missing = [name for name in required_joints if name not in joint_indices]
        if len(missing) == len(required_joints):
            # A message for other joints only (the arm relay publishes the arm on
            # /joint_states too). Not a missing-joint condition; leave the
            # warning state alone, or every such message re-arms the warning.
            return
        if missing:
            if not self.missing_joint_warning_sent:
                self.get_logger().warning(
                    "Waiting for required joints in /joint_states: %s" % ", ".join(missing)
                )
                self.missing_joint_warning_sent = True
            return

        self.missing_joint_warning_sent = False

        drive_positions = {
            name: float(message.position[joint_indices[name]]) for name in self.drive_joint_names
        }
        steering_angles = [float(message.position[joint_indices[name]]) for name in self.steer_joint_names]

        stamp = message.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            stamp = self.get_clock().now().to_msg()

        if self.gyro_bias is not None:
            self._update_standstill(message, joint_indices, drive_positions, steering_angles, stamp)

        if not self.previous_drive_positions:
            self.previous_drive_positions = drive_positions
            self.previous_stamp = stamp
            return

        dt = (
            (stamp.sec - self.previous_stamp.sec)
            + (stamp.nanosec - self.previous_stamp.nanosec) * 1e-9
        )
        if dt <= 0.0:
            return

        if (
            self.wheel_rate_source == "position"
            and self.reset_detection
            and self._crosses_position_reset(drive_positions, dt)
        ):
            # No rate from a step across a reset: re-base the reference and publish
            # nothing, so the integrated pose continues where it was.
            self.previous_drive_positions = drive_positions
            self.previous_stamp = stamp
            self.held_stale_reads = 0
            self.feedback_step_debt = 0
            return

        rate_dt = dt
        if self.wheel_rate_source == "position" and self.stale_sample_check:
            rate_dt = self._feedback_rate_dt(message, joint_indices, drive_positions, dt)
            if rate_dt is None:
                # Stale read held back: the previous sample stays the reference, so
                # the next one spans both reads and no position step is lost.
                return

        wheel_angular_rates = self._extract_wheel_angular_rates(
            message=message,
            joint_indices=joint_indices,
            drive_positions=drive_positions,
            dt=rate_dt,
        )
        linear_x, linear_y, angular_z = self.compute_kinematic_odometry(
            steering_angles=steering_angles,
            wheel_angular_rates=wheel_angular_rates,
        )

        self._integrate_pose(linear_x, linear_y, angular_z, rate_dt)
        self.publish_wheel_odometry(stamp, linear_x, linear_y, angular_z)

        self.previous_drive_positions = drive_positions
        self.previous_stamp = stamp

    def _crosses_position_reset(self, drive_positions: Dict[str, float], dt: float) -> bool:
        """True if the step from the previous sample spans a wheel-position reset.

        All four positions EXACTLY 0.0 is the reset value, not a measurement: the
        firmware clears its encoder counters at boot and zero-fills the frame while
        its motors are not up, and GripperXInterface publishes 0.0 until the first
        feedback after a controller restart. So a step into or out of that state is
        a re-base (out of it: the counter reappearing at its absolute value, or the
        first motion after an ESP32 boot, whose one read of motion is given up).
        A /joint_states gap longer than reset_gap_sec means the feedback chain was
        down (ESP32 reboot, bringup restart), so a step across it is not a rate.
        """
        def all_zero(positions: Dict[str, float]) -> bool:
            return all(positions[name] == 0.0 for name in self.drive_joint_names)

        return (
            dt > self.reset_gap_sec
            or all_zero(drive_positions) != all_zero(self.previous_drive_positions)
        )

    def _feedback_rate_dt(
        self,
        message: JointState,
        joint_indices: Dict[str, int],
        drive_positions: Dict[str, float],
        dt: float,
    ) -> float | None:
        """Time base for a position-difference rate, robust to a stale feedback read.

        /joint_states carries the controller's READ time, not the time the ESP32
        sampled the wheels. Both run at ~30 Hz on unrelated clocks, so when a feedback
        message arrives within jitter of the read, read() sometimes copies the previous
        message again (all four positions bit-identical) and a later read jumps two
        feedback samples. Every repeat is paid back by exactly one double step, so the
        position sum stays exact, but the rate per sample becomes 0 / 2x / 0.5x.

        Returns None to hold back a stale read (at most one in a row — a second one
        means the stream really stopped advancing, which must show as zero). Otherwise
        returns the read period times the number of feedback steps the position
        delta spans. That count is snapped from the reported velocity, and only
        between 1 and (reads spanned + steps still owed by earlier holds): on a stream
        that has never repeated, the result is the read dt unchanged. The velocity
        never becomes the rate — a command echo with a frozen encoder still yields 0.
        """
        velocities = {}
        if len(message.velocity) >= len(message.name):
            for name in self.drive_joint_names:
                value = float(message.velocity[joint_indices[name]])
                if math.isfinite(value) and abs(value) > self.stale_sample_min_wheel_rate:
                    velocities[name] = value

        unchanged = all(
            drive_positions[name] == self.previous_drive_positions[name]
            for name in self.drive_joint_names
        )
        if unchanged and velocities and self.held_stale_reads < 1:
            self.held_stale_reads += 1
            return None

        reads = self.held_stale_reads + 1
        self.held_stale_reads = 0
        if not velocities:
            # Standing or creeping: no step count can be inferred and nothing is owed.
            self.feedback_step_debt = 0
            return dt

        read_period = dt / reads
        ratio = statistics.median(
            (drive_positions[name] - self.previous_drive_positions[name])
            / (velocity * read_period)
            for name, velocity in velocities.items()
        )
        steps = round(ratio)
        if not (
            1 <= steps <= reads + self.feedback_step_debt
            and abs(ratio - steps) <= self.stale_sample_step_tolerance
        ):
            steps = reads
        self.feedback_step_debt = max(0, self.feedback_step_debt + reads - steps)
        return steps * read_period

    def _extract_wheel_angular_rates(
        self,
        message: JointState,
        joint_indices: Dict[str, int],
        drive_positions: Dict[str, float],
        dt: float,
    ) -> list[float]:
        velocity_usable = (
            len(message.velocity) >= len(message.name)
            and all(
                math.isfinite(float(message.velocity[joint_indices[name]]))
                for name in self.drive_joint_names
            )
        )

        if self.wheel_rate_source == "velocity":
            use_reported_velocity = True
        elif self.wheel_rate_source == "velocity_if_valid":
            use_reported_velocity = velocity_usable
        else:  # "position" — see the wheel_rate_source block in __init__
            use_reported_velocity = False

        if use_reported_velocity and not velocity_usable:
            # Asked for velocity, and it is absent or non-finite. Reporting zero would
            # be a lie in the same family this parameter exists to avoid, so say so and
            # take the position instead.
            self.get_logger().error(
                "wheel_rate_source=velocity but /joint_states carries no usable velocity "
                "for the drive joints; falling back to the wheel position this cycle.",
                throttle_duration_sec=5.0,
            )
            use_reported_velocity = False

        wheel_angular_rates = []
        for drive_name, multiplier in zip(self.drive_joint_names, self.drive_joint_multipliers):
            if use_reported_velocity:
                raw_rate = float(message.velocity[joint_indices[drive_name]])
            else:
                raw_rate = (drive_positions[drive_name] - self.previous_drive_positions[drive_name]) / dt
            wheel_angular_rates.append(raw_rate * multiplier)
        return wheel_angular_rates

    def compute_kinematic_odometry(
        self,
        steering_angles: Sequence[float],
        wheel_angular_rates: Sequence[float],
    ) -> tuple[float, float, float]:
        """Compute body-frame odometry twist from wheel states.

        This follows the paper's wheel/body velocity relationship:

            P [vx vy omega]^T = X [v1 v2 v3 v4]^T

        using the forward kinematic mapping implemented in the
        FourWIS4WIDKinematicModel.
        """

        body_twist = self.kinematic_model.forward_kinematics_body_from_wheel_rates(
            steering_angles=steering_angles,
            wheel_angular_rates=wheel_angular_rates,
        )
        return body_twist.vx, body_twist.vy, body_twist.omega

    def _integrate_pose(self, linear_x: float, linear_y: float, angular_z: float, dt: float) -> None:
        delta_yaw = angular_z * dt
        heading = self.pose_yaw + (0.5 * delta_yaw)
        cos_heading = math.cos(heading)
        sin_heading = math.sin(heading)
        delta_x_body = linear_x * dt
        delta_y_body = linear_y * dt
        self.pose_x += (cos_heading * delta_x_body) - (sin_heading * delta_y_body)
        self.pose_y += (sin_heading * delta_x_body) + (cos_heading * delta_y_body)
        self.pose_yaw = normalize_angle(self.pose_yaw + delta_yaw)

    def publish_wheel_odometry(
        self,
        stamp,
        linear_x: float,
        linear_y: float,
        angular_z: float,
    ) -> None:
        quaternion = yaw_to_quaternion(self.pose_yaw)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.pose_x
        odom.pose.pose.position.y = self.pose_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quaternion
        odom.twist.twist.linear.x = linear_x
        odom.twist.twist.linear.y = linear_y
        odom.twist.twist.angular.z = angular_z

        odom.pose.covariance[0] = 0.02
        odom.pose.covariance[7] = 0.02
        odom.pose.covariance[14] = 1e6
        odom.pose.covariance[21] = 1e6
        odom.pose.covariance[28] = 1e6
        odom.pose.covariance[35] = 0.05

        odom.twist.covariance[0] = 0.2
        odom.twist.covariance[7] = 0.5
        odom.twist.covariance[14] = 1e6
        odom.twist.covariance[21] = 1e6
        odom.twist.covariance[28] = 1e6
        odom.twist.covariance[35] = self.wheel_odom_yaw_rate_variance

        self.wheel_odom_publisher.publish(odom)

        if self.tf_broadcaster is None:
            return

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = self.pose_x
        transform.transform.translation.y = self.pose_y
        transform.transform.translation.z = 0.0
        transform.transform.rotation = quaternion
        self.tf_broadcaster.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocalizationInputNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
