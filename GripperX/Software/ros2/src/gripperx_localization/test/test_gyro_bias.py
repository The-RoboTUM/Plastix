"""Gyro bias at standstill (2.3): localization_input_node on synthetic streams.

The node tests run the REAL LocalizationInputNode with the package's own
config/localization.yaml, so the decided values (D7 settle 1.5 s, D8 window 120 s /
minimum 30 s) and the TO-VERIFY thresholds are the ones under test, not copies of
them. Streams: /joint_states at 30 Hz (wheel and steering positions integrated from
the rates set below), IMU at 50 Hz, /cmd_vel at 20 Hz while a command is set.

What would make each test fail, beyond a code change, is stated on the test.
"""
import math
import os
import random
from types import SimpleNamespace

import pytest

rclpy = pytest.importorskip("rclpy")
node_mod = pytest.importorskip("gripperx_localization.localization_input_node")

from builtin_interfaces.msg import Time  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from sensor_msgs.msg import Imu, JointState  # noqa: E402

CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "config",
                      "localization.yaml")
STEP = 1.0 / 512.0            # BNO085 gyro quantisation, rad/s (yaw report 4.5)
BIAS = 0.002                  # rad/s, ~1 step; exact constant unless noise is asked for
SPIN = 0.5                    # rad/s body yaw rate of the synthetic rotations
JS_PERIOD, IMU_PERIOD, CMD_PERIOD = 1.0 / 30.0, 1.0 / 50.0, 1.0 / 20.0


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    rclpy.init(args=["--ros-args", "--params-file", CONFIG])
    yield
    rclpy.shutdown()


class Rig:
    """Drives one node through time with synthetic wheel, steering, IMU and command."""

    def __init__(self, noise_steps=0.0, quantise=False, seed=1, clock_offset=0.0):
        self.node = node_mod.LocalizationInputNode()
        self.t = 1000.0
        self.out = []                         # (t, raw z, published z)
        self.wheel_rate = [0.0] * 4
        self.steer_rate = [0.0] * 4
        self.wheel_pos = [0.0] * 4
        self.steer = [0.0] * 4
        self.body_rate = 0.0
        self.cmd = None                       # None = no publisher; else Twist.angular.z
        self.noise_steps = noise_steps
        self.quantise = quantise
        self.random = random.Random(seed)
        self.accel_random = random.Random(seed + 1000)
        self.accel_frozen = False             # True: accelerometer repeats bit-identically
        self.gyro_override = None             # not None: the gyro z reads this (stuck)
        self._raw = None
        self.node.imu_publisher = SimpleNamespace(
            publish=lambda m: self.out.append((self.t, self._raw, m.angular_velocity.z)))
        # Node clock (now()) = sample time + clock_offset: a bag replayed without
        # use_sim_time has its header stamps hours away from the node clock.
        self.node.get_clock = lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=lambda: self._stamp(self.t + clock_offset)))
        self.next_js = self.t
        self.next_imu = self.t + 0.004
        self.next_cmd = self.t + 0.011
        self.js_enabled = True
        self.js_offset = 0.0                  # shifts the /joint_states stamps only
        self.deliver_js = self.node.joint_state_callback
        self.deliver_imu = self.node.imu_callback
        self.warnings = []
        self.debugs = []
        real_logger = self.node.get_logger()
        self.node.get_logger = lambda: SimpleNamespace(
            warning=lambda text, **kw: self.warnings.append(text),
            debug=lambda text, **kw: self.debugs.append(text),
            info=real_logger.info, error=real_logger.error)

    def jump(self, seconds):
        """Step every clock by `seconds` (negative = backwards), as a bag loop does."""
        self.t += seconds
        self.next_js += seconds
        self.next_imu += seconds
        self.next_cmd += seconds

    @staticmethod
    def _stamp(t):
        return Time(sec=int(t), nanosec=int(round((t - int(t)) * 1e9)))

    def wheels_for_spin(self, omega):
        r = math.hypot(self.node.geometry_a, self.node.geometry_b)
        return [omega * r / self.node.effective_wheel_radius] * 4

    def run(self, seconds):
        end = self.t + seconds
        while True:
            t_next = min(self.next_js, self.next_imu, self.next_cmd)
            if t_next > end:
                break
            dt = t_next - self.t
            self.wheel_pos = [p + w * dt for p, w in zip(self.wheel_pos, self.wheel_rate)]
            self.steer = [s + w * dt for s, w in zip(self.steer, self.steer_rate)]
            self.t = t_next
            if t_next == self.next_js:
                msg = JointState()
                msg.header.stamp = self._stamp(self.t + self.js_offset)
                msg.name = list(self.node.drive_joint_names) + list(self.node.steer_joint_names)
                msg.position = list(self.wheel_pos) + list(self.steer)
                msg.velocity = list(self.wheel_rate) + [0.0] * 4   # steer velocity: 0 on robot
                if self.js_enabled:
                    self.deliver_js(msg)
                self.next_js += JS_PERIOD
            elif t_next == self.next_imu:
                raw = self.body_rate + BIAS + self.random.gauss(0.0, self.noise_steps * STEP)
                if self.quantise:
                    raw = round(raw / STEP) * STEP
                self._raw = raw
                msg = Imu()
                msg.header.stamp = self._stamp(self.t)
                if self.gyro_override is not None:
                    raw = self.gyro_override
                    self._raw = raw
                msg.angular_velocity.z = raw
                # A live accelerometer is never bit-identical twice in a row; the frozen
                # check needs all six axes, so the synthetic one must move too.
                msg.linear_acceleration.x = (
                    0.0 if self.accel_frozen else self.accel_random.gauss(0.0, 0.01))
                self.deliver_imu(msg)
                self.next_imu += IMU_PERIOD
            else:
                if self.cmd is not None:
                    twist = Twist()
                    twist.angular.z = self.cmd
                    self.node.cmd_callback(twist)
                self.next_cmd += CMD_PERIOD
        self.t = end

    def spin(self, seconds, omega=SPIN):
        """Rotation starting NOW on both the gyro and the wheels, then a stop."""
        self.body_rate, self.wheel_rate = omega, self.wheels_for_spin(omega)
        self.run(seconds)
        self.body_rate, self.wheel_rate = 0.0, [0.0] * 4

    @property
    def est(self):
        return self.node.gyro_bias

    def close(self):
        self.node.destroy_node()


@pytest.fixture
def rig():
    made = []

    def make(**kwargs):
        made.append(Rig(**kwargs))
        return made[-1]
    yield make
    for r in made:
        r.close()


def cfg(rig_):
    e = rig_.est
    return e.settle_sec, e.window_sec, e.min_sec


def test_config_carries_decided_values(rig):
    # Fails if localization.yaml loses or changes D7/D8 (a typo falls back to a node
    # default, which would otherwise pass every test below unnoticed).
    import yaml
    with open(CONFIG) as handle:
        params = yaml.safe_load(handle)["localization_input_node"]["ros__parameters"]
    assert params["gyro_bias_estimation"] is True
    assert params["gyro_bias_settle_sec"] == 1.5        # D7
    assert params["gyro_bias_window_sec"] == 120.0      # D8
    assert params["gyro_bias_min_sec"] == 30.0          # D8
    r = rig()
    assert cfg(r) == (1.5, 120.0, 30.0)


def test_constant_bias_removed_after_min_standstill(rig):
    # Fails on config: a longer min_sec than the 40 s fed here, a disabled
    # estimator, or a wheel/steer threshold below zero-motion noise (none here).
    r = rig()
    settle, _, min_sec = cfg(r)
    r.run(settle + min_sec + 5.0)
    late = [pub for t, raw, pub in r.out if t > r.t - 3.0]
    assert r.est.bias() == pytest.approx(BIAS, abs=1e-12)
    assert max(abs(v) for v in late) < 1e-12


def test_bias_below_one_quantisation_step_is_recovered(rig):
    # The real case: bias ~1/3 step, quantised output that mostly reads one value.
    # A deadband or rounding in the estimator would return 0 or one full step here.
    # Fails on config only if window_sec drops so far that the mean gets noisy.
    global BIAS
    saved, BIAS = BIAS, STEP / 3.0
    try:
        r = rig(noise_steps=0.5, quantise=True)
        settle, window, _ = cfg(r)
        r.run(settle + window + 5.0)
        assert r.est.bias() == pytest.approx(BIAS, abs=0.1 * BIAS)
    finally:
        BIAS = saved


def test_nothing_subtracted_before_min_standstill(rig):
    # Fails on config if min_sec is lowered below the time fed here.
    r = rig()
    settle, _, min_sec = cfg(r)
    r.run(settle + min_sec - 2.0)
    assert r.est.bias() is None
    assert all(pub == raw for _, raw, pub in r.out)
    assert 0.0 < r.est.collected_sec() < min_sec


def test_rotation_right_after_standstill_does_not_leak(rig):
    # Uncommanded rotation (no /cmd_vel at all): the gyro turns from the first sample,
    # the wheels are seen moving only at the next /joint_states read. With an exact
    # constant bias any leaked rotation sample moves the mean by >= SPIN/6000.
    # Fails on config if settle_sec were 0 (no look-back) or the wheel threshold
    # rose above the wheel rate of a 0.5 rad/s spin.
    r = rig()
    settle, _, min_sec = cfg(r)
    r.run(settle + min_sec + 10.0)
    assert r.est.bias() == pytest.approx(BIAS, abs=1e-12)
    r.spin(0.2)                       # short, starts mid-way between two reads
    r.run(settle + 10.0)
    r.spin(5.0)
    r.run(1.0)                        # stop shorter than settle: nothing collected
    r.spin(5.0)
    r.run(settle + 10.0)
    assert r.est.bias() == pytest.approx(BIAS, abs=1e-12)


def test_samples_inside_settle_after_motion_are_not_used(rig):
    # After the wheels stop the body still rocks (gyro transient for 1.4 s, inside
    # settle). Fails on config if settle_sec drops below 1.4 s.
    r = rig()
    settle, _, min_sec = cfg(r)
    r.run(settle + min_sec + 5.0)
    r.spin(3.0)
    r.body_rate = 0.05
    r.run(1.4)
    r.body_rate = 0.0
    r.run(settle + 5.0)
    assert r.est.bias() == pytest.approx(BIAS, abs=1e-12)


def test_stop_with_residual_motion_command_does_not_count(rig):
    # Wheels and steering still, but a small non-zero command keeps streaming
    # (e.g. a twist below the drive's breakaway). Then one that is sent once and
    # never followed by a zero command. Fails on config if gyro_bias_cmd_topic no
    # longer names the topic this test publishes through (cmd_callback).
    r = rig()
    settle, _, min_sec = cfg(r)
    r.cmd = 1e-3
    r.run(settle + min_sec + 10.0)
    assert r.est.collected_sec() == 0.0 and r.est.bias() is None
    r.cmd = None                      # stream stops on a non-zero command
    r.run(settle + min_sec + 10.0)
    assert r.est.collected_sec() == 0.0
    r.cmd = 0.0                       # zero command: standstill can start
    r.run(settle + min_sec + 5.0)
    assert r.est.bias() == pytest.approx(BIAS, abs=1e-12)


def test_steering_motion_with_wheels_still_does_not_count(rig):
    # Steering sweeps at 0.3 rad/s (17 deg/s) back and forth, drive wheels still, no
    # command; the body yaws a little (report 4.5, drive15 segment 11). Fails on
    # config if gyro_bias_max_steer_rate rises above 0.3 rad/s.
    r = rig()
    settle, _, min_sec = cfg(r)
    r.body_rate = -0.0013
    for _ in range(int((settle + min_sec + 10.0) / 2.0)):
        r.steer_rate = [0.3] * 4
        r.run(1.0)
        r.steer_rate = [-0.3] * 4
        r.run(1.0)
    assert r.est.collected_sec() == 0.0 and r.est.bias() is None


def test_bias_accumulates_across_separate_standstills(rig):
    # Four standstills of 12 s, each contributing 12 - 2 * settle s. The minimum is
    # reached only by adding them up. Fails on config if settle_sec, min_sec or the
    # standstill length here stop adding up that way (the asserts check the sums).
    r = rig()
    settle, _, min_sec = cfg(r)
    per_stop = 12.0 - 2.0 * settle
    stops = math.ceil(min_sec / per_stop)
    assert stops >= 2, "config makes a single 12 s standstill enough; test means nothing"
    r.spin(2.0)
    for index in range(stops):
        r.run(12.0)
        r.spin(2.0)
        collected = r.est.collected_sec()
        assert collected == pytest.approx(per_stop * (index + 1), abs=0.05)
        if index < stops - 1:
            assert r.est.bias() is None
    assert r.est.bias() == pytest.approx(BIAS, abs=1e-12)


def test_window_tracks_latest_standstill_time():
    # Estimator level: after window_sec at a new bias the old samples are gone.
    est = node_mod.GyroBiasEstimator(settle_sec=1.5, window_sec=120.0, min_sec=30.0,
                                     max_sample_dt=0.1)
    t = 0.0
    est.mark_still(t)
    for bias in (0.001, 0.003):
        for _ in range(int(130.0 / IMU_PERIOD)):
            t += IMU_PERIOD
            est.mark_still(t)
            est.add_sample(t, bias)
        assert est.bias() == pytest.approx(bias, abs=1e-12)
        assert est.collected_sec() >= 120.0 - IMU_PERIOD


def test_imu_dropout_does_not_count_as_standstill_time():
    est = node_mod.GyroBiasEstimator(settle_sec=1.5, window_sec=120.0, min_sec=30.0,
                                     max_sample_dt=0.1)
    est.mark_still(0.0)
    est.add_sample(2.0, BIAS)
    est.add_sample(40.0, BIAS)        # 38 s gap counts as at most max_sample_dt
    est.mark_still(45.0)
    assert est.collected_sec() <= 0.1 and est.bias() is None


def test_backward_time_jump_restarts_the_estimator(rig):
    # Second bag / --loop replay, twin /reset_simulation, backward chrony step. Before
    # the fix every stored time lay in the future after the jump and the estimate
    # froze at the old value forever. Fails on config only via settle/min (sums).
    global BIAS
    r = rig()
    settle, _, min_sec = cfg(r)
    r.run(settle + min_sec + 5.0)
    assert r.est.bias() == pytest.approx(BIAS, abs=1e-12)
    saved, BIAS = BIAS, -0.004
    try:
        r.jump(-600.0)
        before = len(r.out)
        r.run(1.0)
        assert r.est.bias() is None                        # dropped, not frozen
        assert len(r.warnings) == 1 and "backwards" in r.warnings[0]
        assert all(pub == raw for _, raw, pub in r.out[before:])
        r.run(settle + min_sec + 5.0)
        assert r.est.bias() == pytest.approx(BIAS, abs=1e-12)
        assert len(r.warnings) == 1                        # one jump, one WARN
    finally:
        BIAS = saved


def test_joint_state_gap_counts_as_motion(rig):
    # /joint_states silent for 2.5 s (> reset_gap_sec) while the body is turned with
    # the wheels standing (positions identical either side of the gap). Nothing seen
    # during the gap may enter the estimate. Fails on config if reset_gap_sec rises
    # to 2.5 s or more.
    r = rig()
    settle, _, min_sec = cfg(r)
    assert r.node.reset_gap_sec < 2.5, "config makes a 2.5 s gap ordinary; test means nothing"
    r.run(settle + min_sec + 5.0)
    r.js_enabled = False
    r.body_rate = SPIN
    r.run(2.5)
    r.body_rate = 0.0
    r.js_enabled = True
    r.run(settle + 10.0)
    assert r.est.bias() == pytest.approx(BIAS, abs=1e-12)


def test_joint_state_time_alone_going_back_restarts_the_estimator(rig):
    # Bag replay where /joint_states loops 1 s before the IMU does. The /joint_states
    # reset must act on its own: without it the stored motion and stillness times
    # stay in the future of every new /joint_states time, motion there is swallowed
    # by the max() bookkeeping and the old estimate keeps being applied. Fails on
    # config only via settle/min (sums) or if settle_sec reaches 600 s.
    global BIAS
    r = rig()
    settle, _, min_sec = cfg(r)
    assert settle < 600.0, "config makes a 600 s step back small; test means nothing"
    r.run(settle + min_sec + 5.0)
    assert r.est.bias() == pytest.approx(BIAS, abs=1e-12)
    saved, BIAS = BIAS, -0.004
    try:
        r.js_offset = -600.0
        r.run(1.0)                                         # IMU still on the old time
        assert len(r.warnings) == 1 and "/joint_states" in r.warnings[0]
        assert r.est.bias() is None                        # dropped, not frozen
        r.js_offset = 0.0
        r.jump(-600.0)                                     # now the IMU follows
        r.run(settle + min_sec + 5.0)
        assert len(r.warnings) == 2 and "IMU" in r.warnings[1]
        assert r.est.bias() == pytest.approx(BIAS, abs=1e-12)
    finally:
        BIAS = saved


def test_repeated_joint_state_stamp_is_ignored(rig):
    # Every /joint_states message arrives twice with the same stamp (a republisher
    # or a bag with duplicates). Without the equal-stamp guard the rate test divides
    # by dt = 0 and the callback raises. Fails on config only via settle/min (sums).
    r = rig()
    settle, _, min_sec = cfg(r)

    def twice(msg):
        r.node.joint_state_callback(msg)
        r.node.joint_state_callback(msg)
    r.deliver_js = twice
    r.run(settle + min_sec + 5.0)
    assert r.est.bias() == pytest.approx(BIAS, abs=1e-12)


@pytest.mark.parametrize("hours", [22.7, -22.7])
def test_command_uses_sample_time_not_node_clock(rig, hours):
    # Bag replay without use_sim_time: the node clock is hours away from the header
    # stamps. A command stamped with the node clock put the motion 22.7 h ahead of
    # every sample and the bias never activated. Fails on config only via settle/min
    # (sums) or if gyro_bias_cmd_topic stops naming the topic fed here.
    r = rig(clock_offset=hours * 3600.0)
    settle, _, min_sec = cfg(r)
    r.cmd = 1e-3                                           # residual command, wheels still
    r.run(settle + min_sec + 5.0)
    assert r.est.collected_sec() == 0.0
    r.cmd = SPIN
    r.spin(2.0)
    r.cmd = 0.0
    r.run(settle + min_sec + 5.0)
    assert r.est.bias() == pytest.approx(BIAS, abs=1e-12)


def _swap_every(r, period, callback):
    """Deliver one message pair in reverse order every `period` s of sample time."""
    state = {"held": None, "next": r.t + period}

    def deliver(msg):
        if state["held"] is not None:
            callback(msg)
            callback(state["held"])
            state["held"] = None
        elif r.t >= state["next"]:
            state["held"] = msg
            state["next"] += period
        else:
            callback(msg)
    return deliver


def test_small_backward_step_drops_the_sample_only(rig):
    # One swapped IMU pair (20 ms) and one swapped /joint_states pair (33 ms) every
    # 20 s. Before the fix each swap reset the estimator and it never activated.
    # Fails on config if settle_sec drops to 33 ms or below (the swaps then count
    # as large steps), or via settle/min (sums).
    r = rig()
    settle, _, min_sec = cfg(r)
    assert settle > JS_PERIOD, "config makes a swapped pair a large step; test means nothing"
    r.deliver_imu = _swap_every(r, 20.0, r.node.imu_callback)
    r.deliver_js = _swap_every(r, 20.0, r.node.joint_state_callback)
    r.run(settle + min_sec + 25.0)
    assert r.warnings == []
    assert len(r.debugs) >= 4                              # the drops were seen
    assert r.est.bias() == pytest.approx(BIAS, abs=1e-12)


def test_step_back_beyond_settle_resets_once(rig):
    # A step back of twice settle_sec on every stream is a reset, not a drop: one
    # WARN, estimate gone, re-collected from scratch. Fails on config only via
    # settle/min (sums); the threshold is read from the config, not repeated here.
    global BIAS
    r = rig()
    settle, _, min_sec = cfg(r)
    r.run(settle + min_sec + 5.0)
    assert r.est.bias() == pytest.approx(BIAS, abs=1e-12)
    saved, BIAS = BIAS, -0.004
    try:
        r.jump(-2.0 * settle)
        r.run(0.5)
        assert len(r.warnings) == 1 and "backwards" in r.warnings[0]
        assert r.est.bias() is None
        r.run(settle + min_sec + 5.0)
        assert len(r.warnings) == 1
        assert r.est.bias() == pytest.approx(BIAS, abs=1e-12)
    finally:
        BIAS = saved


def test_command_right_after_a_reset_uses_the_new_time_base(rig):
    # The sample that detects a 600 s step back resets the estimator; a non-zero
    # command follows at once, then a zero one. The command must be stamped on the
    # NEW time base: the latest sample time is cleared by the reset, or the
    # pre-jump time (600 s ahead) survives as "latest" and the command freezes the
    # estimator there. Fails on config only via settle/min (sums) or if settle_sec
    # reaches 600 s.
    global BIAS
    r = rig()
    settle, _, min_sec = cfg(r)
    assert settle < 600.0, "config makes a 600 s step back small; test means nothing"
    r.run(settle + min_sec + 5.0)
    saved, BIAS = BIAS, -0.004
    try:
        r.jump(-600.0)
        r.cmd = 1e-3
        r.run(0.2)
        assert len(r.warnings) == 1
        r.cmd = 0.0
        r.run(settle + min_sec + 5.0)
        assert r.est.bias() == pytest.approx(BIAS, abs=1e-12)
    finally:
        BIAS = saved


def test_command_before_the_first_sample_is_kept(rig):
    # One non-zero command arrives before any /joint_states or IMU sample, so there
    # is no time to stamp it with yet. It must still hold collection off until a
    # zero command arrives. Fails on config only via settle/min (sums).
    r = rig()
    settle, _, min_sec = cfg(r)
    twist = Twist()
    twist.angular.z = 1e-3
    r.node.cmd_callback(twist)
    r.run(settle + min_sec + 5.0)
    assert r.est.collected_sec() == 0.0 and r.est.bias() is None
    r.cmd = 0.0
    r.run(settle + min_sec + 5.0)
    assert r.est.bias() == pytest.approx(BIAS, abs=1e-12)


def test_spike_at_standstill_does_not_move_the_bias(rig):
    # One +5 rad/s sample in the middle of a long standstill (the audit's fault
    # injection; unchecked it shifted the estimate by ~0.0018 rad/s for ~210 s).
    # The estimate must equal the clean bias exactly. Fails if the plausibility
    # check is removed or moved AFTER the bias estimator, or if the config lets
    # 5 rad/s through both limits.
    r = rig()
    settle, _, min_sec = cfg(r)
    r.run(settle + 10.0)
    real = r.node.imu_callback
    state = {"done": False}

    def deliver(msg):
        if not state["done"]:
            msg.angular_velocity.z += 5.0
            state["done"] = True
        real(msg)
    r.deliver_imu = deliver
    r.run(0.1)
    r.deliver_imu = real
    r.run(min_sec + 5.0)
    assert state["done"] and r.node.imu_implausible_count == 1
    assert r.est.bias() == pytest.approx(BIAS, abs=1e-12)


def test_in_limit_spike_at_standstill_does_not_move_the_bias(rig):
    # One +1.0 rad/s sample at standstill: inside the plausibility limits, so it
    # reaches the estimator (the audit's case: +0.00031 rad/s on the plain mean for
    # ~210 s). The trimmed mean must leave the estimate unchanged. Fails with
    # gyro_bias_trim_count 0 in the config or a plain mean in bias().
    r = rig()
    settle, _, min_sec = cfg(r)
    assert r.est.trim_count >= 1, "config disables the trim; test means nothing"
    assert 1.0 + BIAS <= r.node.imu_max_abs_yaw_rate and 1.0 <= r.node.imu_max_yaw_rate_step
    # The bias guard would reject this spike before the trim sees it (see the guard
    # tests below); switched off here to test the trim as the second line of defence.
    r.est.band = r.est.cold_band = 0.0
    r.run(settle + min_sec + 5.0)
    real = r.node.imu_callback
    state = {"done": False}

    def deliver(msg):
        if not state["done"]:
            msg.angular_velocity.z += 1.0
            state["done"] = True
        real(msg)
    r.deliver_imu = deliver
    r.run(0.1)
    r.deliver_imu = real
    r.run(settle + 5.0)
    assert state["done"] and r.node.imu_implausible_count == 0
    assert any(rate > 0.5 for rate, _ in r.est.window), "spike never reached the window"
    assert r.est.bias() == pytest.approx(BIAS, abs=1e-12)


def test_trimmed_bias_keeps_a_sub_step_bias_on_quantised_data():
    # Quantised standstill data like the desk bag: mostly exact 0, a few +-1 step,
    # with 3 more at -1 than at +1. The trim must keep that asymmetry (a median
    # reads exactly 0). Fails if bias() becomes a median or trims more samples
    # than one tail holds.
    est = node_mod.GyroBiasEstimator(settle_sec=1.5, window_sec=120.0, min_sec=30.0,
                                     max_sample_dt=0.1, trim_count=3)
    rates = [0.0] * 5900 + [STEP] * 28 + [-STEP] * 31
    est.still_until = 1e9
    for k, rate in enumerate(rates):
        est.pending.append((k * 0.02, rate, 0.02))
    est._commit()
    n = len(rates)
    assert est.bias() == pytest.approx(-3 * STEP / (n - 6), rel=1e-9)


def test_window_eviction_keeps_the_sorted_copy_in_step():
    # Runs the window well past window_sec with asymmetric, varying tails, so the
    # oldest samples are evicted many times. The sorted copy must always equal the
    # window, and the trimmed bias must equal one computed from scratch. Fails if
    # eviction removes the wrong element from the sorted copy (e.g. the smallest).
    est = node_mod.GyroBiasEstimator(settle_sec=1.5, window_sec=10.0, min_sec=5.0,
                                     max_sample_dt=0.1, trim_count=3)
    rng = random.Random(7)
    est.still_until = 1e9
    k = est.trim_count
    for i in range(3000):                                   # 60 s at 50 Hz, window 10 s
        level = rng.choice([0] * 20 + [1] * 3 + [-1] * 1 + [5, -2])
        est.pending.append((i * 0.02, level * STEP + (0.3 if i % 397 == 0 else 0.0), 0.02))
        est._commit()
        if i % 250 == 249 and i > 500:
            rates = sorted(rate for rate, _ in est.window)
            assert est.sorted_rates == rates
            assert est.bias() == pytest.approx(
                math.fsum(rates[k:-k]) / (len(rates) - 2 * k), abs=1e-12)
    assert len(est.window) < 3000


# ------------------------------------------------------------ bias protection (proposal 4)
# Limits come from the active localization.yaml through the node (no copies). They are
# TO-VERIFY there; these tests prove the mechanism, not that the values are right.

def warm(r):
    settle, _, min_sec = cfg(r)
    r.run(settle + min_sec + 10.0)
    assert r.est.bias() == pytest.approx(BIAS, abs=1e-12)


def test_guard_parameters_are_active(rig):
    r = rig()
    assert r.est.band > 0.0 and r.est.cold_band > 0.0
    assert r.node.gyro_bias_frozen_limit > 0


@pytest.mark.parametrize("stuck", [0.5, 0.019])
def test_stuck_gyro_at_rest_after_warm_bias_leaves_bias_unchanged(rig, stuck):
    # Fails without the band around the estimate: 0.5 rad/s for 20 s would move the
    # trimmed 120 s mean by ~0.08 rad/s (turn_stuck5: 0.014 in the replay).
    r = rig()
    warm(r)
    r.gyro_override = stuck
    r.run(20.0)
    assert r.est.rejected_band > 0
    assert r.est.bias() == pytest.approx(BIAS, abs=1e-9)
    r.gyro_override = None
    r.run(5.0)
    assert r.est.bias() == pytest.approx(BIAS, abs=1e-9)


@pytest.mark.parametrize("stuck", [0.5, 0.019])
def test_cold_start_stuck_gyro_gives_no_poisoned_estimate(rig, stuck):
    # Fails without the absolute band: the estimator would publish ``stuck`` as the
    # bias after min_sec and pass the real zero rate on as -stuck.
    r = rig()
    r.gyro_override = stuck
    settle, _, min_sec = cfg(r)
    r.run(settle + min_sec + 10.0)
    assert r.est.bias() is None
    assert r.est.rejected_band > 0
    r.gyro_override = None
    r.run(min_sec + 10.0)
    assert r.est.bias() == pytest.approx(BIAS, abs=1e-9)


def test_frozen_six_axis_run_is_excluded_and_reported(rig):
    # Gyro AND accelerometer bit-identical beyond the limit, at a value INSIDE the band
    # (the band alone would pass it). Fails without the frozen check.
    r = rig()
    warm(r)
    r.gyro_override = BIAS + 0.004
    r.accel_frozen = True
    r.run(10.0)
    assert r.node.imu_frozen_total > 0 and r.est.rejected_frozen > 0
    assert r.est.rejected_band == 0
    assert any("frozen" in w for w in r.warnings)
    assert r.est.bias() == pytest.approx(BIAS, abs=1e-9)


def test_frozen_run_at_cold_start_inside_band_never_builds_an_estimate(rig):
    r = rig()
    r.gyro_override = 0.004
    r.accel_frozen = True
    settle, _, min_sec = cfg(r)
    r.run(settle + min_sec + 10.0)
    # Only the first ``limit`` samples (and not even those once discarded) can enter.
    assert r.est.bias() is None


def test_healthy_noise_is_accepted_by_the_guard(rig):
    # Fails if the provisional band is narrower than healthy quantised noise.
    global BIAS
    saved, BIAS = BIAS, STEP / 3.0
    try:
        r = rig(noise_steps=0.7, quantise=True)
        settle, window, _ = cfg(r)
        r.run(settle + 60.0)
        assert r.est.rejected_band == 0 and r.est.rejected_frozen == 0
        assert r.est.bias() == pytest.approx(BIAS, abs=0.3 * BIAS)
    finally:
        BIAS = saved


def test_run_of_twenty_identical_samples_is_accepted(rig):
    # Longest healthy run counted offline by the audit in the 2026-10-22 bags: 20
    # bit-identical gyro+accel samples (desk_check; rot13 10, drive15 12; n=1 bag
    # each, one robot, one temperature). Must not be classed as frozen.
    r = rig()
    warm(r)
    n = r.est.rejected_frozen
    real = r.node.imu_callback
    ident = {}

    def deliver(msg):
        # Hold the previous message for 19 more samples = 20 identical in total.
        if "msg" not in ident or ident["left"] == 0:
            ident["msg"], ident["left"] = msg, 19
            return real(msg)
        ident["left"] -= 1
        held = Imu()
        held.header.stamp = msg.header.stamp
        held.angular_velocity.z = ident["msg"].angular_velocity.z
        held.linear_acceleration.x = ident["msg"].linear_acceleration.x
        return real(held)
    r.deliver_imu = deliver
    r.run(10.0)
    assert r.node.imu_frozen_run <= 20
    assert r.node.imu_frozen_total == 0 and r.est.rejected_frozen == n
