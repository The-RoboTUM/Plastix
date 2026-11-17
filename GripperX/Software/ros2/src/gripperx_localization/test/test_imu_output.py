"""What localization_input_node hands the EKF on imu_output_topic (2.4).

Since 2.4 robot_localization fuses imu0 (gyro yaw rate), so what this node publishes
there is EKF input. Runs the REAL LocalizationInputNode with the package's own
config/localization.yaml; the publishers are replaced by recorders.

What makes each test fail is stated on it.
"""
import math
import os
from types import SimpleNamespace

import pytest

rclpy = pytest.importorskip("rclpy")
node_mod = pytest.importorskip("gripperx_localization.localization_input_node")

from builtin_interfaces.msg import Time  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from sensor_msgs.msg import Imu  # noqa: E402

CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "config",
                      "localization.yaml")
PERIOD = 0.02                                     # 50 Hz, the BNO085 rate
DATASHEET = [0.00292681, 0.0, 0.0, 0.0, 0.00292681, 0.0, 0.0, 0.0, 0.00292681]


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    rclpy.init(args=["--ros-args", "--params-file", CONFIG])
    yield
    rclpy.shutdown()


def stamp(t):
    return Time(sec=int(t), nanosec=int(round((t - int(t)) * 1e9)))


def seconds(s):
    return s.sec + s.nanosec * 1e-9


class Rig:
    def __init__(self, bias_estimator=True, overrides=None):
        if overrides:
            # Parameters given to the node itself win over the --params-file.
            base_init = node_mod.Node.__init__

            def init_with_overrides(node, name, **kwargs):
                base_init(node, name, parameter_overrides=[
                    Parameter(k, value=v) for k, v in overrides.items()], **kwargs)
            node_mod.Node.__init__ = init_with_overrides
            try:
                self.node = node_mod.LocalizationInputNode()
            finally:
                node_mod.Node.__init__ = base_init
        else:
            self.node = node_mod.LocalizationInputNode()
        if not bias_estimator:
            self.node.gyro_bias = None       # what gyro_bias_estimation: false yields
        self.out = []
        self.node.imu_publisher = SimpleNamespace(publish=self.out.append)
        self.warnings = []
        real = self.node.get_logger()
        self.node.get_logger = lambda: SimpleNamespace(
            warning=lambda text, **kw: self.warnings.append(text),
            debug=lambda text, **kw: None, info=real.info, error=real.error)

    def imu(self, t, z=0.0):
        msg = Imu()
        msg.header.stamp = stamp(t)
        msg.angular_velocity.z = z
        msg.angular_velocity_covariance = list(DATASHEET)
        self.node.imu_callback(msg)

    @property
    def stamps(self):
        return [round(seconds(m.header.stamp), 6) for m in self.out]

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


@pytest.mark.parametrize("bias_estimator", [True, False])
def test_reordered_sample_is_not_published(rig, bias_estimator):
    # A swapped pair (one sample 20 ms late). The late one is older than what was
    # already published: it must not go out, and every published stamp must be
    # >= the one before. Fails on the 2.3 code, which dropped the sample from the
    # bias estimate but still published it (estimator on), or never checked the
    # order at all (estimator off). Fails on config only if gyro_bias_settle_sec
    # (the drop threshold) falls to 20 ms or below.
    r = rig(bias_estimator=bias_estimator)
    assert r.node.imu_backward_drop_sec > PERIOD, "config makes a swap a new time base"
    sent = [100.00, 100.02, 100.06, 100.04, 100.08, 100.10]
    for t in sent:
        r.imu(t)
    assert r.stamps == [100.00, 100.02, 100.06, 100.08, 100.10]
    assert all(b >= a for a, b in zip(r.stamps, r.stamps[1:]))


@pytest.mark.parametrize("bias_estimator", [True, False])
def test_non_finite_rate_is_not_published(rig, bias_estimator):
    # NaN/inf never reach the EKF (imu0 has no Mahalanobis gate to stop them).
    # Fails if the non-finite guard is removed: the 2.3 code skipped the bias
    # update for such a sample but published it unchanged.
    r = rig(bias_estimator=bias_estimator)
    r.imu(100.00, 0.1)
    r.imu(100.02, math.nan)
    r.imu(100.04, math.inf)
    r.imu(100.06, -math.inf)
    r.imu(100.08, 0.1)
    assert r.stamps == [100.00, 100.08]
    assert all(math.isfinite(m.angular_velocity.z) for m in r.out)


@pytest.mark.parametrize("bias_estimator", [True, False])
def test_new_time_base_resumes_output(rig, bias_estimator):
    # A step back of twice the drop threshold (bag loop, simulation reset) is a new
    # time base: output resumes from it with ONE warning, instead of the IMU going
    # silent for good (which would leave the EKF on the encoder yaw rate without
    # anyone noticing). Fails if a long step back is treated like a short one
    # (sample dropped), or if the step back is not logged.
    r = rig(bias_estimator=bias_estimator)
    for k in range(10):
        r.imu(200.0 + k * PERIOD)
    back = 200.0 + 9 * PERIOD - 2.0 * r.node.imu_backward_drop_sec
    for k in range(5):
        r.imu(back + k * PERIOD)
    assert len(r.out) == 15
    assert r.stamps[10] == pytest.approx(back)
    assert len(r.warnings) == 1 and "backwards" in r.warnings[0]


def _configured():
    import yaml
    with open(CONFIG) as handle:
        return yaml.safe_load(handle)["localization_input_node"]["ros__parameters"]


@pytest.mark.parametrize("value", [None, 1.234e-5])
def test_yaw_rate_variance_reaches_the_ekf(rig, value):
    # The z-z angular-velocity variance on the output is imu_yaw_rate_variance --
    # from localization.yaml (None) or from a different value given to the node --
    # overriding the driver's datasheet value; the rest of the matrix passes
    # through. Fails if the override is removed (the EKF then weighs the gyro
    # ~770x looser, report 5.1), if the value is hard-coded in the node instead of
    # read from the parameter, or if the key is dropped from localization.yaml
    # (node default 0 = keep the driver's value).
    want = _configured()["imu_yaw_rate_variance"] if value is None else value
    assert want > 0.0
    r = rig(overrides=None if value is None else {"imu_yaw_rate_variance": value})
    r.imu(100.0, 0.5)
    out = list(r.out[0].angular_velocity_covariance)
    assert out[8] == want
    assert out[:8] == DATASHEET[:8]


@pytest.mark.parametrize("value", [None, 0.123])
def test_wheel_yaw_rate_variance_reaches_the_ekf(rig, value):
    # The encoder yaw-rate variance on /wheel/odom is wheel_odom_yaw_rate_variance
    # (D1 = (a): fallback, loose) -- from localization.yaml (None) or a different
    # value given to the node. Fails if the node goes back to a hard-coded number
    # (the 0.123 case), or the key is dropped from localization.yaml while its
    # value differs from the node default. The D1 floor (>= 0.08) itself is
    # guarded on the effective launch configuration by check_imu_wiring.py.
    want = _configured()["wheel_odom_yaw_rate_variance"] if value is None else value
    r = rig(overrides=None if value is None else {"wheel_odom_yaw_rate_variance": value})
    sent = []
    r.node.wheel_odom_publisher = SimpleNamespace(publish=sent.append)
    r.node.publish_wheel_odometry(stamp(100.0), 0.1, 0.0, 0.2)
    assert sent[0].twist.covariance[35] == want


def _limits(r):
    n = r.node
    assert n.imu_max_abs_yaw_rate > 0.0 and n.imu_max_yaw_rate_step > 0.0
    assert n.imu_max_yaw_rate_slope > 0.0
    return n.imu_max_abs_yaw_rate, n.imu_max_yaw_rate_step, n.imu_max_yaw_rate_slope


@pytest.mark.parametrize("bias_estimator", [True, False])
def test_spike_above_absolute_limit_is_not_published(rig, bias_estimator):
    # One sample at 5 rad/s in a standstill stream (the audit's fault injection).
    # Fails without the absolute check, or if localization.yaml raises
    # imu_max_abs_yaw_rate to 5 rad/s or more (or disables it) -- then the step
    # check alone would have to catch it, which the step-check test covers.
    r = rig(bias_estimator=bias_estimator)
    abs_limit, _, _ = _limits(r)
    spike = 5.0
    if spike <= abs_limit:
        pytest.skip("config allows 5 rad/s; the fault-injection value is no longer a spike")
    r.node.imu_max_yaw_rate_step = 0.0          # absolute check alone
    for k in range(5):
        r.imu(100.0 + k * PERIOD, spike if k == 2 else 0.0)
    assert r.stamps == [100.00, 100.02, 100.06, 100.08]
    assert r.node.imu_implausible_count == 1


@pytest.mark.parametrize("bias_estimator", [True, False])
def test_step_above_limit_is_not_published(rig, bias_estimator):
    # A spike BELOW the absolute limit but a jump larger than the step limit from
    # a standstill. Fails without the step check, or if the config puts the step
    # limit at or above the absolute one (the check could then never fire below
    # the absolute limit -- asserted, not skipped).
    r = rig(bias_estimator=bias_estimator)
    abs_limit, step, _ = _limits(r)
    assert step < abs_limit, "step limit >= absolute limit: the step check is dead"
    spike = 0.5 * (step + abs_limit)
    for k in range(5):
        r.imu(100.0 + k * PERIOD, spike if k == 2 else 0.0)
    assert r.stamps == [100.00, 100.02, 100.06, 100.08]


def test_no_rejection_chain_after_a_spike_in_a_fast_ramp(rig):
    # A ramp whose every step is 0.9 x the step limit (faster than anything
    # measured), from -0.9 to +0.9 x the absolute limit, with one sample replaced
    # by a 2x-absolute spike. Only the spike may go. Fails if the step reference
    # does not widen with the time since the last plausible sample (or the slope
    # is configured below step / 2 periods): the sample after the spike is then
    # 1.8 x step away from the old reference and is rejected, and so is every
    # later one -- the chain.
    r = rig()
    abs_limit, step, slope = _limits(r)
    values, z = [], -0.9 * abs_limit
    while z <= 0.9 * abs_limit:
        values.append(z)
        z += 0.9 * step
    assert len(values) >= 4, "config leaves no room for a ramp; test means nothing"
    spike_at = 1
    for k, value in enumerate(values):
        r.imu(100.0 + k * PERIOD, 2.0 * abs_limit if k == spike_at else value)
    want = [round(100.0 + k * PERIOD, 6) for k in range(len(values)) if k != spike_at]
    assert r.stamps == want
    assert r.node.imu_implausible_count == 1


def test_first_sample_on_a_new_time_base_rejected_warns_once(rig):
    # Bias estimator off. After a long step back the first sample is a spike and
    # is rejected; the next one must still be published WITHOUT a second
    # "backwards" warning. Fails if _imu_sample_admitted keeps the old watermark
    # on a new time base (the next sample is then measured against it again).
    r = rig(bias_estimator=False)
    abs_limit, _, _ = _limits(r)
    for k in range(10):
        r.imu(300.0 + k * PERIOD)
    back = 300.0 - 2.0 * r.node.imu_backward_drop_sec
    r.imu(back, 2.0 * abs_limit)
    for k in range(1, 4):
        r.imu(back + k * PERIOD)
    assert len(r.out) == 13
    assert len([w for w in r.warnings if "backwards" in w]) == 1


@pytest.mark.parametrize("bias_estimator", [True, False])
def test_step_reference_is_forgotten_on_a_new_time_base(rig, bias_estimator):
    # A simulation reset during a spin: the stream ran at +0.9 x the absolute limit,
    # time steps back by twice the drop threshold, and the stream continues at
    # -0.5 rad/s -- more than the step limit away from the old reference. Every
    # sample on the new time base must be published. Fails if the plausibility
    # reference survives the step back: its time then lies in the future, the
    # allowance stays at the step floor, and every sample is rejected (lock-out).
    r = rig(bias_estimator=bias_estimator)
    abs_limit, step, _ = _limits(r)
    before = 0.9 * abs_limit
    after = -0.5
    assert abs(before - after) > step, "config step limit covers the jump; test means nothing"
    for k in range(10):
        r.imu(400.0 + k * PERIOD, before)
    back = 400.0 - 2.0 * r.node.imu_backward_drop_sec
    for k in range(20):
        r.imu(back + k * PERIOD, after)
    assert len(r.out) == 30
    assert r.node.imu_implausible_count == 0
