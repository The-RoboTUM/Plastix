"""The gyro injection of 2.5b (D14) and the other 2.7 replay inputs.

The point of these tests: an injection that does not reach the EKF's input would give a
reassuring table. So they run the REAL LocalizationInputNode on the package's own
config/localization.yaml, take every limit from that file, and follow the fault to the
topic the EKF reads in that same file. The replay itself needs real time and is not run.

Run these outside the replay domain, as all tests: ROS_DOMAIN_ID=99.
"""
import os
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

rclpy = pytest.importorskip("rclpy")
node_mod = pytest.importorskip("gripperx_localization.localization_input_node")

from rclpy.parameter import Parameter  # noqa: E402
from rclpy.serialization import deserialize_message, serialize_message  # noqa: E402
from sensor_msgs.msg import Imu  # noqa: E402

from gripperx_localization import imu_replay as ir  # noqa: E402

PKG = Path(__file__).resolve().parents[1]
CONFIG = PKG / "config" / "localization.yaml"
PLAN = PKG / "config" / "imu_replay_plan_2p6.yaml"
ACTIVE = yaml.safe_load(CONFIG.read_text())
INPUT = ACTIVE["localization_input_node"]["ros__parameters"]
EKF = ACTIVE["ekf_filter_node"]["ros__parameters"]
PERIOD = 0.02


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    rclpy.init(args=["--ros-args", "--params-file", str(CONFIG)])
    yield
    rclpy.shutdown()


class Rig:
    """The real node; what it would hand the EKF lands in self.out."""

    def __init__(self):
        self.node = node_mod.LocalizationInputNode()
        self.out = []
        self.node.imu_publisher = SimpleNamespace(publish=self.out.append)
        self.warnings = []
        real = self.node.get_logger()
        self.node.get_logger = lambda: SimpleNamespace(
            warning=lambda text, **kw: self.warnings.append(text),
            debug=lambda text, **kw: None, info=real.info, error=real.error)


@pytest.fixture
def rig():
    r = Rig()
    yield r
    r.node.destroy_node()


def sample(t, z):
    m = Imu()
    m.header.stamp.sec, m.header.stamp.nanosec = int(t), int(round((t - int(t)) * 1e9))
    m.angular_velocity.z = z
    return m


def stream(rate, t_start=100.0, n=200):
    """Clean turn at a constant rate, then at rest: the fault's interesting edge."""
    return [(t_start + k * PERIOD, rate if k < n // 2 else 0.0) for k in range(n)]


def through(rig, clean, mode, t_inject, duration):
    """clean -> injector -> real node. -> {stamp: z the EKF would get}."""
    inj = ir.GyroInjector(mode, t_inject, duration, t0=0.0)
    for t, z in clean:
        msg = inj.apply(sample(t, z), int(round(t * 1e9)))
        if msg is not None:
            rig.node.imu_callback(msg)
    return {round(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, 3): m.angular_velocity.z
            for m in rig.out}


def test_the_ekf_reads_the_topic_the_tested_publisher_writes(rig):
    # Fails if the EKF is pointed elsewhere: then every number below is about nothing.
    assert EKF["imu0"] == INPUT["imu_output_topic"]
    assert rig.node.imu_output_topic == EKF["imu0"]


def test_node_runs_the_limits_of_the_active_file(rig):
    # A test that read a copied constant would keep passing after a config change.
    assert rig.node.imu_max_yaw_rate_step == INPUT["imu_max_yaw_rate_step"]
    assert rig.node.imu_max_abs_yaw_rate == INPUT["imu_max_abs_yaw_rate"]
    assert rig.node.imu_max_yaw_rate_slope == INPUT["imu_max_yaw_rate_slope"]


def test_stuck_alters_what_the_ekf_gets_and_passes_every_check(rig):
    # Stuck at a turn rate, across the stop: the clean stream goes to 0, the output
    # must not. It is the residual risk of the config, shown on the real checks: none
    # of them fires while the rate stays below imu_max_yaw_rate_step.
    rate = 0.8 * INPUT["imu_max_yaw_rate_step"]
    clean = stream(rate)
    got = through(rig, clean, "stuck", 101.0, 2.0)
    assert len(got) == len(clean)
    changed = [t for t, z in clean if abs(got[round(t, 3)] - z) > 0.5 * rate]
    assert changed, "the injection did not reach the output"
    assert all(102.0 <= t < 103.0 for t in changed)
    assert not [w for w in rig.warnings if "Implausible" in w]


def test_zero_below_the_step_limit_is_not_caught(rig):
    rate = 0.8 * INPUT["imu_max_yaw_rate_step"]
    got = through(rig, stream(rate), "zero", 101.0, 0.5)
    assert got[101.1] == 0.0
    assert not [w for w in rig.warnings if "Implausible" in w]


def test_zero_above_the_step_limit_loses_only_the_first_samples(rig):
    # The check's own widening (slope) lets the fault through after a few samples.
    step, slope = INPUT["imu_max_yaw_rate_step"], INPUT["imu_max_yaw_rate_slope"]
    rate = 1.1 * step
    assert rate < INPUT["imu_max_abs_yaw_rate"]
    t_start, n_lost = 101.0, int(rate / slope / PERIOD)
    got = through(rig, stream(rate), "zero", t_start, 0.5)
    rejected = [k for k in range(10) if round(t_start + k * PERIOD, 3) not in got]
    assert rejected == list(range(n_lost))
    assert got[round(t_start + n_lost * PERIOD, 3)] == 0.0
    assert [w for w in rig.warnings if "Implausible" in w]


def test_silence_removes_samples_and_the_return_passes(rig):
    rate = 0.8 * INPUT["imu_max_yaw_rate_step"]
    got = through(rig, stream(rate), "silent", 101.0, 0.5)
    assert 101.0 not in got and 101.48 not in got
    assert got[101.5] == pytest.approx(rate, abs=1e-6)


def test_recovery_step_after_stuck_is_taken_after_the_widening(rig):
    # Stuck above the step limit, stream back at rest: the stuck samples kept refreshing
    # the reference, so the true return is the step, and it costs a sample or two.
    rate = 1.1 * INPUT["imu_max_yaw_rate_step"]
    got = through(rig, stream(rate), "stuck", 101.0, 2.0)
    assert 103.0 not in got
    assert got[round(103.0 + 3 * PERIOD, 3)] == pytest.approx(0.0, abs=1e-6)
    assert [w for w in rig.warnings if "Implausible" in w]


def test_injector_is_bounded_and_refuses_nonsense():
    with pytest.raises(ir.ReplayError):
        ir.GyroInjector("stuck", 1.0, 0.0, t0=0.0)
    with pytest.raises(ir.ReplayError):
        ir.GyroInjector("frozen", 1.0, 1.0, t0=0.0)


# ------------------------------------------------------------------- at bag level

def _imu_bag(path, rates, hz=50):
    import rosbag2_py
    from tf2_msgs.msg import TFMessage
    w = rosbag2_py.SequentialWriter()
    w.open(rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap"),
           rosbag2_py.ConverterOptions("cdr", "cdr"))
    w.create_topic(rosbag2_py.TopicMetadata(0, "/tf_static", "tf2_msgs/msg/TFMessage", "cdr"))
    w.create_topic(rosbag2_py.TopicMetadata(1, "/imu/bno085/data", "sensor_msgs/msg/Imu", "cdr"))
    t0 = 1_000_000_000
    w.write("/tf_static", serialize_message(TFMessage()), t0)
    for k, z in enumerate(rates):
        ns = t0 + k * 10**9 // hz
        w.write("/imu/bno085/data", serialize_message(sample(ns * 1e-9, z)), ns)
    del w


def _recorded(path, zs, hz=50, topic="/imu/data/filtered", t0=1.0):
    """A stand-in for a run's output bag: zs as recorded from the input node."""
    import rosbag2_py
    w = rosbag2_py.SequentialWriter()
    w.open(rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap"),
           rosbag2_py.ConverterOptions("cdr", "cdr"))
    w.create_topic(rosbag2_py.TopicMetadata(0, topic, "sensor_msgs/msg/Imu", "cdr"))
    for k, z in zs:
        t = t0 + k / hz
        w.write(topic, serialize_message(sample(t, z)), int(t * 1e9))
    del w


def test_prepare_writes_the_fault_into_the_bag(tmp_path):
    src, dst = tmp_path / "s", tmp_path / "d"
    _imu_bag(src, [0.5] * 100 + [0.0] * 100)
    info = ir.prepare(src, dst, gyro_inject={"mode": "stuck", "start": 1.0, "duration": 2.0})
    assert info["injected_samples"] == 100
    t, z = ir.imu_series(dst, "/imu/bno085/data")
    assert len(t) == 200 and z[:150].min() == 0.5 and z[150:].max() == 0.0


def test_prepare_refuses_an_injection_that_touches_nothing(tmp_path):
    src, dst = tmp_path / "s", tmp_path / "d"
    _imu_bag(src, [0.5] * 100)
    with pytest.raises(ir.ReplayError):
        ir.prepare(src, dst, gyro_inject={"mode": "zero", "start": 50.0, "duration": 2.0})


def _inj(clean, mode):
    return {"mode": mode, "start": 1.0, "duration": 2.0, "clean_bag": str(clean), "t0_source": 1.0}


def test_run_check_passes_a_real_fault_and_catches_a_fault_that_did_not_arrive(tmp_path):
    clean = tmp_path / "c"
    rates = [0.5] * 100 + [0.0] * 200
    _imu_bag(clean, rates)
    good = tmp_path / "good"
    _recorded(good, [(k, 0.5 if 50 <= k < 150 else z) for k, z in enumerate(rates)])
    assert ir.injection_faults(good, _inj(clean, "stuck")) == []
    # The override "worked" but the EKF's input is the clean stream.
    bad = tmp_path / "bad"
    _recorded(bad, list(enumerate(rates)))
    assert ir.injection_faults(bad, _inj(clean, "stuck"))


def test_run_check_wants_the_stream_back(tmp_path):
    clean = tmp_path / "c"
    rates = [0.5] * 300
    _imu_bag(clean, rates)
    never = tmp_path / "n"
    _recorded(never, [(k, 0.0 if k >= 50 else 0.5) for k in range(300)])
    assert any("after the end" in f for f in ir.injection_faults(never, _inj(clean, "zero")))


def test_run_check_catches_samples_inside_a_silence(tmp_path):
    clean = tmp_path / "c"
    _imu_bag(clean, [0.5] * 300)
    leak = tmp_path / "l"
    _recorded(leak, [(k, 0.5) for k in range(300)])
    assert any("inside the silence" in f for f in ir.injection_faults(leak, _inj(clean, "silent")))


def test_chain_moves_second_bag_behind_first_with_every_stamp(tmp_path):
    a, b, c = tmp_path / "a", tmp_path / "b", tmp_path / "c"
    _imu_bag(a, [0.1] * 100)
    _imu_bag(b, [0.2] * 100)
    info = ir.prepare_chain(a, b, c, gap_s=2.5)
    t, z = ir.imu_series(c, "/imu/bno085/data")
    assert len(t) == 200
    assert (t[1:] >= t[:-1]).all()
    assert t[100] - t[99] == pytest.approx(2.5, abs=1e-3)
    assert z[99] == pytest.approx(0.1) and z[100] == pytest.approx(0.2)
    assert info["second_starts_s"] == pytest.approx(t[100] - t[0], abs=1e-3)


def test_chain_gap_exceeds_the_active_reset_gap():
    # Below it the seam is a wheel position jump instead of the designed reset.
    plan = yaml.safe_load(PLAN.read_text())
    gaps = [s["chain"]["gap_s"] for s in plan["inputs"].values() if "chain" in s]
    assert gaps and min(gaps) > INPUT["reset_gap_sec"]
    for v in plan["variants"].values():
        if "max_ekf_gap_s" in v:
            assert v["max_ekf_gap_s"] >= max(gaps)


def test_penalty_override_reaches_slam_only():
    o = ir.build_override({"angle_variance_penalty": 3.0})
    assert o == {"slam_toolbox": {"ros__parameters": {"angle_variance_penalty": 3.0}}}


def test_every_planned_injection_is_bounded_and_named_after_a_real_mode():
    plan = yaml.safe_load(PLAN.read_text())
    injected = [s["prepare"]["gyro_inject"] for s in plan["inputs"].values()
                if "gyro_inject" in s.get("prepare", {})]
    assert injected
    for inj in injected:
        ir.GyroInjector(t0=0.0, **inj)


# ------------------------------------------------------------------ analysis

def test_scan_offset_finds_a_known_delay():
    # Rotating scene, rate ramps: the offset is visible only where the rate changes.
    from gripperx_localization.imu_replay_forensics import fit_scan_offset
    import numpy as np
    t_src = np.arange(0.0, 20.0, 0.02)
    rate = 40.0 * (np.sin(t_src * 0.8) > 0)        # deg/s, on and off
    yaw = np.cumsum(rate) * 0.02
    ts = np.arange(0.0, 20.0, 0.1) + 0.103         # stamps say 103 ms later than the truth
    steps = np.diff(np.interp(ts - 0.103, t_src, yaw))
    tau, resid, at_zero = fit_scan_offset(ts, steps, t_src, yaw, np.ones(len(steps), bool))
    assert tau == pytest.approx(-0.103, abs=0.005)
    assert resid < at_zero


def test_azimuth_step_recovers_a_turn_of_the_profile():
    from gripperx_localization.imu_replay_forensics import azimuth_step, range_profile
    import numpy as np
    rng = np.random.default_rng(1)
    base = np.convolve(rng.uniform(1.0, 8.0, 454), np.ones(5) / 5, mode="same")
    base[base < 0.5] = 1.0
    inc = 2 * np.pi / 453
    turned = np.roll(base, -3)     # 3 beams: the sensor turned 3 * inc to the left
    step = azimuth_step(range_profile(0.0, inc, base), range_profile(0.0, inc, turned))
    assert step == pytest.approx(np.degrees(3 * inc), abs=0.1)


def test_d9_alarm_needs_the_disagreement_sustained():
    from gripperx_localization.imu_replay_injection import d9_alarm
    import numpy as np
    t = np.arange(0.0, 20.0, 0.02)
    enc = np.where((t > 5) & (t < 15), 30.0, 0.0)
    assert d9_alarm(t, enc, enc * 1.1) == []      # within 3 + 15 %
    fires = d9_alarm(t, enc, np.zeros_like(t))
    assert fires and 5.9 < fires[0] < 6.5


def test_chain_map_refuses_a_segment_that_does_not_line_up():
    from gripperx_localization.imu_replay_injection import chain_map
    clean = {"segments": [{"index": 3, "t0": 10.0, "t1": 20.0}, {"index": 4, "t0": 30.0, "t1": 40.0}]}
    chained = {"segments": [{"t0": 110.2, "t1": 120.1}, {"t0": 131.5, "t1": 140.0}]}
    assert list(chain_map(chained, clean, 100.0)) == [3]


def test_time_until_held_ignores_a_short_recovery():
    from gripperx_localization.imu_replay_injection import time_until_held
    import numpy as np
    t = np.arange(0.0, 10.0, 0.02)
    ok = (t > 3.0) & ~((t > 3.5) & (t < 3.6)) | (t < 1.0)
    assert time_until_held(t, ok, 2.0) == pytest.approx(1.6, abs=0.05)
    assert time_until_held(t, np.zeros_like(t, bool), 2.0) is None
