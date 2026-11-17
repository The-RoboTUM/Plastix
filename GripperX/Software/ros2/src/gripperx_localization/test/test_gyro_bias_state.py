"""Gyro bias guard state is PUBLISHED on /localization/gyro_bias_state (user decision
2026-11-05: lock-out stays, but must be visible). The REAL LocalizationInputNode with the
package's own config/localization.yaml, driven by the Rig of test_gyro_bias.py; the
payload is received by a real subscriber. Thresholds are TO-VERIFY in the yaml; these
tests prove the mechanism."""
import json
import time

import pytest

import test_gyro_bias as G
from test_gyro_bias import BIAS, cfg, rig, ros_context  # noqa: F401  (fixtures)

from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

TOPIC = G.node_mod.GYRO_BIAS_STATE_TOPIC


def receive(r, wait=3.0, call_publish=True):
    """Message a real subscriber (joining late, TRANSIENT_LOCAL) gets from the node."""
    sub = Node("gyro_state_listener")
    got = []
    sub.create_subscription(
        String, TOPIC, lambda m: got.append(json.loads(m.data)),
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                   reliability=ReliabilityPolicy.RELIABLE))
    if call_publish:
        r.node.publish_gyro_bias_state()
    ex = SingleThreadedExecutor()
    ex.add_node(sub)
    ex.add_node(r.node)
    end = time.monotonic() + wait
    while not got and time.monotonic() < end:
        ex.spin_once(timeout_sec=0.1)
    ex.remove_node(sub)
    ex.remove_node(r.node)
    sub.destroy_node()
    return got[-1] if got else None


def test_state_parameters_come_from_the_yaml(rig):
    r = rig()
    assert r.node.gyro_bias_diag_reject_sec == 10.0
    assert r.node.gyro_bias_diag_no_estimate_sec == 180.0


def test_healthy_warm_state_is_published_ok(rig):
    r = rig()
    settle, _, min_sec = cfg(r)
    r.run(settle + min_sec + 10.0)
    s = receive(r)
    assert s is not None, "nothing on " + TOPIC
    assert s["level"] == 0 and s["estimate_present"] is True
    assert s["bias"] == pytest.approx(BIAS, abs=1e-9)
    assert s["rejected_band"] == 0 and s["rejected_frozen"] == 0 and s["frozen"] is False


def test_timer_publishes_without_a_manual_call(rig):
    r = rig()
    r.run(5.0)
    s = receive(r, wait=4.0, call_publish=False)
    assert s is not None and s["estimate_present"] is False


def test_lockout_after_real_drift_warns(rig):
    # Warm estimate, then the true bias jumps far outside the band: every standstill
    # sample is rejected for good. Must be WARN with counters, bias still the old one.
    r = rig()
    r.run(cfg(r)[0] + cfg(r)[2] + 10.0)
    r.gyro_override = BIAS + 0.05
    r.run(5.0)
    assert receive(r)["level"] == 0              # not yet continuous for long enough
    r.run(20.0)
    s = receive(r)
    assert s["level"] == 1 and "rejected continuously" in s["message"]
    assert s["rejected_band"] > 0 and s["reject_run_sec"] >= r.node.gyro_bias_diag_reject_sec
    assert s["estimate_present"] is True and s["bias"] == pytest.approx(BIAS, abs=1e-9)


def test_rejection_run_ends_when_samples_are_accepted_again(rig):
    r = rig()
    r.run(cfg(r)[0] + cfg(r)[2] + 10.0)
    r.gyro_override = 0.5
    r.run(20.0)
    assert receive(r)["level"] == 1
    r.gyro_override = None
    r.run(3.0)
    s = receive(r)
    assert s["level"] == 0 and s["reject_run_sec"] < 3.0


def test_no_estimate_after_cold_window_warns(rig):
    r = rig()
    r.gyro_override = 0.5                         # cold start, never an estimate
    r.run(r.node.gyro_bias_diag_no_estimate_sec + 5.0)
    s = receive(r)
    assert s["level"] == 1 and s["estimate_present"] is False and s["bias"] is None
    assert "no bias estimate" in s["message"]


def test_frozen_sensor_state_is_reported(rig):
    r = rig()
    r.run(cfg(r)[0] + cfg(r)[2] + 10.0)
    r.gyro_override = BIAS + 0.004
    r.accel_frozen = True
    r.run(3.0)
    s = receive(r)
    assert s["frozen"] is True and s["level"] == 1 and s["rejected_frozen"] > 0
    assert s["frozen_total"] > 0


def test_driving_after_one_rejected_sample_does_not_warn(rig):
    # Audit repro 1: the first gyro sample of an uncommanded rotation is seen before
    # the wheels move and is rejected; the 15 s drive that follows is not standstill.
    r = rig()
    r.run(cfg(r)[0] + cfg(r)[2] + 10.0)
    r.spin(15.0)
    s = receive(r)
    assert s["level"] == 0 and s["reject_run_sec"] < 1.0, s


def test_commanded_drive_after_a_short_transient_does_not_warn(rig):
    # Audit repro 2: 0.2 s of rocking past settle is rejected, then 15 s commanded drive.
    r = rig()
    r.run(cfg(r)[0] + cfg(r)[2] + 10.0)
    r.gyro_override = BIAS + 0.05
    r.run(0.2)
    r.gyro_override = None
    r.cmd = 0.3
    r.body_rate, r.wheel_rate = 0.3, r.wheels_for_spin(0.3)
    r.run(15.0)
    s = receive(r)
    assert s["level"] == 0 and s["reject_run_sec"] < 1.0, s


def test_lockout_keeps_counting_across_short_standstills(rig):
    r = rig()
    r.run(cfg(r)[0] + cfg(r)[2] + 10.0)
    r.gyro_override = BIAS + 0.05
    r.run(8.0)
    r.gyro_override = None
    r.spin(5.0)                                   # drive in between: adds nothing
    r.gyro_override = BIAS + 0.05
    r.run(6.0)
    assert receive(r)["level"] == 1


def test_time_base_reset_restarts_the_cold_start_clock(rig):
    # Fails if _gyro_bias_time_went_back forgets to clear gyro_bias_first_time: the old
    # first sample would then still date the "no estimate" window.
    r = rig()
    r.run(200.0)
    r.jump(-150.0)
    r.run(20.0)
    s = receive(r)
    assert s["estimate_present"] is False
    assert s["since_first_sample_sec"] < 25.0, s


def test_silent_imu_warns_and_age_is_published(rig):
    r = rig()
    r.run(cfg(r)[0] + cfg(r)[2] + 10.0)
    clock = {"t": r.node.gyro_bias_last_imu_wall}
    r.node._wall_now = lambda: clock["t"]
    assert receive(r)["level"] == 0
    clock["t"] += r.node.gyro_bias_diag_imu_silent_sec + 1.0
    s = receive(r)
    assert s["level"] == 1 and "IMU silent" in s["message"]
    assert s["imu_age_sec"] >= r.node.gyro_bias_diag_imu_silent_sec


def test_drive_between_two_short_rejection_stretches_adds_nothing(rig):
    # Fails if the gap between rejected samples is not capped at max_sample_dt: the 20 s
    # drive in between would then count as rejection time.
    r = rig()
    r.run(cfg(r)[0] + cfg(r)[2] + 10.0)
    r.gyro_override = BIAS + 0.05
    r.run(4.0)
    r.gyro_override = None
    r.spin(20.0)
    r.gyro_override = BIAS + 0.05
    r.run(cfg(r)[0] + 1.0)
    s = r.node.gyro_bias_state()
    assert s["reject_run_sec"] < 7.0 and s["level"] == 0, s


def test_imu_age_is_refreshed_by_each_imu_callback(rig):
    # Fails if imu_callback stops stamping the wall time of the last sample.
    r = rig()
    clock = {"t": 5000.0}
    r.node._wall_now = lambda: clock["t"]
    r.node.gyro_bias_last_imu_wall = clock["t"]
    clock["t"] += 100.0
    assert r.node.gyro_bias_state()["imu_age_sec"] == pytest.approx(100.0)
    r.run(0.1)
    assert r.node.gyro_bias_state()["imu_age_sec"] == pytest.approx(0.0)


# ---------------------------------------------------------------- twin IMU noise (B)
XACRO = (G.os.path.dirname(G.os.path.abspath(__file__)) + "/../../gripperx_description"
         "/urdf/gripperx_v1.gazebo.xacro")


def test_twin_imu_declares_gaussian_noise_on_all_six_axes_and_no_bias():
    import xml.etree.ElementTree as ET
    sensor = ET.parse(XACRO).getroot().find(".//sensor[@name='imu_sensor']")
    for group in ("angular_velocity", "linear_acceleration"):
        for axis in "xyz":
            noise = sensor.find(f"imu/{group}/{axis}/noise")
            assert noise is not None, f"{group}/{axis} has no noise"
            assert noise.get("type") == "gaussian"
            assert float(noise.find("mean").text) == 0.0
            assert float(noise.find("stddev").text) > 0.0
            assert noise.find("bias_mean") is None and noise.find("dynamic_bias_stddev") is None


def test_noiseless_twin_imu_never_estimates_a_bias_and_warns(rig):
    # What the twin did before the noise: every sample repeats bit-identically, so the
    # frozen check swallows the whole standstill. Confirms the finding behind B.
    r = rig()
    r.gyro_override = BIAS
    r.accel_frozen = True
    r.run(cfg(r)[0] + cfg(r)[2] + 20.0)
    s = receive(r)
    assert s["estimate_present"] is False and s["frozen"] is True and s["level"] == 1


def test_twin_noise_level_gives_an_estimate_and_no_frozen_samples(rig):
    # Gyro z sigma as written in the xacro (0.000333 rad/s), continuous, not quantised.
    import xml.etree.ElementTree as ET
    sensor = ET.parse(XACRO).getroot().find(".//sensor[@name='imu_sensor']")
    sigma = float(sensor.find("imu/angular_velocity/z/noise/stddev").text)
    r = rig(noise_steps=sigma / G.STEP)
    r.run(cfg(r)[0] + cfg(r)[2] + 20.0)
    s = receive(r)
    assert s["estimate_present"] is True and s["rejected_frozen"] == 0 and s["level"] == 0
    assert s["bias"] == pytest.approx(BIAS, abs=2e-4)
