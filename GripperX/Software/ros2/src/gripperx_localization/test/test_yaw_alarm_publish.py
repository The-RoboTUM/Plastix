"""odom_divergence_monitor really publishes the latched alarm on /localization/yaw_alarm
(TRANSIENT_LOCAL, republished each tick), and a subscriber that joins late receives it.
In-process rclpy, no robot; parameters from the active localization.yaml."""
import json
import os
import sys
import time
from pathlib import Path

import pytest

os.environ.setdefault("ROS_DOMAIN_ID", "222")
os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")
HERE = Path(__file__).resolve()
sys.path.insert(0, str(HERE.parents[1] / "src"))
sys.path.append(str(HERE.parents[2] / "gripperx_control" / "src"))
rclpy = pytest.importorskip("rclpy")
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from gripperx_localization.odom_divergence_monitor import (  # noqa: E402
    YAW_ALARM_TOPIC, OdomDivergenceMonitor)
import test_yaw_alarm_latch as L  # noqa: E402  (config P, signals)

YAML = HERE.parents[1] / "config" / "localization.yaml"


@pytest.fixture
def monitor():
    rclpy.init(args=["--ros-args", "--params-file", str(YAML), "-p", "use_sim_time:=false"])
    n = OdomDivergenceMonitor()
    yield n
    n.destroy_node()
    rclpy.shutdown()


def _late_receive(monitor):
    sub_node = Node("late_joiner")
    got = []
    sub_node.create_subscription(
        String, YAW_ALARM_TOPIC, lambda m: got.append(m.data),
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                   reliability=ReliabilityPolicy.RELIABLE))
    ex = SingleThreadedExecutor()
    ex.add_node(sub_node)
    ex.add_node(monitor)
    end = time.monotonic() + 8.0
    while not got and time.monotonic() < end:
        ex.spin_once(timeout_sec=0.1)
    ex.remove_node(sub_node)
    sub_node.destroy_node()
    return got


def test_publisher_qos(monitor):
    info = monitor.get_publishers_info_by_topic(YAW_ALARM_TOPIC)
    assert len(info) == 1
    q = info[0].qos_profile
    assert q.durability == DurabilityPolicy.TRANSIENT_LOCAL
    assert q.reliability == ReliabilityPolicy.RELIABLE


def test_latched_alarm_is_published_and_reaches_a_late_joiner(monitor):
    d = monitor.yaw
    now = monitor._now()
    t0 = now - 40.0                            # replayed on the node clock, ending in the past
    t = t0
    while t < t0 + P_STEP_END():
        bad = t >= t0 + L.P["startup_grace_sec"] + 2.0
        d.push("enc", t, t, __import__("math").radians(L.BAD if bad else L.GYRO))
        d.push("gyro", t, t, __import__("math").radians(L.GYRO))
        d.evaluate(t)
        t += 0.02
    monitor._log_episode()
    assert d.latched
    monitor._tick()                            # publishes before the subscriber exists
    got = _late_receive(monitor)
    assert got, "no /localization/yaw_alarm message reached a late-joining subscriber"
    m = json.loads(got[-1])
    assert m["latched"] is True and m["episodes"] == 1
    assert m["first_episode_utc"].endswith("UTC")


def P_STEP_END():
    return L.P["startup_grace_sec"] + 2.0 + 3 * L.P["yaw_warn_sustain_sec"] + L.P["yaw_window_sec"]


def test_warning_episode_is_logged_at_warn_not_error(monitor):
    calls = []

    class _Log:
        def warning(self, m):
            calls.append(("warn", m))

        def error(self, m):
            calls.append(("error", m))

        def info(self, m):
            calls.append(("info", m))

    monitor.get_logger = lambda: _Log()
    d = monitor.yaw
    t = monitor._now() - 60.0
    tol, win = L.P["yaw_abs_tol_dps"], L.P["yaw_error_window_sec"]
    end_clean = t + L.P["startup_grace_sec"] + win + 2.0
    end = end_clean + L.P["yaw_warn_sustain_sec"] + 0.8     # a warning, far short of an error
    while t < end:
        enc = tol + 1.0 if t >= end_clean else 0.0
        d.push("enc", t, t, __import__("math").radians(enc))
        d.push("gyro", t, t, 0.0)
        d.evaluate(t)
        monitor._log_episode()
        t += 0.02
    assert d.warning_episodes == 1 and not d.latched
    assert [k for k, _ in calls] == ["warn"], calls
