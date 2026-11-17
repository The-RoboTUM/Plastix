"""P carries the arm node's stop epoch (user decision 2026-10-12: Space must always
abort P). keyboard_teleop_node.trigger_pick, no hardware, nothing on the wire: the
action client is replaced by a recorder. Self-contained (own fixture)."""

import os

# Discovery OFF and a twin domain, as in test_leader_keys.py (process-local).
os.environ.setdefault('ROS_DOMAIN_ID', '221')
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'OFF'
os.environ.pop('ROS_LOCALHOST_ONLY', None)
os.environ.pop('ROS_STATIC_PEERS', None)

import pytest  # noqa: E402
import rclpy  # noqa: E402
from std_msgs.msg import Int64  # noqa: E402

from gripperx_geometry.constants import (  # noqa: E402
    HALF_TRACK_KINGPIN as B,
    HALF_WHEELBASE_KINGPIN as A,
    WHEEL_RADIUS_EFFECTIVE as R,
)
from gripperx_teleop import keyboard_teleop_node as ktn  # noqa: E402


class _Sink:
    def publish(self, msg):
        pass


class _Client:
    def __init__(self, ready=True):
        self.goals, self.ready = [], ready

    def wait_for_server(self, timeout_sec=0.0):
        return self.ready

    def send_goal_async(self, goal):
        self.goals.append(goal)

        class F:
            def add_done_callback(self, cb):
                pass
        return F()


@pytest.fixture
def pick_node():
    rclpy.init(args=['--ros-args', '-p', f'a:={A}', '-p', f'b:={B}', '-p', f'wheel_radius:={R}'])
    n = ktn.KeyboardTeleopNode()
    n._leader_pub = n._arm_pub = n._arm_stop_pub = _Sink()
    n._pick_client = _Client()
    yield n
    n.destroy_node()
    rclpy.shutdown()


def test_pick_goal_carries_the_last_stop_epoch(pick_node):
    n = pick_node
    n._on_stop_epoch(Int64(data=4711))
    assert n.trigger_pick() is True
    assert [g.stop_epoch for g in n._pick_client.goals] == [4711]


def test_no_pick_goal_before_a_stop_epoch_arrived(pick_node):
    n = pick_node
    assert n._stop_epoch is None                 # nothing published in this process
    assert n.trigger_pick() is False
    assert n._pick_client.goals == [] and not n._pick_busy


def test_trigger_pick_reports_every_path_that_sends_nothing(pick_node):
    n = pick_node
    n._on_stop_epoch(Int64(data=7))
    n._pick_client.ready = False
    assert n.trigger_pick() is False             # server unreachable
    n._pick_client.ready = True
    assert n.trigger_pick() is True
    assert n.trigger_pick() is False             # already running
    assert len(n._pick_client.goals) == 1


def test_stop_epoch_subscription_is_reliable_and_latched(pick_node):
    """It must match the arm's RELIABLE + TRANSIENT_LOCAL depth-1 publisher, or this node
    never learns the epoch (and every P fails closed)."""
    from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
    subs = [s for s in pick_node.subscriptions if s.topic_name == '/arm/stop_epoch']
    assert len(subs) == 1
    q = subs[0].qos_profile
    assert (q.reliability, q.durability, q.depth) == (
        ReliabilityPolicy.RELIABLE, DurabilityPolicy.TRANSIENT_LOCAL, 1)
