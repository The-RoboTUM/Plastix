"""The web UI's P event says what trigger_pick did: 'sent' only when a goal was sent
(it can refuse: busy, server unreachable, no stop epoch yet). No hardware, nothing on
the wire. Self-contained fixture."""

import os

os.environ.setdefault('ROS_DOMAIN_ID', '221')
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'OFF'
os.environ.pop('ROS_LOCALHOST_ONLY', None)
os.environ.pop('ROS_STATIC_PEERS', None)

import pytest  # noqa: E402
import rclpy  # noqa: E402

from gripperx_geometry.constants import (  # noqa: E402
    HALF_TRACK_KINGPIN as B,
    HALF_WHEELBASE_KINGPIN as A,
    WHEEL_RADIUS_EFFECTIVE as R,
)


class _Sink:
    def publish(self, msg):
        pass


@pytest.fixture
def web():
    from gripperx_teleop import web_teleop_node as wtn
    rclpy.init(args=['--ros-args', '-p', f'a:={A}', '-p', f'b:={B}', '-p', f'wheel_radius:={R}'])
    n = wtn.WebTeleopNode()
    n._leader_pub = n._arm_pub = n._arm_stop_pub = n._mode_pub = _Sink()
    yield n
    n.destroy_node()
    rclpy.shutdown()


def _pick_events(n):
    return [(e['text'], e['level']) for e in n._telemetry()['events']
            if e['text'].startswith('pick_plastic')]


@pytest.mark.parametrize('ret,text,level', [
    (True, 'pick_plastic: sent', 'info'),
    (False, 'pick_plastic: NOT sent - see the teleop log', 'alarm'),
    (None, 'pick_plastic: requested', 'info'),
])
def test_pick_event_follows_what_trigger_pick_did(web, ret, text, level):
    calls = []
    web.trigger_pick = lambda: calls.append(1) or ret
    web._on_client_input('s1', [], ['pick|1|5'], True)
    assert calls == [1]
    assert _pick_events(web) == [(text, level)]
