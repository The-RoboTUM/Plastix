"""hw_firmware_mock's simulated encoders (user decision 2026-10-16) and its real-robot guard.

Pure-logic checks of SimulatedEncoder, plus node checks on ROS_DOMAIN_ID 220 (a twin
domain), discovery limited to this host. Nothing here can reach a robot.
"""

import math
import os
import time

import pytest

os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

import rclpy  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from std_msgs.msg import Float64MultiArray  # noqa: E402

from gripperx_control.hw_firmware_mock import (  # noqa: E402
    COUNT_RAD, HwFirmwareMock, PROVENANCE_INIT_FAILED, PROVENANCE_LIVE,
    PROVENANCE_LIVE_UNCONFIRMED, PROVENANCE_NO_ENCODER, SimulatedEncoder,
)

DT = 1.0 / 30.0


# ------------------------------------------------------------------ pure logic
def test_ok_encoder_is_unconfirmed_until_the_first_count_then_live():
    enc = SimulatedEncoder('ok')
    assert enc.provenance == PROVENANCE_LIVE_UNCONFIRMED
    for _ in range(300):                      # standing still: never promoted
        enc.step(0.0, DT)
    assert enc.provenance == PROVENANCE_LIVE_UNCONFIRMED
    enc.step(0.5 * COUNT_RAD / DT, DT)        # half a count
    assert enc.provenance == PROVENANCE_LIVE_UNCONFIRMED
    enc.step(0.6 * COUNT_RAD / DT, DT)        # past one count in total
    assert enc.provenance == PROVENANCE_LIVE


def test_ok_encoder_integrates_the_command():
    enc = SimulatedEncoder('ok')
    for _ in range(30):
        enc.step(-4.0, DT)
    assert enc.velocity == -4.0
    assert math.isclose(enc.position, -4.0, rel_tol=1e-9)
    assert enc.provenance == PROVENANCE_LIVE   # reverse counts too


@pytest.mark.parametrize('mode,code,velocity', [
    ('unconfirmed', PROVENANCE_LIVE_UNCONFIRMED, 0.0),
    ('none', PROVENANCE_NO_ENCODER, 4.0),       # firmware getRPM() echoes the target
    ('init_failed', PROVENANCE_INIT_FAILED, 4.0),
])
def test_missing_encoders_never_count_and_never_promote(mode, code, velocity):
    enc = SimulatedEncoder(mode)
    for _ in range(300):
        enc.step(4.0, DT)
    assert enc.provenance == code
    assert enc.position == 0.0
    assert enc.velocity == velocity


def test_unknown_mode_is_refused():
    with pytest.raises(ValueError):
        SimulatedEncoder('broken')


# ------------------------------------------------------------------ node
@pytest.fixture
def ros():
    context = rclpy.Context()
    rclpy.init(context=context, domain_id=220)
    executor = SingleThreadedExecutor(context=context)
    nodes = []

    yield context, executor, nodes, None
    for n in nodes:
        executor.remove_node(n)
        n.destroy_node()
    executor.shutdown()
    rclpy.shutdown(context=context)


def _make(ros_fixture, encoders):
    """HwFirmwareMock in the test context (its __init__ takes no context argument)."""
    context, executor, nodes, _ = ros_fixture
    original = Node.__init__

    def patched(self, name, **kwargs):
        kwargs.setdefault('context', context)
        kwargs.setdefault('parameter_overrides', [
            Parameter('simulated_encoders', value=encoders),
            Parameter('startup_check_sec', value=0.5),
        ])
        original(self, name, **kwargs)

    Node.__init__ = patched
    try:
        mock = HwFirmwareMock()
    finally:
        Node.__init__ = original
    nodes.append(mock)
    executor.add_node(mock)
    return mock


def _probe(ros_fixture):
    context, executor, nodes, _ = ros_fixture
    probe = Node('mock_probe', context=context)
    nodes.append(probe)
    executor.add_node(probe)
    states = []
    probe.create_subscription(Float64MultiArray, '/hw/joint_states',
                              lambda m: states.append(list(m.data)), 10)
    commands = probe.create_publisher(Float64MultiArray, '/hw/joint_commands', 10)
    return probe, states, commands


def _spin(executor, seconds, each=None):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if each:
            each()
        executor.spin_once(timeout_sec=0.02)


def test_desk_oct13_shape(ros):
    _, executor, _, _ = ros
    _make(ros, ['ok', 'ok', 'ok', 'unconfirmed'])
    _probe_node, states, commands = _probe(ros)
    drive = Float64MultiArray(data=[0.0] * 4 + [4.0] * 4)
    _spin(executor, 2.5, lambda: commands.publish(drive))
    assert states, 'mock published nothing'
    last = states[-1]
    assert len(last) == 16
    assert last[12:16] == [PROVENANCE_LIVE] * 3 + [PROVENANCE_LIVE_UNCONFIRMED]
    assert all(p > 0.5 for p in last[8:11]) and last[11] == 0.0
    assert last[7] == 0.0 and last[4] == 4.0


def test_default_is_four_working_encoders(ros):
    _, executor, _, _ = ros
    mock = _make(ros, ['ok', 'ok', 'ok', 'ok'])
    assert [e.mode for e in mock.encoders] == ['ok'] * 4


def test_refuses_to_publish_next_to_another_states_writer(ros):
    context, executor, nodes, _ = ros
    firmware = Node('fake_esp32', context=context)
    nodes.append(firmware)
    executor.add_node(firmware)
    firmware.create_publisher(Float64MultiArray, '/hw/joint_states', 10)
    mock = _make(ros, ['ok', 'ok', 'ok', 'ok'])
    _probe_node, states, _ = _probe(ros)
    _spin(executor, 2.5)
    assert mock.refused
    assert states == [], 'the mock published next to a real firmware writer'
