"""drive_fault_nav_guard against a stand-in NavigateToPose server.

No Nav2, no controller, no motion: a fake action server that runs until it is
cancelled, the guard node, and a latched DriveFault published by this test.
ROS_DOMAIN_ID 220 (a twin domain, same convention as test_check_scripts.py) so
nothing here can reach a real robot's graph.
"""

import threading
import time

import pytest

rclpy = pytest.importorskip('rclpy')
from action_msgs.msg import GoalStatus  # noqa: E402
from gripperx_control.drive_fault_nav_guard import (  # noqa: E402
    active_goal_count, cancel_all_request, drive_fault_where, DriveFaultNavGuard, FAULT_QOS,
)
from gripperx_control_msgs.msg import DriveFault  # noqa: E402
from nav2_msgs.action import NavigateToPose  # noqa: E402
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse  # noqa: E402
from rclpy.callback_groups import ReentrantCallbackGroup  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402


class FakeNavigator(Node):
    """Accepts every goal, runs until cancelled."""

    def __init__(self, **node_kwargs):
        super().__init__('fake_bt_navigator', **node_kwargs)
        self.server = ActionServer(
            self, NavigateToPose, '/navigate_to_pose', self._execute,
            goal_callback=lambda _g: GoalResponse.ACCEPT,
            cancel_callback=lambda _g: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup())

    def _execute(self, goal_handle):
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return NavigateToPose.Result()
            time.sleep(0.02)
        goal_handle.succeed()
        return NavigateToPose.Result()


_DOMAINS = iter(range(220, 230))  # the twin band (SR-8), one fresh domain per test


@pytest.fixture
def graph():
    # A FRESH DOMAIN PER TEST: on a shared domain a previous test's latched
    # action status (TRANSIENT_LOCAL) could still arrive and overwrite this
    # test's goal count -- seen as a flaky retry test.
    context = rclpy.Context()
    rclpy.init(context=context, domain_id=next(_DOMAINS))
    executor = MultiThreadedExecutor(num_threads=4, context=context)
    nav = FakeNavigator(context=context)
    guard = DriveFaultNavGuard(context=context, parameter_overrides=[
        Parameter('recancel_period_sec', value=0.2),
        Parameter('cancel_retry_limit', value=30),
    ])
    client_node = Node('nav_client', context=context)
    client = ActionClient(client_node, NavigateToPose, '/navigate_to_pose')
    fault_pub = client_node.create_publisher(DriveFault, '/swerve_controller/drive_fault',
                                             FAULT_QOS)
    for n in (nav, guard, client_node):
        executor.add_node(n)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    try:
        yield client, fault_pub, guard
    finally:
        executor.shutdown()
        for n in (nav, guard, client_node):
            n.destroy_node()
        rclpy.shutdown(context=context)


def _wait(predicate, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return predicate()


def _send_goal(client):
    assert client.wait_for_server(timeout_sec=5.0)
    future = client.send_goal_async(NavigateToPose.Goal())
    assert _wait(future.done, 5.0)
    handle = future.result()
    assert handle.accepted
    return handle, handle.get_result_async()


def _fault(active=True):
    msg = DriveFault()
    msg.active = active
    msg.reason = DriveFault.REASON_ENCODER_MISSING if active else DriveFault.REASON_NONE
    msg.reason_text = 'ENCODER_MISSING' if active else 'NONE'
    msg.wheel_index = 3 if active else -1
    msg.wheel_name = 'b_rightwheel' if active else ''
    return msg


def test_helpers():
    from action_msgs.msg import GoalStatusArray
    arr = GoalStatusArray()
    for status in (GoalStatus.STATUS_EXECUTING, GoalStatus.STATUS_SUCCEEDED,
                   GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_CANCELED):
        s = GoalStatus()
        s.status = status
        arr.status_list.append(s)
    assert active_goal_count(arr) == 2
    request = cancel_all_request()
    assert bytes(request.goal_info.goal_id.uuid) == bytes(16)
    assert request.goal_info.stamp.sec == 0 and request.goal_info.stamp.nanosec == 0


def test_running_goal_is_cancelled_when_the_fault_latches(graph):
    client, fault_pub, _guard = graph
    _handle, result = _send_goal(client)
    # No fault (an explicit "inactive" message, as the controller publishes on
    # configure): the goal keeps running.
    fault_pub.publish(_fault(active=False))
    assert not _wait(result.done, 1.0)
    fault_pub.publish(_fault(active=True))
    assert _wait(result.done, 5.0), 'goal still running after the drive fault latched'
    assert result.result().status == GoalStatus.STATUS_CANCELED


def test_a_goal_sent_after_the_fault_is_cancelled_too(graph):
    client, fault_pub, guard = graph
    fault_pub.publish(_fault(active=True))
    assert _wait(lambda: guard.fault_active, 5.0)
    time.sleep(1.1)  # past recancel_period_sec of the edge-triggered cancel
    _handle, result = _send_goal(client)
    assert _wait(result.done, 5.0), 'late goal was not cancelled'
    assert result.result().status == GoalStatus.STATUS_CANCELED


def test_cancel_is_retried_when_the_service_was_not_yet_discovered(graph):
    """F6: at fault time the cancel service is 'not ready'; the bounded retry gets it."""
    client, fault_pub, guard = graph
    _handle, result = _send_goal(client)
    # The guard must have SEEN the executing goal before the fault (status topic).
    assert _wait(lambda: guard._active_goals.get('/navigate_to_pose', 0) > 0, 5.0)
    cancel_client = guard._cancel_clients['/navigate_to_pose']
    real_ready = cancel_client.service_is_ready
    calls = {'n': 0}

    def not_ready_at_first():
        calls['n'] += 1
        return calls['n'] > 3 and real_ready()

    cancel_client.service_is_ready = not_ready_at_first
    fault_pub.publish(_fault(active=True))
    assert _wait(result.done, 5.0), (
        'goal still running: the cancel was not retried '
        f'(ready-checks={calls["n"]}, attempts={guard._attempts}, '
        f'active={guard._active_goals}, gave_up={guard._gave_up}, '
        f'fault={guard.fault_active})')
    assert result.result().status == GoalStatus.STATUS_CANCELED
    assert calls['n'] > 3


def test_a_clear_stops_the_cancelling_but_resumes_nothing(graph):
    """SR-18 amendment 2026-11-09 (L3): after active = false a goal cancelled by the
    fault stays cancelled, and only a goal sent after the clear runs."""
    client, fault_pub, guard = graph
    _handle, aborted = _send_goal(client)
    fault_pub.publish(_fault(active=True))
    assert _wait(aborted.done, 5.0)
    assert aborted.result().status == GoalStatus.STATUS_CANCELED
    cleared = _fault(active=False)
    cleared.clear_count = 1
    cleared.last_cleared_reason = DriveFault.REASON_STALL_LATCHED_HELD
    cleared.last_clear_requester = 'cli'
    fault_pub.publish(cleared)
    assert _wait(lambda: not guard.fault_active, 5.0)
    # Nothing re-sent: the guard has no action client at all, and no goal is
    # running on the server after the clear.
    time.sleep(1.0)
    assert guard._active_goals.get('/navigate_to_pose', 0) == 0
    _handle2, fresh = _send_goal(client)
    assert not _wait(fresh.done, 1.5), 'a goal sent after the clear was cancelled'
    assert aborted.result().status == GoalStatus.STATUS_CANCELED


def test_feedback_lost_is_logged_as_the_link_not_a_wheel():
    """L2: FEEDBACK_LOST has wheel_index -1 and no wheel_name -- name the link."""
    m = DriveFault()
    m.active = True
    m.reason = DriveFault.REASON_FEEDBACK_LOST
    m.reason_text = 'FEEDBACK_LOST'
    m.wheel_index = -1
    m.other_wheel_index = 0
    assert drive_fault_where(m) == 'wheel feedback link, all wheels'
    assert drive_fault_where(_fault(active=True)) == 'wheel b_rightwheel'
