#!/usr/bin/env python3
"""Cancel every Nav2 goal while the whole-drive fault is latched.

User decision 2026-10-16: a missing wheel encoder or a persistent stall stops the
whole drive (swerve_controller, /swerve_controller/drive_fault, latched until the
stack restarts). The controller already forces all four wheel commands to 0.0;
this node makes the navigation stack STOP TRYING, so the fault reads as "goal
cancelled" instead of a long failing recovery dance against wheels that cannot
turn.

Deliberately OUTSIDE the realtime update loop: it is an ordinary node that
subscribes to the latched fault topic and uses the standard action cancel
service (`<action>/_action/cancel_goal` with an all-zero goal id and stamp =
"cancel all goals"). It also watches `<action>/_action/status`, so a goal sent
AFTER the fault (Nav2 started later, an operator re-sending) is cancelled as
soon as it is accepted.

It never publishes a velocity and never commands anything: the zero at the
wheels is the controller's job, and stays there if this node is not running.

OPERATOR CLEAR (SR-18 amendment 2026-11-09, item 6 L3): when the fault topic goes
back to active = false (an operator clear of a STALL_* reason), this node simply
stops cancelling. It NEVER resumes a goal the fault aborted or cancelled -- it
keeps no goals and has no action client to send one; only a goal sent after the
clear runs. A client that re-sends goals by itself will make the robot drive.
"""
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from gripperx_control_msgs.msg import DriveFault
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

ACTIVE_GOAL_STATES = frozenset({GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING})

# rcl's default action-status QoS: KeepLast(1), RELIABLE, TRANSIENT_LOCAL.
STATUS_QOS = QoSProfile(
    depth=1,
    history=HistoryPolicy.KEEP_LAST,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
FAULT_QOS = STATUS_QOS


def active_goal_count(status_array):
    """Goals in a GoalStatusArray that are still accepted or executing."""
    return sum(1 for s in status_array.status_list if s.status in ACTIVE_GOAL_STATES)


def cancel_all_request():
    """Build the CancelGoal request meaning "every goal": zero goal id AND zero stamp."""
    return CancelGoal.Request()


def drive_fault_where(msg):
    """Return 'wheel <name>', or the feedback link for FEEDBACK_LOST (wheel_index -1)."""
    if msg.wheel_index >= 0 and msg.wheel_name:
        return f'wheel {msg.wheel_name}'
    if msg.reason == DriveFault.REASON_FEEDBACK_LOST:
        return 'wheel feedback link, all wheels'
    return 'wheel not identified'


class DriveFaultNavGuard(Node):

    def __init__(self, **node_kwargs):
        super().__init__('drive_fault_nav_guard', **node_kwargs)
        self.declare_parameter('drive_fault_topic', '/swerve_controller/drive_fault')
        # navigate_to_pose / navigate_through_poses are bt_navigator's; follow_path
        # is controller_server's (the one producing cmd_vel); backup /
        # drive_on_heading / crab_walk are the behavior_server plugins in
        # gripperx_planning/config/nav2.yaml that drive the wheels themselves.
        self.declare_parameter('action_names', [
            '/navigate_to_pose', '/navigate_through_poses', '/follow_path',
            '/backup', '/drive_on_heading', '/crab_walk',
        ])
        # Minimum spacing of cancel requests per action while goals keep being
        # accepted. A request cadence, not a safety value.
        self.declare_parameter('recancel_period_sec', 1.0)
        # BOUNDED RETRY: a cancel service not yet discovered at fault time (Nav2
        # still starting, DDS discovery lagging) must not leave a goal running.
        # While the fault is active and an action reports an accepted/executing
        # goal, the cancel is retried every recancel_period_sec, at most this
        # many times per action; then an ERROR says the guard gave up.
        self.declare_parameter('cancel_retry_limit', 30)

        self._fault = None
        self._recancel = float(self.get_parameter('recancel_period_sec').value)
        self._last_cancel = {}
        self._cancel_clients = {}
        self._active_goals = {}
        self._attempts = {}
        self._gave_up = set()
        self._retry_limit = int(self.get_parameter('cancel_retry_limit').value)
        for name in self.get_parameter('action_names').value:
            name = str(name).rstrip('/')
            self._cancel_clients[name] = self.create_client(
                CancelGoal, f'{name}/_action/cancel_goal')
            self.create_subscription(
                GoalStatusArray, f'{name}/_action/status',
                lambda msg, n=name: self._on_status(n, msg), STATUS_QOS)

        self.create_subscription(
            DriveFault, str(self.get_parameter('drive_fault_topic').value),
            self._on_fault, FAULT_QOS)
        self.create_timer(self._recancel, self._retry)
        self.get_logger().info(
            'drive_fault_nav_guard: cancels %s while %s is active.'
            % (', '.join(self._cancel_clients), self.get_parameter('drive_fault_topic').value))

    @property
    def fault_active(self):
        return bool(self._fault is not None and self._fault.active)

    def _on_fault(self, msg):
        was = self.fault_active
        self._fault = msg
        if msg.active and not was:
            self.get_logger().error(
                'DRIVE FAULT (%s, %s): cancelling every Nav2 goal. %s'
                % (msg.reason_text, drive_fault_where(msg), msg.detail))
            for name in self._cancel_clients:
                self._cancel(name, force=True)
        elif was and not msg.active:
            requester = getattr(msg, 'last_clear_requester', '') or 'unknown'
            self.get_logger().warn(
                f'drive fault cleared (by {requester}): new Nav2 goals are no longer '
                'cancelled. Goals cancelled during the fault are NOT resumed.')
            self._attempts.clear()
            self._gave_up.clear()

    def _on_status(self, name, msg):
        count = active_goal_count(msg)
        self._active_goals[name] = count
        if count == 0:
            self._attempts.pop(name, None)
            self._gave_up.discard(name)
        if self.fault_active and count > 0:
            self._cancel(name, force=False)

    def _retry(self):
        if not self.fault_active:
            return
        for name, count in self._active_goals.items():
            if count == 0 or name in self._gave_up:
                continue
            attempts = self._attempts.get(name, 0)
            if attempts >= self._retry_limit:
                self._gave_up.add(name)
                self.get_logger().error(
                    f'{name}: {count} goal(s) still active after {attempts} cancel attempts '
                    'during the drive fault -- giving up. The wheels stay at 0.0 (the '
                    'controller enforces that); cancel the goal by hand.')
                continue
            self._attempts[name] = attempts + 1
            self._cancel(name, force=False)

    def _cancel(self, name, force):
        now = self.get_clock().now().nanoseconds * 1e-9
        last = self._last_cancel.get(name)
        if not force and last is not None and 0.0 <= now - last < self._recancel:
            return
        client = self._cancel_clients[name]
        if not client.service_is_ready():
            # Nav2 not running (or that server not up): nothing to cancel now;
            # the status subscription catches any goal it accepts later.
            self.get_logger().info(f'{name}: cancel service not available, nothing to cancel')
            return
        self._last_cancel[name] = now
        future = client.call_async(cancel_all_request())
        future.add_done_callback(lambda f, n=name: self._on_cancel_done(n, f))

    def _on_cancel_done(self, name, future):
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001 -- report, never crash the guard
            self.get_logger().error(f'{name}: cancel request failed: {exc}')
            return
        self.get_logger().warn(
            f'{name}: cancel-all sent because of the drive fault -> return_code '
            f'{response.return_code}, {len(response.goals_canceling)} goal(s) canceling')


def main(args=None):
    rclpy.init(args=args)
    node = DriveFaultNavGuard()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
