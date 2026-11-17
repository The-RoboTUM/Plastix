#!/usr/bin/env python3
"""
Browser teleop for GripperX — the same teleop, driven from a web page.

What this is, and what it deliberately is not
---------------------------------------------
It is `keyboard_teleop_node` with a different input device. The class below
SUBCLASSES the terminal node and adds nothing to the control path: the same
`press()`, the same dead-man windows, the same `TransitionGuard`, the same
`center()` for the emergency stop, the same topics. Every safety property that
was accepted for the terminal node holds here for the same reason it held
there — it is the same code, not a second copy of it.

It is NOT a replacement for `keyboard_teleop_node`. That node is untouched and
still works; run whichever fits. Running BOTH at once is the one thing to
avoid: two nodes publishing the same cmd_vel would race.

The dead-man switch across a network
------------------------------------
A browser has something the terminal never had: a real key-up event. It is
used, but it is not TRUSTED — a key-up that never arrives (closed lid, killed
tab, dropped Wi-Fi) must not leave the robot driving. So the page re-sends the
COMPLETE set of keys it is holding ~20x/s, and this node refreshes the parent's
key timestamps from that set. Stop hearing from the page and the timestamps go
stale on their own, which is precisely the condition the parent already stops
on (SR-3). A key-up therefore only makes the stop FASTER; its absence cannot
make the stop fail to happen.

Three further rules the terminal did not need:

* One session drives, the rest watch. Two tabs refreshing competing key sets
  would fight over the dead-man. The emergency stop is exempt — any open page
  may stop the robot, holder or not.
* The emergency stop latches. `center()` clears the held keys, but the driving
  page would re-assert them on its next beat 50 ms later and drive straight
  back out of the stop. So after a stop, input stays ignored until the page
  reports an EMPTY key set — i.e. until the operator has physically let go.
* The listening socket binds to localhost by default. This page drives a
  robot; putting it on 0.0.0.0 is a deliberate act, and the node says so in
  the log when you do it.
"""
import collections
import json
import math
import threading
import time
import webbrowser

import rclpy
from rclpy.signals import SignalHandlerOptions
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import String

from gripperx_teleop.keyboard_teleop_node import (
    ALL_KEYS, KeyboardTeleopNode, _signal_desk_quit, _spin, log_stop_signals,
    route_stop_signals, stop_spinning,
)
from gripperx_teleop.manoeuvre import HUMAN_LABEL
from gripperx_teleop.web_server import CameraRelay, TeleopWebServer

# Keys the page may report as held. Anything else is dropped rather than
# forwarded to press(), which would raise a KeyError inside the tick.
HOLDABLE_KEYS = frozenset(ALL_KEYS)

# One-shot operator actions. Held keys are a set; these are edges, and the page
# sends each exactly once.
_OBSERVER_ACTIONS = frozenset({'estop'})

EVENT_LOG_DEPTH = 60


# Shown (line under the keys + banner, via the align-state display) when a J finds no
# leader_arm_node: before 2026-10-16 the request went nowhere and the operator saw nothing
# until a second J logged "leader alignment CANCELLED".
LEADER_UNAVAILABLE = ('unavailable: leader process not running - start it '
                      '(gripperx_desk.sh starts it when the leader is plugged in)')


YAW_ALARM_TOPIC = '/localization/yaw_alarm'
# odom_divergence_monitor republishes every 0.5 s; this many seconds without a
# message mean the monitor is absent, never "OK".
YAW_ALARM_SILENT_SEC = 3.0


# live states in which the cross-check cannot judge: "OK" must not be shown for them.
YAW_BLIND_STATES = frozenset({
    'IMU_SILENT', 'WHEEL_SILENT', 'BOTH_SILENT', 'NO_MOUNT', 'NO_DATA', 'WARMUP',
    'INTERMITTENT'})


def yaw_alarm_view(data, age_sec, silent_sec=YAW_ALARM_SILENT_SEC):
    """Page state of the two-stage yaw-rate divergence alarm (user decisions 2026-11-05, D17 2026-11-13).

    data: last message text (JSON) or None; age_sec: seconds since it arrived (None if
    never). -> dict(state='alarm'|'warn'|'ok'|'unknown'|'silent', episodes, first_utc,
    live_state, warning_episodes, live).
    'alarm' = the latched ERROR stage; `live` then says whether the divergence still
    persists ('persists': live WARN / DIVERGED, or DIVERGED_HELD = ended less than the
    monitor's hold ago), has 'cleared' (live OK) or cannot be told ('unknown': monitor
    silent or blind). 'warn' = the non-latched WARNING stage, shown only while it lasts.
    'unknown': the monitor speaks but is blind (YAW_BLIND_STATES), not latched. A latched alarm
    stays 'alarm' even if the monitor then goes quiet: silence must not hide it.
    """
    msg = None
    if data is not None:
        try:
            msg = json.loads(data)
            if not isinstance(msg, dict):
                msg = None
        except ValueError:
            msg = None
    fresh = msg is not None and age_sec is not None and age_sec <= silent_sec
    live = ''
    if msg is not None and msg.get('latched') is True:
        state = 'alarm'
        ls = msg.get('live_state')
        if not fresh or ls in YAW_BLIND_STATES:
            live = 'unknown'
        elif ls in ('WARN', 'DIVERGED', 'DIVERGED_HELD'):
            live = 'persists'
        elif ls == 'OK':
            live = 'cleared'
        else:
            live = 'unknown'
    elif not fresh:
        state = 'silent'
    elif msg.get('live_state') in YAW_BLIND_STATES:
        state = 'unknown'
    elif msg.get('live_state') in ('WARN', 'DIVERGED') or msg.get('warning') is True:
        state = 'warn'
    else:
        state = 'ok'
    msg = msg or {}
    return {'state': state, 'episodes': msg.get('episodes'),
            'first_utc': msg.get('first_episode_utc'),
            'live_state': msg.get('live_state') or '',
            'warning_episodes': msg.get('warning_episodes'), 'live': live}


def wall_fault(text: str) -> bool:
    """A leader wall state that is an alarm: a wall fault (monitor, read/write error)
    or a torque-off not confirmed (leader_arm_node wall texts)."""
    return 'wall fault' in text or 'NOT confirmed' in text


# DriveFault REASON_* codes, as labels (the message's own constants are the
# source; this is only for the last_cleared_reason code, which has no text field).
DRIVE_FAULT_LABELS = {
    0: 'NONE', 1: 'ENCODER_MISSING', 2: 'STALL_RETRIP', 3: 'STALL_TRIP_COUNT',
    4: 'STALL_MULTI_WHEEL', 5: 'STALL_LATCHED_HELD', 6: 'FEEDBACK_LOST',
}
# Reasons for which the page OFFERS the clear button (SR-18 amendment 2026-11-09,
# D1). Display only: the controller's fixed list (drive_fault_monitor.hpp) decides,
# and the page shows the service's answer -- a refusal included.
CLEARABLE_DRIVE_FAULT_REASONS = frozenset({2, 3, 4, 5})
# Event the page sends to request the clear:
# 'clear_drive_fault:<reason code shown>:<fault_id shown>'. Both values come from
# the page and are range-checked (parse_clear_event) before anything is built.
CLEAR_DRIVE_FAULT_EVENT = 'clear_drive_fault:'
_UINT32_MAX = 2 ** 32 - 1


def parse_clear_event(name: str):
    """'clear_drive_fault:<reason>:<fault_id>' -> (reason, fault_id), or None.

    The reason must be a DriveFault REASON_* code and the fault id a non-zero
    uint32: the page always sends the identity of the fault its dialog showed
    (0, "any", is for the CLI only).
    """
    parts = name[len(CLEAR_DRIVE_FAULT_EVENT):].split(':')
    if len(parts) != 2:
        return None
    try:
        reason, fault_id = int(parts[0]), int(parts[1])
    except ValueError:
        return None
    if reason not in DRIVE_FAULT_LABELS or not 0 < fault_id <= _UINT32_MAX:
        return None
    return reason, fault_id
JOINT_ORDER_LABELS = ('FL', 'FR', 'BL', 'BR')


def drive_fault_other(msg) -> str:
    """The second wheel of a fault (STALL_MULTI_WHEEL: the one already latched), or ''."""
    i = int(msg.other_wheel_index)
    if 0 <= i < len(JOINT_ORDER_LABELS):
        return f'{JOINT_ORDER_LABELS[i]} (joint index {i})'
    return ''


def drive_fault_where(msg) -> str:
    """Where a DriveFault is: 'wheel <name>', or the feedback link for FEEDBACK_LOST.

    FEEDBACK_LOST (reason 6) carries wheel_index -1 and an empty wheel_name on
    purpose -- it is not one wheel -- so no wheel may be named for it.
    """
    if msg.wheel_index >= 0 and msg.wheel_name:
        return f'wheel {msg.wheel_name}'
    if msg.reason_text == 'FEEDBACK_LOST':
        return 'wheel feedback link, all wheels'
    return 'wheel not identified'


class WebTeleopNode(KeyboardTeleopNode):

    def __init__(self):
        super().__init__(node_name='web_teleop_node')

        self.declare_parameter('web_host', '127.0.0.1')
        self.declare_parameter('web_port', 8080)
        self.declare_parameter('web_stream_hz', 20.0)
        # Ride-through window: how long the page may go silent before the
        # robot is stopped. Wi-Fi to this robot is known to be unstable
        # (NFR-8), so a 200 ms hiccup should NOT jolt the robot to a halt --
        # but once this expires the held keys are actively released rather
        # than left to age out, so the stop happens within one publish tick
        # of it. Worst case from link loss to stop is therefore this value
        # plus a tick (~0.52 s), which is inside the terminal node's own
        # worst case of drive_hold_sec (0.6 s). The browser is never slower
        # to stop than the terminal it replaces.
        self.declare_parameter('client_timeout_sec', 0.5)
        # One-shot events older than this (page-side age, see app.js) are dropped:
        # a J/P/I/O must not act long after it was pressed (audit LA1 H1). TO-VERIFY.
        self.declare_parameter('event_max_age_sec', 1.0)
        # How long a J may wait for any /arm/leader/align_state answer before the page
        # says the leader process is not answering and the request is cancelled. The
        # leader node answers every align request at once; a display choice, not a
        # safety value.
        self.declare_parameter('leader_answer_timeout_sec', 3.0)
        # Rate at which the held-key set is re-asserted into the parent's
        # timestamps. Must be well above 1/drive_hold or driving would stutter.
        self.declare_parameter('refresh_rate_hz', 50.0)
        self.declare_parameter('takeover_sec', 2.0)
        self.declare_parameter('open_browser', False)
        self.declare_parameter('active_mode_topic', '/teleop/active_mode')
        # Second source for the MEASURED steering angles, used only when
        # /hw/steer_states has nothing fresh. On the real robot that topic is
        # the servo's own feedback and stays authoritative; in the twin nothing
        # publishes it at all, so without this the wheel view could only ever
        # show the commanded pose and the alignment readout would say "no steer
        # feedback" forever. The joint order is the one used everywhere in this
        # stack -- FL, FR, BL, BR -- and the names are the URDF's, which are
        # NOT consistently spelled (f_left_steer but b_leftsteer), so they are
        # listed rather than derived.
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('steer_joint_names', [
            'f_left_steer', 'f_right_steer', 'b_leftsteer', 'b_rightsteer',
        ])
        self.declare_parameter('arming_state_topic',
                               '/gripperx/external/arming_state')

        self._client_timeout = float(self.get_parameter('client_timeout_sec').value)
        self._event_max_age = float(self.get_parameter('event_max_age_sec').value)
        self._leader_answer_timeout = float(
            self.get_parameter('leader_answer_timeout_sec').value)
        self._align_requested_at = None     # monotonic time of an unanswered J align request
        self._seen_event_ids = collections.deque(maxlen=512)   # (session, id), dedup

        # The tracker starts in the FALLBACK regime because a terminal does not
        # know yet whether it will get real releases. A browser is not in that
        # position: keyup exists, this front-end acts on it, and the beat
        # protocol carries the complete held set so a missed release is
        # corrected on the next beat 50 ms later rather than waited out. So the
        # upgrade the terminal reader performs after negotiating with the
        # terminal is done here at construction instead.
        #
        # The ceiling does not move. held() still requires an event inside
        # drive_hold_sec, so a browser that dies without sending anything stops
        # the robot exactly as before -- releases only ever make the stop
        # earlier, never later.
        self._keys.release_reporting = True

        # ── Snapshot state (written on the ROS thread, read by HTTP threads) ──
        self._web_lock = threading.Lock()
        self._snap = {}
        self._events = []
        self._event_seq = 0
        # Event ids restart at 1 with every node start; a page left open across a
        # restart kept its high last-seen id and silently skipped every new event
        # (align refusals among them, 2026-10-15). The page resets on a new epoch.
        self._events_epoch = f'{time.time():.6f}'
        self._align_state_rx = None         # monotonic receive time of a FRESH align state
        self._align_seen = False            # any align state received yet (replay filter)
        # Leader virtual wall (2026-10-15): last /arm/leader/wall_state text, same replay
        # filter as the align state.
        self._wall_state = ''
        self._wall_state_rx = None
        self._wall_seen = False
        # Whole-drive fault (2026-10-16): last /swerve_controller/drive_fault, or None.
        self._drive_fault = None
        # Operator clear (SR-18 amendment 2026-11-09): service client, the
        # request in flight, and the last answer shown on the page.
        self._clear_client = None
        self._clear_srv_type = None
        self._clear_pending = False
        self._clear_result = None
        # Latched yaw-rate divergence alarm (2026-11-05): last message text and arrival.
        self._yaw_alarm_data = None
        self._yaw_alarm_rx = None

        # ── Operator input state ─────────────────────────────────────────────
        self._input_lock = threading.Lock()
        # Shutdown gate for page input, see begin_ending(). Both under _input_lock.
        self._input_closed = False
        self._input_busy = 0          # _on_client_input() calls in flight
        self._input_idle = threading.Condition(self._input_lock)
        self._client_keys = frozenset()
        self._client_seen = 0.0
        self._client_session = None
        # Set by the emergency stop, cleared when the driving page reports that
        # every key has been released. See the module docstring.
        self._estop_latched = False
        self._link_lost = False
        self._leader_link_dropped = False   # leader/alignment already ended on link loss
        # Audit LA2 T4: a POST that hung in TCP arrives after the link was declared lost,
        # with a fresh-looking page-side age. The first beat after a declared loss
        # therefore carries no one-shot except Space.
        self._drop_next_oneshots = False

        self._active_mode = None
        self._gate = None
        # Last psi and when it last moved discontinuously. See _observe().
        self._last_psi = None
        self._psi_jump_at = 0.0
        self._joint_steer = None
        self._joint_steer_t = 0.0
        self._measured_source = None
        self.stop_event = threading.Event()
        # Set only by _quit() (the UI's Shift+Q / quit button), never by an
        # external shutdown. main() reads it AFTER center() has published to
        # decide whether to tell gripperx_desk.sh to tear the whole desk
        # session down -- see _signal_desk_quit() in keyboard_teleop_node.py.
        self._deliberate_quit = False

        self.create_subscription(
            String,
            str(self.get_parameter('active_mode_topic').value),
            self._on_active_mode,
            10,
        )
        self._subscribe_arming_state()
        self._subscribe_wall_state()
        self._subscribe_drive_fault()
        self._subscribe_yaw_alarm()

        self._steer_joint_names = [
            str(n) for n in self.get_parameter('steer_joint_names').value
        ]
        self.create_subscription(
            JointState,
            str(self.get_parameter('joint_states_topic').value),
            self._on_joint_states,
            10,
        )

        self._init_camera()

        self._server = TeleopWebServer(
            camera=self._camera,
            camera_profile=self.request_camera_profile,
            sink=self._on_client_input,
            snapshot=self._telemetry,
            host=str(self.get_parameter('web_host').value),
            port=int(self.get_parameter('web_port').value),
            stream_hz=float(self.get_parameter('web_stream_hz').value),
            takeover_sec=float(self.get_parameter('takeover_sec').value),
            logger=self.get_logger(),
        )

        refresh_hz = float(self.get_parameter('refresh_rate_hz').value)
        self.create_timer(1.0 / refresh_hz, self._refresh_held_keys)
        self.create_timer(0.25, self._check_leader_answer)

    # ── Camera tile ──────────────────────────────────────────────────────────
    # The image subscription exists only while a browser streams (CameraRelay
    # counts the viewers); with none, this node generates no camera DDS traffic
    # and the camera node itself stops capturing. Nothing here may block: the
    # parameter service is only ever called asynchronously.

    CAMERA_PARAM_TIMEOUT_SEC = 3.0

    def _init_camera(self):
        self.declare_parameter('camera_image_topic', '/camera/front/image_raw/compressed')
        self.declare_parameter('camera_node', '/camera/front/camera_front')
        self.declare_parameter('camera_stale_sec', 1.0)
        self._camera_topic = str(self.get_parameter('camera_image_topic').value)
        node = str(self.get_parameter('camera_node').value).rstrip('/')
        self._camera_profiles = []           # profile_names, read from the camera node
        self._camera_stale = float(self.get_parameter('camera_stale_sec').value)
        self._camera = CameraRelay(self._camera_subscribe, self._camera_unsubscribe)
        self._camera_sub = None
        self._camera_lock = threading.Lock()
        self._camera_profile_now = None      # last known/confirmed profile of the camera node
        self._camera_note = ''               # last parameter result, for the tile
        self._camera_pending = None          # (profile, future, t0)
        self._camera_read = None             # (future, t0) of the GetParameters
        group = MutuallyExclusiveCallbackGroup()
        self._camera_set = self.create_client(
            SetParameters, node + '/set_parameters', callback_group=group)
        self._camera_get = self.create_client(
            GetParameters, node + '/get_parameters', callback_group=group)
        self.create_timer(0.5, self._camera_watchdog, callback_group=group)

    def _camera_subscribe(self):
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._camera_sub = self.create_subscription(
            CompressedImage, self._camera_topic, self._on_camera_frame, qos,
            callback_group=MutuallyExclusiveCallbackGroup())
        self.event('camera view on', 'info')
        if self._camera_get.service_is_ready():
            req = GetParameters.Request()
            req.names = ['profile', 'profile_names']
            future = self._camera_get.call_async(req)
            with self._camera_lock:
                self._camera_read = (future, time.monotonic())
            future.add_done_callback(self._on_camera_profile_read)

    def _camera_unsubscribe(self):
        sub, self._camera_sub = self._camera_sub, None
        if sub is not None:
            self.destroy_subscription(sub)
        self.event('camera view off (subscription closed)', 'info')

    def _on_camera_frame(self, msg):
        self._camera.push(bytes(msg.data))

    def _on_camera_profile_read(self, future):
        with self._camera_lock:
            self._camera_read = None
        try:
            values = future.result().values
            if values and values[0].type == ParameterType.PARAMETER_STRING:
                self._camera_profile_now = values[0].string_value
            if len(values) > 1 and values[1].type == ParameterType.PARAMETER_STRING_ARRAY:
                self._camera_profiles = list(values[1].string_array_value)
        except Exception:  # noqa: BLE001 -- display only
            pass

    def request_camera_profile(self, name: str) -> dict:
        """Start a profile change on the camera node; never blocks."""
        # Any name is passed on: the camera node owns the list and refuses unknown ones.
        if not name or len(name) > 32:
            return {'ok': False, 'detail': 'bad profile name'}
        if not self._camera_set.service_is_ready():
            self._camera_note = 'camera node not running (no parameter service)'
            self.event(f'camera profile {name}: {self._camera_note}', 'warn')
            return {'ok': False, 'detail': self._camera_note}
        with self._camera_lock:
            if self._camera_pending is not None:
                return {'ok': False, 'detail': 'a profile change is still pending'}
            req = SetParameters.Request()
            req.parameters = [ParameterMsg(
                name='profile',
                value=ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=name))]
            future = self._camera_set.call_async(req)
            self._camera_pending = (name, future, time.monotonic())
        future.add_done_callback(self._on_camera_profile_set)
        self._camera_note = f'switching to {name} ...'
        return {'ok': True, 'pending': True}

    def _on_camera_profile_set(self, future):
        with self._camera_lock:
            pending, self._camera_pending = self._camera_pending, None
        if pending is None:
            return  # already timed out
        name = pending[0]
        try:
            result = future.result().results[0]
            ok, reason = result.successful, result.reason
        except Exception as exc:  # noqa: BLE001
            ok, reason = False, str(exc)
        if ok:
            self._camera_profile_now = name
            self._camera_note = f'profile {name} set'
            self.event(f'camera profile {name} set', 'ok')
        else:
            self._camera_note = f'profile {name} refused: {reason}'
            self.event(f'camera profile {name} refused: {reason}', 'warn')

    def _camera_watchdog(self):
        now = time.monotonic()
        limit = self.CAMERA_PARAM_TIMEOUT_SEC
        with self._camera_lock:
            read, pending = self._camera_read, self._camera_pending
            if read is not None and now - read[1] < limit:
                read = None
            elif read is not None:
                self._camera_read = None
            if pending is not None and now - pending[2] < limit:
                pending = None
            elif pending is not None:
                self._camera_pending = None
        if read is not None:
            self._camera_get.remove_pending_request(read[0])
        if pending is not None:
            self._camera_set.remove_pending_request(pending[1])
            self._camera_note = f'profile {pending[0]}: no answer from the camera node'
            self.event(self._camera_note, 'warn')

    def _camera_view(self):
        st = self._camera.status()
        view = {'on': st['viewers'] > 0, **st,
                'profile': self._camera_profile_now, 'note': self._camera_note,
                'profiles': self._camera_profiles}
        if view['on']:
            node_up = self._camera_set.service_is_ready()
            view['node'] = node_up
            publishers = self.count_publishers(self._camera_topic)
            view['publishers'] = publishers
            age = st['age_sec']
            if publishers == 0:
                view['state'] = 'no camera node'
            elif age is None:
                view['state'] = 'waiting for frames'
            elif age > self._camera_stale:
                view['state'] = 'stale'
            else:
                view['state'] = 'live'
        else:
            view['state'] = 'off'
        return view

    # ── Optional subscriptions ───────────────────────────────────────────────

    def _subscribe_arming_state(self):
        """External authority gate state, if that package is even installed.

        The gate is an optional part of the system (Octopus link). Teleop must
        start on a robot that has never had it built, so an import failure is a
        note in the log, not a startup failure.
        """
        try:
            from gripperx_external_msgs.msg import ArmingState
        except ImportError:
            self.get_logger().info(
                'gripperx_external_msgs not available — the UI will show the '
                'external gate as unknown (U/L still send their service call)'
            )
            return
        from rclpy.qos import (
            DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
        )
        self.create_subscription(
            ArmingState,
            str(self.get_parameter('arming_state_topic').value),
            self._on_arming_state,
            # Latched, matching the gateway's publisher: a page opened after
            # the gate was armed must still learn that it is armed.
            QoSProfile(
                depth=1,
                history=HistoryPolicy.KEEP_LAST,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

    def _subscribe_wall_state(self):
        from rclpy.qos import (
            DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
        )
        from std_msgs.msg import String
        self.create_subscription(
            String, '/arm/leader/wall_state', self._on_wall_state,
            # transient_local, matching leader_arm_node's publisher.
            QoSProfile(
                depth=1,
                history=HistoryPolicy.KEEP_LAST,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

    def _subscribe_drive_fault(self):
        """Whole-drive fault from swerve_controller (user decision 2026-10-16).

        Optional like the arming state: a teleop built without
        gripperx_control_msgs still starts, and says the fault is not shown.
        """
        try:
            from gripperx_control_msgs.msg import DriveFault
        except ImportError:
            self.get_logger().warn(
                'gripperx_control_msgs not available -- the UI CANNOT show the '
                'whole-drive fault (/swerve_controller/drive_fault)')
            return
        from rclpy.qos import (
            DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
        )
        try:
            from gripperx_control_msgs.srv import ClearDriveFault
        except ImportError:
            self.get_logger().warn(
                'gripperx_control_msgs has no ClearDriveFault -- the UI CANNOT clear a '
                'drive fault (/swerve_controller/clear_drive_fault)')
        else:
            self._clear_srv_type = ClearDriveFault
            self._clear_client = self.create_client(
                ClearDriveFault, '/swerve_controller/clear_drive_fault')
        self.create_subscription(
            DriveFault, '/swerve_controller/drive_fault', self._on_drive_fault,
            # Latched, matching swerve_controller: a page opened after the trip
            # must still show it.
            QoSProfile(
                depth=1,
                history=HistoryPolicy.KEEP_LAST,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

    def _subscribe_yaw_alarm(self):
        from rclpy.qos import (
            DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
        )
        self.create_subscription(
            String, YAW_ALARM_TOPIC, self._on_yaw_alarm,
            QoSProfile(
                depth=1,
                history=HistoryPolicy.KEEP_LAST,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

    def _on_yaw_alarm(self, msg):
        was = yaw_alarm_view(self._yaw_alarm_data, 0.0)['state'] == 'alarm'
        self._yaw_alarm_data = msg.data
        self._yaw_alarm_rx = time.monotonic()
        view = yaw_alarm_view(msg.data, 0.0)
        if view['state'] == 'alarm' and not was:
            self.event('YAW-RATE DIVERGENCE ALARM LATCHED: stop, restart the stack before '
                       'moving again (hardware switch S1).', 'alarm')

    def _on_drive_fault(self, msg):
        was = bool(self._drive_fault is not None and self._drive_fault.active)
        self._drive_fault = msg
        if msg.active and not was:
            until = ('until the stack restarts or an operator clears it'
                     if int(msg.reason) in CLEARABLE_DRIVE_FAULT_REASONS
                     else 'until the stack restarts (Mode R)')
            self.event(
                f'DRIVE FAULT ({msg.reason_text}, {drive_fault_where(msg)}): all four '
                f'drive motors held at zero {until}. {msg.detail}',
                'alarm')
        elif was and not msg.active:
            requester = getattr(msg, 'last_clear_requester', '') or 'unknown'
            self.event(f'drive fault cleared (by {requester}): the drive is released -- '
                       'Nav2 goals cancelled by the fault are NOT resumed', 'ok')

    def request_drive_fault_clear(self, expected_reason: int, expected_fault_id: int = 0):
        """Call /swerve_controller/clear_drive_fault for the reason the dialog showed.

        Non-blocking: the answer arrives in _on_clear_response and is shown on the
        page (event log and the line under the fault). A clear releases the drive:
        a held drive command (from anywhere) drives off at once (D2).
        """
        if self._clear_client is None:
            self._clear_result = {'success': False, 'pending': False,
                                  'message': 'clear service type not available in this build'}
            self.event('drive fault clear NOT sent: no ClearDriveFault in this build', 'alarm')
            return False
        if self._clear_pending:
            self.event('drive fault clear: a request is already in flight', 'alarm')
            return False
        if not self._clear_client.service_is_ready():
            self._clear_result = {'success': False, 'pending': False,
                                  'message': '/swerve_controller/clear_drive_fault not available'}
            self.event('drive fault clear NOT sent: /swerve_controller/clear_drive_fault '
                       'not available', 'alarm')
            return False
        request = self._clear_srv_type.Request()
        request.requester = 'web_teleop'
        request.expected_reason = int(expected_reason)
        request.expected_fault_id = int(expected_fault_id)
        self._clear_pending = True
        self._clear_result = {'success': None, 'pending': True, 'message': 'clear requested...'}
        future = self._clear_client.call_async(request)
        future.add_done_callback(self._on_clear_response)
        self.event(f'drive fault clear requested (expected reason '
                   f'{DRIVE_FAULT_LABELS.get(int(expected_reason), expected_reason)})', 'info')
        return True

    def _on_clear_response(self, future):
        self._clear_pending = False
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001 -- show it, never crash the page
            self._clear_result = {'success': False, 'pending': False,
                                  'message': f'clear call failed: {exc}'}
            self.event(f'drive fault clear call failed: {exc}', 'alarm')
            return
        self._clear_result = {'success': bool(response.success), 'pending': False,
                              'message': str(response.message)}
        if response.success:
            self.event(f'drive fault clear ACCEPTED: {response.message}', 'ok')
        else:
            self.event(f'drive fault clear REJECTED: {response.message}', 'alarm')

    def _drive_fault_view(self):
        """The telemetry fields of the clear (button, dialog, last clear, answer)."""
        f = self._drive_fault
        active = bool(f is not None and f.active)
        reason = int(f.reason) if f is not None else 0
        count = int(getattr(f, 'clear_count', 0)) if f is not None else 0
        last = None
        if count > 0:
            stamp = f.last_clear_stamp
            last = {
                'stamp_sec': stamp.sec + stamp.nanosec * 1e-9,
                'reason': DRIVE_FAULT_LABELS.get(int(f.last_cleared_reason),
                                                 str(f.last_cleared_reason)),
                'requester': f.last_clear_requester or 'unspecified',
            }
        return {
            'drive_fault_reason_code': reason if active else 0,
            'drive_fault_id': int(getattr(f, 'fault_id', 0)) if active else 0,
            'drive_fault_clearable': active and reason in CLEARABLE_DRIVE_FAULT_REASONS,
            'drive_fault_other': drive_fault_other(f) if active else '',
            'drive_fault_clear_count': count,
            'drive_fault_last_clear': last,
            'drive_fault_clear_result': self._clear_result,
        }

    def _on_wall_state(self, msg):
        # The first message may be the transient_local replay of a state published
        # before this node started: shown under the keys, but no banner age and no
        # alarm event. Only releases go to the event log; held/outside changes are on
        # the line under the keys (they can change every few ticks). No rclpy logging
        # here: the page is the display.
        replay = not self._wall_seen
        self._wall_seen = True
        self._wall_state = msg.data
        if replay:
            if msg.data.startswith('released') or wall_fault(msg.data):
                self.event(f'leader wall (last state, from before this node started): '
                           f'{msg.data}', 'info')
            return
        self._wall_state_rx = time.monotonic()
        if wall_fault(msg.data):
            self.event(f'leader wall: {msg.data}', 'alarm')
        elif msg.data.startswith('released'):
            self.event(f'leader wall: {msg.data}', 'info')

    def _on_joint_states(self, msg):
        """Pick the four steering joints out, in the stack's joint order.

        By NAME, never by index: /joint_states interleaves steer and wheel
        joints and its order is whatever the broadcaster produced. An index
        assumption here would show a wheel's accumulated rotation as a steering
        angle and look almost plausible while doing it.
        """
        try:
            angles = [msg.position[msg.name.index(n)]
                      for n in self._steer_joint_names]
        except (ValueError, IndexError):
            return          # a joint is missing -- leave the previous sample
        with self._lock:
            self._joint_steer = [float(a) for a in angles]
            self._joint_steer_t = time.monotonic()

    def _measured_steer(self, now):
        """Measured steering angles and where they came from.

        /hw/steer_states first, always: on the real robot it is the servo's own
        feedback, and it is the only one of the two that can disagree with what
        ros2_control believes. Falling back the other way round would hide
        exactly the fault the alignment readout exists to show.
        """
        hardware = self._fresh_steer_states(now)
        if hardware is not None:
            return hardware, 'hw'
        with self._lock:
            fresh = (
                self._joint_steer is not None
                and (now - self._joint_steer_t) <= self._steer_states_timeout
            )
            if fresh:
                return list(self._joint_steer), 'joint_states'
        return None, None

    def _on_active_mode(self, msg: String):
        with self._web_lock:
            self._active_mode = msg.data

    def _on_arming_state(self, msg):
        with self._web_lock:
            self._gate = {
                'armed': bool(msg.armed),
                'seconds_remaining': float(msg.seconds_remaining),
                'armed_by': str(msg.armed_by),
                'last_disarm_detail': str(msg.last_disarm_detail),
            }

    # ── Operator feedback (replaces the parent's raw-tty writes) ─────────────

    def _announce(self, status: str):
        """Same contract as the parent, without the terminal escape codes.

        The parent writes the status line straight to a raw tty. There is no
        raw tty here, so those bytes would land in the ROS log as visible
        control characters. The topic publish and the log line — the parts
        other subscribers depend on — are kept exactly as they were.
        """
        with self._lock:
            if status == self._status_line:
                return
            previous = self._status_line
            self._status_line = status

        msg = String()
        msg.data = status
        self._manoeuvre_pub.publish(msg)
        if previous.split('|')[0] != status.split('|')[0]:
            self.get_logger().info(f'Manoeuvre: {status}')
            self.event(status.split('|')[0].strip(), 'manoeuvre')

    def event(self, text: str, level: str = 'info'):
        """Append to the ring buffer the page renders as its event log."""
        with self._web_lock:
            self._event_seq += 1
            self._events.append({
                'id': self._event_seq,
                'level': level,
                'text': text,
                'wall': time.strftime('%H:%M:%S'),
            })
            del self._events[:-EVENT_LOG_DEPTH]

    # ── Observation hook: what the parent just published ─────────────────────

    def _observe(self, manoeuvre, steer_angle, target, cmd, armed, pose_on, status):
        now = time.monotonic()
        self._note_psi_jump(now)
        measured, source = self._measured_steer(now)
        self._measured_source = source
        with self._web_lock:
            self._snap = {
                'manoeuvre': manoeuvre,
                'manoeuvre_label': HUMAN_LABEL.get(manoeuvre, manoeuvre),
                'status': status,
                'guard_state': self._guard.state,
                'drive_allowed': bool(armed),
                'pose_commanded': bool(pose_on),
                'pose_reachable': bool(self._guard.pose_reachable),
                'armed_without_feedback': bool(self._guard.armed_without_feedback),
                'steer_deg': math.degrees(steer_angle),
                # Joint order FL, FR, BL, BR throughout — same order the
                # servos, /hw/steer_states and STEER_PATTERN use.
                'target_deg': (
                    [math.degrees(v) for v in target] if target is not None else None
                ),
                'measured_deg': (
                    [math.degrees(v) for v in measured] if measured is not None else None
                ),
                'measured_source': source,
                'cmd': {
                    'vx': cmd.linear.x,
                    'vy': cmd.linear.y,
                    'wz': cmd.angular.z,
                },
                # Steerable crab. psi is the DIRECTION OF TRAVEL, not a wheel
                # angle: arrow up/down rotate it, and it is None whenever no
                # crab is active -- which is not the same as 0.0, because 0.0
                # is a legitimate heading (straight ahead). Read here rather
                # than threaded through _observe(), so the parent's tick
                # signature stays what the terminal node defines.
                'crab_psi_deg': (
                    None if self._crab_psi is None
                    else math.degrees(self._crab_psi)
                ),
                'crab_psi_snap': bool(self._crab_psi_snap),
                'psi_jump_ago_sec': (
                    None if self._psi_jump_at <= 0.0
                    else time.monotonic() - self._psi_jump_at
                ),
            }

    def _note_psi_jump(self, now):
        """Notice when the crab heading crosses a dead band, and say so.

        Steering a crab is continuous only inside the reachable arcs; between
        them psi JUMPS the dead-band gap (crab_psi_snap), modules swinging with
        traction still on. The teleop TransitionGuard does not cover this -- it
        withholds drive on a change of MANOEUVRE, and a steered crab stays
        crab_left throughout. swerve_controller's alignment gate is meant to
        cover it, but ships DISABLED by default (ros2_controllers.yaml).

        Not a defect to fix here -- a regime the operator must be able to see.
        psi moves a couple of degrees per tick, so anything above 20 deg in one
        tick is a gap crossing and nothing else.
        """
        psi = self._crab_psi
        if psi is None:
            self._last_psi = None
            return
        previous = self._last_psi
        self._last_psi = psi
        if previous is not None and abs(psi - previous) > math.radians(20.0):
            self._psi_jump_at = now
            self.event(
                f'crab heading jumped {math.degrees(abs(psi - previous)):.0f}° '
                'across a dead band — modules swinging, traction not withheld',
                'warn',
            )

    # ── Telemetry frame ──────────────────────────────────────────────────────

    def _telemetry(self):
        now = time.monotonic()
        with self._input_lock:
            held = sorted(self._client_keys)
            silent = now - self._client_seen
            fresh = self._client_seen > 0.0 and silent <= self._client_timeout
            estop = self._estop_latched
            session = self._client_session
        with self._web_lock:
            frame = dict(self._snap)
            frame['events'] = list(self._events)
            frame['mode'] = self._active_mode
            frame['gate'] = self._gate
        frame['camera'] = self._camera_view()
        frame.update({
            'held': held if fresh else [],
            'link_fresh': fresh,
            'silent_sec': round(silent, 3) if self._client_seen > 0.0 else None,
            'estop_latched': estop,
            # The real state, mirrored from the /arm/leader/enable echo.
            'leader_enabled': bool(self._leader_enabled),
            # Self-alignment (2026-10-13): last /arm/leader/align_state text.
            'leader_aligning': bool(self._align_pending),
            'leader_align_state': self._align_state,
            # Age of that text, so the page can raise a refusal/abort in the banner
            # for a few seconds instead of only in the event log at the page bottom.
            'leader_align_age_sec': (None if self._align_state_rx is None
                                     else round(now - self._align_state_rx, 2)),
            # Leader virtual wall (2026-10-15): last /arm/leader/wall_state text.
            'leader_wall_state': self._wall_state,
            'leader_wall_age_sec': (None if self._wall_state_rx is None
                                    else round(now - self._wall_state_rx, 2)),
            'events_epoch': self._events_epoch,
            # Whole-drive fault (2026-10-16), latched until the stack restarts.
            'drive_fault_active': bool(self._drive_fault is not None
                                       and self._drive_fault.active),
            'drive_fault_reason': (self._drive_fault.reason_text
                                   if self._drive_fault is not None else ''),
            'drive_fault_wheel': (self._drive_fault.wheel_name
                                  if self._drive_fault is not None else ''),
            # Display text: 'wheel <name>' or, for FEEDBACK_LOST, the feedback link.
            'drive_fault_where': (drive_fault_where(self._drive_fault)
                                  if self._drive_fault is not None else ''),
            'drive_fault_detail': (self._drive_fault.detail
                                   if self._drive_fault is not None else ''),
            **self._drive_fault_view(),
            **{'yaw_alarm_' + k: v for k, v in yaw_alarm_view(
                self._yaw_alarm_data,
                None if self._yaw_alarm_rx is None else now - self._yaw_alarm_rx).items()},
            'driver': session,
            'rivals': list(self._rivals),
            'limits': {
                'steer_limit_deg': math.degrees(self._limit),
                # A/D is MOMENTARY: straight ahead is the resting state, and
                # the angle springs back to exactly 0 when neither key is
                # held. The page has to say which of the two models it is
                # showing, because the difference is invisible in a still
                # picture and decisive in a moving one.
                'steer_rate_deg_s': math.degrees(self._rate),
                'steer_return_rate_deg_s': math.degrees(self._return_rate),
                'linear_vel_m_s': self._lin_vel,
                'crab_speed_m_s': self._crab_speed,
                'spin_speed_rad_s': self._spin_speed,
                'drive_hold_sec': self._drive_hold,
                # What held() is ACTUALLY using right now. With releases
                # reported this is the ceiling; the terminal's fallback regime
                # measures the repeat interval instead and lands well below it.
                'deadman_window_sec': self._keys.window('w'),
                'align_tolerance_deg': math.degrees(self._guard.align_tolerance_rad),
            },
            'geometry': {
                'a': self._model.a,
                'b': self._model.b,
                'wheel_radius': self._model.wheel_radius,
            },
            # Which headings a pure translation can actually reach, straight
            # out of the same SteeringLimits the pose resolution uses. The page
            # draws the gaps between them as what they are -- four 45 deg bands
            # where no pose exists -- instead of letting the operator discover
            # them by steering into one.
            'translation_arcs_deg': [
                [math.degrees(low), math.degrees(high)]
                for low, high in self._translation_arcs
            ],
        })
        return frame

    # ── Input from the page ──────────────────────────────────────────────────

    def _accept_event(self, session, raw):
        """'name|id|age_ms' from the page -> name, or None for a duplicate (same id
        from the same page), a stale event, or one without id/age (rejected outright,
        coordinator decision 2026-10-13). Space is accepted in any form, never dropped."""
        parts = str(raw).split('|')
        name = parts[0]
        if name == 'estop':
            return name
        if len(parts) != 3:
            self.get_logger().warn(f'web event {raw!r} without id/age: rejected')
            return None
        try:
            event_id, age = parts[1], float(parts[2]) / 1000.0
        except ValueError:
            return None
        key = (session, event_id)
        if key in self._seen_event_ids:
            self.get_logger().warn(f'web event {name} #{event_id}: duplicate, dropped')
            return None
        self._seen_event_ids.append(key)
        if age > self._event_max_age:
            self.get_logger().warn(f'web event {name}: {age:.2f} s old, dropped '
                                   f'(event_max_age_sec {self._event_max_age})')
            self.event(f'{name}: arrived {age:.1f} s late - dropped, press again', 'alarm')
            return None
        return name

    def _on_client_input(self, session, keys, events, has_control):
        with self._input_lock:
            closed = self._input_closed
            if not closed:
                self._input_busy += 1
        if closed:
            # Shutting down: an HTTP/1.1 keep-alive connection outlives
            # stop_server(), so input still arrives here. Only the e-stop is
            # honoured (it can only stop); held keys and every other one-shot
            # (mode, pick, home, leader, gate) are dropped.
            if any(str(raw).split('|')[0] == 'estop' for raw in events):
                self._do_estop()
            return
        try:
            self._handle_client_input(session, keys, events, has_control)
        finally:
            with self._input_idle:
                self._input_busy -= 1
                self._input_idle.notify_all()

    def _handle_client_input(self, session, keys, events, has_control):
        now = time.monotonic()
        wanted = frozenset(k for k in keys if k in HOLDABLE_KEYS)
        with self._input_lock:
            after_loss = self._drop_next_oneshots and has_control
            if after_loss:
                self._drop_next_oneshots = False

        for raw in events:
            name = self._accept_event(session, raw)
            if name is None:
                continue
            if after_loss and name != 'estop':
                self.get_logger().warn(f'web event {name}: first beat after a link loss, dropped')
                self.event(f'{name}: arrived after the link was lost - dropped, press again',
                           'alarm')
                continue
            if name == 'estop':
                # Never gated on holding control: anybody watching may stop.
                self._do_estop()
            elif not has_control:
                continue
            elif name in _OBSERVER_ACTIONS:
                continue
            else:
                self._dispatch(name)

        if not has_control:
            return

        with self._input_lock:
            if self._estop_latched:
                if wanted:
                    # Still holding something after the stop — keep ignoring
                    # input, and keep saying so on the page.
                    wanted = frozenset()
                else:
                    self._estop_latched = False
                    self.get_logger().info(
                        'emergency stop cleared — all keys released'
                    )
                    self.event('E-STOP cleared, control returned', 'ok')
            if self._link_lost:
                self._link_lost = False
                self.event('operator link restored', 'ok')
            self._leader_link_dropped = False
            dropped = self._client_keys - wanted
            self._client_keys = wanted
            self._client_seen = now
            self._client_session = session

        # Outside the lock: _release_keys takes the parent's lock, and this
        # module keeps a single lock order (input -> parent/web, never back).
        if dropped:
            self._release_keys(dropped)

    def _release_keys(self, keys=None):
        """Zero the parent's key timestamps -- i.e. let go, now.

        Without this, letting go of W would only stop the robot once the
        timestamp aged out of `drive_hold_sec` (0.6 s). That delay exists in
        the terminal node because a terminal has no key-up event to work with
        and must infer the release from key repeat stopping. A browser does
        have one, so a deliberate release should be acted on immediately --
        the dead-man window stays as the backstop for the releases that never
        arrive, which is what it was always for.
        """
        now = time.monotonic()
        with self._lock:
            if keys is None:
                self._keys.clear_all(now)
            else:
                for key in keys:
                    # clear() rather than an on_event(RELEASE): the tracker
                    # distinguishes "the operator let go" from "this code is
                    # letting go on their behalf", and a link timeout is the
                    # second one.
                    self._keys.clear(key, now)

    def _on_align_state(self, msg):
        # The first message may be the transient_local replay of a state published
        # before this node started (audit LA4 L4): it updates the visible line but is
        # no fresh notice - no banner age, an info event - unless a J of this node is
        # waiting for its answer.
        self._align_requested_at = None     # any answer: the leader process is there
        requested = bool(self._align_pending)
        replay = not self._align_seen and not requested
        self._align_seen = True
        super()._on_align_state(msg)
        if msg.data.startswith('aligning'):
            # The leader is about to move itself: a wall-fault notice still within its
            # banner time would outrank "LEADER ALIGNING ... Hands clear" (audit LW1b).
            # It stays on the line under the keys and in the event log.
            self._wall_state_rx = None
        if replay:
            self.event(f'leader alignment (last state, from before this node started): '
                       f'{msg.data}', 'info')
            return
        self._align_state_rx = time.monotonic()
        self.event(f'leader alignment: {msg.data}',
                   'alarm' if msg.data.startswith(('aborted', 'refused')) else 'info')

    # ── Leader process presence (2026-10-16) ─────────────────────────────────

    def toggle_leader(self):
        """J, as in the terminal node, plus: an align request that no leader node
        can receive is not sent but shown, and one that gets no answer is cancelled
        and shown (_check_leader_answer)."""
        if not self._leader_enabled and not self._align_pending:
            if self._align_pub.get_subscription_count() == 0:
                self._leader_unavailable('nothing subscribes to /arm/leader/align; '
                                         'J was not sent')
                return
            # Before the publish: the answer can arrive before super() returns.
            self._align_requested_at = time.monotonic()
        super().toggle_leader()

    def disarm_leader(self):
        self._align_requested_at = None
        super().disarm_leader()

    def _check_leader_answer(self):
        start = self._align_requested_at
        if start is None or time.monotonic() - start < self._leader_answer_timeout:
            return
        # Cancel first: a leader node that is merely stalled must not start an
        # alignment the page has already declared dead.
        self.disarm_leader()
        self._leader_unavailable(
            f'no answer on /arm/leader/align_state within '
            f'{self._leader_answer_timeout:.1f} s; request cancelled')

    def _leader_unavailable(self, detail):
        text = f'{LEADER_UNAVAILABLE} ({detail})'
        # Through the align-state display: the line under the keys and, while fresh,
        # the banner. The leader node's own first state replaces it when it starts.
        self._align_state = text
        self._align_state_rx = time.monotonic()
        self.event(f'leader alignment: {text}', 'alarm')
        self.get_logger().warn(f'leader alignment: {text}')

    def _do_estop(self):
        self.center()
        with self._input_lock:
            self._client_keys = frozenset()
            self._estop_latched = True
        self.event('EMERGENCY STOP — release every key to regain control', 'alarm')

    def _dispatch(self, name):
        if self._input_closed:
            return   # shutting down: see begin_ending()
        actions = {
            'mode_keyboard':   (lambda: self.set_mode('keyboard'),
                                'mode → keyboard'),
            'mode_autonomous': (lambda: self.set_mode('autonomous'),
                                'mode → autonomous (Nav2)'),
            'pick':            (self.trigger_pick,    'pick_plastic: sent'),
            'open_gripper':    (self.open_gripper,    'gripper: open'),
            'home':            (self.go_home,         'arm: home position'),
            'leader':          (self.toggle_leader,   'leader: J (align / cancel / off)'),
            'arm_gate':        (self.arm_gateway,     'external gate: ARM requested'),
            'disarm_gate':     (self.disarm_gateway,  'external gate: DISARM requested'),
            'quit':            (self._quit,           'shutdown requested from the UI'),
        }
        if name.startswith(CLEAR_DRIVE_FAULT_EVENT):
            parsed = parse_clear_event(name)
            if parsed is None:
                # Dropped HERE, so the rest of the batch and the held keys still apply.
                self.get_logger().warn(f'web event {name[:80]!r}: malformed or out of range, '
                                       'dropped')
                self.event('drive fault clear NOT sent: malformed request from the page',
                           'alarm')
                return
            self.request_drive_fault_clear(*parsed)
            return
        entry = actions.get(name)
        if entry is None:
            return
        action, text = entry
        if name == 'pick':
            # Announced only once trigger_pick says what happened: it can refuse to
            # send (busy, server unreachable, no stop epoch yet). True/False from
            # trigger_pick; None = a trigger_pick that does not report (unknown).
            sent = action()
            if sent is False:
                self.event('pick_plastic: NOT sent - see the teleop log', 'alarm')
            else:
                self.event(text if sent else 'pick_plastic: requested', 'info')
            return
        self.event(text, 'alarm' if name == 'quit' else 'info')
        action()

    def _quit(self):
        self._deliberate_quit = True
        self.stop_event.set()

    # ── Dead-man refresh ─────────────────────────────────────────────────────

    def _refresh_held_keys(self):
        """Re-assert the page's held keys into the parent's timestamps.

        This is the whole dead-man mechanism. `press()` is what the terminal
        node's key-repeat calls; here the page's beat plays that role. When the
        beats stop, this loop stops calling press(), the timestamps age out of
        `drive_hold_sec` and the parent stops the robot — no message has to
        arrive for the stop to happen.
        """
        now = time.monotonic()
        lost = False
        drop_leader = False
        with self._input_lock:
            if self._input_closed or self._client_seen <= 0.0:
                return
            silent = now - self._client_seen
            if silent > self._client_timeout:
                self._drop_next_oneshots = True
                # Leader mode / a running alignment end with the link (audit LA1 L1).
                drop_leader = ((self._leader_enabled or self._align_pending)
                               and not self._leader_link_dropped)
                if drop_leader:
                    self._leader_link_dropped = True
                lost = bool(self._client_keys) and not self._link_lost
                if lost:
                    self._link_lost = True
                self._client_keys = frozenset()
                keys = None
            else:
                keys = self._client_keys

        if drop_leader:
            self.disarm_leader()
            self.get_logger().warn('operator page silent: leader mode / alignment switched off')
            self.event('operator link lost - leader mode / alignment switched off', 'alarm')
        if keys is None:
            # Do not merely stop refreshing -- actively let go, so the stop
            # lands on the next publish tick instead of one drive_hold later.
            self._release_keys()
            if lost:
                self.get_logger().warn(
                    f'operator page silent for {silent:.2f} s — keys released'
                )
                self.event('operator link lost — keys released', 'alarm')
            return

        for key in keys:
            # press() enforces the mutual exclusion between manoeuvre, drive
            # and steer keys itself, so a page that reports two conflicting
            # keys held cannot produce a state the terminal could not.
            self.press(key)

    def _announce_rivals(self, appeared, rivals):
        """Same warning, put where THIS operator is looking.

        The parent writes a red block into a raw terminal, which this
        front-end does not have. Here it belongs in the event log and, through
        the `rivals` field of the telemetry frame, in a banner that outranks
        every other banner on the page -- including the emergency stop's own,
        because this is the one state in which the stop does not necessarily
        stop the robot.
        """
        if appeared:
            self.event(f'another teleop is running: {", ".join(appeared)}', 'alarm')
        elif not rivals:
            self.event('the other teleop is gone', 'ok')

    # ── Outcome events worth showing on the page ─────────────────────────────

    def _on_pick_done(self, future):
        result = future.result().result
        self.event(
            f'pick_plastic: {"OK" if result.success else "ERROR"} — {result.message}',
            'ok' if result.success else 'alarm',
        )
        super()._on_pick_done(future)

    def _on_arming_response(self, future, arm: bool):
        try:
            response = future.result()
        except Exception:  # noqa: BLE001 — the parent logs the detail
            self.event('set_arming call failed', 'alarm')
        else:
            if response.success:
                self.event(
                    f'external gate {"ARMED" if arm else "DISARMED"} — '
                    f'{response.message}',
                    'ok',
                )
            else:
                self.event(f'set_arming REFUSED — {response.message}', 'alarm')
        super()._on_arming_response(future, arm)

    # ── Lifecycle ────────────────────────────────────────────────────────────

    def start_server(self):
        host = str(self.get_parameter('web_host').value)
        self._server.start()
        url = self._server.url
        self.get_logger().info(f'Teleop UI: {url}')
        if host not in ('127.0.0.1', 'localhost', '::1'):
            self.get_logger().warn(
                f'web_host={host} — the drive controls of this robot are now '
                'reachable from the network. Anyone who can open that URL can '
                'drive it. Use 127.0.0.1 unless that is what you meant.'
            )
        self.event(f'teleop UI serving on {url}', 'ok')
        if bool(self.get_parameter('open_browser').value):
            threading.Thread(
                target=lambda: webbrowser.open(url), daemon=True
            ).start()

    def stop_server(self):
        self._server.stop()

    def begin_ending(self, drain_timeout: float = 0.02):
        """Close page input for good, BEFORE the final center().

        Measured 2026-10-22: a quit that arrives while the page holds W was
        followed by non-zero Twists between the stop and the exit --
        _refresh_held_keys() re-pressed the held keys, and a keep-alive HTTP/1.1
        connection kept delivering new ones even after stop_server(). Now:

        * no new page input is acted on (_on_client_input drops it, e-stop aside),
          and no one-shot dispatches any more (_dispatch() checks the gate);
        * the parent's gate makes press() refuse every key, so a held-key
          refresh that read the keys before this cannot press after it, and
          trigger_pick() refuses a goal once its server wait returns;
        * one-shots already past the _dispatch() check are waited for, so none
          (mode 'autonomous', home, leader) can publish after the stop.

        The wait is short on purpose: with the gates above it only has to cover
        the microseconds between a passed check and a non-blocking publish. The
        one dispatch that blocks -- P, in wait_for_server() for up to 0.5 s with
        no arm server -- is held by the gate in trigger_pick() instead, and must
        not delay the stop (re-audit 2026-10-22: 0.2 s here gave 201.6 ms from
        signal to /arm/stop with a pick in flight).
        """
        with self._input_idle:
            self._input_closed = True
            self._client_keys = frozenset()
            self._client_seen = 0.0
            super().begin_ending()   # lock order input -> parent, as everywhere here
            self._input_idle.wait_for(lambda: self._input_busy == 0, drain_timeout)


def main():
    # Our own signal handling, as in the terminal node (route_stop_signals()):
    # a signal only sets the stop event, so the stop below is published while
    # the context is still up. The early event covers node construction.
    early = threading.Event()
    received = []
    route_stop_signals(early, received)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = WebTeleopNode()
    route_stop_signals(node.stop_event, received)
    if early.is_set():
        node.stop_event.set()
    spin_thread = threading.Thread(
        target=_spin, args=(node, node.stop_event), daemon=True)
    spin_thread.start()
    try:
        node.start_server()
        # The node has no terminal input to block on, so the main thread just
        # waits for a signal, the spin thread ending, or the UI's quit.
        while not node.stop_event.is_set():
            node.stop_event.wait(0.25)
    finally:
        try:
            # Input closed first, then the stop at once. The server goes down
            # only afterwards: shutting it down waits out serve_forever's poll
            # interval, and the stop must not wait for that.
            node.begin_ending()
            log_stop_signals(node, received)
            if rclpy.ok():
                # Same shutdown as the terminal node: stop, straighten, keyboard
                # mode, and give the last publish a moment to leave.
                node.center(reason='shutdown')
                time.sleep(0.15)
                # Only _quit() (the UI's quit button/Shift+Q) sets this; no
                # signal path does, so only a deliberate UI quit tears the
                # desk session down.
                if node._deliberate_quit:
                    _signal_desk_quit()
            else:
                # Not reachable through a signal any more (they no longer take
                # the context down); kept for anything else that might.
                node.get_logger().warn(
                    'context already down — no explicit stop published; '
                    'downstream timeouts are what stop the robot here'
                )
            try:
                node.stop_server()
            except Exception as exc:   # the stop is out; still shut ROS down
                node.get_logger().error(f'stopping the web server failed: {exc!r}')
        finally:
            stop_spinning(node, spin_thread)


if __name__ == '__main__':
    main()
