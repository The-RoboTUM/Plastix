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
import math
import threading
import time
import webbrowser

import rclpy
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from gripperx_teleop.keyboard_teleop_node import ALL_KEYS, KeyboardTeleopNode
from gripperx_teleop.manoeuvre import HUMAN_LABEL
from gripperx_teleop.web_server import TeleopWebServer

# Keys the page may report as held. Anything else is dropped rather than
# forwarded to press(), which would raise a KeyError inside the tick.
HOLDABLE_KEYS = frozenset(ALL_KEYS)

# One-shot operator actions. Held keys are a set; these are edges, and the page
# sends each exactly once.
_OBSERVER_ACTIONS = frozenset({'estop'})

EVENT_LOG_DEPTH = 60


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

        # ── Operator input state ─────────────────────────────────────────────
        self._input_lock = threading.Lock()
        self._client_keys = frozenset()
        self._client_seen = 0.0
        self._client_session = None
        # Set by the emergency stop, cleared when the driving page reports that
        # every key has been released. See the module docstring.
        self._estop_latched = False
        self._link_lost = False

        self._active_mode = None
        self._gate = None
        # Last psi and when it last moved discontinuously. See _observe().
        self._last_psi = None
        self._psi_jump_at = 0.0
        self._joint_steer = None
        self._joint_steer_t = 0.0
        self._measured_source = None
        self.stop_event = threading.Event()

        self.create_subscription(
            String,
            str(self.get_parameter('active_mode_topic').value),
            self._on_active_mode,
            10,
        )
        self._subscribe_arming_state()

        self._steer_joint_names = [
            str(n) for n in self.get_parameter('steer_joint_names').value
        ]
        self.create_subscription(
            JointState,
            str(self.get_parameter('joint_states_topic').value),
            self._on_joint_states,
            10,
        )

        self._server = TeleopWebServer(
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
        them psi JUMPS the 45 deg gap (crab_psi_snap). Measured on the desk rig
        2026-08-24: the modules swing that 45 deg WITH TRACTION STILL ON. The
        teleop TransitionGuard does not cover it -- it withholds drive on a
        change of MANOEUVRE, and a steered crab stays crab_left throughout. The
        thing that is meant to cover it is swerve_controller's alignment gate,
        which ships DISABLED on purpose (ros2_controllers.yaml: "the ONE key to
        change to try it").

        So this is not a defect to fix here; it is a regime the operator has to
        be able to see they are in. psi moves at the crab steering rate, a
        couple of degrees per tick, so anything above 20 deg in one tick is a
        gap crossing and nothing else.
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
        frame.update({
            'held': held if fresh else [],
            'link_fresh': fresh,
            'silent_sec': round(silent, 3) if self._client_seen > 0.0 else None,
            'estop_latched': estop,
            'driver': session,
            'rivals': list(self._rivals),
            'limits': {
                'steer_limit_deg': math.degrees(self._limit),
                # A/D became MOMENTARY on 2026-08-24: straight ahead is the
                # resting state, and the angle springs back to exactly 0 when
                # neither key is held. The page has to say which of the two
                # models it is showing, because the difference is invisible in
                # a still picture and decisive in a moving one.
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

    def _on_client_input(self, session, keys, events, has_control):
        now = time.monotonic()
        wanted = frozenset(k for k in keys if k in HOLDABLE_KEYS)

        for name in events:
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

    def _do_estop(self):
        self.center()
        with self._input_lock:
            self._client_keys = frozenset()
            self._estop_latched = True
        self.event('EMERGENCY STOP — release every key to regain control', 'alarm')

    def _dispatch(self, name):
        actions = {
            'mode_keyboard':   (lambda: self.set_mode('keyboard'),
                                'mode → keyboard'),
            'mode_autonomous': (lambda: self.set_mode('autonomous'),
                                'mode → autonomous (Nav2)'),
            'pick':            (self.trigger_pick,    'pick_plastic: sent'),
            'open_gripper':    (self.open_gripper,    'gripper: open'),
            'home':            (self.go_home,         'arm: home position'),
            'arm_gate':        (self.arm_gateway,     'external gate: ARM requested'),
            'disarm_gate':     (self.disarm_gateway,  'external gate: DISARM requested'),
            'quit':            (self._quit,           'shutdown requested from the UI'),
        }
        entry = actions.get(name)
        if entry is None:
            return
        action, text = entry
        self.event(text, 'alarm' if name == 'quit' else 'info')
        action()

    def _quit(self):
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
        with self._input_lock:
            if self._client_seen <= 0.0:
                return
            silent = now - self._client_seen
            if silent > self._client_timeout:
                lost = bool(self._client_keys) and not self._link_lost
                if lost:
                    self._link_lost = True
                self._client_keys = frozenset()
                keys = None
            else:
                keys = self._client_keys

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


def _spin(node):
    """Spin until the context goes down, then release the main thread.

    rclpy installs handlers for SIGINT *and SIGTERM*, and SIGTERM is what
    `ros2 launch` sends on teardown. It shuts the context down underneath this
    thread and raises here. Without handing that on, the main thread below
    would keep waiting on an event nobody will ever set, and the process would
    sit there with its HTTP port still bound and a half-dead node still in the
    graph -- a teleop page that looks alive and controls nothing. Measured, not
    theorised: a plain `kill` did exactly that.
    """
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop_event.set()


def main():
    rclpy.init()
    node = WebTeleopNode()
    spin_thread = threading.Thread(target=_spin, args=(node,), daemon=True)
    spin_thread.start()
    node.start_server()
    try:
        # The node has no terminal input to block on, so the main thread just
        # waits for Ctrl+C, an external shutdown, or the UI's shutdown button.
        while not node.stop_event.is_set():
            node.stop_event.wait(0.25)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_server()
        if rclpy.ok():
            # Same shutdown as the terminal node: stop, straighten, keyboard
            # mode, and give the last publish a moment to leave.
            node.center()
            time.sleep(0.15)
        else:
            # Killed from outside: the context is already down, so nothing can
            # be published. The stop is not lost -- whatever took the context
            # down stopped this node publishing at all, and both teleop_mux
            # (cmd_timeout_sec) and the dead-man downstream treat silence as
            # zero. This is the one path where the explicit stop is skipped,
            # and it is skipped because it is impossible, not because it is
            # unnecessary.
            node.get_logger().warn(
                'external shutdown — no explicit stop published; downstream '
                'timeouts are what stop the robot here'
            )
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
