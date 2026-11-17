"""Reads the SO101 leader arm and publishes it. Passive (torque OFF) unless asked to align.

Topics (all on the ROS domain of the shell, normally ROS_DOMAIN_ID=20):
  /arm/leader/ticks         sensor_msgs/JointState  position = raw ticks (float)
  /arm/leader/joint_states  sensor_msgs/JointState  position = radians (only with calibration_file)
  /arm/leader/active        std_msgs/Bool           heartbeat at rate_hz; True = last read ok
                            (stops arriving when the node dies: the follower's watchdog input)
  /arm/leader/calibration_verified  std_msgs/Bool, transient_local  calibration_verified of
                            the calibration file (False without one)
  /arm/leader/enable        std_msgs/Bool, transient_local  streaming REQUEST (teleop key).
                            Also tracked here: this node publishes False on read errors / shutdown.
  /arm/leader/align         std_msgs/Bool (in, volatile)  True = align request (teleop J),
                            False = cancel. Idempotent: a True while aligning is ignored.
  /arm/leader/align_state   std_msgs/String, transient_local  idle | aligning ... |
                            done ... | aborted: ... | refused: ... | direct: ...
                            (direct = within the follower's activation tolerance, or
                            alignment disabled: nothing moved)
  /joint_states             (in) the follower's calibrated URDF angles (arm_joint_state_relay)
  /arm/stop                 (in) Space: aborts a running alignment, releases the wall
  /arm/leader/state         (in, transient_local) the follower's leader-mode state;
                            'active...' arms the virtual wall
  /arm/leader/wall_state    std_msgs/String, transient_local  disabled | unavailable: ... |
                            idle | armed[; held: ...][; outside: ...] | released: ...
                            ('wall fault' or 'NOT confirmed' in a released text = fault;
                            'outside: <joint> ... (overload release)' = let go after
                            pushing without progress, limp until back inside)
This node never arms anything; `enable` is a request, the follower decides.

VIRTUAL WALL (user decisions 2026-10-15/16, progressive wall 2026-10-20, wall.py): while
leader mode is active, a leader joint beyond the follower's command edge (joint limit minus
leader.limit_command_margin_ticks, read from the follower's config) gets torque on its own
servo only, its goal 2 ticks beyond the wall and a Torque_Limit that grows with the depth beyond
(wall_min_torque_limit at the first tick, wall_torque_limit at wall_full_force_depth_rad):
a spring, engaged only beyond by more than wall_engage_depth_rad (a deadband against
toggling); back inside: torque off at once, never Torque_Limit 0 with torque on. Let go
per joint after holding a high load without inward progress for wall_overload_time_s
(overload release). Released (torque off, all servos, confirmed) on Space, J, enable
withdrawn, follower leaving leader mode, read error and shutdown. wall_enabled: false
switches it off. Every first run of it is a test under the per-test motion approval (SR-1).

SELF-ALIGNMENT (user decision 2026-10-13, FR-17): on an align request the leader drives
itself to the follower's present pose, goes limp, and only then publishes enable=True.
That MOVES THE LEADER: every first run of it is a test under the per-test motion approval
(SR-1). Nothing but the startup torque-off is written unless an align request arrives.
"""

import signal
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty, String

from gripperx_leader.aligner import AlignAbort, AlignConfig, Aligner, AlignRefused

from gripperx_leader.calibration import load_calibration
from gripperx_leader.follower_config import activation_tolerance, command_band
from gripperx_leader.leader_bus import (
    DEFAULT_IDS, LeaderBus, LeaderReadError, detect_port, joint_names)
from gripperx_leader.wall import LeaderWall, WallConfig, WallFault

WALL_TIMING_LOG_S = 2.0         # wall timing report period while a joint is held
FOLLOWER_JOINTS = ['arm_limb1_joint', 'arm_limb2_joint', 'arm_limb3_joint',
                   'arm_limb4_joint', 'arm_limb5_joint', 'arm_gripper_joint']

ENABLE_QOS = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                        reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)


class LeaderArmNode(Node):
    def __init__(self, bus=None):
        super().__init__('leader_arm_node')
        self.declare_parameter('port', '')
        self.declare_parameter('baud', 1_000_000)
        self.declare_parameter('servo_ids', [1, 2, 3, 4, 5, 6])
        self.declare_parameter('rate_hz', 30.0)
        self.declare_parameter('calibration_file', '')
        self.declare_parameter('max_consecutive_read_errors', 5)
        d = AlignConfig()
        self.declare_parameter('align_enabled', True)
        self.declare_parameter('align_max_joint_velocity_rad_s', d.max_joint_velocity_rad_s)
        self.declare_parameter('align_torque_limit', d.torque_limit)
        self.declare_parameter('align_deviation_tolerance_rad', d.deviation_tolerance_rad)
        self.declare_parameter('align_arrival_tolerance_rad', d.arrival_tolerance_rad)
        self.declare_parameter('align_settle_timeout_sec', d.settle_timeout_sec)
        self.declare_parameter('align_voltage_min_v', d.voltage_min_v)
        self.declare_parameter('align_voltage_max_v', d.voltage_max_v)
        self.declare_parameter('follower_joint_states_topic', '/joint_states')
        self.declare_parameter('follower_joint_names', FOLLOWER_JOINTS)
        self.declare_parameter('follower_pose_timeout_sec', d.follower_pose_timeout_sec)
        self.declare_parameter('follower_motion_tolerance_rad', d.follower_motion_tolerance_rad)
        self.declare_parameter('align_still_check_sec', d.still_check_sec)
        self.declare_parameter('align_still_tolerance_rad', d.still_tolerance_rad)
        self.declare_parameter('align_torque_on_tolerance_rad', d.torque_on_tolerance_rad)
        self.declare_parameter('align_travel_margin_rad', d.travel_margin_rad)
        # "" = gripperx_arm share config/arm_safety_filter.yaml (follower_config.py).
        self.declare_parameter('follower_safety_filter_file', '')
        w = WallConfig()
        self.declare_parameter('wall_enabled', True)
        self.declare_parameter('wall_torque_limit', w.torque_limit)
        self.declare_parameter('wall_min_torque_limit', w.min_torque_limit)
        self.declare_parameter('wall_full_force_depth_rad', w.full_force_depth_rad)
        self.declare_parameter('wall_engage_depth_rad', w.engage_depth_rad)
        self.declare_parameter('wall_overload_time_s', w.overload_time_s)
        self.declare_parameter('wall_overload_load', w.overload_load)
        self.declare_parameter('wall_overload_progress_rad', w.overload_progress_rad)
        self.declare_parameter('follower_leader_state_topic', '/arm/leader/state')

        ids = [int(i) for i in self.get_parameter('servo_ids').value]
        self._names = joint_names(ids)
        self._max_errors = int(self.get_parameter('max_consecutive_read_errors').value)
        self._errors = 0

        cal_file = str(self.get_parameter('calibration_file').value)
        self._cal = None
        if cal_file:
            try:
                self._cal = load_calibration(cal_file).select(ids)
            except ValueError as exc:
                raise SystemExit(f'calibration {cal_file}: {exc}')
            if not self._cal.verified:
                self.get_logger().warn(
                    'calibration_verified is false: /arm/leader/joint_states radians are '
                    'NOT verified (zero hand-aligned, signs placeholders)')

        if bus is None:
            port = str(self.get_parameter('port').value) or detect_port()
            if not port:
                raise SystemExit('port parameter required: no unambiguous leader port found '
                                 '(/dev/gripperx_leader or a single /dev/serial/by-id entry)')
            bus = LeaderBus(port, int(self.get_parameter('baud').value), ids)
            bus.open()
        self._bus = bus
        self._ids = ids
        self._startup_check(ids)
        self._log_diagnostics()

        self._ticks_pub = self.create_publisher(JointState, '/arm/leader/ticks', 10)
        self._rad_pub = self.create_publisher(JointState, '/arm/leader/joint_states', 10)
        self._active_pub = self.create_publisher(Bool, '/arm/leader/active', 10)
        self._enable_pub = self.create_publisher(Bool, '/arm/leader/enable', ENABLE_QOS)
        self._cal_pub = self.create_publisher(Bool, '/arm/leader/calibration_verified', ENABLE_QOS)
        self._cal_pub.publish(Bool(data=bool(self._cal is not None and self._cal.verified
                                             and self._offset_problem is None)))
        self._setup_align(ids)
        self._setup_wall(ids)
        self._rate_hz = float(self.get_parameter('rate_hz').value)
        self._wall_t_prev = None
        self._wall_timing_t0 = None
        self.wall_tick_max_ms = self.wall_interval_max_ms = 0.0
        self.create_timer(1.0 / self._rate_hz, self._tick)
        self._publish_enable(False)   # start disarmed, overriding any stale latched True
        self.get_logger().info('leader streaming DISARMED; torque off, nothing is commanded')

    # ------------------------------------------------------------------ alignment
    def _setup_align(self, ids):
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self._align_enabled = bool(g('align_enabled'))
        cfg = AlignConfig(
            max_joint_velocity_rad_s=float(g('align_max_joint_velocity_rad_s')),
            torque_limit=int(g('align_torque_limit')),
            deviation_tolerance_rad=float(g('align_deviation_tolerance_rad')),
            arrival_tolerance_rad=float(g('align_arrival_tolerance_rad')),
            settle_timeout_sec=float(g('align_settle_timeout_sec')),
            follower_pose_timeout_sec=float(g('follower_pose_timeout_sec')),
            follower_motion_tolerance_rad=float(g('follower_motion_tolerance_rad')),
            voltage_min_v=float(g('align_voltage_min_v')),
            voltage_max_v=float(g('align_voltage_max_v')),
            still_check_sec=float(g('align_still_check_sec')),
            still_tolerance_rad=float(g('align_still_tolerance_rad')),
            torque_on_tolerance_rad=float(g('align_torque_on_tolerance_rad')),
            travel_margin_rad=float(g('align_travel_margin_rad')))
        self._act_problem = None
        try:
            cfg.activation_tolerance_rad = activation_tolerance(
                str(g('follower_safety_filter_file')))
            if not cfg.arrival_tolerance_rad < cfg.activation_tolerance_rad:
                self._act_problem = (
                    f'align_arrival_tolerance_rad {cfg.arrival_tolerance_rad} not below the '
                    f'follower activation tolerance {cfg.activation_tolerance_rad}')
        except ValueError as exc:
            self._act_problem = f'follower activation tolerance unknown: {exc}'
        if self._act_problem:
            self.get_logger().error(self._act_problem + ' - every align request is refused')
        else:
            self.get_logger().info(
                f'leader-mode activation tolerance {cfg.activation_tolerance_rad} rad '
                '(arm_action_server leader.activation_match_tolerance_rad): align requests '
                'within it on every joint enable leader mode directly, nothing moves')
        if cfg.max_joint_velocity_rad_s <= 0.0:
            raise SystemExit('align_max_joint_velocity_rad_s must be > 0')
        self._align_cfg = cfg
        all_names = list(g('follower_joint_names'))
        if len(all_names) != len(DEFAULT_IDS):
            raise SystemExit(f'follower_joint_names needs {len(DEFAULT_IDS)} entries (by servo id)')
        self._follower_names = [all_names[i - 1] for i in ids]
        self._follower_q = None
        self._follower_prev = None
        self._follower_rx = -float('inf')
        self._enable_last = False
        self._unconfirmed_reported = None
        self._tick_max_ms = 0.0
        # Follower joint limits: the single source gripperx_geometry (SR-17 rule 3), by id.
        from gripperx_geometry.constants import ARM_JOINT_LOWER, ARM_JOINT_UPPER
        limits = [(ARM_JOINT_LOWER[i - 1], ARM_JOINT_UPPER[i - 1]) for i in ids]
        self._aligner = None if self._cal is None else Aligner(
            self._bus, self._cal, cfg, self._names, limits)
        self._align_state_pub = self.create_publisher(String, '/arm/leader/align_state', ENABLE_QOS)
        self.create_subscription(Bool, '/arm/leader/align', self._on_align, 10)
        self.create_subscription(Bool, '/arm/leader/enable', self._on_enable, ENABLE_QOS)
        self.create_subscription(Empty, '/arm/stop', self._on_stop, 10)
        self.create_subscription(JointState, str(g('follower_joint_states_topic')),
                                 self._on_follower, qos_profile_sensor_data)
        self._set_align_state('idle' if self._align_enabled else
                              'idle (alignment disabled: J requests leader mode directly)')

    # ------------------------------------------------------------------ virtual wall
    def _setup_wall(self, ids):
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self._wall = None
        self._wall_text = None
        self._follower_leader_active = False
        # Set by every release of an armed wall (Space, J, read error, fault, ...): no
        # re-arming until the follower enters leader mode afresh - the late 'off' of the
        # follower would otherwise race a re-armed wall at the very joint it just let go.
        self._wall_latched = False
        self._wall_state_pub = self.create_publisher(String, '/arm/leader/wall_state', ENABLE_QOS)
        self.create_subscription(String, str(g('follower_leader_state_topic')),
                                 self._on_follower_leader_state, ENABLE_QOS)
        if not bool(g('wall_enabled')):
            self._set_wall_state('disabled (wall_enabled false)')
            return
        why = None
        if self._cal is None:
            why = 'no calibration_file: leader angles unknown'
        elif not self._cal.verified:
            why = 'leader calibration not verified'
        elif self._offset_problem is not None:
            why = 'leader Homing_Offset does not match the calibration'
        if why is None:
            try:
                band = command_band(str(g('follower_safety_filter_file')))
                cfg = WallConfig(
                    torque_limit=int(g('wall_torque_limit')),
                    min_torque_limit=int(g('wall_min_torque_limit')),
                    full_force_depth_rad=float(g('wall_full_force_depth_rad')),
                    engage_depth_rad=float(g('wall_engage_depth_rad')),
                    overload_time_s=float(g('wall_overload_time_s')),
                    overload_load=int(g('wall_overload_load')),
                    overload_progress_rad=float(g('wall_overload_progress_rad')),
                    voltage_min_v=float(g('align_voltage_min_v')),
                    voltage_max_v=float(g('align_voltage_max_v')))
                # Bands by servo id 1..6; the gripper (id 6) has its wall like the other
                # joints (user decision 2026-11-13).
                self._wall = LeaderWall(self._bus, self._cal, cfg, self._names,
                                        [band[i - 1] for i in ids])
            except ValueError as exc:
                why = str(exc)
        if why is not None:
            self.get_logger().error(f'virtual wall unavailable: {why}')
            self._set_wall_state(f'unavailable: {why}')
            return
        self.get_logger().info(
            'virtual wall at the follower command band (rad, by id): '
            + ', '.join(f'{i}: [{b[0]:.4f}, {b[1]:.4f}]'
                        for i, b in zip(ids, self._wall._bands) if b is not None))
        self._set_wall_state('idle')

    def _set_wall_state(self, text: str):
        if text != self._wall_text:
            self._wall_text = text
            self._wall_state_pub.publish(String(data=text))

    def _on_follower_leader_state(self, msg: String):
        was, self._follower_leader_active = (self._follower_leader_active,
                                             msg.data.startswith('active'))
        if self._follower_leader_active and not was:
            self._wall_latched = False
        if not self._follower_leader_active:
            self._wall_release(f'follower leader mode off ({msg.data})')

    def _wall_blocker(self):
        """Why the wall must not be armed now; None = it may."""
        if self._wall_latched:
            return 'released; re-arms at the next leader-mode activation'
        if not self._enable_last:
            return 'leader mode not requested'
        if not self._follower_leader_active:
            return 'follower not in leader mode'
        if self._aligning():
            return 'aligning'
        if self._errors:
            return 'leader read errors'
        if self._wall.unconfirmed or (self._aligner is not None and self._aligner.unconfirmed):
            return 'torque-off unconfirmed'
        return None

    def _wall_release(self, reason: str):
        """Torque off on every leader servo, at once, if the wall is armed."""
        if self._wall is None or not self._wall.armed:
            return
        text = self._wall.release_all(reason)
        self._report_wall_release(text)

    def _report_wall_release(self, text: str):
        self._wall_latched = True
        if self._wall.unconfirmed:
            self.get_logger().error(f'virtual wall released: {text}')
            self._set_wall_state(f'released: {text}')
        else:
            self.get_logger().info(f'virtual wall released: {text}')
            self._set_wall_state(f'released: {text}; re-arms at the next leader-mode '
                                 'activation')

    def _wall_retry(self):
        bad = self._wall.retry_release()
        if not bad:
            self.get_logger().info('wall torque-off confirmed on retry')
            self._set_wall_state('idle (torque off confirmed on retry; re-arms at the next '
                                 'leader-mode activation)')

    def _wall_step(self, ticks):
        if self._wall is None:
            return
        why = self._wall_blocker()
        if why is not None:
            if self._wall.armed:
                self._wall_release(why)
            return
        try:
            if not self._wall.armed:
                for note in self._wall.arm(ticks):
                    self.get_logger().warn(f'virtual wall: {note}')
            for event in self._wall.step(ticks, time.monotonic()):
                self.get_logger().info(f'virtual wall: {event}')
        except WallFault as exc:
            self._report_wall_release(str(exc))
            return
        held, out = self._wall.held(), self._wall.outside()
        self._set_wall_state('armed' + (f'; held: {", ".join(held)}' if held else '')
                             + (f'; outside: {", ".join(out)}' if out else ''))

    def _set_align_state(self, text: str):
        self.align_state = text
        self._align_state_pub.publish(String(data=text))

    def _aligning(self) -> bool:
        return self._aligner is not None and self._aligner.active

    def _on_follower(self, msg: JointState):
        pos = dict(zip(msg.name, msg.position))
        if all(n in pos for n in self._follower_names):   # /joint_states also carries wheels
            self._follower_prev = self._follower_q
            self._follower_q = [float(pos[n]) for n in self._follower_names]
            self._follower_rx = time.monotonic()

    def _on_enable(self, msg: Bool):
        self._enable_last = bool(msg.data)
        if not msg.data:
            self._wall_release('leader-mode request withdrawn (J / O / I / P / Space)')
        if self._aligning():
            self._abort_align('leader-mode request ' + ('withdrawn (J / O / I / P / Space)'
                              if not msg.data else 'set by another publisher'))

    def _on_stop(self, _msg):
        self._wall_release('Space (/arm/stop)')
        if self._aligning():
            self._abort_align('Space (/arm/stop)')

    def _on_align(self, msg: Bool):
        self._wall_release('J (/arm/leader/align)')
        if not msg.data:
            if self._aligning():
                self._abort_align('cancelled (J / O / I / P / Space / teleop exit)')
            return
        if self._aligning():
            return                                    # idempotent
        if self._offset_problem is not None and self._cal is not None:
            self._set_align_state(f'refused: {self._offset_problem}')
            self.get_logger().warn(f'align request refused: {self._offset_problem}')
            return
        if not self._align_enabled:
            self._publish_enable(True)
            self._set_align_state('direct: alignment disabled, leader mode requested')
            self.get_logger().info('align request: alignment disabled, enable published directly')
            return
        why = self._act_problem or self._align_refusal()
        if why is None:
            try:
                self._bus.status_bits.clear()
                diag = self._bus.read_diagnostics()        # registers first (audit LA1 M2)
                ticks = self._bus.read_positions()
                moving = self._aligner.begin(time.monotonic(), ticks, self._follower_q, diag)
            except LeaderReadError as exc:
                why = f'leader read failed: {exc}'
            except AlignRefused as exc:
                why = str(exc)
        if why is not None:
            self._set_align_state(f'refused: {why}')
            self.get_logger().warn(f'alignment refused: {why}')
            return
        if not moving:
            if self._aligner.direct:
                self._set_align_state(
                    f'direct: within the activation tolerance '
                    f'({self._align_cfg.activation_tolerance_rad:.3f} rad) on every joint, '
                    'nothing moved; leader mode requested')
            else:
                self._set_align_state('done: already aligned, nothing moved; leader mode '
                                      'requested')
            self.get_logger().info(f'align request: {self.align_state}')
            self._publish_enable(True)
            return
        self._tick_max_ms = 0.0
        clamp = (f'; clamped to the travel band: {", ".join(self._aligner.clamped)}'
                 if self._aligner.clamped else '')
        self._set_align_state(f'aligning: checking the leader is still '
                              f'({self._align_cfg.still_check_sec} s), then torque on{clamp}')
        self.get_logger().warn('alignment started - THE LEADER WILL MOVE')

    def _align_refusal(self):
        if self._aligner is None:
            return 'no calibration_file: leader angles unknown'
        if not self._cal.verified:
            return 'leader calibration not verified'
        if self._enable_last:
            return 'leader mode already requested'
        if self._errors:
            return 'leader read errors in progress'
        if self._aligner.unconfirmed:
            return f'torque-off still unconfirmed on servo(s) {self._aligner.unconfirmed}'
        if self._wall is not None and self._wall.unconfirmed:
            return f'wall torque-off still unconfirmed on servo(s) {self._wall.unconfirmed}'
        if self._wall is not None:
            # The aligner takes the register as the servo's own Torque_Limit: a restore that
            # failed after a wall release would leave the wall's spring value there.
            failed = self._wall.restore_pending()
            if failed:
                return f'wall Torque_Limit restore still pending on servo(s) {failed}'
        age = time.monotonic() - self._follower_rx
        if self._follower_q is None or age > self._align_cfg.follower_pose_timeout_sec:
            return (f'no fresh follower pose on {self.get_parameter("follower_joint_states_topic").value} '
                    f'with {self._follower_names} (timeout {self._align_cfg.follower_pose_timeout_sec} s)')
        # MessageInfo carries no publisher id, and /joint_states has several legitimate
        # publishers (wheels): two arm sources show up as disagreeing consecutive poses.
        if self._follower_prev is None:
            return 'follower pose seen only once: need two consistent messages'
        diff = max(abs(a - b) for a, b in zip(self._follower_q, self._follower_prev))
        if diff > self._align_cfg.follower_motion_tolerance_rad:
            return (f'follower pose inconsistent between the last two messages ({diff:.3f} rad): '
                    'follower moving, or a second source of the arm joints')
        return None

    def _abort_align(self, reason: str):
        text = self._aligner.abort(reason)
        self._unconfirmed_reported = list(self._aligner.unconfirmed) or None
        self._set_align_state(f'aborted: {text}')
        self.get_logger().error(f'alignment aborted: {text}')

    def _align_step(self, ticks):
        was_settling = self._aligner.phase == 'settling'
        try:
            arrived = self._aligner.step(time.monotonic(), ticks, self._follower_q,
                                         self._follower_rx)
        except AlignAbort as exc:
            self._unconfirmed_reported = list(self._aligner.unconfirmed) or None
            self._set_align_state(f'aborted: {exc}')
            self.get_logger().error(f'alignment aborted: {exc}')
            return
        if was_settling and self._aligner.phase == 'ramp':
            clamp = (f'; clamped to the travel band: {", ".join(self._aligner.clamped)}'
                     if self._aligner.clamped else '')
            self._set_align_state(
                f'aligning: leader to the follower pose, {self._aligner.duration:.1f} s at '
                f'{self._align_cfg.max_joint_velocity_rad_s} rad/s{clamp}')
        if arrived:
            self._set_align_state('done: aligned, torque off confirmed; leader mode requested '
                                  f'(max tick {self._tick_max_ms:.1f} ms)')
            self.get_logger().info('alignment done, torque off confirmed: enable published')
            self._publish_enable(True)

    def _check_offsets(self, diag) -> None:
        """Audit LA2 E2: the calibration's ticks are only valid with the Homing_Offsets
        they were measured with. Mismatch or unknown -> calibration reported unverified,
        alignment and direct enable refused."""
        self._offset_problem = None
        if self._cal is None:
            return
        rec = self._cal.homing_offset_raw
        bad = []
        for i, sid in enumerate(self._ids):
            want = None if rec is None else rec[i]
            got = (diag or {}).get(sid, {}).get('offset')
            if want is None or got != want:
                bad.append(f'servo {sid}: EEPROM {got}, calibration {want}')
        bad += self._cal.pair_problems()
        if bad:
            self._offset_problem = (
                'Homing_Offset does not match the calibration (' + '; '.join(bad) + '): '
                'leader angles are WRONG. Fix: update zero_ticks + homing_offset_raw together '
                '(README "Leader EEPROM step") or roll back with leader_eeprom --restore')
            self.get_logger().error(self._offset_problem + ' - calibration reported UNVERIFIED')

    def _log_diagnostics(self):
        """Read-only startup report: model, voltage and its limits, mode, position limits,
        Torque_Limit, torque, Homing_Offset, CW/CCW_Dead_Zone (the virtual wall's hold tick
        assumes the servo holds within about a tick of its goal - TO-VERIFY)."""
        self._offset_problem = 'servo diagnostics not read yet'
        try:
            diag = self._bus.read_diagnostics()
        except Exception as exc:  # noqa: BLE001 - a report, never a start blocker
            self.get_logger().warn(f'servo diagnostics unreadable: {exc}')
            self._check_offsets(None)
            return
        self._check_offsets(diag)
        for sid, d in diag.items():
            v = d.get('voltage')
            self.get_logger().info(
                f'servo {sid}: model {d.get("model")}, Present_Voltage {v} raw'
                + (f' (= {v * 0.1:.1f} V if 0.1 V/LSB, TO-VERIFY)' if v is not None else '')
                + f', Min/Max_Voltage_Limit {d.get("min_voltage")}/{d.get("max_voltage")} raw'
                f', Operating_Mode {d.get("mode")}, position limits '
                f'{d.get("min_pos")}..{d.get("max_pos")}, Torque_Limit {d.get("torque_limit")}'
                f', torque {d.get("torque")}, Homing_Offset {d.get("offset")}'
                f', CW/CCW_Dead_Zone {d.get("cw_dead_zone")}/{d.get("ccw_dead_zone")}')

    def _startup_check(self, ids):
        present = self._bus.ping_all(DEFAULT_IDS)
        names = dict(zip(DEFAULT_IDS, joint_names(DEFAULT_IDS)))
        for sid in DEFAULT_IDS:
            self.get_logger().info(
                f'servo {sid} ({names[sid]}): '
                f'{"present" if present.get(sid) else "ABSENT"}, '
                f'{"configured" if sid in ids else "not configured"}')
        missing = [i for i in ids if not present.get(i)]
        if missing:
            self._bus.close()
            raise SystemExit(
                f'configured servo id(s) {missing} do not answer. If they are not wired, '
                f'run with -p servo_ids:="{[i for i in ids if i not in missing]}"')

    def _publish_enable(self, value: bool):
        if value:
            # Before the echo arrives (audit LA1 L2): a J in between must be refused.
            self._enable_last = True
        self._enable_pub.publish(Bool(data=value))

    def _tick(self):
        t_start = time.monotonic()
        self._tick_body()
        if self._aligning():
            self._tick_max_ms = max(self._tick_max_ms, (time.monotonic() - t_start) * 1e3)
        self._wall_timing(t_start)

    def _wall_timing(self, t_start: float):
        """While the wall holds a joint: the longest tick and the longest interval between
        tick starts, logged every WALL_TIMING_LOG_S - does the wall fit the tick period?"""
        prev, self._wall_t_prev = self._wall_t_prev, t_start
        if self._wall is None or 'held' not in self._wall.state:
            self._wall_timing_t0 = None
            return
        now = time.monotonic()
        if self._wall_timing_t0 is None:
            self._wall_timing_t0, self.wall_tick_max_ms, self.wall_interval_max_ms = now, 0.0, 0.0
        self.wall_tick_max_ms = max(self.wall_tick_max_ms, (now - t_start) * 1e3)
        if prev is not None:
            self.wall_interval_max_ms = max(self.wall_interval_max_ms, (t_start - prev) * 1e3)
        if now - self._wall_timing_t0 >= WALL_TIMING_LOG_S:
            self.get_logger().info(
                f'virtual wall timing while held: max tick {self.wall_tick_max_ms:.1f} ms, '
                f'max tick interval {self.wall_interval_max_ms:.1f} ms (period '
                f'{1e3 / self._rate_hz:.1f} ms)')
            self._wall_timing_t0, self.wall_tick_max_ms, self.wall_interval_max_ms = now, 0.0, 0.0

    def _retry_torque_off(self):
        bad = self._aligner.retry_release()
        if bad != self._unconfirmed_reported:
            self._unconfirmed_reported = bad
            if bad:
                self._set_align_state(f'aborted: torque-off NOT confirmed on servo(s) {bad} '
                                      '- retrying every tick, check the leader')
                self.get_logger().error(f'leader torque-off still unconfirmed on {bad}')
            else:
                self._set_align_state('aborted: torque off now confirmed (retry)')
                self.get_logger().info('leader torque-off confirmed on retry')

    def _tick_body(self):
        if self._aligner is not None and self._aligner.unconfirmed and not self._aligning():
            self._retry_torque_off()
        if self._wall is not None and self._wall.unconfirmed:
            self._wall_retry()
        ok = True
        try:
            ticks = self._bus.read_positions()
            self._errors = 0
        except LeaderReadError as exc:
            ok = False
            self._errors += 1
            self.get_logger().warn(f'leader read error: {exc}', throttle_duration_sec=2.0)
            if self._aligning():
                self._abort_align(f'leader read error: {exc}')
            self._wall_release(f'leader read error: {exc}')
            if self._errors == self._max_errors:
                self._publish_enable(False)
                self.get_logger().error('too many read errors: streaming request withdrawn')
        self._active_pub.publish(Bool(data=ok))
        if not ok:
            return
        stamp = self.get_clock().now().to_msg()
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = self._names
        msg.position = [float(t) for t in ticks]
        self._ticks_pub.publish(msg)
        if self._cal is not None:
            rad = JointState()
            rad.header.stamp = stamp
            rad.name = self._names
            rad.position = self._cal.to_radians(ticks)
            self._rad_pub.publish(rad)
        if self._aligning():
            self._align_step(ticks)
        self._wall_step(ticks)

    def shutdown(self):
        try:
            if self._aligning():
                self._abort_align('node shutdown')
        except Exception:
            pass
        try:
            self._wall_release('node shutdown')
        except Exception:
            pass
        try:
            self._publish_enable(False)
            self._active_pub.publish(Bool(data=False))
        except Exception:
            pass
        # Unconditional, whatever the aligner and wall think: an interrupt between a
        # state change and its torque-off (aligner._release sets IDLE first) would
        # otherwise leave the steps above skipping a servo that is still torqued.
        try:
            bad = self._bus.torque_off_verified()
            if bad:
                self.get_logger().error(f'shutdown: torque-off NOT confirmed on servo(s) {bad}')
        except Exception:
            pass
        try:
            self._bus.close()
        except Exception:
            pass


def _hangup_as_interrupt(_signum, _frame):
    raise KeyboardInterrupt


def main():
    rclpy.init(args=sys.argv)
    # SIGHUP (the terminal that started this node was closed - gripperx_desk.sh's,
    # or a hand start's) would otherwise kill the process without the finally
    # below: no wall release, no alignment abort, a servo can stay torqued. Taken
    # like SIGINT, which rclpy delivers as KeyboardInterrupt.
    signal.signal(signal.SIGHUP, _hangup_as_interrupt)
    node = LeaderArmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # A second SIGHUP/SIGINT (or TERM) would raise into shutdown() and abort the
        # torque-off half way: from here on the node only finishes stopping.
        for sig in (signal.SIGHUP, signal.SIGINT, signal.SIGTERM):
            signal.signal(sig, signal.SIG_IGN)
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
