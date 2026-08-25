#!/usr/bin/env python3
"""
Keyboard teleop for GripperX — runs on the laptop.

Controls:
  W / S       Forward / backward (deadman: only drives while held)
  A / D       Steer left / right  (cumulative — stays put when released)
  Arrow ←/→   Crab walk left / right (sideways, deadman)        FR-7
  Arrow ↑/↓   Spin in place clockwise / counter-clockwise (deadman)
  Space       EMERGENCY STOP: stop + straight ahead + back to keyboard mode
  K           Mode: keyboard (manual control)
  G           Mode: autonomous (Nav2 takes over, goal via RViz)
  P           Start full grip sequence
  O           Only open gripper (joint 6)
  I           Arm to home position
  U           Arm the external authority gate (Octopus goals), 120 s window
  L           Disarm the external authority gate (Octopus goals), immediately
  Q / Ctrl+C  Quit

Two routes, one node
--------------------
A/D steering keeps the legacy DIRECT route: a fixed per-wheel pattern on
`/teleop/direct_steer`, which `steer_servo_node` applies as an override on top
of `/hw/joint_commands`. That route has no kinematics behind it and no per-wheel
limit awareness beyond the servo node's own clamp.

Crab and spin need wheel poses a fixed pattern cannot express (all four at
-+90 deg, and -+50.8 deg outward respectively), and they need the calibrated
per-wheel steering windows so no wheel is silently clamped out of the pose. So
they go the IK route instead: cmd_vel (linear.y / angular.z) -> teleop_mux ->
swerve_cmd_node -> per-wheel limits -> ros2_control. While a manoeuvre is
active this node STOPS publishing `/teleop/direct_steer`, so the override in
steer_servo_node lapses after its `direct_timeout_sec` and the IK path actually
owns the steering. The legacy route is bypassed, not removed.

Because the two routes park the wheels in completely different poses, every
switch between them runs through `manoeuvre.TransitionGuard`, which withholds
traction until the modules have measurably reached the new pose.
"""
import sys
import tty
import termios
import select
import math
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.action import ActionClient
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, String
from gripperx_arm_msgs.action import PickPlastic
from gripperx_external_msgs.srv import SetArming

from gripperx_control.steering_limits import (
    DEFAULT_INWARD_LIMIT_DEG,
    DEFAULT_OUTWARD_LIMIT_DEG,
    DEFAULT_OUTWARD_SIGN,
    SteeringLimits,
)
from gripperx_control.swerve_kinematic_model import FourWIS4WIDKinematicModel

from gripperx_teleop.key_input import (
    PRESS,
    RELEASE,
    KeyStateTracker,
    disable_release_reporting,
    enable_release_reporting,
    read_sequence,
)
from gripperx_teleop.manoeuvre import (
    CORNERING,
    CRAB_LEFT,
    CRAB_RIGHT,
    CRAB_STEER_KEYS,
    KEY_TO_MANOEUVRE,
    MANOEUVRE_KEY_PRECEDENCE,
    TransitionGuard,
    manoeuvre_pose,
    manoeuvre_twist,
    reachable_translation_arcs,
    snap_psi_into_reach,
)

STEER_JOINT_COUNT = 4

# Node names that publish the same cmd_vel this one does. Two of them at once is
# the single genuinely dangerous way to run this system: both publish at their
# own rate onto the same topic, teleop_mux forwards whichever arrived last, and
# NEITHER OPERATOR'S DEAD-MAN COVERS THE OTHER'S TRAFFIC. Releasing every key
# here does not stop a robot the other one is driving, and neither does the
# space bar -- center() zeroes this node's twist, and the other node overwrites
# it on its next tick. Impossible to notice by eye, cheap to detect, so it is
# detected. Both front-ends are listed: two terminals is as bad as one of each.
RIVAL_NODE_NAMES = ("keyboard_teleop_node", "web_teleop_node")

# On a timer, not only at start-up: the likelier mistake is a second teleop
# started AFTERWARDS, by the operator who forgot about the first.
RIVAL_CHECK_PERIOD_SEC = 2.0

# Operator default arming window for the external authority gate
# (gripperx_external's goal_gateway_node, service /gripperx/external/set_arming).
# Mirrors AGREED_DEFAULT_ARMING_DURATION_SEC in goal_gateway_node.py — the
# gateway refuses a SetArming request with a missing/zero duration outright
# (there is no indefinite arming window), so this must always be sent
# explicitly and non-zero.
ARM_REQUEST_DURATION_SEC = 120.0

# Drive keys (dead-man, SR-3) versus the cumulative steering keys. The arrows
# are drive keys: they command traction, so releasing one must stop the robot.
DRIVE_KEYS = ("w", "s")
MANOEUVRE_KEYS = tuple(KEY_TO_MANOEUVRE)
STEER_KEYS = ("a", "d")
ALL_KEYS = DRIVE_KEYS + STEER_KEYS + MANOEUVRE_KEYS + CRAB_STEER_KEYS

# Per-wheel steering factors this node publishes on /teleop/direct_steer for one
# operator steering value (joint order FL, FR, BL, BR): front and rear axle
# counter-rotate, which is what makes A/D corner instead of crab.
STEER_PATTERN = (1.0, 1.0, -1.0, -1.0)


def _pattern_steer_limit(outward_rad, inward_rad, outward_sign, pattern):
    """Largest |steering value| this node may emit without being clamped.

    The reachable joint window is asymmetric and per wheel (calibrated
    2026-08-13, gripperx_control/config/steer_servo.yaml): ~100 deg outward,
    ~30 deg inward, with `outward_sign` saying which sign of the joint angle is
    outward for that wheel (MEASURED: [-1, +1, +1, -1] — do not derive it from
    the URDF, that gives the wrong answer on the front pair).

    Because A/D drives all four wheels from ONE value and in BOTH directions,
    the usable envelope is the tightest bound over all wheels and both signs.
    With the counter-rotating pattern above, every steering direction puts two
    wheels on their inward side, so the answer is the inward limit — 30 deg.
    That is a real mechanical bound, not a policy choice: asking for more would
    only be clamped by steer_servo_node, silently, leaving the four wheels in a
    pose that matches no single instantaneous centre of rotation.
    """

    bound = float('inf')
    for factor, sign in zip(pattern, outward_sign):
        if factor == 0.0:
            continue
        upper = outward_rad if sign > 0 else inward_rad
        lower = -inward_rad if sign > 0 else -outward_rad
        for direction in (1.0, -1.0):
            edge = upper if (direction * factor) > 0 else lower
            bound = min(bound, abs(edge / factor))
    return bound


def update_steering_angle(
    current: float,
    steering_left: bool,
    steering_right: bool,
    rate: float,
    return_rate: float,
    limit: float,
    dt: float,
) -> float:
    """One tick of the MOMENTARY A/D steering angle.

    Pure, and module-level for the same reason _pattern_steer_limit above is:
    the behaviour the operator will judge this branch by is decided here, so it
    has to be checkable without a robot (test/check_steering_return.py).

    Holding both A and D is not an error and not a special case — the two
    increments simply cancel, which is also what the operator would expect from
    pressing both. Holding neither runs the angle back to EXACTLY zero.
    """

    if steering_left:
        current = min(limit, current + rate * dt)
    if steering_right:
        current = max(-limit, current - rate * dt)
    if steering_left or steering_right:
        return current

    step = return_rate * dt
    if abs(current) <= step:
        return 0.0
    return current - math.copysign(step, current)


class KeyboardTeleopNode(Node):

    def __init__(self, node_name: str = 'keyboard_teleop_node'):
        # The name is a parameter only so a second front-end can reuse this
        # node without colliding in the ROS graph (web_teleop_node). Default
        # unchanged -- every existing launch and `ros2 run` invocation keeps
        # the name it had.
        super().__init__(node_name)

        self.declare_parameter('steer_rate_rad_s',  0.6)
        # SELF-CENTRING RETURN RATE (user decision 2026-08-24). A/D are now
        # momentary: the angle grows while the key is held and runs back to zero
        # when it is not, so STRAIGHT AHEAD IS THE RESTING STATE and the robot
        # can no longer be driven off with a steering angle somebody set minutes
        # ago and forgot. That forgotten-angle case was the reason the cumulative
        # model needed a separate centring command at all (FR-13).
        #
        # Separate from steer_rate_rad_s rather than reusing it, because the two
        # are not the same judgement: the outward rate is how fast the operator
        # may ASK for angle, the return rate is how fast the machine takes it
        # away again. Defaulted EQUAL to it, so the symmetric behaviour is what
        # ships and an asymmetry has to be chosen deliberately.
        #
        # It is NOT a stop path and must not be read as one: the drive is
        # commanded by W/S through the dead-man and is unaffected by this. The
        # emergency stop keeps its own step to zero.
        self.declare_parameter('steer_return_rate_rad_s', 0.6)
        # Reachable steering window, mirroring gripperx_control/config/steer_servo.yaml
        # (calibrated 2026-08-13, source of truth). Joint order FL, FR, BL, BR.
        self.declare_parameter('steer_outward_limit_rad', math.radians(100.0))
        self.declare_parameter('steer_inward_limit_rad',  math.radians(35.0))
        self.declare_parameter('steer_outward_sign',      [-1, 1, 1, -1])
        # Operator cap on the A/D steering value. It is additionally capped at
        # what the window above actually allows for this node's steering pattern
        # (see _pattern_steer_limit) — an operator can only ever ask for LESS
        # than the mechanics allow, never for an angle the servo node would then
        # silently clamp. 35 deg is exactly that mechanical bound — written as
        # radians(35) rather than a rounded 0.6109, which would sit a hair ABOVE
        # the limit and trip the guard below on every start.
        self.declare_parameter('steer_limit_rad',   math.radians(35.0))
        self.declare_parameter('publish_rate_hz',   50.0)
        self.declare_parameter('linear_vel_m_s',    0.5)
        # DEAD-MAN CEILING for driving. Its MEANING CHANGED with the key input
        # layer (gripperx_teleop/key_input.py) and it is no longer the normal
        # stopping time:
        #
        #   * terminal WITH the kitty keyboard protocol — the robot stops on the
        #     real key-release event, typically within one publish tick (20 ms).
        #     This value then only covers a terminal that died without sending
        #     the release, which is the case the 06.07. incident is about.
        #   * terminal WITHOUT it — the tracker MEASURES the auto-repeat interval
        #     and stops roughly 3 repeats after the last one (~0.1 s at a 30 ms
        #     repeat rate). This value applies in full only until the first
        #     repeat has been seen, i.e. across the terminal's ~0.5 s initial
        #     repeat delay, which is the interval it was sized for and the one
        #     reason it cannot simply be lowered.
        #
        # It is a CEILING in both regimes: nothing can extend the dead-man past
        # it, and the measured window is clamped to it. The old advice to raise
        # the X11 repeat rate (`xset r rate`) is dropped — it is X11-only, and
        # tying the robot's stopping distance to a desktop setting was never a
        # property anyone could verify.
        self.declare_parameter('drive_hold_sec',     0.6)
        self.declare_parameter('direct_steer_topic', '/teleop/direct_steer')
        self.declare_parameter('cmd_vel_topic',      '/teleop/keyboard/cmd_vel')
        self.declare_parameter('arm_command_topic',  '/arm/command')
        # DT-4/M2 digital twin: in the sim there is no steer_servo_node
        # to consume /teleop/direct_steer. Default false → cmd_vel.angular.z
        # always stays 0, byte-identical real behavior. true (sim launch only)
        # additionally mirrors the cumulative A/D steering angle as angular.z onto
        # cmd_vel_topic, so that teleop_mux (keyboard_pass_angular_z=true) →
        # swerve_cmd_node can take over steering. See DT-10 for the
        # planned real servo steering path in the sim.
        self.declare_parameter('publish_steer_cmd_vel', False)
        self.declare_parameter('steer_to_omega_gain',   1.0)

        # ── Crab / spin (arrow keys) ──────────────────────────────────────
        # Deliberately slower than linear_vel_m_s: both manoeuvres put the
        # wheels in a pose the operator cannot read off the chassis at a glance,
        # and crab in particular moves the robot along an axis it has no sensor
        # coverage for.
        self.declare_parameter('crab_speed_m_s',   0.25)
        # 0.60 -> 0.55 on 2026-08-21, to match FollowPath.max_vel_theta so a
        # teleop turn and a Nav2 turn smear the scan by the same amount. Below
        # ~0.45 rad/s the wheels drop under the 0.12 m/s floor and the robot
        # stops turning rather than turning slowly, so this cannot go much
        # lower: at r_eff 0.2665 m, 0.55 rad/s puts the wheels at 0.147 m/s.
        # The residual 3.1 deg of LiDAR smear per scan is a property of a 10 Hz
        # sensor on a machine with a rotation floor, not something a speed
        # setting can remove. Removing it needs scan de-skewing.
        self.declare_parameter('spin_speed_rad_s', 0.55)
        # Traction while the modules are still slewing into the new pose. The
        # IK's steering angle atan2(vy_i, vx_i) is INVARIANT under a positive
        # scaling of the whole twist, so a scaled-down twist commands exactly
        # the target pose at a fraction of the wheel speed. It cannot be 0:
        # a zero twist is not "the same pose slowly", it is "wheels straight".
        # There is no steer-only command in this chain (see the note in
        # docs/TELEOP_MANOEUVRES.md) — 2 % of 0.25 m/s for the ~1 s of slewing
        # is ~5 mm of wheel travel, against the full speed it would be without
        # the guard.
        self.declare_parameter('manoeuvre_pose_scale', 0.02)
        # ── Steering a crab (arrow up/down), added 2026-08-24 ──────────────
        # Rate at which arrow up/down rotate the crab's DIRECTION OF TRAVEL psi.
        # Matched to steer_rate_rad_s so both steering keys move the machine at
        # the same rate and the operator only has one number to learn.
        self.declare_parameter('crab_steer_rate_rad_s', 0.6)
        # WHAT HAPPENS AT A DEAD BAND, and this is the one behavioural choice in
        # the feature.
        #
        # A pure translation puts all four modules on the SAME angle, and the
        # asymmetric windows leave only four reachable arcs with four 45 deg dead
        # bands between them (see reachable_translation_arcs() in manoeuvre.py):
        #
        #     reachable  [-180,-145] [-100,-80] [-35,+35] [+80,+100] [+145,+180]
        #
        # So psi CANNOT sweep continuously from pure crab (+-90, and it has only
        # +-10 deg of room) into the forward cone (+-35). In between there is no
        # pose at all, and swerve_controller would return kRejected, zero the
        # drive and hold the steering for the whole 45 deg.
        #
        #   true  -- psi JUMPS across the gap to the far edge and the robot pauses
        #            while the modules swing the 45 deg. The pause is real, it is
        #            what the machine has to do, and it is exactly the transition
        #            the guard (and swerve_controller's alignment gate) exist for.
        #   false -- psi STOPS at the edge of the arc. Honest, but then steering a
        #            crab means +-10 deg and nothing more.
        #
        # Defaulted true: stopping at +-80 deg makes the keys look broken, and the
        # jump is bounded, guarded and announced.
        self.declare_parameter('crab_psi_snap', True)
        # Must be >= steer_servo_node's direct_timeout_sec (0.5 s), otherwise
        # the pose is commanded while the direct-steer override still wins.
        self.declare_parameter('direct_release_sec',   0.7)
        self.declare_parameter('align_tolerance_rad',  math.radians(6.0))
        # Fallback only, for when /hw/steer_states does not reach the laptop.
        # Worst case slew is ~140 deg at roughly 0.3-0.5 s per 90 deg.
        self.declare_parameter('align_timeout_sec',    1.5)
        self.declare_parameter('use_steer_feedback',   True)
        self.declare_parameter('steer_states_topic',   '/hw/steer_states')
        self.declare_parameter('steer_states_timeout_sec', 0.5)
        self.declare_parameter('manoeuvre_topic',      '/teleop/manoeuvre')
        # Geometry and steering window used ONLY to predict the pose
        # swerve_cmd_node will command, so the guard waits for the right angles.
        # Mirrors gripperx_control/config/swerve_cmd.yaml and, through it,
        # config/steer_servo.yaml (the calibrated source of truth).
        # NOTE: this node is superseded by gripperx_swerve_controller/SwerveController
        # and is on the deletion-round list (out of the active path since 19c33c4),
        # still present on disk pending a separate, user-owned removal.
        # GEOMETRY COMES FROM THE SINGLE SOURCE OF TRUTH, NOT FROM A LOCAL DEFAULT.
        # Declared WITHOUT a default on purpose: gripperx_teleop/config/keyboard_teleop.yaml
        # supplies a, b and wheel_radius, and that file is held to
        # gripperx_geometry/config/geometry.yaml by colcon test. A node-level default here
        # would be a second copy that can drift, and drift is exactly what bit this file.
        #
        # WHY, kept from the branch that carried the interim stop-gap (2026-08-24), because
        # the reasoning is the reason the guard is moving into the controller at all:
        # the retired values were a = 0.203 / b = 0.16556, which are not the numbers the
        # controller plans with. That was not cosmetic -- it made the guard structurally
        # unable to do its job for one of the two manoeuvres:
        #
        #     controller commands the spin pose at  atan2(0.1809, 0.1087) = 58.999 deg
        #     this node predicted it at             atan2(0.203,  0.16556) = 50.80 deg
        #     difference                                                     8.20 deg
        #     align_tolerance_rad                                            6.00 deg
        #
        # so the measured angles could NEVER come within tolerance of the pose being waited
        # for, and every in-place spin armed on the TIMEOUT -- "pose NOT confirmed", 1.5 s
        # after the key, whether or not the modules had arrived. Crab was unaffected,
        # because +-90 deg is a property of a pure translation and not of a or b, which is
        # why this stayed invisible.
        #
        # The branch's stop-gap (a hardcoded 0.180 / 0.110) is SUPERSEDED by the parameter
        # file rather than merged: it was still a second copy, and not even the CAD pair.
        self.declare_parameter('a',            Parameter.Type.DOUBLE)
        self.declare_parameter('b',            Parameter.Type.DOUBLE)
        self.declare_parameter('wheel_radius', Parameter.Type.DOUBLE)
        self.declare_parameter('steering_outward_limit_deg', DEFAULT_OUTWARD_LIMIT_DEG)
        self.declare_parameter('steering_inward_limit_deg',  DEFAULT_INWARD_LIMIT_DEG)
        self.declare_parameter('steering_outward_sign',      list(DEFAULT_OUTWARD_SIGN))

        rate          = float(self.get_parameter('publish_rate_hz').value)
        self._rate    = float(self.get_parameter('steer_rate_rad_s').value)
        mechanical_limit = _pattern_steer_limit(
            float(self.get_parameter('steer_outward_limit_rad').value),
            float(self.get_parameter('steer_inward_limit_rad').value),
            [int(v) for v in self.get_parameter('steer_outward_sign').value],
            STEER_PATTERN,
        )
        requested_limit = float(self.get_parameter('steer_limit_rad').value)
        self._limit = min(requested_limit, mechanical_limit)
        if requested_limit > mechanical_limit + 1e-9:
            self.get_logger().error(
                f'steer_limit_rad={requested_limit:.4f} rad '
                f'({math.degrees(requested_limit):.1f} deg) exceeds what the steering '
                f'can reach in this pattern ({math.degrees(mechanical_limit):.1f} deg); '
                'reduced to that. Anything above would be clamped by steer_servo_node '
                'without the operator noticing.'
            )
        self._return_rate = float(self.get_parameter('steer_return_rate_rad_s').value)
        self._lin_vel = float(self.get_parameter('linear_vel_m_s').value)
        self._drive_hold = float(self.get_parameter('drive_hold_sec').value)
        steer_topic   = str(self.get_parameter('direct_steer_topic').value)
        cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        arm_topic     = str(self.get_parameter('arm_command_topic').value)
        self._publish_steer_cmd_vel = bool(self.get_parameter('publish_steer_cmd_vel').value)
        self._steer_to_omega_gain   = float(self.get_parameter('steer_to_omega_gain').value)

        self._crab_speed = float(self.get_parameter('crab_speed_m_s').value)
        self._spin_speed = float(self.get_parameter('spin_speed_rad_s').value)
        self._pose_scale = float(self.get_parameter('manoeuvre_pose_scale').value)
        self._crab_steer_rate = float(self.get_parameter('crab_steer_rate_rad_s').value)
        self._crab_psi_snap = bool(self.get_parameter('crab_psi_snap').value)
        self._use_steer_feedback = bool(self.get_parameter('use_steer_feedback').value)
        self._steer_states_timeout = float(
            self.get_parameter('steer_states_timeout_sec').value
        )

        self._model = FourWIS4WIDKinematicModel(
            a=float(self.get_parameter('a').value),
            b=float(self.get_parameter('b').value),
            wheel_radius=float(self.get_parameter('wheel_radius').value),
        )
        self._model_limits = SteeringLimits.from_outward_inward(
            math.radians(float(self.get_parameter('steering_outward_limit_deg').value)),
            math.radians(float(self.get_parameter('steering_inward_limit_deg').value)),
            [int(v) for v in self.get_parameter('steering_outward_sign').value],
        ).in_model_order()
        # Which translation directions this chassis can actually reach. Computed
        # from the SAME windows the pose resolution uses, so the two cannot drift
        # apart if the calibration moves — see reachable_translation_arcs().
        self._translation_arcs = reachable_translation_arcs(self._model_limits)

        self._guard = TransitionGuard(
            release_sec=float(self.get_parameter('direct_release_sec').value),
            align_timeout_sec=float(self.get_parameter('align_timeout_sec').value),
            align_tolerance_rad=float(self.get_parameter('align_tolerance_rad').value),
        )

        self._steer_pub   = self.create_publisher(Float64MultiArray, steer_topic,         10)
        self._cmd_vel_pub = self.create_publisher(Twist,             cmd_vel_topic,       10)
        self._arm_pub     = self.create_publisher(String,            arm_topic,           10)
        self._mode_pub    = self.create_publisher(String,            '/teleop/set_mode',  10)
        # Latched: whoever subscribes late (a second terminal, rqt) still learns
        # which manoeuvre the robot is in without waiting for the next change.
        self._manoeuvre_pub = self.create_publisher(
            String,
            str(self.get_parameter('manoeuvre_topic').value),
            QoSProfile(
                depth=1,
                history=HistoryPolicy.KEEP_LAST,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self._pick_client = ActionClient(self, PickPlastic, 'pick_plastic')
        self._arming_client = self.create_client(
            SetArming, '/gripperx/external/set_arming'
        )

        self._lock      = threading.Lock()
        # Key state. `release_reporting` is decided by the reader thread once it
        # has a raw tty and has negotiated with the terminal (_key_reader), so
        # the tracker starts in the FALLBACK regime and is upgraded in place.
        # Starting the other way round would mean a terminal without the
        # protocol spends its first seconds believing every key is still down.
        self._keys      = KeyStateTracker(
            ALL_KEYS,
            ceiling_sec=float(self.get_parameter('drive_hold_sec').value),
            release_reporting=False,
        )
        self._steer     = 0.0
        # Direction of travel of the active crab, robot frame, rad. None whenever
        # no crab is active — NOT 0.0, because 0.0 is a legitimate psi (straight
        # ahead as a pure translation) and the two must not look alike.
        self._crab_psi  = None
        self._dt        = 1.0 / rate
        self._pick_busy = False
        self._steer_states = None
        self._steer_states_t = 0.0
        self._status_line = ''

        if self._use_steer_feedback:
            # BEST_EFFORT on purpose: this crosses the (unstable, NFR-8) hotspot
            # link from the Pi. A dropped sample only delays arming by one tick;
            # a retransmit queue would not help and costs bandwidth. RELIABLE
            # publisher + BEST_EFFORT subscriber is a compatible pairing.
            self.create_subscription(
                Float64MultiArray,
                str(self.get_parameter('steer_states_topic').value),
                self._on_steer_states,
                QoSProfile(
                    depth=1,
                    history=HistoryPolicy.KEEP_LAST,
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                ),
            )

        self.create_timer(self._dt, self._publish)

        self._rivals = ()
        self.create_timer(RIVAL_CHECK_PERIOD_SEC, self._check_for_rivals)

        self.get_logger().info(
            f'Keyboard teleop started | Steer→{steer_topic} '
            f'(max {math.degrees(self._limit):.1f} deg) | '
            f'Drive→{cmd_vel_topic} | Arm→{arm_topic}'
        )
        # Follows the 2026-08-24 rebinding, which the banner box below already
        # carries: arrow up/down steer the crab, the spins are on 0 and 9. This
        # line was missed in that pass and still advertised the old binding --
        # a start-up log that names the wrong keys is worse than none.
        self.get_logger().info(
            f'Manoeuvres: arrow ←/→ crab at {self._crab_speed:.2f} m/s '
            f'(arrow ↑/↓ steer it), 0 spin CW / 9 spin CCW at '
            f'{self._spin_speed:.2f} rad/s '
            f'(IK route via cmd_vel; steer feedback '
            f'{"on" if self._use_steer_feedback else "OFF — arming on timeout"})'
        )

    # ── Another teleop on the same cmd_vel ───────────────────────────────────

    def _check_for_rivals(self):
        """Notice a second teleop publishing the same cmd_vel as this one.

        Deliberately does NOT kill the other node. It might be the one an
        operator is actually holding a key on, and killing it blind would be a
        worse failure than the one being prevented -- a robot mid-manoeuvre
        whose commands stop arriving is not the same as a robot that was told
        to stop. So this says so, and leaves the decision to the human who can
        see both.
        """
        own = self.get_name()
        names = self.get_node_names()
        rivals = set()
        for name in names:
            if name == own:
                # Our own entry -- but a SECOND node with our own name is still
                # a rival, so count rather than skip.
                if names.count(name) > 1:
                    rivals.add(name)
            elif name in RIVAL_NODE_NAMES:
                rivals.add(name)

        found = tuple(sorted(rivals))
        if found == self._rivals:
            return
        appeared = tuple(sorted(set(found) - set(self._rivals)))
        self._rivals = found

        if appeared:
            self.get_logger().error(
                f'ANOTHER TELEOP IS RUNNING ({", ".join(appeared)}). Both '
                'publish the same cmd_vel, so neither operator can stop what '
                'the other commands. Shut one down.'
            )
        elif not found:
            self.get_logger().info('the other teleop is gone')
        self._announce_rivals(appeared, found)

    def _announce_rivals(self, appeared, rivals):
        """Put it where THIS front-end's operator is looking.

        Here that is a scrolling raw terminal, where a log line is one line
        among many and scrolls away. The browser front-end overrides this and
        puts a banner across the top of the page instead.
        """
        if not rivals:
            sys.stdout.write('\r\x1b[2K  \x1b[32m>> the other teleop is gone'
                             '\x1b[0m\r\n')
            sys.stdout.flush()
            return
        listed = ', '.join(rivals)
        sys.stdout.write(
            '\r\x1b[2K\r\n'
            '\x1b[1;41;97m  !! ANOTHER TELEOP IS RUNNING: ' + listed + '  \x1b[0m\r\n'
            '\x1b[1;31m  !! It publishes the same cmd_vel as this one.\x1b[0m\r\n'
            '\x1b[1;31m  !! Your SPACE BAR does not stop what it commands.\x1b[0m\r\n'
            '\x1b[1;31m  !! Shut one of them down.\x1b[0m\r\n\r\n'
        )
        sys.stdout.flush()

    # ── Steering feedback / manoeuvre selection ──────────────────────────────

    def _on_steer_states(self, msg: Float64MultiArray):
        if len(msg.data) < STEER_JOINT_COUNT:
            return
        with self._lock:
            # Joint order FL, FR, BL, BR — same order as STEER_PATTERN.
            self._steer_states = [float(v) for v in msg.data[:STEER_JOINT_COUNT]]
            self._steer_states_t = time.monotonic()

    def _fresh_steer_states(self, now: float):
        """Measured servo angles, or None if too old / never seen."""
        if self._steer_states is None:
            return None
        if (now - self._steer_states_t) > self._steer_states_timeout:
            return None
        return self._steer_states

    def _active_manoeuvre(self, now: float) -> str:
        """Which manoeuvre the currently held arrow key asks for.

        The manoeuvre is defined solely by the held arrow — no latch, so
        releasing it ends the manoeuvre within the dead-man window (SR-3) and
        the wheels return to straight ahead. `press()` already invalidates the
        competing keys, so at most one can be inside the window; the loop below
        is only a deterministic tie-breaker.
        """
        for key in MANOEUVRE_KEY_PRECEDENCE:
            if self._keys.held(key, now, self._drive_hold):
                return KEY_TO_MANOEUVRE[key]
        return CORNERING

    def _update_crab_psi(self, manoeuvre: str, now: float) -> float:
        """One tick of the crab's direction of travel. Caller holds the lock.

        psi starts at the plain crab heading (+-90 deg) and arrow up/down rotate
        it. THE RULE IS THE SAME ON BOTH SIDES: up steers towards the FRONT
        (psi -> 0), down towards the REAR (psi -> +-180). Expressed through the
        side sign rather than as two cases, so a crab-left and a crab-right
        cannot drift into behaving differently.

        psi is CLAMPED TO THE ROBOT'S OWN SIDE. Letting a crab-left steer through
        zero into a crab-right would silently change which manoeuvre is active
        while the operator is still holding the left arrow.
        """

        side = 1.0 if manoeuvre == CRAB_LEFT else -1.0
        psi = self._crab_psi
        if psi is None:
            psi = side * (math.pi / 2.0)

        up = self._held('up')
        down = self._held('down')
        if up == down:
            # Neither, or both cancelling. psi HOLDS — unlike A/D it does not
            # spring back, because there is no direction here that is more
            # "neutral" than another: the operator picked a heading and the robot
            # is travelling along it.
            return psi

        step = self._crab_steer_rate * self._dt
        moving_towards_zero = up
        psi -= side * step if up else -side * step

        # Stay on this crab's side of the robot, and inside the half turn.
        low, high = (0.0, math.pi) if side > 0 else (-math.pi, 0.0)
        psi = min(high, max(low, psi))

        if self._crab_psi_snap:
            snapped = snap_psi_into_reach(psi, self._translation_arcs, moving_towards_zero)
            if snapped != psi:
                self.get_logger().info(
                    f'crab steering: psi {math.degrees(psi):+.1f} deg is in a dead band '
                    f'(no pose exists for a translation in that direction) -- jumping to '
                    f'{math.degrees(snapped):+.1f} deg. The robot pauses while the modules '
                    f'swing; that pause is the machine, not the software.'
                )
                psi = snapped
        else:
            # No snap: refuse to enter the dead band at all, i.e. stop at the arc
            # edge. Without this psi would walk into a region where
            # swerve_controller rejects every twist, and the operator would see
            # the robot simply stop with no reason given.
            reachable = snap_psi_into_reach(psi, self._translation_arcs, moving_towards_zero)
            if reachable != psi:
                psi = self._crab_psi if self._crab_psi is not None else side * (math.pi / 2.0)

        return psi

    # ── Publish tick ─────────────────────────────────────────────────────────

    def _publish(self):
        now = time.monotonic()
        with self._lock:
            manoeuvre = self._active_manoeuvre(now)
            crabbing = manoeuvre in (CRAB_LEFT, CRAB_RIGHT)
            if not crabbing:
                # Leaving a crab forgets its heading. Carrying psi across would
                # mean the NEXT crab starts wherever the last one was steered to,
                # minutes later, which is the same forgotten-state trap the
                # cumulative A/D angle has.
                self._crab_psi = None
            if self._guard.request(manoeuvre, now) and manoeuvre != CORNERING:
                # The cumulative A/D angle describes a pose the manoeuvre is
                # about to leave. Zero it, so releasing the arrow key returns the
                # wheels to straight ahead instead of snapping back to a stale
                # steering angle the operator set minutes ago.
                self._steer = 0.0

            if crabbing:
                self._crab_psi = self._update_crab_psi(manoeuvre, now)

            # A/D are MOMENTARY, and W/S are deliberately not consulted here:
            # steering and driving are independent axes of one command, so the
            # operator can hold W and correct with A at the same time. `press()`
            # already declines to invalidate the drive keys when a steer key
            # arrives, so nothing above this line stands in the way either.
            #
            # NOTE FOR THE OPERATOR, and it is not a defect of this branch: a
            # TERMINAL cannot report two keys held at once — it auto-repeats only
            # the LAST key pressed — so under a plain terminal, tapping A while
            # holding W still lets the drive lapse. What removes that is the key
            # input layer on Theo-teleop-responsive; this branch supplies the
            # SEMANTICS and that one supplies the key state. Each is testable
            # alone; together they are the feature.
            # Self-centring converges on EXACTLY zero rather than
            # asymptotically: a residual of a few milliradians would keep
            # /teleop/direct_steer alive at a non-zero angle for ever, and that
            # override wins over the IK inside swerve_controller (A2), so
            # "almost straight" would quietly hold the servos off the IK path
            # indefinitely. See update_steering_angle().
            self._steer = update_steering_angle(
                self._steer,
                self._held('a'),
                self._held('d'),
                self._rate,
                self._return_rate,
                self._limit,
                self._dt,
            )
            angle = self._steer

            # Deadman: only drives while the key is held (key-repeat window).
            # No latch — safety incident 06.07.: a latched W in a
            # forgotten terminal left the motors running continuously. The
            # arrow keys are drive keys too and use the same window.
            if manoeuvre == CORNERING:
                if self._held('w', self._drive_hold):
                    drive = 1
                elif self._held('s', self._drive_hold):
                    drive = -1
                else:
                    drive = 0
                # Legacy route: the pose the direct-steer pattern is asking for.
                target = [angle * factor for factor in STEER_PATTERN]
            else:
                drive = 0
                # IK route: exactly the pose swerve_cmd_node will command for
                # this twist, resolved against the same per-wheel windows.
                target = manoeuvre_pose(
                    manoeuvre,
                    self._model,
                    self._model_limits,
                    self._crab_speed,
                    self._spin_speed,
                    self._fresh_steer_states(now),
                    self._crab_psi,
                )

            self._guard.update(now, target, self._fresh_steer_states(now))
            armed    = self._guard.drive_allowed
            pose_on  = self._guard.pose_commanded
            status   = self._guard.describe()

        # Steering → direct_steer (steer_servo_node)
        # Front axle and rear axle counter-rotating → cornering instead of crab walk.
        # Same pattern the reachable limit was derived from — keep them together.
        if manoeuvre == CORNERING:
            steer_msg = Float64MultiArray()
            steer_msg.data = [angle * factor for factor in STEER_PATTERN]  # FL, FR, BL, BR
            self._steer_pub.publish(steer_msg)
        # else: deliberately SILENT. steer_servo_node applies /teleop/direct_steer
        # as an override on top of /hw/joint_commands for direct_timeout_sec
        # (0.5 s) after the last message. Staying quiet lets that override lapse,
        # which is the only way the IK path can own the steering servos. The
        # legacy route is bypassed here, not removed.

        # Drive → cmd_vel (teleop_mux → swerve_cmd_node → controller)
        cmd = Twist()
        if manoeuvre == CORNERING:
            cmd.linear.x = self._lin_vel * drive if armed else 0.0
            if self._publish_steer_cmd_vel:
                # Sim helper steering (DT-4/M2, see DT-10): mirror the same cumulative
                # steering angle that goes to direct_steer above additionally as omega
                # onto cmd_vel — a single source of truth for "how far
                # steered", on the real robot the value stays unused (default false).
                cmd.angular.z = angle * self._steer_to_omega_gain
        elif pose_on:
            vx, vy, omega = manoeuvre_twist(
                manoeuvre, self._crab_speed, self._spin_speed, self._crab_psi
            )
            scale = 1.0 if armed else self._pose_scale
            cmd.linear.x  = vx * scale
            cmd.linear.y  = vy * scale
            cmd.angular.z = omega * scale
        # RELEASING: zero twist. The direct-steer override still owns the
        # servos, so commanding the new pose now would only apply traction in
        # the OLD pose — the exact thing this guard exists to prevent.
        self._cmd_vel_pub.publish(cmd)

        # Observation hook. A no-op here; a front-end that wants to SHOW what
        # was just published (the browser UI) overrides it instead of
        # recomputing the tick for itself. Recomputation would be a second,
        # silently diverging copy of a safety-relevant decision -- this way the
        # display can only ever show what actually went on the wire.
        self._observe(manoeuvre, angle, target, cmd, armed, pose_on, status)

        self._announce(status)

    def _observe(self, manoeuvre, steer_angle, target, cmd, armed, pose_on, status):
        """Called once per publish tick with the values just published."""

    # ── Operator feedback ────────────────────────────────────────────────────

    def _announce(self, status: str):
        """Make the active manoeuvre visible — it is no longer inferable.

        Which key is held no longer tells the operator what the wheels are
        doing: arrow-left first swings all four modules to 90 deg before
        anything moves sideways, and that is alarming unannounced.
        """
        with self._lock:
            if status == self._status_line:
                return
            previous = self._status_line
            self._status_line = status

        msg = String()
        msg.data = status
        self._manoeuvre_pub.publish(msg)
        # Raw tty: carriage return + erase line, and \r\n instead of \n.
        sys.stdout.write(f'\r\x1b[2K  >> {status}\r\n')
        sys.stdout.flush()
        if previous.split('|')[0] != status.split('|')[0]:
            self.get_logger().info(f'Manoeuvre: {status}')

    # ── Keys ───────────────────────────────────────────────────────────────

    def press(self, key: str, kind: int = PRESS):
        """One key event from the reader thread.

        `kind` is the kitty-protocol event type (press / repeat / release). A
        RELEASE only ever clears state, so it must not run the mutual-exclusion
        block below — letting go of W would otherwise invalidate a manoeuvre key
        the operator is still holding.
        """
        now = time.monotonic()
        with self._lock:
            self._keys.on_event(key, kind, now)
            if kind == RELEASE:
                return
            # Mutual exclusion. Invalidating the competing keys immediately
            # (instead of letting them time out) means a direction or mode
            # change takes effect at once and two manoeuvres can never be
            # requested together.
            if key in MANOEUVRE_KEYS:
                # One manoeuvre at a time, and a manoeuvre supersedes W/S.
                for other in MANOEUVRE_KEYS + DRIVE_KEYS:
                    if other != key:
                        self._keys.clear(other, now)
            elif key in DRIVE_KEYS:
                # W and S are mutually exclusive, and asking to drive
                # forward/backward means leaving crab/spin.
                for other in DRIVE_KEYS + MANOEUVRE_KEYS:
                    if other != key:
                        self._keys.clear(other, now)
            elif key in STEER_KEYS:
                # A/D are cornering keys — steering means leaving crab/spin.
                for other in MANOEUVRE_KEYS:
                    self._keys.clear(other, now)
            elif key in CRAB_STEER_KEYS:
                # Arrow up/down are MODIFIERS of an active crab, not manoeuvres,
                # so they invalidate NOTHING — the operator has to be able to
                # hold left and up at once. That also means pressing one while no
                # crab is active does nothing at all, which is the right answer:
                # there is no heading to steer.
                pass

    def center(self):
        # EMERGENCY STOP: forces keyboard mode, so the space bar also stops
        # in autonomous mode — teleop_mux publishes zero immediately on the
        # switch and ignores Nav2 commands from then on. Unconditional in every
        # manoeuvre (SR-2): stop, straighten, back to keyboard mode.
        mode_msg = String()
        mode_msg.data = 'keyboard'
        self._mode_pub.publish(mode_msg)
        now = time.monotonic()
        with self._lock:
            was_manoeuvring = self._guard.manoeuvre != CORNERING
            self._keys.clear_all(now)
            self._steer = 0.0
            self._crab_psi = None
            # rearm only if there was nothing to swing back: that keeps the
            # accepted SR-2/SR-3 behaviour byte-identical for plain cornering
            # (W drives again immediately after the stop). Out of a crab/spin
            # pose the wheels are up to 90 deg off and traction stays withheld
            # until they are measurably straight again.
            self._guard.force_cornering(now, rearm=not was_manoeuvring)
            status = self._guard.describe()
        # Publish stop immediately
        self._cmd_vel_pub.publish(Twist())
        # …and straight ahead on the legacy route: this is the command that
        # actually swings the modules back out of a crab/spin pose, because it
        # re-arms the direct-steer override in steer_servo_node at once.
        straight = Float64MultiArray()
        straight.data = [0.0] * STEER_JOINT_COUNT
        self._steer_pub.publish(straight)
        self.get_logger().info('EMERGENCY STOP: stop + center + keyboard mode')
        self._announce(status)

    def _held(self, key: str, window: float = 0.15) -> bool:
        return self._keys.held(key, time.monotonic(), window)

    def enable_release_regime(self) -> None:
        """Called by the reader thread when the terminal agreed to report releases."""
        with self._lock:
            self._keys.release_reporting = True

    # ── Arm commands ──────────────────────────────────────────────────────────

    def trigger_pick(self):
        if self._pick_busy:
            self.get_logger().info('pick_plastic already running')
            return
        if not self._pick_client.wait_for_server(timeout_sec=0.5):
            self.get_logger().warn('pick_plastic server unreachable')
            return
        self._pick_busy = True
        goal = PickPlastic.Goal()
        future = self._pick_client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)
        self.get_logger().info('pick_plastic: sent')

    def _on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('pick_plastic: goal rejected')
            self._pick_busy = False
            return
        handle.get_result_async().add_done_callback(self._on_pick_done)

    def _on_pick_done(self, future):
        result = future.result().result
        self._pick_busy = False
        self.get_logger().info(
            f'pick_plastic: {"OK" if result.success else "ERROR"} — {result.message}'
        )

    def set_mode(self, mode: str):
        msg = String(); msg.data = mode
        self._mode_pub.publish(msg)
        self.get_logger().info(f'Teleop mode → {mode}')

    def open_gripper(self):
        msg = String(); msg.data = 'open_gripper'
        self._arm_pub.publish(msg)
        self.get_logger().info('Arm: opening gripper')

    def go_home(self):
        msg = String(); msg.data = 'go_home'
        self._arm_pub.publish(msg)
        self.get_logger().info('Arm: home position')

    # ── External authority gate (Octopus goal gateway) ─────────────────────
    #
    # SR-15 rule 4: arming may only ever happen through an explicit operator
    # act. These two methods are that act — one per direction, deliberately
    # NOT one toggle key: a toggle can arm by accident (operator unsure of
    # the current state, or a stray keypress) with no way to tell from the
    # keypress alone which direction it just took. Two dedicated keys make
    # every press unambiguous, at the cost of one more key to remember.
    # There is no auto-arm, no arm-on-startup and no re-arm-on-expiry
    # anywhere in this node — arming happens ONLY here, from a keypress.

    def _call_arming(self, arm: bool, duration_sec: float):
        if not self._arming_client.service_is_ready():
            # Non-blocking on purpose: a dead/absent gateway must never stall
            # the key loop. wait_for_service()/spin_until_future_complete()
            # with a real timeout would still block this thread for that
            # long; service_is_ready() returns immediately.
            self.get_logger().warn(
                'set_arming service unavailable (gateway not running?) — '
                f'{"arm" if arm else "disarm"} request NOT sent'
            )
            return
        request = SetArming.Request()
        request.arm = arm
        request.duration_sec = duration_sec if arm else 0.0
        request.requested_by = 'keyboard_teleop'
        future = self._arming_client.call_async(request)
        future.add_done_callback(
            lambda f, arm=arm: self._on_arming_response(f, arm)
        )
        self.get_logger().info(
            f'set_arming: sent {"ARM" if arm else "DISARM"} request'
            + (f' (duration {duration_sec:.0f} s)' if arm else '')
        )

    def _on_arming_response(self, future, arm: bool):
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001 — report, don't crash the node
            self.get_logger().error(f'set_arming call failed: {exc}')
            return
        # The gateway's refusal messages are informative (clock not
        # advancing, duration too long, already armed, …) — always show them,
        # success or not.
        if response.success and arm:
            self.get_logger().info(
                f'ARMED — {response.message} '
                f'(granted window: {response.state.seconds_remaining:.0f} s remaining)'
            )
        elif response.success:
            self.get_logger().info(f'DISARMED — {response.message}')
        else:
            self.get_logger().warn(f'set_arming REFUSED — {response.message}')

    def arm_gateway(self):
        self._call_arming(True, ARM_REQUEST_DURATION_SEC)

    def disarm_gateway(self):
        self._call_arming(False, 0.0)


# The rest of an escape sequence follows within microseconds of the ESC when a
# key generates it; a human pressing ESC produces nothing after it. 50 ms
# separates the two reliably without making a bare ESC feel sluggish.
_ESC_SEQUENCE_TIMEOUT = 0.05


def _dispatch(node: KeyboardTeleopNode, key: str, kind: int, stop_event) -> None:
    """Act on one decoded key event.

    ONE dispatch for both input regimes. Under the kitty protocol every key
    arrives as an escape sequence carrying an event type; without it, plain
    bytes arrive and the caller synthesises `PRESS`. Everything below the reader
    therefore sees the same thing either way, which is what keeps the fallback
    from being a second, subtly different teleop.
    """

    if key in ("q", "ctrl_c"):
        # Quit on the PRESS edge. Acting on a release instead would leave the
        # operator holding a key that has visibly done nothing.
        if kind != RELEASE:
            stop_event.set()
        return

    if key in ALL_KEYS:
        # The movement keys are the only ones that care about press vs release —
        # they are the dead-man (SR-3).
        node.press(key, kind)
        return

    # Command keys: edge-triggered, and deliberately NOT on repeat. Holding P
    # must request one pick, not one per repeat interval.
    if kind != PRESS:
        return
    if key == "space":
        node.center()
    elif key == "k":
        node.set_mode('keyboard')
    elif key == "g":
        node.set_mode('autonomous')
    elif key == "p":
        node.trigger_pick()
    elif key == "o":
        node.open_gripper()
    elif key == "i":
        node.go_home()
    elif key == "u":
        node.arm_gateway()
    elif key == "l":
        node.disarm_gateway()


def _key_reader(node: KeyboardTeleopNode, stop_event: threading.Event):
    tty_file = open('/dev/tty', 'rb', buffering=0)
    fd  = tty_file.fileno()
    old = termios.tcgetattr(fd)
    tty.setraw(fd)

    _BANNER = (
        '\r\n'
        '╔══════════════════════════════════════════════════════╗\r\n'
        '║  GripperX Keyboard Teleop                            ║\r\n'
        '║  W = Forward  S = Backward  (hold = drive)           ║\r\n'
        '║  A/D = Steer (hold; springs back to straight)        ║\r\n'
        '║  ←/→ = Crab walk left/right   (hold)                 ║\r\n'
        '║  ↑/↓ = steer the crab (front / rear) while crabbing  ║\r\n'
        '║    …only ±10° near pure crab, then a 45° dead band   ║\r\n'
        '║       the wheels must jump: the robot pauses there.  ║\r\n'
        '║  0 = Spin CW   9 = Spin CCW   (hold)                 ║\r\n'
        '║    …←/→/0/9 first swing all four wheels into pose,   ║\r\n'
        '║       the robot only moves once they are there.      ║\r\n'
        '║  K = Keyboard mode   G = Autonomous (Nav2)           ║\r\n'
        '║  P = Grip  O = Gripper open  I = Arm home            ║\r\n'
        '║  U = Arm external gate (120s)  L = Disarm gate       ║\r\n'
        '║  Q / Ctrl+C = Quit                                   ║\r\n'
        '╚══════════════════════════════════════════════════════╝\r\n\n'
    )
    sys.stdout.write(_BANNER)
    sys.stdout.flush()

    # Negotiate AFTER raw mode: the reply is an escape sequence, and a cooked
    # terminal would line-buffer it and echo it at the operator instead.
    releases = enable_release_reporting(tty_file)
    if releases:
        node.enable_release_regime()
        sys.stdout.write(
            '  key release reporting: ON — the robot drives only while the key is down\r\n\n'
        )
    else:
        sys.stdout.write(
            '  key release reporting: not supported by this terminal — falling back to\r\n'
            '  measured auto-repeat timing. Expect a short run-on after releasing W/S.\r\n'
            '  A terminal with the kitty keyboard protocol (kitty, foot, ghostty,\r\n'
            '  WezTerm, recent xterm) removes it.\r\n\n'
        )
    sys.stdout.flush()

    try:
        while not stop_event.is_set():
            r, _, _ = select.select([tty_file], [], [], 0.05)
            if not r:
                continue
            raw = tty_file.read(1).decode('utf-8', errors='ignore')
            if raw == '\x1b':
                # Resolve BEFORE lowercasing: the cursor-key finals are 'A'..'D',
                # and 'A'.lower() == 'a' would read as steer-left.
                decoded = read_sequence(tty_file, _ESC_SEQUENCE_TIMEOUT)
                if decoded is not None:
                    _dispatch(node, decoded[0], decoded[1], stop_event)
                continue
            # Plain bytes. Under flag 8 nothing reaches here at all; without the
            # protocol this is the whole input path, and a plain byte carries no
            # event type, so it is a press by definition.
            ch = raw.lower()
            if ch == '\x03':
                stop_event.set()
                break
            if ch == ' ':
                _dispatch(node, 'space', PRESS, stop_event)
            # 0 and 9 are SPIN CW / CCW since 2026-08-24. They belong in this
            # list for the same reason every other bound key does: under the
            # kitty protocol nothing arrives as plain bytes, and a key missing
            # from the fallback path is a key the operator cannot press when
            # the protocol is absent.
            elif ch in ('w', 's', 'a', 'd', '0', '9',
                        'q', 'k', 'g', 'p', 'o', 'i', 'u', 'l'):
                _dispatch(node, ch, PRESS, stop_event)
    finally:
        # Pop the flags BEFORE restoring the termios settings: leaving them set
        # would hand the operator's shell a terminal that reports every keystroke
        # as an escape sequence — a broken shell after a clean quit.
        disable_release_reporting()
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        tty_file.close()
        sys.stdout.write('\r\nKeyboard teleop ended.\r\n')
        sys.stdout.flush()


def main():
    rclpy.init()
    node = KeyboardTeleopNode()
    stop_event = threading.Event()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    _key_reader(node, stop_event)
    node.center()
    time.sleep(0.15)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
