#!/usr/bin/env python3
"""ARM action server — fixed pick trajectory via raw Feetech servo positions."""

import sys
import time
import math
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import JointState
from std_msgs.msg import String

sys.path.insert(0, '/home/ubuntu/.local/lib/python3.12/site-packages')
import scservo_sdk as scs

from gripperx_arm_msgs.action import PickPlastic

SERVO_PORT  = '/dev/arm_servo'
BAUDRATE    = 1_000_000
IDS         = [1, 2, 3, 4, 5, 6]
JOINT_NAMES = ['limb1', 'limb2', 'limb3', 'limb4', 'limb5', 'gripper']
GRIPPER_ID  = 6

ADDR_GOAL_POS   = 42
ADDR_MOVE_TIME  = 44
ADDR_PRESENT    = 56
ADDR_TORQUE     = 40

POS_TOLERANCE   = 80   # ticks — target position is considered reached within this value

# 2 Hz instead of 20 Hz: each tick reads 6 servos blocking over serial (the SDK
# busy-polls without sleep) — at 20 Hz this cost an entire CPU core (97%, audit 2026-07-02).
JOINT_STATE_RATE_HZ = 2.0

# These Feetech servos ignore MOVE_TIME/GOAL_SPEED in practice and always move
# at a fixed internal speed (verified 2026-07-06). Speed is therefore enforced
# via a software ramp: many small intermediate setpoints instead of a single
# jump to the target position.
#
# CONSEQUENCE OF THE 2026-08-25 SPEED-UP, worth knowing before shortening these times
# further: step_delay is 1/MOVE_STEP_HZ regardless of duration, so a shorter move_time
# means FEWER setpoints over the same travel, not faster setpoints. The servo still runs
# at its own fixed internal speed. Once the setpoints advance faster than the servo can
# follow, the ramp stops setting the speed and _move() simply returns while the arm is
# still catching up — _verify_positions would then warn (it only warns, it does not fail
# the action). The wait_extra_ms settle times are the margin against that.
MOVE_STEP_HZ = 30.0

# Fallback poses, in IDS order. These are the values recorded before the 2026-08-13
# rework and are only used when no parameter file is loaded. The live values come from
# config/arm_poses.yaml — re-record with `arm_pose_teach` after any mechanical change
# instead of editing them here.
DEFAULT_HOME_POS   = [2069, 963, 3077, 1031, 2042, 1356]
DEFAULT_GRIP_POS   = [2106, 3036, 1513, 2934, 2113, 2652]
DEFAULT_GRIP_OPEN  = 2656   # gripper fully open
DEFAULT_GRIP_CLOSE = 1497   # gripper closed on a plastic item

# TODO(#340): enter the real per-servo travel limits once the Front_Lid is printed and
# the arm is fully assembled. Until then this is the full 12-bit servo range, i.e. the
# clamp in _move() is deliberately inert — the hook exists so the limits have one
# obvious home instead of having to be retrofitted into every call site later.
# The arm joints are also still absent from the URDF (gripperx_v1.core.xacro has no
# limb*/gripper joints), so the digital twin has no limits either. Same issue.
DEFAULT_LIMIT_MIN = [0] * len(IDS)
DEFAULT_LIMIT_MAX = [4095] * len(IDS)


class ArmActionServer(Node):
    def __init__(self):
        super().__init__('arm_action_server')
        self._cb_group = ReentrantCallbackGroup()
        self._serial_lock = threading.Lock()

        self._declare_and_read_parameters()

        self._port = scs.PortHandler(SERVO_PORT)
        self._pkt  = scs.PacketHandler(0)

        if not self._port.openPort():
            raise RuntimeError(f'Cannot open {SERVO_PORT}')
        self._port.setBaudRate(BAUDRATE)

        for i in IDS:
            self._pkt.write1ByteTxRx(self._port, i, ADDR_TORQUE, 1)
        self.get_logger().info(f'Arm connected on {SERVO_PORT}, Torque ON')

        # Startup homing is a real movement the operator has not asked for, and it runs
        # on every respawn. It must be switchable off while the home pose is being
        # re-taught, otherwise each bringup drives the arm onto stale tick values.
        if self._home_on_startup:
            self.get_logger().info('Startup: moving to home position...')
            self._move(self._home_pos, 5067)
            self.get_logger().info('Startup: home position reached')
        else:
            self.get_logger().warn(
                'Startup homing DISABLED (home_on_startup=false) — arm stays where it is. '
                'Send "go_home" on /arm/command when the home pose is confirmed.'
            )

        self._joint_pub = self.create_publisher(JointState, '/arm/joint_states', 10)
        self.create_timer(1.0 / JOINT_STATE_RATE_HZ, self._publish_joint_states)

        self.create_subscription(String, '/arm/command', self._on_command,
                                 10, callback_group=self._cb_group)

        self._action_server = ActionServer(
            self, PickPlastic, 'pick_plastic',
            execute_callback=self._execute_cb,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self._cb_group,
        )
        self.get_logger().info(
            'arm_action_server ready — Action: /pick_plastic | Topic: /arm/command'
        )

    def _declare_and_read_parameters(self):
        """Poses and travel limits come from config/arm_poses.yaml.

        Everything falls back to the DEFAULT_* constants, so running the node without a
        parameter file behaves exactly as it did before the poses were made configurable.
        """
        self.declare_parameter('poses.home', DEFAULT_HOME_POS)
        self.declare_parameter('poses.grip', DEFAULT_GRIP_POS)
        self.declare_parameter('gripper.open', DEFAULT_GRIP_OPEN)
        self.declare_parameter('gripper.close', DEFAULT_GRIP_CLOSE)
        self.declare_parameter('joint_limits.min', DEFAULT_LIMIT_MIN)
        self.declare_parameter('joint_limits.max', DEFAULT_LIMIT_MAX)
        self.declare_parameter('home_on_startup', True)

        self._home_pos = self._pose_from_parameter('poses.home')
        self._grip_pos = self._pose_from_parameter('poses.grip')
        self._grip_open = int(self.get_parameter('gripper.open').value)
        self._grip_close = int(self.get_parameter('gripper.close').value)
        self._limit_min = self._ticks_from_parameter('joint_limits.min')
        self._limit_max = self._ticks_from_parameter('joint_limits.max')
        self._home_on_startup = bool(self.get_parameter('home_on_startup').value)

        for index, sid in enumerate(IDS):
            if self._limit_min[index] > self._limit_max[index]:
                raise RuntimeError(
                    f'joint_limits: min > max for servo id={sid} '
                    f'({self._limit_min[index]} > {self._limit_max[index]})'
                )

        # Sequence: (step_name, positions_dict, move_time_ms, wait_extra_ms)
        #
        # SPEED, 2026-08-25: every move_time in this node was shortened to 2/3 of its
        # previous value — arm motion 50 % faster, on user instruction. The six values
        # (5067 / 4267 / 4000 / 2400 / 2400 / 1333 ms) were scaled AS A SET; if the speed
        # is changed again, change them together, or the sequence loses its shape.
        #
        # grip_close was 3600 ms and is now 2400. It had been set slow DELIBERATELY
        # (instead of 800 ms) because a short move_time forces the servo to reach the
        # target in a short time -> high current spike while gripping, and slower movement
        # reduces the current draw. That trade-off was put to the user and the speed-up was
        # confirmed for the whole arm including this step. 2400 ms is still far above the
        # 800 ms the original comment warned about, but the current draw while gripping HAS
        # NOT BEEN MEASURED at the new value. If gripping starts browning out the rail or
        # tripping the servo, this is the first number to put back.
        #
        # wait_extra_ms is NOT scaled: it is settle time, not motion. It is the margin that
        # lets the servos catch up before _verify_positions reads them, and shorter moves
        # need that margin more, not less.
        self._pick_sequence = [
            ('approach',    {**self._grip_pos, GRIPPER_ID: self._grip_open}, 4000, 800),
            ('grip_close',  {GRIPPER_ID: self._grip_close},                  2400, 700),
            ('return_home', dict(self._home_pos),                            4267, 900),
        ]

    def _ticks_from_parameter(self, name: str) -> list:
        values = [int(v) for v in self.get_parameter(name).value]
        if len(values) != len(IDS):
            raise RuntimeError(
                f'Parameter {name} has {len(values)} values, expected {len(IDS)} '
                f'(one per servo, in id order {IDS})'
            )
        return values

    def _pose_from_parameter(self, name: str) -> dict:
        return dict(zip(IDS, self._ticks_from_parameter(name)))

    def _clamp_ticks(self, sid: int, ticks: int) -> int:
        """Keep a goal position inside the configured travel limits for that servo.

        Inert while joint_limits is the full 0..4095 range — see TODO(#340) above.
        """
        index = IDS.index(sid)
        return max(self._limit_min[index], min(self._limit_max[index], int(ticks)))

    def _move(self, positions: dict, move_time_ms: int) -> dict:
        """Moves linearly interpolated to the target position over move_time_ms.

        Blocks for the duration of the movement (ramp instead of firmware timing).
        Returns the positions actually commanded, i.e. after clamping — verification
        has to compare against those, not against the requested ones.
        """
        # Clamp the goals once up front, so a bad pose in the config cannot be driven
        # to. Each interpolated setpoint is clamped again below, because the ramp starts
        # from the measured position, which may itself sit outside the limits.
        targets = {sid: self._clamp_ticks(sid, ticks) for sid, ticks in positions.items()}
        for sid, ticks in positions.items():
            if targets[sid] != ticks:
                self.get_logger().warn(
                    f'Goal for servo {sid} clamped {ticks} -> {targets[sid]} by joint_limits'
                )

        with self._serial_lock:
            starts = {}
            for sid in targets:
                val, res, _ = self._pkt.read2ByteTxRx(self._port, sid, ADDR_PRESENT)
                starts[sid] = val if res == scs.COMM_SUCCESS else targets[sid]

            duration_s = move_time_ms / 1000.0
            steps = max(1, int(duration_s * MOVE_STEP_HZ))
            step_delay = duration_s / steps

            for i in range(1, steps + 1):
                frac = i / steps
                for sid, target in targets.items():
                    pos = int(round(starts[sid] + (target - starts[sid]) * frac))
                    self._pkt.write2ByteTxRx(
                        self._port, sid, ADDR_GOAL_POS, self._clamp_ticks(sid, pos)
                    )
                time.sleep(step_delay)

        return targets

    def _on_command(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd == 'open_gripper':
            self.get_logger().info('arm/command: opening gripper')
            self._move({GRIPPER_ID: self._grip_open}, 2400)
        elif cmd == 'go_home':
            self.get_logger().info('arm/command: home position')
            self._move(self._home_pos, 4267)
        else:
            self.get_logger().warn(f'arm/command: unknown "{cmd}"')

    def _read_pos(self, sid: int):
        # Lock mandatory: runs in parallel with _move() (timer vs. action/command callback),
        # concurrent bus access corrupts packets.
        with self._serial_lock:
            val, res, _ = self._pkt.read2ByteTxRx(self._port, sid, ADDR_PRESENT)
        return val if res == scs.COMM_SUCCESS else None

    def _verify_positions(self, targets: dict, step: str):
        """Log warning if any servo didn't reach its target."""
        for sid, target in targets.items():
            actual = self._read_pos(sid)
            if actual is None:
                self.get_logger().warn(f'{step}: ID {sid} not responding — position unknown')
                continue
            delta = abs(actual - target)
            if delta > POS_TOLERANCE:
                self.get_logger().warn(
                    f'{step}: ID {sid} Target={target} Actual={actual} Delta={delta} — not reached!'
                )

    def _publish_joint_states(self):
        if not rclpy.ok():
            return
        try:
            positions = []
            for sid in IDS:
                val = self._read_pos(sid)
                if val is None:
                    self.get_logger().warn(
                        f'joint_states: servo {sid} not responding',
                        throttle_duration_sec=10.0,
                    )
                    return
                positions.append(math.radians(val / 4096 * 360))
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = JOINT_NAMES
            msg.position = positions
            self._joint_pub.publish(msg)
        except Exception as exc:
            self.get_logger().warn(f'joint_states: {exc}', throttle_duration_sec=10.0)

    def _execute_cb(self, goal_handle):
        self.get_logger().info('pick_plastic: start')
        feedback = PickPlastic.Feedback()
        total = len(self._pick_sequence)

        for idx, (step, positions, move_ms, extra_ms) in enumerate(self._pick_sequence):
            if goal_handle.is_cancel_requested:
                self.get_logger().info('pick_plastic: canceled — moving home')
                self._move(self._home_pos, 1333)
                goal_handle.canceled()
                result = PickPlastic.Result()
                result.success = False
                result.message = 'Canceled'
                return result

            self.get_logger().info(f'pick_plastic: {idx+1}/{total} — {step}')
            commanded = self._move(positions, move_ms)  # already blocks for move_ms (ramp)

            feedback.progress = (idx + 1) / total
            feedback.current_step = step
            goal_handle.publish_feedback(feedback)

            time.sleep(extra_ms / 1000.0)
            # Verify against what was actually commanded, so a clamped goal does not
            # report as "not reached".
            self._verify_positions(commanded, step)

        goal_handle.succeed()
        result = PickPlastic.Result()
        result.success = True
        result.message = 'Grip sequence completed'
        self.get_logger().info('pick_plastic: done')
        return result

    def destroy_node(self):
        self._port.closePort()
        super().destroy_node()


def main():
    rclpy.init()
    node = ArmActionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
