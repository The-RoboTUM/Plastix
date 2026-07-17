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
MOVE_STEP_HZ = 30.0

# Recorded raw positions
GRIP_POS  = {1: 2106, 2: 3036, 3: 1513, 4: 2934, 5: 2113, 6: 2652}
HOME_POS  = {1: 2069, 2:  963, 3: 3077, 4: 1031, 5: 2042, 6: 1356}
GRIP_OPEN = 2656   # Joint 6 fully open

# Sequence: (step_name, positions_dict, move_time_ms, wait_extra_ms)
# grip_close deliberately slow (instead of 800ms): a short move_time forces the servo
# to reach the target position in a short time -> high current spike while
# gripping. Slower movement reduces the current draw.
PICK_SEQUENCE = [
    ('approach',      {**GRIP_POS, 6: GRIP_OPEN}, 6000, 800),
    ('grip_close',    {6: 1497},                   3600, 700),
    ('return_home',   HOME_POS,                    6400, 900),
]


class ArmActionServer(Node):
    def __init__(self):
        super().__init__('arm_action_server')
        self._cb_group = ReentrantCallbackGroup()
        self._serial_lock = threading.Lock()

        self._port = scs.PortHandler(SERVO_PORT)
        self._pkt  = scs.PacketHandler(0)

        if not self._port.openPort():
            raise RuntimeError(f'Cannot open {SERVO_PORT}')
        self._port.setBaudRate(BAUDRATE)

        for i in IDS:
            self._pkt.write1ByteTxRx(self._port, i, ADDR_TORQUE, 1)
        self.get_logger().info(f'Arm connected on {SERVO_PORT}, Torque ON')
        self.get_logger().info('Startup: moving to home position...')
        self._move(HOME_POS, 7600)
        self.get_logger().info('Startup: home position reached')

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

    def _move(self, positions: dict, move_time_ms: int):
        """Moves linearly interpolated to the target position over move_time_ms.
        Blocks for the duration of the movement (ramp instead of firmware timing)."""
        with self._serial_lock:
            starts = {}
            for sid in positions:
                val, res, _ = self._pkt.read2ByteTxRx(self._port, sid, ADDR_PRESENT)
                starts[sid] = val if res == scs.COMM_SUCCESS else positions[sid]

            duration_s = move_time_ms / 1000.0
            steps = max(1, int(duration_s * MOVE_STEP_HZ))
            step_delay = duration_s / steps

            for i in range(1, steps + 1):
                frac = i / steps
                for sid, target in positions.items():
                    pos = int(round(starts[sid] + (target - starts[sid]) * frac))
                    self._pkt.write2ByteTxRx(self._port, sid, ADDR_GOAL_POS, pos)
                time.sleep(step_delay)

    def _on_command(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd == 'open_gripper':
            self.get_logger().info('arm/command: opening gripper')
            self._move({6: GRIP_OPEN}, 3600)
        elif cmd == 'go_home':
            self.get_logger().info('arm/command: home position')
            self._move(HOME_POS, 6400)
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
        total = len(PICK_SEQUENCE)

        for idx, (step, positions, move_ms, extra_ms) in enumerate(PICK_SEQUENCE):
            if goal_handle.is_cancel_requested:
                self.get_logger().info('pick_plastic: canceled — moving home')
                self._move(HOME_POS, 2000)
                goal_handle.canceled()
                result = PickPlastic.Result()
                result.success = False
                result.message = 'Canceled'
                return result

            self.get_logger().info(f'pick_plastic: {idx+1}/{total} — {step}')
            self._move(positions, move_ms)  # already blocks for move_ms (ramp)

            feedback.progress = (idx + 1) / total
            feedback.current_step = step
            goal_handle.publish_feedback(feedback)

            time.sleep(extra_ms / 1000.0)
            self._verify_positions(positions, step)

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
