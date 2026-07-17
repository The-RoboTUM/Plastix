#!/usr/bin/env python3
"""
Keyboard teleop for GripperX — runs on the laptop.

Controls:
  W / S       Forward / backward (deadman: only drives while held)
  A / D       Steer left / right  (cumulative — stays put when released)
  Space       EMERGENCY STOP: stop + straight ahead + back to keyboard mode
  K           Mode: keyboard (manual control)
  G           Mode: autonomous (Nav2 takes over, goal via RViz)
  P           Start full grip sequence
  O           Only open gripper (joint 6)
  I           Arm to home position
  Q / Ctrl+C  Quit
"""
import sys
import tty
import termios
import select
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, String
from gripperx_arm_msgs.action import PickPlastic

STEER_JOINT_COUNT = 4


class KeyboardTeleopNode(Node):

    def __init__(self):
        super().__init__('keyboard_teleop_node')

        self.declare_parameter('steer_rate_rad_s',  0.6)
        self.declare_parameter('steer_limit_rad',   0.785)
        self.declare_parameter('publish_rate_hz',   50.0)
        self.declare_parameter('linear_vel_m_s',    0.5)
        # Deadman window for driving: larger than the steering window (0.15 s), because
        # the terminal waits ~0.5 s after the first keypress before key repeats
        # arrive — otherwise the start-off moment stutters. Releasing = stop after
        # at most this time. For a faster stop: speed up the X11 repeat rate
        # (xset r rate 200 40) and lower this value.
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

        rate          = float(self.get_parameter('publish_rate_hz').value)
        self._rate    = float(self.get_parameter('steer_rate_rad_s').value)
        self._limit   = float(self.get_parameter('steer_limit_rad').value)
        self._lin_vel = float(self.get_parameter('linear_vel_m_s').value)
        self._drive_hold = float(self.get_parameter('drive_hold_sec').value)
        steer_topic   = str(self.get_parameter('direct_steer_topic').value)
        cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        arm_topic     = str(self.get_parameter('arm_command_topic').value)
        self._publish_steer_cmd_vel = bool(self.get_parameter('publish_steer_cmd_vel').value)
        self._steer_to_omega_gain   = float(self.get_parameter('steer_to_omega_gain').value)

        self._steer_pub   = self.create_publisher(Float64MultiArray, steer_topic,         10)
        self._cmd_vel_pub = self.create_publisher(Twist,             cmd_vel_topic,       10)
        self._arm_pub     = self.create_publisher(String,            arm_topic,           10)
        self._mode_pub    = self.create_publisher(String,            '/teleop/set_mode',  10)
        self._pick_client = ActionClient(self, PickPlastic, 'pick_plastic')

        self._lock      = threading.Lock()
        self._key_t     = {k: 0.0 for k in ('a', 'd', 'w', 's')}
        self._steer     = 0.0
        self._dt        = 1.0 / rate
        self._pick_busy = False

        self.create_timer(self._dt, self._publish)

        self.get_logger().info(
            f'Keyboard teleop started | Steer→{steer_topic} | '
            f'Drive→{cmd_vel_topic} | Arm→{arm_topic}'
        )

    # ── Publish tick ─────────────────────────────────────────────────────────

    def _publish(self):
        with self._lock:
            if self._held('a'):
                self._steer = min(self._limit,  self._steer + self._rate * self._dt)
            if self._held('d'):
                self._steer = max(-self._limit, self._steer - self._rate * self._dt)
            angle = self._steer
            # Deadman: only drives while W/S is held (key-repeat window).
            # No latch — safety incident 06.07.: a latched W in a
            # forgotten terminal left the motors running continuously.
            if self._held('w', self._drive_hold):
                drive = 1
            elif self._held('s', self._drive_hold):
                drive = -1
            else:
                drive = 0

        # Steering → direct_steer (steer_servo_node)
        # Front axle and rear axle counter-rotating → cornering instead of crab walk
        steer_msg = Float64MultiArray()
        steer_msg.data = [angle, angle, -angle, -angle]  # FL, FR, BL, BR
        self._steer_pub.publish(steer_msg)

        # Drive → cmd_vel (teleop_mux → swerve_cmd_node → controller)
        cmd = Twist()
        cmd.linear.x = self._lin_vel * drive
        if self._publish_steer_cmd_vel:
            # Sim helper steering (DT-4/M2, see DT-10): mirror the same cumulative
            # steering angle that goes to direct_steer above additionally as omega
            # onto cmd_vel — a single source of truth for "how far
            # steered", on the real robot the value stays unused (default false).
            cmd.angular.z = self._steer * self._steer_to_omega_gain
        self._cmd_vel_pub.publish(cmd)

    # ── Keys ───────────────────────────────────────────────────────────────

    def press(self, key: str):
        with self._lock:
            self._key_t[key] = time.monotonic()
            # W and S are mutually exclusive — invalidate the opposite key immediately,
            # so a direction change doesn't first have to wait out the hold window.
            if key == 'w':
                self._key_t['s'] = 0.0
            elif key == 's':
                self._key_t['w'] = 0.0

    def center(self):
        # EMERGENCY STOP: forces keyboard mode, so the space bar also stops
        # in autonomous mode — teleop_mux publishes zero immediately on the
        # switch and ignores Nav2 commands from then on.
        mode_msg = String()
        mode_msg.data = 'keyboard'
        self._mode_pub.publish(mode_msg)
        with self._lock:
            for k in self._key_t:
                self._key_t[k] = 0.0
            self._steer = 0.0
        # Publish stop immediately
        self._cmd_vel_pub.publish(Twist())
        self.get_logger().info('EMERGENCY STOP: stop + center + keyboard mode')

    def _held(self, key: str, window: float = 0.15) -> bool:
        return (time.monotonic() - self._key_t[key]) < window

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
        '║  A/D = Steer (hold)  Space = EMERGENCY STOP          ║\r\n'
        '║  K = Keyboard mode   G = Autonomous (Nav2)           ║\r\n'
        '║  P = Grip  O = Gripper open  I = Arm home            ║\r\n'
        '║  Q / Ctrl+C = Quit                                   ║\r\n'
        '╚══════════════════════════════════════════════════════╝\r\n\n'
    )
    sys.stdout.write(_BANNER)
    sys.stdout.flush()

    try:
        while not stop_event.is_set():
            r, _, _ = select.select([tty_file], [], [], 0.05)
            if not r:
                continue
            ch = tty_file.read(1).decode('utf-8', errors='ignore').lower()
            if ch in ('\x03', 'q'):
                stop_event.set()
                break
            elif ch in ('w', 's', 'a', 'd'):
                node.press(ch)
            elif ch == ' ':
                node.center()
            elif ch == 'k':
                node.set_mode('keyboard')
            elif ch == 'g':
                node.set_mode('autonomous')
            elif ch == 'p':
                node.trigger_pick()
            elif ch == 'o':
                node.open_gripper()
            elif ch == 'i':
                node.go_home()
    finally:
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
