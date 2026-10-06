#!/usr/bin/env python3
"""Cross-check two odometry sources against each other and say so when they part.

WHY THIS EXISTS. On 2026-08-21 a Nav2 run in the twin reported SUCCEEDED with the
robot 2.47 m off target and ZERO recovery behaviours. Nothing in the stack
objected, and nothing could have: every Nav2 tolerance compares the robot pose
against the goal in the SAME estimated frame, so an error in the estimate cancels
out of the comparison exactly. The goal checker is satisfied when the ESTIMATE
arrives, wherever the machine actually is.

Ground truth does not exist outside simulation, so no tolerance can be checked
against "the truth". But two odometry sources measuring the same motion by
physically different means ARE independent of each other, and that is checkable.
In the failure above, wheel odometry advanced 2.618 m while laser odometry
advanced 0.02 m over the same 5.4 s. This node would have said so.

WHAT IT DOES NOT DO. It does not decide which source is right — it cannot. It
reports that they disagree, with both numbers, and leaves the verdict to whoever
reads it. It changes no command and touches no control path.

THRESHOLD PROVENANCE, measured rather than picked: over a 2.0 s sliding window,
    healthy run   median 0.021 m   p95 0.063 m   max 0.087 m
    locked-up run median 0.369 m   p95 0.931 m   max 0.961 m
The default 0.25 m sits ~2.9x above the worst healthy value and well below the
median of the failure. Re-derive it if the window changes; the two do not scale
independently.
"""
from collections import deque

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from rclpy.node import Node


class DivergenceWindow:
    """The whole decision, with no ROS in it, so it can be tested against a CSV."""

    OK = "OK"
    DIVERGED = "DIVERGED"
    INSUFFICIENT_MOTION = "INSUFFICIENT_MOTION"
    NO_DATA = "NO_DATA"

    def __init__(self, window_sec: float, min_travel_m: float, max_divergence_m: float):
        self.window_sec = float(window_sec)
        self.min_travel_m = float(min_travel_m)
        self.max_divergence_m = float(max_divergence_m)
        self._a = deque()  # (t, x, y) source A
        self._b = deque()

    @staticmethod
    def _travel(buf) -> float:
        if len(buf) < 2:
            return 0.0
        return ((buf[-1][1] - buf[0][1]) ** 2 + (buf[-1][2] - buf[0][2]) ** 2) ** 0.5

    def _trim(self, buf, now):
        while len(buf) > 1 and now - buf[0][0] > self.window_sec:
            buf.popleft()

    def push_a(self, t, x, y):
        self._a.append((t, x, y))
        self._trim(self._a, t)

    def push_b(self, t, x, y):
        self._b.append((t, x, y))
        self._trim(self._b, t)

    def evaluate(self):
        """-> (state, travel_a, travel_b, divergence)"""
        if len(self._a) < 2 or len(self._b) < 2:
            return self.NO_DATA, 0.0, 0.0, 0.0
        ta, tb = self._travel(self._a), self._travel(self._b)
        div = abs(ta - tb)
        # Below the floor the comparison is dominated by quantisation and says
        # nothing. Reporting OK there would be a false assurance, so it gets its
        # own state rather than being folded into OK.
        if max(ta, tb) < self.min_travel_m:
            return self.INSUFFICIENT_MOTION, ta, tb, div
        if div > self.max_divergence_m:
            return self.DIVERGED, ta, tb, div
        return self.OK, ta, tb, div


class OdomDivergenceMonitor(Node):
    def __init__(self):
        super().__init__("odom_divergence_monitor")
        self.declare_parameter("source_a_topic", "/wheel/odom")
        self.declare_parameter("source_b_topic", "/laser/odom")
        self.declare_parameter("window_sec", 2.0)
        self.declare_parameter("min_travel_m", 0.10)
        self.declare_parameter("max_divergence_m", 0.25)
        self.declare_parameter("publish_period_sec", 0.5)

        a_topic = str(self.get_parameter("source_a_topic").value)
        b_topic = str(self.get_parameter("source_b_topic").value)
        self.window = DivergenceWindow(
            self.get_parameter("window_sec").value,
            self.get_parameter("min_travel_m").value,
            self.get_parameter("max_divergence_m").value,
        )
        self.a_topic, self.b_topic = a_topic, b_topic
        self.create_subscription(Odometry, a_topic, self._on_a, 20)
        self.create_subscription(Odometry, b_topic, self._on_b, 20)
        self.diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.create_timer(float(self.get_parameter("publish_period_sec").value), self._tick)
        self._last_state = None
        self.get_logger().info(
            f"comparing {a_topic} against {b_topic}: window "
            f"{self.window.window_sec:.1f} s, alarm above "
            f"{self.window.max_divergence_m:.2f} m, floor {self.window.min_travel_m:.2f} m"
        )

    @staticmethod
    def _stamp(msg) -> float:
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _on_a(self, msg):
        self.window.push_a(self._stamp(msg), msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _on_b(self, msg):
        self.window.push_b(self._stamp(msg), msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _tick(self):
        state, ta, tb, div = self.window.evaluate()
        level = {
            DivergenceWindow.OK: DiagnosticStatus.OK,
            DivergenceWindow.DIVERGED: DiagnosticStatus.ERROR,
            DivergenceWindow.INSUFFICIENT_MOTION: DiagnosticStatus.OK,
            DivergenceWindow.NO_DATA: DiagnosticStatus.WARN,
        }[state]

        status = DiagnosticStatus()
        status.level = level
        status.name = "localization: odometry cross-check"
        status.hardware_id = "gripperx"
        status.message = {
            DivergenceWindow.OK: "sources agree",
            DivergenceWindow.DIVERGED: (
                f"SOURCES DISAGREE by {div:.3f} m over {self.window.window_sec:.1f} s "
                f"- one of them is not measuring the ground"
            ),
            DivergenceWindow.INSUFFICIENT_MOTION: "too little motion to judge",
            DivergenceWindow.NO_DATA: "waiting for both sources",
        }[state]
        status.values = [
            KeyValue(key="state", value=state),
            KeyValue(key="source_a", value=self.a_topic),
            KeyValue(key="source_b", value=self.b_topic),
            KeyValue(key="travel_a_m", value=f"{ta:.4f}"),
            KeyValue(key="travel_b_m", value=f"{tb:.4f}"),
            KeyValue(key="divergence_m", value=f"{div:.4f}"),
            KeyValue(key="threshold_m", value=f"{self.window.max_divergence_m:.4f}"),
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status.append(status)
        self.diag_pub.publish(array)

        # Edge-triggered on the log, so a sustained fault does not drown the log
        # while a transition is never silent.
        if state != self._last_state:
            if state == DivergenceWindow.DIVERGED:
                self.get_logger().error(
                    f"{self.a_topic} travelled {ta:.3f} m while {self.b_topic} travelled "
                    f"{tb:.3f} m over {self.window.window_sec:.1f} s "
                    f"(divergence {div:.3f} m > {self.window.max_divergence_m:.2f} m). "
                    "One source is not measuring ground motion; the pose estimate "
                    "downstream cannot be trusted until this clears."
                )
            elif self._last_state == DivergenceWindow.DIVERGED:
                self.get_logger().info("odometry sources agree again")
            self._last_state = state


def main(args=None):
    rclpy.init(args=args)
    node = OdomDivergenceMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
