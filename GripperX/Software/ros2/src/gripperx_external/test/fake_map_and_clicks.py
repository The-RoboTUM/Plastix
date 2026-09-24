#!/usr/bin/env python3
"""A slam_toolbox stand-in and an RViz operator, for the twin checks only.

    ROS_DOMAIN_ID=221 python3 test/fake_map_and_clicks.py --a -1 0 --b 1 0

It does two things and nothing else:

* holds a publisher on ``/map`` (a tiny latched ``OccupancyGrid``). Its DDS
  endpoint GID (global identifier) is what the line calibration records as the
  map session, so killing this process and starting another one IS a
  slam_toolbox restart as far as the calibration node and the gateway can tell;
* plays the operator: publishes clicks on ``/clicked_point`` in ``map``, A then
  B, and repeats the pair until the calibration node reports ``calibrated``
  (the first pair can land before that node has discovered ``/map``, which it
  then refuses - correctly - with NO_MAP_SESSION).

Without ``--a/--b`` it clicks nothing until told to on stdin:
``pair ax ay bx by`` (a full pair, retried like above) or ``click x y`` (one
single click, e.g. to start a recalibration).

It refuses the real robot's domain: it publishes a ``/map``.
"""

import argparse
import json
import os
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String

STATUS_TOPIC = "/gripperx/external/line_calibration"


def _latched():
    return QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                      reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


class FakeMapAndClicks(Node):
    def __init__(self):
        super().__init__("fake_map_and_clicks")
        self._map_pub = self.create_publisher(OccupancyGrid, "/map", _latched())
        self._click_pub = self.create_publisher(PointStamped, "/clicked_point", 10)
        self._status = {}
        self._lock = threading.Lock()
        self.create_subscription(String, STATUS_TOPIC, self._on_status, _latched())
        grid = OccupancyGrid()
        grid.header.frame_id = "map"
        grid.info.resolution = 0.5
        grid.info.width = grid.info.height = 2
        grid.info.origin.orientation.w = 1.0
        grid.data = [0] * 4
        self._map_pub.publish(grid)
        self.get_logger().info("holding /map (this process = one map session)")

    def _on_status(self, msg):
        with self._lock:
            self._status = json.loads(msg.data)

    def status(self):
        with self._lock:
            return dict(self._status)

    def click(self, x, y):
        p = PointStamped()
        p.header.frame_id = "map"
        p.header.stamp = self.get_clock().now().to_msg()
        p.point.x, p.point.y = float(x), float(y)
        self._click_pub.publish(p)
        self.get_logger().info(f"clicked ({x:.3f}, {y:.3f})")

    def pair(self, a, b, attempts=15):
        deadline = time.time() + 30.0
        while self._click_pub.get_subscription_count() < 1 and time.time() < deadline:
            time.sleep(0.2)
        before = self.status().get("calibration_count", 0)
        for _ in range(attempts):
            self.click(*a)
            # Let the A click's status arrive (it clears any earlier refusal),
            # so an old refusal is not mistaken for this pair's verdict.
            end = time.time() + 2.0
            while time.time() < end and self.status().get("rejected"):
                time.sleep(0.05)
            time.sleep(0.3)
            self.click(*b)
            end = time.time() + 2.0
            while time.time() < end:
                s = self.status()
                if s.get("state") == "calibrated" and s.get("calibration_count", 0) > before:
                    self.get_logger().info(
                        f"CALIBRATED #{s['calibration_id']} L={s['length_m']:.3f}")
                    return True
                if s.get("rejected") and s.get("reason") != "NO_MAP_SESSION":
                    # A refusal on the pair itself (length range, degenerate):
                    # clicking it again would be refused again. The operator
                    # would pick new points; the scenario decides that.
                    self.get_logger().warn(f"pair refused: {s.get('reason')}")
                    return False
                time.sleep(0.1)
        self.get_logger().error("calibration did not complete")
        return False


def main():
    if os.environ.get("ROS_DOMAIN_ID") in (None, "", "20"):
        print("refusing: this fixture publishes /map and is for twin domains only",
              file=sys.stderr)
        return 2
    parser = argparse.ArgumentParser()
    parser.add_argument("--a", nargs=2, type=float)
    parser.add_argument("--b", nargs=2, type=float)
    args = parser.parse_args()

    rclpy.init()
    node = FakeMapAndClicks()
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()
    try:
        if args.a and args.b:
            node.pair(args.a, args.b)
        for line in sys.stdin:
            words = line.split()
            if not words:
                continue
            if words[0] == "pair" and len(words) == 5:
                v = [float(w) for w in words[1:]]
                node.pair(v[:2], v[2:])
            elif words[0] == "click" and len(words) == 3:
                node.click(float(words[1]), float(words[2]))
        while rclpy.ok():
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
