#!/usr/bin/env python3
"""SR-1 WARNING -- THIS SCRIPT COMMANDS MOTION.
Steers all four wheels to 0 deg via /teleop/direct_steer, republished for
HOLD_SEC (first positional arg, default 10.0 s). Steering only -- see below
-- but steering IS motion. SR-1 (the project's motion-approval rule) requires an explicit
user approval for EACH individual run; an approval relayed by an agent does
not satisfy it.

Steer all four wheels to 0 deg via /teleop/direct_steer, then report.

STEERING ONLY. Publishes nothing on /cmd_vel and commands no wheel velocity.
The override is freshness-gated (direct_timeout 0.50 s), so it must be
republished; one participant does both the publishing and the measuring.
"""
import sys, time, math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64MultiArray


HOLD_SEC = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
RATE_HZ = 10.0
LABELS = ["FL", "FR", "BL", "BR"]

class SteerStraight(Node):
    def __init__(self):
        super().__init__("steer_straight_tool")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(Float64MultiArray, "/teleop/direct_steer", qos)
        self.sub = self.create_subscription(Float64MultiArray, "/hw/steer_states", self.on_steer, qos)
        self.latest = None
        self.first = None

    def on_steer(self, msg):
        self.latest = list(msg.data[:4])
        if self.first is None:
            self.first = list(self.latest)

def main():
    rclpy.init()
    n = SteerStraight()
    # settle: wait for the first measurement before commanding anything
    t0 = time.time()
    while n.latest is None and time.time() - t0 < 5.0:
        rclpy.spin_once(n, timeout_sec=0.1)
    if n.latest is None:
        print("ABBRUCH: keine Messung auf /hw/steer_states -- nichts kommandiert.")
        return 2
    print("VORHER  (rad): " + "  ".join(f"{l}={v:+.4f}" for l, v in zip(LABELS, n.first)))
    print("VORHER  (deg): " + "  ".join(f"{l}={math.degrees(v):+7.2f}" for l, v in zip(LABELS, n.first)))

    msg = Float64MultiArray()
    msg.data = [0.0, 0.0, 0.0, 0.0]
    deadline = time.time() + HOLD_SEC
    while time.time() < deadline:
        n.pub.publish(msg)
        rclpy.spin_once(n, timeout_sec=1.0 / RATE_HZ)

    # let the last override expire, then read the settled angles
    t1 = time.time()
    while time.time() - t1 < 1.0:
        rclpy.spin_once(n, timeout_sec=0.05)

    print("NACHHER (rad): " + "  ".join(f"{l}={v:+.4f}" for l, v in zip(LABELS, n.latest)))
    print("NACHHER (deg): " + "  ".join(f"{l}={math.degrees(v):+7.2f}" for l, v in zip(LABELS, n.latest)))
    worst = max(abs(math.degrees(v)) for v in n.latest)
    print(f"groesste Restabweichung: {worst:.2f} deg")
    print("ERGEBNIS: " + ("geradeaus (<2 deg)" if worst < 2.0 else "NICHT geradeaus -- siehe oben"))
    n.destroy_node(); rclpy.shutdown()
    return 0

sys.exit(main())
