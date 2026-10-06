#!/usr/bin/env python3
"""SR-1 WARNING -- THIS SCRIPT COMMANDS MOTION.
Publishes Twist (drive, linear.x) on --topic (default
/teleop/keyboard/cmd_vel): default --vx 0.16 m/s held for --sec default
3.0 s, i.e. ~0.48 m straight forward at default settings. SR-1
(the project's motion-approval rule) requires an explicit user approval for EACH
individual run; an approval relayed by an agent does not satisfy it.

Does wheel odometry decode a STRAIGHT DRIVE as a straight drive?

The Pi runs drive_joint_multipliers [1,1,-1,-1] over
[f_leftwheel, b_leftwheel, b_rightwheel, f_rightwheel] -- it negates the right
side. That is the SIM convention (the URDF used to mirror the right wheel joint
axes). On hardware the firmware has ALREADY normalised into the robot frame, so
the negation should turn a straight drive into an apparent in-place rotation.

Derived, never measured. This measures it.
"""
import sys, time, threading, argparse, math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

ap = argparse.ArgumentParser()
ap.add_argument("--vx", type=float, default=0.16)
ap.add_argument("--sec", type=float, default=3.0)
ap.add_argument("--topic", default="/teleop/keyboard/cmd_vel")
a = ap.parse_args()

class O(Node):
    def __init__(self):
        super().__init__("odom_check_tool")
        q = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(Twist, a.topic, q)
        self.create_subscription(Odometry, "/wheel/odom", self.cb, q)
        self.lock = threading.Lock(); self.s = []
    def cb(self, m):
        with self.lock:
            self.s.append((m.twist.twist.linear.x, m.twist.twist.linear.y,
                           m.twist.twist.angular.z))

print("commanding vx = %+.3f m/s STRAIGHT for %.1f s" % (a.vx, a.sec))
print("  correct  -> odom linear.x ~ %+.3f, angular.z ~ 0" % a.vx)
print("  defective-> odom linear.x ~ 0, angular.z large\n")
rclpy.init(); n = O()
threading.Thread(target=rclpy.spin, args=(n,), daemon=True).start()
t0 = time.time()
while not n.s and time.time() - t0 < 8:
    time.sleep(0.05)
if not n.s:
    print("ABORT: nothing on /wheel/odom -- nothing commanded."); sys.exit(2)
with n.lock:
    n.s.clear()
tw = Twist(); tw.linear.x = a.vx
t0 = time.time()
while time.time() - t0 < a.sec:
    n.pub.publish(tw); time.sleep(0.05)
for _ in range(30):
    n.pub.publish(Twist()); time.sleep(0.05)
with n.lock:
    S = list(n.s)
if len(S) < 10:
    print("ABORT: too few samples (%d)." % len(S)); sys.exit(3)
tail = S[len(S)//3:]
vx = sum(s[0] for s in tail)/len(tail)
vy = sum(s[1] for s in tail)/len(tail)
wz = sum(s[2] for s in tail)/len(tail)
print("=== ODOMETRY REPORTED (steady state, %d samples) ===" % len(tail))
print("  linear.x  %+.4f m/s   (commanded %+.4f)" % (vx, a.vx))
print("  linear.y  %+.4f m/s" % vy)
print("  angular.z %+.4f rad/s  = %+.1f deg/s" % (wz, math.degrees(wz)))
print()
if abs(vx) > 0.5*abs(a.vx) and abs(wz) < 0.2:
    print("=== VERDICT: odometry decodes the straight drive CORRECTLY.")
    print("    The derivation was WRONG. Do not change the multipliers.")
elif abs(wz) > 0.3 and abs(vx) < 0.5*abs(a.vx):
    print("=== VERDICT: DEFECT CONFIRMED -- a straight drive is decoded as ROTATION.")
    print("    drive_joint_multipliers must become [1,1,1,1] on real hardware.")
else:
    print("=== VERDICT: NEITHER cleanly. Do not act on this run; investigate.")
rclpy.shutdown()
