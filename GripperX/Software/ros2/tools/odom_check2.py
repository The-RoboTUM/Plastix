#!/usr/bin/env python3
"""SR-1 WARNING -- THIS SCRIPT COMMANDS MOTION.
Publishes Twist (drive, linear.x) on --topic (default
/teleop/keyboard/cmd_vel): default --vx 0.16 m/s held for --sec default
3.0 s, i.e. ~0.48 m straight forward at default settings. SR-1
(the project's motion-approval rule) requires an explicit user approval for EACH
individual run; an approval relayed by an agent does not satisfy it.

Straight drive: what the wheels DID, what /joint_states CARRIES, what odometry SAYS.

The first attempt reported odom linear.x ~ 0 (commanded 0.16) with angular.z
-0.22 rad/s. The zero forward speed is damning on its own, but the rotation is a
quarter of what a negated right side predicts (-v/0.164 = -0.97), so the simple
"multipliers are wrong" story does not fit the number. Two things were missing:
whether the robot moved at all, and what the node actually reads.

This measures all three in ONE run so they cannot disagree about which run they
describe.
"""
import sys, time, threading, argparse, math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from gripperx_control_msgs.msg import WheelVelocityReport

ap = argparse.ArgumentParser()
ap.add_argument("--vx", type=float, default=0.16)
ap.add_argument("--sec", type=float, default=3.0)
ap.add_argument("--topic", default="/teleop/keyboard/cmd_vel")
a = ap.parse_args()


class O(Node):
    def __init__(self):
        super().__init__("odom_check2_tool")
        q = QoSProfile(depth=30, reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(Twist, a.topic, q)
        self.create_subscription(Odometry, "/wheel/odom", self.cb_o, q)
        self.create_subscription(WheelVelocityReport,
                                 "/swerve_controller/wheel_velocities", self.cb_w, q)
        self.create_subscription(JointState, "/joint_states", self.cb_j, q)
        self.lock = threading.Lock()
        self.o, self.w, self.j = [], [], []

    def cb_o(self, m):
        with self.lock:
            self.o.append((m.twist.twist.linear.x, m.twist.twist.linear.y,
                           m.twist.twist.angular.z))

    def cb_w(self, m):
        with self.lock:
            self.w.append((list(m.joint_names), list(m.measured)))

    def cb_j(self, m):
        with self.lock:
            self.j.append((list(m.name), list(m.velocity)))


print("commanding vx = %+.3f m/s STRAIGHT for %.1f s\n" % (a.vx, a.sec))
rclpy.init()
n = O()
threading.Thread(target=rclpy.spin, args=(n,), daemon=True).start()
t0 = time.time()
while (not n.o or not n.w or not n.j) and time.time() - t0 < 10:
    time.sleep(0.05)
missing = [t for t, v in (("/wheel/odom", n.o),
                          ("/swerve_controller/wheel_velocities", n.w),
                          ("/joint_states", n.j)) if not v]
if missing:
    print("ABORT: no data on %s -- nothing commanded." % missing)
    sys.exit(2)
with n.lock:
    n.o.clear(); n.w.clear(); n.j.clear()

tw = Twist(); tw.linear.x = a.vx
t0 = time.time()
while time.time() - t0 < a.sec:
    n.pub.publish(tw); time.sleep(0.05)
for _ in range(30):
    n.pub.publish(Twist()); time.sleep(0.05)

with n.lock:
    O_, W_, J_ = list(n.o), list(n.w), list(n.j)
if len(O_) < 10 or len(W_) < 10 or len(J_) < 10:
    print("ABORT: too few samples (%d/%d/%d)." % (len(O_), len(W_), len(J_)))
    sys.exit(3)

print("=== 1. DID IT MOVE?  /swerve_controller/wheel_velocities ===")
names, _ = W_[0]
wt = W_[len(W_) // 3:]
wm = [sum(x[1][i] for x in wt) / len(wt) for i in range(len(names))]
for nm, v in zip(names, wm):
    print("    %-16s %+.4f rad/s  (= %+.4f m/s at r=0.070)" % (nm, v, v * 0.070))

print("\n=== 2. WHAT THE NODE READS:  /joint_states ===")
jn, jv = J_[len(J_) // 2]
for nm, v in zip(jn, jv):
    if "wheel" in nm:
        print("    %-16s %+.4f rad/s" % (nm, v))

print("\n=== 3. WHAT ODOMETRY SAYS:  /wheel/odom ===")
ot = O_[len(O_) // 3:]
vx = sum(s[0] for s in ot) / len(ot)
vy = sum(s[1] for s in ot) / len(ot)
wz = sum(s[2] for s in ot) / len(ot)
print("    linear.x  %+.4f m/s   (commanded %+.4f)" % (vx, a.vx))
print("    linear.y  %+.4f m/s" % vy)
print("    angular.z %+.4f rad/s = %+.1f deg/s" % (wz, math.degrees(wz)))

print("\n=== READING ===")
moved = max(abs(v) for v in wm) > 1.0
print("    wheels turned: %s" % ("YES" if moved else "NO -- everything else is moot"))
if moved:
    print("    odometry forward speed is %.0f %% of commanded" % (100.0 * vx / a.vx))
    print("    a negated right side would predict angular.z = %+.3f rad/s"
          % (-a.vx / 0.164222))
rclpy.shutdown()
