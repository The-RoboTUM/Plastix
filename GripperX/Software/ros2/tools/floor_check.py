#!/usr/bin/env python3
"""SR-1 WARNING -- THIS SCRIPT COMMANDS MOTION.
Publishes Twist (drive, linear.x) on --topic (default
/teleop/keyboard/cmd_vel): default --speed 0.06 m/s held for --sec default
6.0 s. The speed is chosen below the regulator floor (0.140 m/s) so no
motion is expected -- but "no motion expected" is a prediction, not a
guarantee, and the drive IS commanded regardless. SR-1 (the project's motion-approval
rule) requires an explicit user approval for EACH individual run; an
approval relayed by an agent does not satisfy it.

FR-14 item 12 slow-end floor -- no-motion check. DEPLOY_2026-08-21 section 5.

Commands BELOW the floor (stall_min_command_rad_s) and verifies that every
wheel reports REGULATOR_OFF_BELOW_FLOOR=7 with a correction of exactly 0.0.
At this speed the machine does not break away, so no motion is expected --
but the drive IS commanded.
"""
import sys, time, threading, argparse, math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from gripperx_control_msgs.msg import WheelVelocityReport

R = 0.070
STATUS = {0:"DISABLED",1:"ACTIVE",2:"AT_AUTHORITY_LIMIT",3:"OFF_PROVENANCE",
          4:"OFF_NO_MEASUREMENT",5:"OFF_STALE_FEEDBACK",6:"OFF_STALL_LATCHED",
          7:"OFF_BELOW_FLOOR"}

class F(Node):
    def __init__(self, topic):
        super().__init__("floor_check_tool")
        q = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(Twist, topic, q)
        self.create_subscription(WheelVelocityReport, "/swerve_controller/wheel_velocities",
                                 self.cb, q)
        self.lock = threading.Lock(); self.r = None; self.n = 0
    def cb(self, m):
        with self.lock: self.r = m; self.n += 1

ap = argparse.ArgumentParser()
ap.add_argument("--speed", type=float, default=0.06)
ap.add_argument("--sec", type=float, default=6.0)
ap.add_argument("--topic", default="/teleop/keyboard/cmd_vel")
a = ap.parse_args()

wheel = a.speed / R
print(f"commanded {a.speed} m/s -> wheel {wheel:.4f} rad/s   (floor is 2.0 rad/s = 0.140 m/s)")
print(f"expectation: every wheel reports status 7 OFF_BELOW_FLOOR, correction exactly 0.0\n")

rclpy.init(); n = F(a.topic)
threading.Thread(target=rclpy.spin, args=(n,), daemon=True).start()
t0=time.time()
while n.r is None and time.time()-t0 < 8:
    time.sleep(0.05)
if n.r is None:
    print("ABORT: no report on /swerve_controller/wheel_velocities"); sys.exit(2)

tw = Twist(); tw.linear.x = a.speed
samples = []
t0 = time.time()
while time.time() - t0 < a.sec:
    n.pub.publish(tw)
    time.sleep(0.05)
    with n.lock: r = n.r
    if r: samples.append((list(r.commanded), list(r.correction),
                          list(r.regulator_status), list(r.measured),
                          list(r.joint_names)))
# stop, exact zero
for _ in range(30):
    n.pub.publish(Twist()); time.sleep(0.05)

if not samples:
    print("ABORT: no samples"); sys.exit(2)
names = samples[-1][4]
print(f"{len(samples)} Samples ausgewertet\n")
ok = True
for i, nm in enumerate(names):
    cmds = [s[0][i] for s in samples if abs(s[0][i]) > 1e-9]
    cors = [s[1][i] for s in samples]
    sts  = [s[2][i] for s in samples]
    meas = [s[3][i] for s in samples]
    maxcor = max(abs(c) for c in cors)
    stset = sorted(set(sts))
    cmax = max(abs(c) for c in cmds) if cmds else 0.0
    good = (maxcor == 0.0) and stset == [7]
    ok &= good
    print(f"  {nm}")
    print(f"    commanded max |.|   {cmax:.4f} rad/s  ({'below' if cmax<2.0 else 'ABOVE'} floor)")
    print(f"    correction max |.|  {maxcor:.6f}      {'EXACTLY ZERO' if maxcor==0.0 else 'NOT ZERO'}")
    print(f"    status gesehen      {[STATUS.get(s,s) for s in stset]}")
    print(f"    measured max |.|    {max(abs(m) for m in meas):.4f} rad/s")
    print(f"    -> {'PASS' if good else 'FAIL'}\n")
print("=== ERGEBNIS:", "PASS -- Floor arbeitet wie spezifiziert" if ok else "FAIL -- siehe oben", "===")
rclpy.shutdown()
sys.exit(0 if ok else 1)
