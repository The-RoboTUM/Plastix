#!/usr/bin/env python3
"""SR-1 WARNING -- THIS SCRIPT COMMANDS MOTION.
Publishes a spin Twist (angular.z, default omega=0.15 rad/s for --sec
default 4.0 s) on --topic (default /teleop/keyboard/cmd_vel), sized to stay
below the measured breakaway threshold -- but "no motion expected" is a
prediction, not a guarantee. Also drives all four wheels to a spin pose via
/teleop/direct_steer (steering motion, up to --align-sec default 30 s).
SR-1 (the project's motion-approval rule) requires an explicit user approval for EACH
individual run; an approval relayed by an agent does not satisfy it.

Bench check for the contact-point correction — OP29_SPIN_REPAIR.md step 0.

Reads what the controller COMMANDS for a spin twist and checks it against the
design. It falsifies the implementation without the robot travelling anywhere:
omega is chosen so the contact speed sits below the measured breakaway
(0.050 m/s), and the machine has been shown today not to move there.

WHAT IT CATCHES. The one failure mode that makes this repair WORSE than doing
nothing is applying the correction BEFORE the +-180 deg module fold, which
inverts it on exactly the wheels that roll backwards -- FL and BL in a spin.
That shows up here as FL/BL commanded LOW instead of high, while FR/BR are high.
"""
import sys, time, math, threading, argparse
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from gripperx_control_msgs.msg import WheelVelocityReport

R = 0.070
R_KINGPIN = math.hypot(0.180, 0.110)
H = [0.055572, -0.055372, 0.055572, -0.055571]      # joint order FL, FR, BL, BR
SPIN_DEG = [-58.570, +58.570, +58.570, -58.570]
WINDOW_DEG = [(-100.0, 35.0), (-35.0, 100.0), (-35.0, 100.0), (-100.0, 35.0)]
LABELS = ["FL", "FR", "BL", "BR"]


class Bench(Node):
    def __init__(self, topic):
        super().__init__("bench_check_tool")
        q = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST)
        self.cmd = self.create_publisher(Twist, topic, q)
        self.steer = self.create_publisher(Float64MultiArray, "/teleop/direct_steer", q)
        self.create_subscription(Float64MultiArray, "/hw/steer_states", self.on_steer, q)
        self.create_subscription(WheelVelocityReport,
                                 "/swerve_controller/wheel_velocities", self.on_rep, q)
        self.lock = threading.Lock()
        self.steer_meas = None
        self.rep = None

    def on_steer(self, m):
        with self.lock:
            self.steer_meas = list(m.data[:4])

    def on_rep(self, m):
        with self.lock:
            self.rep = m


ap = argparse.ArgumentParser()
ap.add_argument("--omega", type=float, default=0.15)
ap.add_argument("--sec", type=float, default=4.0)
ap.add_argument("--align-sec", type=float, default=30.0)
ap.add_argument("--topic", default="/teleop/keyboard/cmd_vel")
a = ap.parse_args()

contact = abs(a.omega) * R_KINGPIN
base = abs(a.omega) * R_KINGPIN / R
exp = [abs(a.omega) * (R_KINGPIN + abs(h)) / R for h in H]

print("BENCH CHECK — contact-point correction")
print("  omega %+.4f rad/s -> contact speed %.4f m/s (breakaway 0.050) -> NO TRAVEL expected"
      % (a.omega, contact))
print("  uncorrected wheel command would be   %.4f rad/s on all four" % base)
print("  corrected expectation (joint order): %s"
      % ["%.4f" % e for e in exp])
print("  steering MUST be unchanged at %s\n" % SPIN_DEG)

rclpy.init()
n = Bench(a.topic)
threading.Thread(target=rclpy.spin, args=(n,), daemon=True).start()
t0 = time.time()
while (n.steer_meas is None or n.rep is None) and time.time() - t0 < 8:
    time.sleep(0.05)
if n.steer_meas is None or n.rep is None:
    print("ABORT: no feedback."); sys.exit(2)

# align first, or the slew brake scales the very number we are reading
target = [math.radians(d) for d in SPIN_DEG]
lo = [math.radians(w[0]) for w in WINDOW_DEG]
hi = [math.radians(w[1]) for w in WINDOW_DEG]
cmd = list(target)
aligned = False


def settle_and_read(command, hold_sec):
    """Publish one fixed command, let the servos actually get there, then read.

    The previous version integrated the error every 50 ms while the servo was
    still slewing. With 31 deg to travel the setpoint wound up into the window
    stop within a second and the pose ended nowhere near the target. Windup,
    plain and simple -- so: command, WAIT, then look.
    """
    m = Float64MultiArray(); m.data = command
    t_end = time.time() + hold_sec
    while time.time() < t_end:
        n.steer.publish(m)
        time.sleep(0.05)
    with n.lock:
        return list(n.steer_meas)


for attempt in range(4):
    cur = settle_and_read(cmd, 6.0 if attempt == 0 else 4.0)
    err = [t - c for t, c in zip(target, cur)]
    worst = max(abs(e) for e in err)
    print("    attempt %d: reached %s  worst %.2f deg"
          % (attempt + 1, ["%+.2f" % math.degrees(x) for x in cur], math.degrees(worst)))
    if worst < math.radians(0.6):
        aligned = True
        break
    # ONE full-error trim per settled reading, clamped to each wheel's window
    cmd = [min(max(c + e, l), h) for c, e, l, h in zip(cmd, err, lo, hi)]

hold = Float64MultiArray(); hold.data = cmd
for _ in range(10):
    n.steer.publish(hold); time.sleep(0.05)

tw = Twist(); tw.angular.z = a.omega
samples = []
t0 = time.time()
while time.time() - t0 < a.sec:
    n.cmd.publish(tw); time.sleep(0.05)
    with n.lock:
        if n.rep:
            samples.append((list(n.rep.commanded), list(n.rep.measured)))
for _ in range(30):
    n.cmd.publish(Twist()); time.sleep(0.05)

if len(samples) < 5:
    print("ABORT: too few samples."); sys.exit(3)
# steady state: the last third
tail = samples[len(samples) // 3:]
cmd_mean = [sum(abs(s[0][i]) for s in tail) / len(tail) for i in range(4)]
meas_max = [max(abs(s[1][i]) for s in tail) for i in range(4)]

print("\n=== COMMANDED WHEEL SPEEDS (steady state) ===")
ok = True
for i, l in enumerate(LABELS):
    ratio = cmd_mean[i] / base if base else 0.0
    good = abs(cmd_mean[i] - exp[i]) < 0.01
    ok &= good
    print("  %s commanded %.4f rad/s   expected %.4f   ratio vs uncorrected %.4f  -> %s"
          % (l, cmd_mean[i], exp[i], ratio, "PASS" if good else "FAIL"))
print("\n=== THE FOLD-SIGN TRAP ===")
print("  FL and BL are the FOLDED wheels in a spin. If the correction went in")
print("  before the fold they would read LOW (%.4f) while FR/BR read high."
      % (abs(a.omega) * (R_KINGPIN - abs(H[0])) / R))
folded_low = cmd_mean[0] < base and cmd_mean[1] > base
print("  observed: %s" % ("INVERTED — do not deploy" if folded_low else "not inverted, correct"))
print("\n=== NO-TRAVEL CHECK ===")
print("  max |measured| %s rad/s" % ["%.3f" % m for m in meas_max])
print("\n=== RESULT: %s ===" % ("PASS" if ok and not folded_low else "FAIL"))
rclpy.shutdown()
sys.exit(0 if (ok and not folded_low) else 1)
