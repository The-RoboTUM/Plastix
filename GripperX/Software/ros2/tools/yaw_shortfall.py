#!/usr/bin/env python3
"""SR-1 WARNING -- THIS SCRIPT COMMANDS MOTION.
Publishes a spin Twist (angular.z) on --topic (default
/teleop/keyboard/cmd_vel): default --omega 0.75847 rad/s commanding --turns
nominal revolutions (default 1.0), i.e. a full in-place spin at default
settings. Also drives all four wheels to the spin pose via
/teleop/direct_steer (steering motion, up to --align-sec default 25 s).
SR-1 (the project's motion-approval rule) requires an explicit user approval for EACH
individual run; an approval relayed by an agent does not satisfy it.

OP-29 spin half -- the YAW-RATE SHORTFALL, replacing the withdrawn chirality test.

WHY THE OLD TEST IS GONE. DEPLOY_2026-08-21.md section 2 asked for a 2-vs-2
split of the contact radii (0.15470 / 0.26585, ratio 1.718) that MIRRORS when
omega reverses. That premise assumed the king-pin -> wheel offset points the
same way on all four corners. It does not: gripperx_v1.core.xacro L303-306 has
+0.055572 / +0.055572 / -0.055571 / -0.055372, i.e. MIRRORED, outboard on every
corner -- and the tape settles it (predicted width over the tyre outer faces
408.4 mm, measured on the robot 2026-08-19: 409 mm; a same-side offset would
predict 297 mm). A mirror-symmetric contact set in a mirror-symmetric pose
cannot be chiral. All four contact radii come out at 0.2666 +- 0.0002 m.

WHAT SURVIVES IS UNIFORM AND TESTABLE. The controller commands each wheel
omega * |r_kingpin| while the contact point needs omega * |r_contact|, so the
machine turns at |r_kingpin| / |r_contact| = 0.7914 of the commanded rate --
a 21 % shortfall, slip-free, identical on all four wheels. It is INVISIBLE on
/swerve_controller/wheel_velocities: four clean wheels are not evidence
against it.

HOW THIS MEASURES IT WITHOUT AN IMU. There is no BNO085 connected, and the
odometry path cannot be used either -- drive_joint_multipliers [1,1,-1,-1] in
gripperx_localization decodes a clean spin as pure translation with exactly
zero yaw. So: command EXACTLY ONE NOMINAL REVOLUTION and let a human read the
chalk mark. Controller's model right -> back on the mark. Contact-point model
right -> 75 deg short.

The encoders still contribute: integrating the MEASURED wheel speeds gives the
distance each tyre actually rolled, which turns into a predicted rotation under
each hypothesis. The observed angle then picks a winner instead of merely
showing that something is off.
"""
import sys, os, json, time, math, threading, argparse
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from gripperx_control_msgs.msg import WheelVelocityReport

R = 0.070                 # wheel radius
A_HALF, B_HALF = 0.1809, 0.1087   # geometry SoT (gripperx_geometry/config/geometry.yaml)
R_KINGPIN = math.hypot(A_HALF, B_HALF)        # 0.210950 -- what the controller uses
R_CONTACT = 0.266571                          # audit, from the URDF offsets
# SPIN_DEG derived from the geometry single source of truth, NOT hand-written.
# 2026-08-25: the CAD pair (a=0.1809, b=0.1087, GQ-1/GQ-4) moved the commanded spin pose
# 58.570 -> 58.999 deg. The old literal was 0.429 deg off against --align-tol-deg 0.6, i.e.
# 71 % of the tolerance budget, and would have aborted this run as a false misalignment.
# R_KINGPIN and R_CONTACT above shift by < 0.05 % under the same change and are left as they
# are; R_CONTACT is pose-dependent (the contact offset rotates with the steer angle) and was
# re-derived as correct, not stale.
_SPIN_POSE_DEG = math.degrees(math.atan2(A_HALF, B_HALF))   # 58.999 with the SoT pair
SPIN_DEG = [-_SPIN_POSE_DEG, +_SPIN_POSE_DEG, +_SPIN_POSE_DEG, -_SPIN_POSE_DEG]  # FL, FR, BL, BR
# steering_outward_sign [-1,+1,+1,-1]: outward limit 100 deg, inward 35 deg
WINDOW_DEG = [(-100.0, 35.0), (-35.0, 100.0), (-35.0, 100.0), (-100.0, 35.0)]
LABELS = ["FL", "FR", "BL", "BR"]
STATUS = {0: "DISABLED", 1: "ACTIVE", 2: "AT_LIMIT", 3: "OFF_PROVENANCE",
          4: "OFF_NO_MEAS", 5: "OFF_STALE", 6: "OFF_STALL", 7: "OFF_BELOW_FLOOR"}


class Yaw(Node):
    def __init__(self, topic):
        super().__init__("yaw_shortfall_tool")
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
        self.rep_t = 0.0
        self.samples = []

    def on_steer(self, m):
        with self.lock:
            self.steer_meas = list(m.data[:4])

    def on_rep(self, m):
        with self.lock:
            self.rep = m
            self.rep_t = time.time()
            self.samples.append({"t": self.rep_t,
                                 "commanded": list(m.commanded),
                                 "measured": list(m.measured),
                                 "status": list(m.regulator_status),
                                 "latched": list(m.stall_latched)})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--omega", type=float, default=0.75847,
                    help="rad/s about z; default puts contact speed at 0.16 m/s")
    ap.add_argument("--turns", type=float, default=1.0, help="NOMINAL turns to command")
    ap.add_argument("--align-sec", type=float, default=25.0)
    ap.add_argument("--align-tol-deg", type=float, default=0.6)
    ap.add_argument("--topic", default="/teleop/keyboard/cmd_vel")
    ap.add_argument("--out", default=os.path.expanduser("~/ws/runs/yaw"))
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args()

    wheel_omega = abs(a.omega) * R_KINGPIN / R
    contact_v = abs(a.omega) * R_KINGPIN
    spin_sec = a.turns * 2.0 * math.pi / abs(a.omega)
    ratio = R_KINGPIN / R_CONTACT
    predicted_deg = a.turns * 360.0 * ratio

    os.makedirs(a.out, exist_ok=True)
    jpath = os.path.join(a.out, "yaw_%s.jsonl" % time.strftime("%Y%m%d_%H%M%S"))
    jf = open(jpath, "w")

    def rec(**kw):
        jf.write(json.dumps(kw) + "\n"); jf.flush()

    print("OP-29 yaw-rate shortfall")
    print("  omega commanded      %+.5f rad/s" % a.omega)
    print("  wheel speed          %.4f rad/s  (%s the 2.0 floor)"
          % (wheel_omega, "ABOVE" if wheel_omega > 2.0 else "below"))
    print("  contact speed        %.4f m/s" % contact_v)
    print("  NOMINAL %.2f turn(s) = %.3f s of commanding" % (a.turns, spin_sec))
    print()
    print("  PREDICTIONS -- read the chalk mark, they differ by %.0f deg:"
          % (a.turns * 360.0 - predicted_deg))
    print("    controller's king-pin model  r=%.6f m  -> %.1f deg (back on the mark)"
          % (R_KINGPIN, a.turns * 360.0))
    print("    contact-point model          r=%.6f m  -> %.1f deg (%.0f deg SHORT)"
          % (R_CONTACT, predicted_deg, a.turns * 360.0 - predicted_deg))
    print()
    if wheel_omega > 2.0:
        print("  HWR-30a IS ARMED at this wheel speed -- a latch aborts the run.")
    rec(event="derivation", omega=a.omega, wheel_omega=wheel_omega, spin_sec=spin_sec,
        r_kingpin=R_KINGPIN, r_contact=R_CONTACT, predicted_deg=predicted_deg,
        nominal_deg=a.turns * 360.0)

    rclpy.init()
    n = Yaw(a.topic)
    threading.Thread(target=rclpy.spin, args=(n,), daemon=True).start()
    t0 = time.time()
    while (n.steer_meas is None or n.rep is None) and time.time() - t0 < 8.0:
        time.sleep(0.05)
    if n.steer_meas is None or n.rep is None:
        print("ABORT: no /hw/steer_states or no wheel report -- nothing commanded.")
        return 2
    with n.lock:
        if any(n.rep.stall_latched):
            print("ABORT: a wheel is already latched off by HWR-30a: %s"
                  % list(n.rep.stall_latched))
            return 2
    print("  steering now (deg): %s"
          % ["%+.2f" % math.degrees(x) for x in n.steer_meas])
    if a.dry_run:
        print("\nDRY RUN -- nothing published, nothing moved.")
        return 0

    # ---- phase 1: pre-align the steering, WITHOUT driving -----------------
    print("\n  ALIGN: closed-loop to the spin pose via /teleop/direct_steer (no drive)")
    print("    trimming the COMMANDED angle until the MEASURED angle is on target")
    target = [math.radians(d) for d in SPIN_DEG]
    lo = [math.radians(w[0]) for w in WINDOW_DEG]
    hi = [math.radians(w[1]) for w in WINDOW_DEG]
    tol = math.radians(a.align_tol_deg)
    cmd = list(target)
    aligned = False
    t0 = time.time()
    last_report = 0.0
    while time.time() - t0 < a.align_sec:
        msg = Float64MultiArray(); msg.data = cmd
        n.steer.publish(msg)
        time.sleep(0.05)
        with n.lock:
            cur = list(n.steer_meas) if n.steer_meas else None
        if not cur:
            continue
        err = [t - c for t, c in zip(target, cur)]
        worst = max(abs(e) for e in err)
        if worst < tol:
            aligned = True
            break
        cmd = [min(max(c + 0.30 * e, l), h)
               for c, e, l, h in zip(cmd, err, lo, hi)]
        if time.time() - last_report > 4.0:
            last_report = time.time()
            print("      worst error %.2f deg, trim now %s"
                  % (math.degrees(worst),
                     ["%+.2f" % math.degrees(c - t) for c, t in zip(cmd, target)]))
    with n.lock:
        cur = list(n.steer_meas)
    worst_deg = math.degrees(max(abs(c - t) for c, t in zip(cur, target)))
    print("    commanded (deg): %s" % ["%+.2f" % math.degrees(x) for x in cmd])
    print("    reached   (deg): %s" % ["%+.2f" % math.degrees(x) for x in cur])
    print("    worst error:     %.2f deg (tolerance %.2f) -> %s"
          % (worst_deg, a.align_tol_deg, "ALIGNED" if aligned else "NOT aligned"))
    rec(event="align", target=target, commanded=cmd, reached=cur,
        aligned=aligned, worst_deg=worst_deg)
    if not aligned:
        print("ABORT: steering did not reach the spin pose; not commanding a spin.")
        for _ in range(20):
            n.cmd.publish(Twist()); time.sleep(0.05)
        return 3
    hold = Float64MultiArray(); hold.data = cmd
    for _ in range(10):
        n.steer.publish(hold); time.sleep(0.05)

    # keep the override alive while we hand over to the twist, so the wheels
    # never see a pose they must slew away from
    print("\n  >>> SPIN: commanding %.3f s. Watch the chalk mark. <<<" % spin_sec)
    tw = Twist(); tw.angular.z = a.omega
    with n.lock:
        n.samples.clear()
    t_start = time.time()
    last = t_start
    intervals = []
    aborted = None
    while time.time() - t_start < spin_sec:
        now = time.time()
        intervals.append(now - last); last = now
        n.cmd.publish(tw)
        time.sleep(0.05)
        with n.lock:
            latched = list(n.rep.stall_latched) if n.rep else [False] * 4
            age = time.time() - n.rep_t
        if any(latched):
            aborted = "HWR-30a latched a wheel off: %s" % latched
            break
        if age > 0.5:
            aborted = "SR-13: wheel report silent > 0.5 s"
            break
    for _ in range(40):
        n.cmd.publish(Twist()); time.sleep(0.05)

    with n.lock:
        S = list(n.samples)
    for s in S:
        rec(event="sample", **s)

    worst = max(intervals) if intervals else 0.0
    print("\n=== PUBLISHER HEALTH ===")
    print("  %d intervals, worst %.3f s (mux timeout 0.50 s) -> %s"
          % (len(intervals), worst, "OK" if worst < 0.3 else "STALLED, run INVALID"))
    rec(event="publisher", n=len(intervals), worst=worst, ok=worst < 0.3)
    if aborted:
        print("\nABORTED: %s" % aborted)
        rec(event="abort", why=aborted)
        print("The run is INVALID -- do not read the chalk mark as a result.")
        return 3
    if len(S) < 10:
        print("\nToo few samples (%d) -- inconclusive." % len(S))
        return 3

    # ---- integrate what the wheels actually rolled ------------------------
    theta = [0.0] * 4
    for i in range(1, len(S)):
        dt = S[i]["t"] - S[i - 1]["t"]
        if dt <= 0 or dt > 0.5:
            continue
        for w in range(4):
            theta[w] += abs(S[i]["measured"][w]) * dt
    arc = [t * R for t in theta]          # metres each tyre rolled
    mean_arc = sum(arc) / 4.0
    rot_kingpin = math.degrees(mean_arc / R_KINGPIN)
    rot_contact = math.degrees(mean_arc / R_CONTACT)
    statuses = sorted({x for s in S for x in s["status"]})

    print("\n=== WHAT THE WHEELS DID ===")
    print("  regulator status seen: %s" % [STATUS.get(x, x) for x in statuses])
    for l, x in zip(LABELS, arc):
        print("    %s rolled %.4f m" % (l, x))
    print("  mean rolled: %.4f m" % mean_arc)
    print("\n=== ROTATION IMPLIED BY THE ENCODERS ===")
    print("  if the effective radius is the KING PIN   (%.6f m): %.1f deg"
          % (R_KINGPIN, rot_kingpin))
    print("  if the effective radius is the CONTACT PT (%.6f m): %.1f deg"
          % (R_CONTACT, rot_contact))
    print("\n=== NOW READ THE CHALK MARK ===")
    print("  How far did the CHASSIS actually turn?")
    print("    ~%.0f deg  -> the controller's king-pin model is right;" % rot_kingpin)
    print("                 the contact-point mechanism is REFUTED on hardware.")
    print("    ~%.0f deg  -> the contact-point model is right; OP-29's spin half" % rot_contact)
    print("                 has its cause, and the 21 %% shortfall is real.")
    print("  Anything else is neither, and needs its own explanation.")
    rec(event="result", arc=arc, mean_arc=mean_arc,
        rot_if_kingpin=rot_kingpin, rot_if_contact=rot_contact, statuses=statuses)
    print("\nrun record: %s" % jpath)
    jf.close()
    return 0


sys.exit(main())
