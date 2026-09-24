#!/usr/bin/env python3
"""SR-1 WARNING -- THIS SCRIPT COMMANDS MOTION.
Publishes Twist (lateral drive, linear.y) on --topic (default
/teleop/keyboard/cmd_vel): default --vy 0.16 m/s held for --sec default
3.0 s, i.e. ~0.48 m sideways at default settings. Also drives all four
wheels to the crab pose via /teleop/direct_steer (steering motion, up to
--align-sec default 30 s). SR-1 (the project's motion-approval rule) requires an explicit
user approval for EACH individual run; an approval relayed by an agent does
not satisfy it.

OP-29 crab half — does the diagonal wheel fold produce a YAW COUPLE?

Crab is the ONLY manoeuvre whose reversed wheels form a DIAGONAL pair (FL and
BR), rather than the symmetric pair a spin reverses. So any forward/reverse
asymmetry in the drivetrain — PWM deadband, breakaway, backlash taken up in
one direction only — shows up as a symmetric speed error in a spin but as a
ROTATION in a crab. Structural; entirely unmeasured on hardware.

THE DISCRIMINATOR is the SIGN under reversal: a yaw couple from drivetrain
asymmetry flips with the crab direction. A steering zero-offset does not.

HOW IT IS MEASURED WITHOUT AN IMU. In the crab pose every wheel rolls laterally,
and the lateral speed of wheel i is `vy + omega * x_i` — it depends on the
LONGITUDINAL position only. Front wheels sit at x = +0.1809, rear at -0.1809, so

    yaw angle  =  (front lateral arc - rear lateral arc) / (2 * 0.1809)

and a LEFT/RIGHT difference cannot be yaw at all, because the lateral component
of omega x r does not depend on y. One run separates rotation from drive
asymmetry. The BNO085 is not connected and the odometry path is known-broken on
hardware, so this is the only quantitative route available.
"""
import sys, os, json, time, math, threading, argparse
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from gripperx_control_msgs.msg import WheelVelocityReport

R = 0.070
X_HALF = 0.180900              # king pin longitudinal, localization.yaml
CRAB_DEG = [-90.0, +90.0, +90.0, -90.0]        # FL, FR, BL, BR
# Per-wheel steering window, joint order FL, FR, BL, BR. Mirrors
# gripperx_control/config/steer_servo.yaml, which is the source of truth.
WINDOW_DEG = [(-125.0, 35.0), (-35.0, 125.0), (-35.0, 125.0), (-125.0, 35.0)]
LABELS = ["FL", "FR", "BL", "BR"]
STATUS = {0: "DISABLED", 1: "ACTIVE", 2: "AT_LIMIT", 3: "OFF_PROVENANCE",
          4: "OFF_NO_MEAS", 5: "OFF_STALE", 6: "OFF_STALL", 7: "OFF_BELOW_FLOOR"}


class Crab(Node):
    def __init__(self, topic):
        super().__init__("crab_yaw_tool")
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
                                 "measured": list(m.measured),
                                 "steer": list(self.steer_meas) if self.steer_meas else None,
                                 "status": list(m.regulator_status),
                                 "latched": list(m.stall_latched)})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--vy", type=float, default=0.16,
                    help="lateral speed, m/s. Positive = crab LEFT.")
    ap.add_argument("--sec", type=float, default=3.0)
    ap.add_argument("--align-sec", type=float, default=30.0)
    ap.add_argument("--align-tol-deg", type=float, default=0.8)
    ap.add_argument("--topic", default="/teleop/keyboard/cmd_vel")
    ap.add_argument("--out", default=os.path.expanduser("~/ws/runs/crab"))
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args()

    wheel_omega = abs(a.vy) / R
    os.makedirs(a.out, exist_ok=True)
    jpath = os.path.join(a.out, "crab_%s.jsonl" % time.strftime("%Y%m%d_%H%M%S"))
    jf = open(jpath, "w")

    def rec(**kw):
        jf.write(json.dumps(kw) + "\n"); jf.flush()

    print("OP-29 crab half — yaw couple from the diagonal fold?")
    print("  vy %+.3f m/s (%s), %.2f s -> nominal travel %.3f m"
          % (a.vy, "LEFT" if a.vy > 0 else "RIGHT", a.sec, abs(a.vy) * a.sec))
    print("  wheel speed %.4f rad/s (%s the 2.0 floor)"
          % (wheel_omega, "ABOVE" if wheel_omega > 2.0 else "below"))
    print("  crab pose %s deg — FL and BR fold and roll BACKWARDS (the diagonal)"
          % CRAB_DEG)
    print("  yaw is read as (front arc - rear arc) / %.4f m" % (2 * X_HALF))
    if wheel_omega > 2.0:
        print("  HWR-30a IS ARMED — a latch aborts the run.")
    rec(event="derivation", vy=a.vy, sec=a.sec, wheel_omega=wheel_omega,
        crab_deg=CRAB_DEG, x_half=X_HALF)

    rclpy.init()
    n = Crab(a.topic)
    threading.Thread(target=rclpy.spin, args=(n,), daemon=True).start()
    t0 = time.time()
    while (n.steer_meas is None or n.rep is None) and time.time() - t0 < 8.0:
        time.sleep(0.05)
    if n.steer_meas is None or n.rep is None:
        print("ABORT: no /hw/steer_states or no wheel report — nothing commanded.")
        return 2
    with n.lock:
        if any(n.rep.stall_latched):
            print("ABORT: a wheel is already latched off: %s" % list(n.rep.stall_latched))
            return 2
    if a.dry_run:
        print("\n  steering now (deg): %s"
              % ["%+.2f" % math.degrees(x) for x in n.steer_meas])
        print("DRY RUN — nothing published, nothing moved.")
        return 0

    # ---- closed-loop alignment to the crab pose, no drive ------------------
    print("\n  ALIGN: closed-loop to the crab pose (no drive). 90 deg outward,")
    print("         window is 125 deg outward, so 35 deg of margin.")
    target = [math.radians(d) for d in CRAB_DEG]
    lo = [math.radians(w[0]) for w in WINDOW_DEG]
    hi = [math.radians(w[1]) for w in WINDOW_DEG]
    tol = math.radians(a.align_tol_deg)
    # ALIGNMENT IS SETTLE-THEN-TRIM, AND IT IS HARD-BOUNDED. An earlier integrator
    # loop (exit as soon as the MEASURED value crossed inside tolerance, then a
    # continuous proportional trim) wound the command PAST the target while the
    # servo was still travelling, ran to the window clamp within about half a
    # second, and made all four steering couplings slip against their wheels --
    # it looked like success on the console right up to costing a day's
    # calibration.
    #
    # A commanded angle is reached faithfully -- commanding 77 deg settles at
    # 77 deg. The loop only has to compensate travel TIME, which is not an
    # error and cannot be trimmed away. The rules here, each tied to the
    # failure mode above:
    #   * The command only ever changes when the steering has STOPPED MOVING. Settling
    #     is judged on the measurement standing still, not on the error being small --
    #     a servo passing through the target is standing still on neither count.
    #   * A trim applies the FULL settled residual once, then waits to settle again.
    #     That is deadbeat on a real offset and impossible to wind up.
    #   * The command is clamped to target +- MAX_TRIM_DEG, on top of the window clamp.
    #     This is the backstop: even if settling detection fails completely, the command
    #     cannot walk to the window limit.
    #   * Alignment is accepted only when the error is inside tol AND settled.
    SETTLE_EPS = math.radians(0.15)   # movement below this counts as "stopped"
    SETTLE_SAMPLES = 10               # ~0.5 s of standing still at the 0.05 s period
    MAX_TRIMS = 6
    MAX_TRIM_DEG = 10.0               # hard bound on how far a trim may leave the target

    trim_lo = [max(l, tg - math.radians(MAX_TRIM_DEG)) for l, tg in zip(lo, target)]
    trim_hi = [min(h, tg + math.radians(MAX_TRIM_DEG)) for h, tg in zip(hi, target)]

    cmd = [min(max(tg, l), h) for tg, l, h in zip(target, trim_lo, trim_hi)]
    aligned = False
    trims = 0
    still = 0
    prev = None
    t0 = time.time()
    while time.time() - t0 < a.align_sec:
        msg = Float64MultiArray(); msg.data = cmd
        n.steer.publish(msg)
        time.sleep(0.05)
        with n.lock:
            cur = list(n.steer_meas) if n.steer_meas else None
        if not cur:
            continue

        if prev is not None and max(abs(c - p) for c, p in zip(cur, prev)) < SETTLE_EPS:
            still += 1
        else:
            still = 0
        prev = cur
        if still < SETTLE_SAMPLES:
            continue

        err = [tg - c for tg, c in zip(target, cur)]
        worst = max(abs(e) for e in err)
        if worst < tol:
            aligned = True
            break
        if trims >= MAX_TRIMS:
            print("    settled %.2f deg off target after %d trims -- not converging."
                  % (math.degrees(worst), trims))
            break
        cmd = [min(max(c + e, l), h) for c, e, l, h in zip(cmd, err, trim_lo, trim_hi)]
        trims += 1
        still = 0
        print("    trim %d: settled %.2f deg off, commanding %s"
              % (trims, math.degrees(worst),
                 ["%+.2f" % math.degrees(x) for x in cmd]))

    with n.lock:
        cur = list(n.steer_meas)
    worst_deg = math.degrees(max(abs(c - t) for c, t in zip(cur, target)))
    print("    reached (deg): %s" % ["%+.2f" % math.degrees(x) for x in cur])
    print("    worst error:   %.2f deg -> %s"
          % (worst_deg, "ALIGNED" if aligned else "NOT aligned"))
    rec(event="align", target=target, commanded=cmd, reached=cur,
        aligned=aligned, worst_deg=worst_deg, trims=trims,
        settled=(still >= SETTLE_SAMPLES))
    if not aligned:
        print("ABORT: steering did not reach the crab pose; not commanding a crab.")
        for _ in range(20):
            n.cmd.publish(Twist()); time.sleep(0.05)
        return 3
    hold = Float64MultiArray(); hold.data = cmd
    for _ in range(10):
        n.steer.publish(hold); time.sleep(0.05)

    # ---- crab ---------------------------------------------------------------
    print("\n  >>> CRAB %s for %.2f s. Watch whether the NOSE swings. <<<"
          % ("LEFT" if a.vy > 0 else "RIGHT", a.sec))
    tw = Twist(); tw.linear.y = a.vy
    with n.lock:
        n.samples.clear()
    t_start = time.time()
    last = t_start
    intervals = []
    aborted = None
    while time.time() - t_start < a.sec:
        now = time.time()
        intervals.append(now - last); last = now
        n.cmd.publish(tw)
        time.sleep(0.05)
        with n.lock:
            latched = list(n.rep.stall_latched) if n.rep else [False] * 4
            age = time.time() - n.rep_t
        if any(latched):
            aborted = "HWR-30a latched a wheel off: %s" % latched; break
        if age > 0.5:
            aborted = "SR-13: wheel report silent > 0.5 s"; break
    for _ in range(40):
        n.cmd.publish(Twist()); time.sleep(0.05)

    with n.lock:
        S = list(n.samples)
    for s in S:
        rec(event="sample", **s)

    worst = max(intervals) if intervals else 0.0
    print("\n=== PUBLISHER HEALTH ===")
    print("  %d intervals, worst %.3f s -> %s"
          % (len(intervals), worst, "OK" if worst < 0.3 else "STALLED, run INVALID"))
    rec(event="publisher", n=len(intervals), worst=worst, ok=worst < 0.3)
    if aborted:
        print("\nABORTED: %s\nThe run is INVALID." % aborted)
        rec(event="abort", why=aborted)
        return 3
    if len(S) < 10:
        print("\nToo few samples (%d) — inconclusive." % len(S))
        return 3

    # ---- integrate the LATERAL ground distance each wheel actually rolled ---
    lat = [0.0] * 4
    for i in range(1, len(S)):
        dt = S[i]["t"] - S[i - 1]["t"]
        if dt <= 0 or dt > 0.5:
            continue
        st = S[i]["steer"] or S[i - 1]["steer"]
        if not st:
            continue
        for w in range(4):
            # ground velocity of the wheel = signed wheel speed along d(delta);
            # its lateral (y) component is speed * sin(delta). The fold is
            # handled by construction: a folded wheel has a negative speed AND
            # a delta 180 deg away, and the product is identical.
            lat[w] += S[i]["measured"][w] * R * math.sin(st[w]) * dt

    front = 0.5 * (lat[0] + lat[1])          # FL, FR
    rear = 0.5 * (lat[2] + lat[3])           # BL, BR
    left = 0.5 * (lat[0] + lat[2])           # FL, BL
    right = 0.5 * (lat[1] + lat[3])          # FR, BR
    mean_lat = sum(lat) / 4.0
    yaw_rad = (front - rear) / (2.0 * X_HALF)
    statuses = sorted({x for s in S for x in s["status"]})

    print("\n=== LATERAL GROUND DISTANCE PER WHEEL ===")
    print("  regulator status seen: %s" % [STATUS.get(x, x) for x in statuses])
    for l, x in zip(LABELS, lat):
        print("    %s %+.4f m" % (l, x))
    print("  mean travel   %+.4f m   (nominal %+.4f m)" % (mean_lat, a.vy * a.sec))
    print("\n=== THE TWO SEPARATE QUESTIONS ===")
    print("  front %+.4f  vs rear %+.4f   -> difference %+.4f m" % (front, rear, front - rear))
    print("     => YAW %+.2f deg   (this is rotation; only x-position can cause it)"
          % math.degrees(yaw_rad))
    print("  left  %+.4f  vs right %+.4f  -> difference %+.4f m" % (left, right, left - right))
    print("     => NOT yaw. A left/right split is drive asymmetry, not rotation.")
    print("\n  Reverse the direction and compare the SIGN of the yaw:")
    print("    sign FLIPS  -> drivetrain forward/reverse asymmetry acting through")
    print("                   the diagonal fold. The hypothesis holds.")
    print("    sign STAYS  -> something direction-independent (steering zero")
    print("                   offsets, geometry). The hypothesis is refuted.")
    rec(event="result", lateral=lat, front=front, rear=rear, left=left, right=right,
        mean=mean_lat, yaw_rad=yaw_rad, yaw_deg=math.degrees(yaw_rad), statuses=statuses)
    print("\nrun record: %s" % jpath)
    jf.close()
    return 0


sys.exit(main())
