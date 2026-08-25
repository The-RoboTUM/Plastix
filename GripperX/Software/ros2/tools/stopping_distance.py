#!/usr/bin/env python3
"""SR-1 WARNING -- THIS SCRIPT COMMANDS MOTION.
Publishes Twist (drive, linear.x) on --topic (default
/teleop/keyboard/cmd_vel): default --speed 0.06 m/s, ramped for
--accel-sec (default 4.0 s) then cut to exact zero, repeated for --runs
(default 5) runs, each guarded at --max-distance-m (default 1.5 m). With
--alternate, direction reverses every other run. SR-1 (the project's motion-approval
rule) requires an explicit user approval for EACH individual run; an
approval relayed by an agent does not satisfy it.

Stopping distance, open-loop. DEPLOY_2026-08-21.md section 6 / Octopus.

Commands a steady speed, cuts the command with an EXACT ZERO twist (what Nav2
does at a goal, i.e. what Octopus actually experiences), and reads the encoder
POSITION delta from the cut until the wheels stand still.

Writes every sample incrementally. Never buffers a run.
"""
import sys, os, json, time, math, threading, argparse
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, String
from gripperx_control_msgs.msg import WheelVelocityReport

R = 0.070                    # wheel radius, ros2_controllers.yaml
IDX_VEL, IDX_POS, IDX_ENC = 4, 8, 12
LABELS = ["FL", "FR", "BL", "BR"]
STALL_MIN_CMD = 2.0          # stall_min_command_rad_s

class Stopper(Node):
    def __init__(self, topic):
        super().__init__("stopping_distance_tool")
        q = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(Twist, topic, q)
        self.create_subscription(Float64MultiArray, "/hw/joint_states", self.on_state, q)
        self.create_subscription(String, "/teleop/active_mode", self.on_mode, q)
        self.create_subscription(WheelVelocityReport,
                                 "/swerve_controller/wheel_velocities", self.on_report, q)
        self.latched = [False] * 4
        self.rstatus = [0] * 4
        self.correction = [0.0] * 4
        self.regulating = [False] * 4
        self.lock = threading.Lock()
        self.vel = None; self.pos = None; self.enc = None
        self.stamp = 0.0; self.mode = None

    def on_state(self, msg):
        d = msg.data
        if len(d) < 16: return
        with self.lock:
            self.vel = list(d[IDX_VEL:IDX_VEL+4])
            self.pos = list(d[IDX_POS:IDX_POS+4])
            self.enc = [int(x) for x in d[IDX_ENC:IDX_ENC+4]]
            self.stamp = time.time()

    def on_mode(self, msg): self.mode = msg.data

    def on_report(self, msg):
        with self.lock:
            self.latched = list(msg.stall_latched)
            self.rstatus = list(msg.regulator_status)
            self.correction = list(msg.correction)
            self.regulating = list(msg.regulating)

    def snap(self):
        with self.lock:
            return (list(self.vel) if self.vel else None,
                    list(self.pos) if self.pos else None,
                    list(self.enc) if self.enc else None, self.stamp)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--speed", type=float, default=0.06)
    ap.add_argument("--accel-sec", type=float, default=4.0)
    ap.add_argument("--runs", type=int, default=5)
    ap.add_argument("--max-distance-m", type=float, default=1.5)
    ap.add_argument("--topic", default="/teleop/keyboard/cmd_vel")
    ap.add_argument("--out", default=os.path.expanduser("~/ws/runs/stopping"))
    ap.add_argument("--alternate", action="store_true",
                    help="reverse direction every other run to stay inside the lane")
    ap.add_argument("--dry-run", action="store_true", help="publish NOTHING; check preconditions only")
    a = ap.parse_args()

    os.makedirs(a.out, exist_ok=True)
    ts = time.strftime("%Y%m%d_%H%M%S")
    jpath = os.path.join(a.out, f"stopping_{ts}.jsonl")
    jf = open(jpath, "w")
    def rec(**kw):
        jf.write(json.dumps(kw) + "\n"); jf.flush()

    wheel_omega = a.speed / R
    print(f"commanded {a.speed:.3f} m/s -> wheel {wheel_omega:.4f} rad/s "
          f"({'BELOW' if wheel_omega < STALL_MIN_CMD else 'ABOVE'} stall_min_command {STALL_MIN_CMD}) ")
    if wheel_omega >= STALL_MIN_CMD:
        print("  HWR-30a CAN arm at this speed -- a wheel may be latched off mid-run.")
    else:
        print("  HWR-30a cannot arm: no wheel can be silently latched off.")
    rec(event="derivation", speed=a.speed, wheel_omega=wheel_omega,
        stall_min_command=STALL_MIN_CMD, arms=wheel_omega >= STALL_MIN_CMD, radius=R)

    rclpy.init()
    n = Stopper(a.topic)
    spin = threading.Thread(target=rclpy.spin, args=(n,), daemon=True); spin.start()

    t0 = time.time()
    while n.snap()[0] is None and time.time() - t0 < 8.0:
        time.sleep(0.05)
    v, p, e, st = n.snap()
    if v is None:
        print("ABORT: no /hw/joint_states -- nothing commanded."); return 2
    print(f"mode={n.mode}  provenance={e}  (3=Live, 2=LiveUnconfirmed)")
    rec(event="precondition", mode=n.mode, provenance=e, pos=p, vel=v)
    if n.mode is not None and n.mode != "keyboard":
        print(f"ABORT: mux mode is '{n.mode}', this tool publishes to {a.topic}."); return 2
    if a.dry_run:
        print("DRY RUN -- nothing published, nothing moved."); return 0

    results = []
    for run in range(1, a.runs + 1):
        print(f"\n--- run {run}/{a.runs} ---")
        direction = -1.0 if (a.alternate and run % 2 == 0) else 1.0
        tw = Twist(); tw.linear.x = a.speed * direction
        print("  Richtung: %s" % ("VORWAERTS" if direction > 0 else "RUECKWAERTS"))
        start_pos = n.snap()[1]
        t_start = time.time()
        aborted = None
        seen_status = set()
        max_corr = 0.0
        regulating_seen = False
        # ACCELERATE + CRUISE
        while time.time() - t_start < a.accel_sec:
            n.pub.publish(tw)
            v, p, e, st = n.snap()
            if time.time() - st > 0.5:
                aborted = "SR-13: /hw/joint_states silent > 0.5 s"; break
            if any(n.latched):
                aborted = "HWR-30a latched a wheel off: %s -- run INVALID" % (n.latched,)
                break
            dist = abs(sum(p) / 4.0 - sum(start_pos) / 4.0) * R
            if dist > a.max_distance_m:
                aborted = f"distance guard {a.max_distance_m} m"; break
            with n.lock:
                st_now = list(n.rstatus); co_now = list(n.correction)
                rg_now = list(n.regulating)
            seen_status.update(st_now)
            max_corr = max([max_corr] + [abs(c) for c in co_now])
            regulating_seen = regulating_seen or any(rg_now)
            rec(event="drive", run=run, t=time.time(), vel=v, pos=p,
                rstatus=st_now, correction=co_now, regulating=rg_now)
            time.sleep(0.05)
        if aborted:
            n.pub.publish(Twist()); print(f"ABORT: {aborted}"); rec(event="abort", run=run, why=aborted)
            break
        # PRE-CUT GUARD -- the wheels must ACTUALLY be turning, or the run is
        # vacuous. The first version of this tool reported a confident
        # "-0.03 cm, LESS than 2 cm" while the machine stood still.
        v_now = n.snap()[0]
        moving = [abs(x) > 0.2 * wheel_omega for x in v_now]
        if not any(moving):
            n.pub.publish(Twist())
            msg = ("no wheel is turning at the cut (measured %s rad/s, commanded %.3f). "
                   "The machine did not break away -- NOTHING TO MEASURE."
                   % (["%+.3f" % x for x in v_now], wheel_omega))
            print("ABORT run %d: %s" % (run, msg))
            rec(event="abort", run=run, why="no motion at cut", vel=v_now, cmd=wheel_omega)
            break
        if not all(moving):
            print("  WARNING: only %d/4 wheels turning: " % sum(moving)
                  + " ".join("%s=%s" % (l, "yes" if m else "NO") for l, m in zip(LABELS, moving)))
            rec(event="partial_motion", run=run, moving=moving, vel=v_now)
        # CUT -- exact zero twist
        v_cut, p_cut, _, _ = n.snap()
        t_cut = time.time()
        zero = Twist()
        rec(event="cut", run=run, t=t_cut, vel=v_cut, pos=p_cut)
        print(f"  cut at wheel speeds {['%+.3f' % x for x in v_cut]} rad/s")
        # COAST until still
        still_since = None
        p_end = p_cut
        while time.time() - t_cut < 5.0:
            n.pub.publish(zero)
            v, p, e, st = n.snap()
            rec(event="coast", run=run, t=time.time(), vel=v, pos=p)
            p_end = p
            if max(abs(x) for x in v) < 0.02:
                still_since = still_since or time.time()
                if time.time() - still_since > 0.6: break
            else:
                still_since = None
            time.sleep(0.02)
        per_wheel = [abs((pe - pc) * R) for pe, pc in zip(p_end, p_cut)]
        mean_m = sum(per_wheel) / 4.0
        results.append((mean_m, per_wheel, v_cut, all(moving)))
        print(f"  stopping distance: {mean_m*100:+.2f} cm   per wheel(cm): "
              + " ".join(f"{l}={x*100:+.2f}" for l, x in zip(LABELS, per_wheel)))
        rec(event="result", run=run, mean_m=mean_m, per_wheel=per_wheel, v_cut=v_cut,
            regulator_status_while_driving=sorted(seen_status),
            max_correction=max_corr, regulating_seen=regulating_seen,
            latched=n.latched)
        names = {0: "DISABLED", 1: "ACTIVE", 2: "AT_LIMIT", 3: "OFF_PROVENANCE",
                 4: "OFF_NO_MEAS", 5: "OFF_STALE", 6: "OFF_STALL", 7: "OFF_BELOW_FLOOR"}
        print("  regulator WHILE DRIVING: %s | max |correction| %.4f rad/s | regulating=%s"
              % ([names.get(x, x) for x in sorted(seen_status)], max_corr, regulating_seen))
        # settle between runs
        t = time.time()
        while time.time() - t < 2.0:
            n.pub.publish(zero); time.sleep(0.05)

    # final stop, exact zero, held
    for _ in range(40):
        n.pub.publish(Twist()); time.sleep(0.05)

    if results:
        valid = [r for r in results if r[3]]
        dropped = len(results) - len(valid)
        if dropped:
            print("\n%d von %d Laeufen VERWORFEN: nicht alle vier Raeder drehten."
                  % (dropped, len(results)))
        if not valid:
            print("\n=== KEIN ERGEBNIS ===")
            print("  Kein einziger Lauf hatte alle vier Raeder in Bewegung.")
            print("  Es gibt keinen Bremsweg zu berichten -- die Zahl waere erfunden.")
            rec(event="no_result", why="no run with all four wheels moving",
                runs=len(results))
            print("\nrun record: %s" % jpath)
            jf.close(); rclpy.shutdown(); return 3
        xs = [r[0] * 100 for r in valid]
        mean = sum(xs) / len(xs)
        sd = (sum((x - mean) ** 2 for x in xs) / len(xs)) ** 0.5
        print("\n================ SUMMARY ================")
        for i, x in enumerate(xs, 1): print(f"  run {i}: {x:+.2f} cm")
        print(f"  mean {mean:+.2f} cm   sd {sd:.2f} cm   min {min(xs):+.2f}  max {max(xs):+.2f}")
        print(f"  OCTOPUS QUESTION -- more or less than ~2 cm?  -> {'MORE' if abs(mean) > 2.0 else 'LESS'}")
        rec(event="summary", runs_cm=xs, mean_cm=mean, sd_cm=sd)
    print(f"\nrun record: {jpath}")
    jf.close(); rclpy.shutdown(); return 0

sys.exit(main())
