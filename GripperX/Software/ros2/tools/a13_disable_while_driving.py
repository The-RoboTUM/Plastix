#!/usr/bin/env python3
"""SR-1 WARNING -- THIS SCRIPT COMMANDS MOTION.
Publishes Twist (drive) on --topic, default /teleop/keyboard/cmd_vel:
constant forward --speed (default 0.16 m/s) held for --settle-sec +
--after-sec (default 4.0 + 3.0 = 7 s), i.e. up to ~1.1 m at default
settings, with the wheel regulator toggled off mid-run while still driving.
SR-1 (the project's motion-approval rule) requires an explicit user approval for EACH
individual run; an approval relayed by an agent does not satisfy it.

FR-14 A13 -- disable the wheel regulator WHILE DRIVING.

DEPLOY_2026-08-21.md section 3. The permanent enable gave up R1
(disabled-by-default); what replaced it is R2, the runtime switch, and R2's
own acceptance criterion has never been run. Every disable so far happened on
blocks or after the run had ended.

Expected: the correction steps to zero in ONE cycle -- a step bounded by the
authority limit (at most 30 % of the setpoint). FR-14 item 4 was corrected on
exactly this point: zeroing buys freedom from RETAINED STATE, not from a STEP.
Whether that step is visible as a lurch is the MEASUREMENT; the requirement
may not assert it.

THE TRAP THIS IS BUILT AROUND (DEPLOY section 8): a blocking
subprocess.run(["ros2","param","set", ...]) inside the command publisher stalls
the loop, the mux sees 0.5 s of silence, correctly zeroes /cmd_vel, and the
wheels coast. A coasting robot looks exactly like the lurch being hunted. So
the switch goes through the set_parameters SERVICE from this same node, off the
publish thread, and the publish intervals are recorded so the log can PROVE the
publisher never stalled.
"""
import sys, os, json, time, math, threading, argparse
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from gripperx_control_msgs.msg import WheelVelocityReport

R = 0.070
MUX_TIMEOUT = 0.5
STATUS = {0: "DISABLED", 1: "ACTIVE", 2: "AT_LIMIT", 3: "OFF_PROVENANCE",
          4: "OFF_NO_MEAS", 5: "OFF_STALE", 6: "OFF_STALL", 7: "OFF_BELOW_FLOOR"}


class A13(Node):
    def __init__(self, topic):
        super().__init__("a13_tool")
        q = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(Twist, topic, q)
        self.create_subscription(WheelVelocityReport,
                                 "/swerve_controller/wheel_velocities", self.on_report, q)
        self.cli = self.create_client(SetParameters, "/swerve_controller/set_parameters")
        self.lock = threading.Lock()
        self.samples = []
        self.switch_t = None
        self.switch_done_t = None

    def on_report(self, msg):
        with self.lock:
            self.samples.append({
                "t": time.time(),
                "commanded": list(msg.commanded),
                "measured": list(msg.measured),
                "correction": list(msg.correction),
                "status": list(msg.regulator_status),
                "regulating": list(msg.regulating),
                "latched": list(msg.stall_latched),
            })

    def set_regulator(self, value: bool):
        """Fire the switch WITHOUT blocking anything that matters."""
        p = Parameter()
        p.name = "wheel_regulator_enabled"
        p.value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=value)
        req = SetParameters.Request(parameters=[p])
        self.switch_t = time.time()
        fut = self.cli.call_async(req)
        fut.add_done_callback(lambda f: setattr(self, "switch_done_t", time.time()))
        return fut


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--speed", type=float, default=0.16,
                    help="must be ABOVE the slow-end floor (0.14 m/s) or the regulator is off anyway")
    ap.add_argument("--settle-sec", type=float, default=4.0)
    ap.add_argument("--after-sec", type=float, default=3.0)
    ap.add_argument("--topic", default="/teleop/keyboard/cmd_vel")
    ap.add_argument("--out", default=os.path.expanduser("~/ws/runs/a13"))
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args()

    wheel = a.speed / R
    os.makedirs(a.out, exist_ok=True)
    jpath = os.path.join(a.out, "a13_%s.jsonl" % time.strftime("%Y%m%d_%H%M%S"))
    jf = open(jpath, "w")

    def rec(**kw):
        jf.write(json.dumps(kw) + "\n"); jf.flush()

    print("A13 -- disable the regulator WHILE DRIVING")
    print("  speed %.3f m/s -> wheel %.4f rad/s (floor 2.0; %s)"
          % (a.speed, wheel, "ABOVE, regulator can act" if wheel > 2.0 else "BELOW -- USELESS"))
    print("  authority bound 30%% of setpoint = %.4f rad/s" % (0.30 * wheel))
    if wheel <= 2.0:
        print("ABORT: at or below the floor the regulator is off anyway; nothing to disable.")
        return 2
    rec(event="derivation", speed=a.speed, wheel=wheel, authority=0.30 * wheel)

    rclpy.init()
    n = A13(a.topic)
    ex = MultiThreadedExecutor()
    ex.add_node(n)
    threading.Thread(target=ex.spin, daemon=True).start()

    if not n.cli.wait_for_service(timeout_sec=8.0):
        print("ABORT: /swerve_controller/set_parameters not available -- nothing commanded.")
        return 2
    t0 = time.time()
    while not n.samples and time.time() - t0 < 8.0:
        time.sleep(0.05)
    if not n.samples:
        print("ABORT: no report on /swerve_controller/wheel_velocities -- nothing commanded.")
        return 2
    st0 = n.samples[-1]["status"]
    print("  regulator status at rest: %s" % [STATUS.get(x, x) for x in st0])
    if a.dry_run:
        print("DRY RUN -- nothing published, nothing moved.")
        return 0

    # ---- drive, switch, keep driving -------------------------------------
    tw = Twist(); tw.linear.x = a.speed
    intervals = []
    stop_pub = threading.Event()

    def publisher():
        last = None
        while not stop_pub.is_set():
            now = time.time()
            if last is not None:
                intervals.append(now - last)
            last = now
            n.pub.publish(tw)
            time.sleep(0.05)          # 20 Hz, mux timeout is 0.5 s

    th = threading.Thread(target=publisher, daemon=True)
    with n.lock:
        n.samples.clear()
    th.start()
    time.sleep(a.settle_sec)

    print("\n  >>> switching wheel_regulator_enabled -> FALSE, still driving <<<")
    n.set_regulator(False)
    time.sleep(a.after_sec)

    stop_pub.set(); th.join(timeout=2.0)
    # exact zero stop, held
    for _ in range(40):
        n.pub.publish(Twist()); time.sleep(0.05)

    with n.lock:
        S = list(n.samples)
    tsw = n.switch_t

    # ---- did the publisher ever stall? -----------------------------------
    worst = max(intervals) if intervals else 0.0
    print("\n=== PUBLISHER HEALTH (the trap) ===")
    print("  %d intervals, worst %.3f s, mux timeout %.2f s -> %s"
          % (len(intervals), worst, MUX_TIMEOUT,
             "OK, never starved" if worst < 0.3 else "STALLED -- run INVALID"))
    rec(event="publisher", n=len(intervals), worst=worst, ok=worst < 0.3)
    if worst >= 0.3:
        print("  The mux may have zeroed /cmd_vel. A coast is NOT a lurch. Discard this run.")

    # ---- the step --------------------------------------------------------
    before = [s for s in S if s["t"] < tsw][-6:]
    after = [s for s in S if s["t"] >= tsw][:6]
    for s in S:
        rec(event="sample", **s)
    if not before or not after:
        print("no samples around the switch -- inconclusive")
        return 3

    print("\n=== THE SWITCH ===")
    print("  service call issued at t=0.000, reply after %.4f s"
          % ((n.switch_done_t - tsw) if n.switch_done_t else float("nan")))
    lastb = before[-1]
    print("\n  LAST SAMPLE BEFORE  (%.3f s before the call)" % (tsw - lastb["t"]))
    print("    status     %s" % [STATUS.get(x, x) for x in lastb["status"]])
    print("    correction %s" % ["%+.4f" % c for c in lastb["correction"]])
    print("    measured   %s" % ["%+.3f" % c for c in lastb["measured"]])
    maxcorr_before = max(abs(c) for c in lastb["correction"])

    print("\n  SAMPLES AFTER")
    for i, s in enumerate(after):
        print("    +%.3f s  status %s  corr %s  meas %s"
              % (s["t"] - tsw,
                 [STATUS.get(x, x) for x in s["status"]],
                 ["%+.4f" % c for c in s["correction"]],
                 ["%+.3f" % c for c in s["measured"]]))

    zeroed_at = None
    for s in after:
        if max(abs(c) for c in s["correction"]) == 0.0:
            zeroed_at = s["t"] - tsw
            break

    print("\n=== RESULT ===")
    print("  correction magnitude removed by the switch: %.4f rad/s" % maxcorr_before)
    print("     = %.1f %% of the setpoint (%.4f rad/s), authority bound is 30 %%"
          % (100.0 * maxcorr_before / wheel, wheel))
    print("  correction reached exactly 0.0 after: %s"
          % ("%.3f s" % zeroed_at if zeroed_at is not None else "NEVER within the window"))
    mb = [abs(x) for x in lastb["measured"]]
    ma = [abs(x) for x in after[-1]["measured"]]
    print("  |measured| before %s" % ["%.3f" % x for x in mb])
    print("  |measured| after  %s" % ["%.3f" % x for x in ma])
    print("  largest per-wheel speed change across the switch: %.4f rad/s"
          % max(abs(x - y) for x, y in zip(mb, ma)))
    print("\n  NOTE: this measures the COMMAND step and the wheel-speed response.")
    print("  Whether the chassis visibly LURCHED is an observation only a human")
    print("  can make -- there is no IMU on this robot (BNO085 not connected).")
    rec(event="result", maxcorr_before=maxcorr_before, wheel=wheel,
        zeroed_after=zeroed_at, measured_before=mb, measured_after=ma)
    print("\nrun record: %s" % jpath)
    jf.close()
    return 0


sys.exit(main())
