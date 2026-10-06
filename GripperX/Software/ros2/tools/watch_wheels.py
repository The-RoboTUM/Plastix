#!/usr/bin/env python3
"""Watch wheel encoder positions live. COMMANDS NOTHING -- read-only.

Turn each wheel BY HAND and watch which counters move. Separates
"the wheel is not driven" from "the encoder is not counting".
"""
import sys, time, math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64MultiArray

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
LABELS = ["FL", "FR", "BL", "BR"]
R = 0.070

class W(Node):
    def __init__(self):
        super().__init__("watch_wheels_tool")
        q = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Float64MultiArray, "/hw/joint_states", self.cb, q)
        self.base = None; self.cur = None; self.enc = None
        self.span = [0.0]*4          # max-min excursion per wheel
        self.lo = None; self.hi = None

    def cb(self, msg):
        d = msg.data
        if len(d) < 16: return
        pos = list(d[8:12]); self.enc = [int(x) for x in d[12:16]]
        if self.base is None:
            self.base = list(pos); self.lo = list(pos); self.hi = list(pos)
        self.lo = [min(a,b) for a,b in zip(self.lo,pos)]
        self.hi = [max(a,b) for a,b in zip(self.hi,pos)]
        self.span = [h-l for h,l in zip(self.hi,self.lo)]
        self.cur = pos

rclpy.init(); n = W()
t0 = time.time(); last = 0
print("Drehe die Raeder VON HAND. Nichts wird kommandiert.  (Strg-C beendet)\n")
try:
    while time.time() - t0 < DUR:
        rclpy.spin_once(n, timeout_sec=0.1)
        if n.cur and time.time() - last > 1.0:
            last = time.time()
            print("  " + " | ".join(
                f"{l}: {math.degrees(s):7.1f}deg bewegt  prov={e}"
                for l, s, e in zip(LABELS, n.span, n.enc)))
except KeyboardInterrupt:
    pass
print("\n=== ERGEBNIS: Gesamtausschlag je Rad ===")
for l, s, e in zip(LABELS, n.span, n.enc):
    verdict = "ENCODER ZAEHLT" if abs(math.degrees(s)) > 5.0 else "KEINE BEWEGUNG GEMESSEN"
    print(f"  {l}: {math.degrees(s):8.1f} deg   provenance={e}   -> {verdict}")
rclpy.shutdown()
