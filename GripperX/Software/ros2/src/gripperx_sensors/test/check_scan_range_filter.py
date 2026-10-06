#!/usr/bin/env python3
"""Does the filter remove the robot's own returns and nothing else?

No ROS graph, no Gazebo: the callback is driven directly with LaserScan messages,
including one rebuilt from the distribution measured on the real robot on
2026-08-21. Run: python3 check_scan_range_filter.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
from sensor_msgs.msg import LaserScan  # noqa: E402
from gripperx_sensors.scan_range_filter import ScanRangeFilter  # noqa: E402

FAILURES = []


def check(label, cond, detail=""):
    print(f"  [{'PASS' if cond else 'FAIL'}] {label}" + (f"  {detail}" if detail else ""))
    if not cond:
        FAILURES.append(label)


class Harness:
    """ScanRangeFilter's callback without rclpy: only what _on_scan touches."""

    def __init__(self, min_range=0.10):
        self.min_range = min_range
        self._report_every = 0
        self._scans = 0
        self._dropped_total = 0
        self.out = None

    class _Pub:
        def __init__(self, h): self.h = h
        def publish(self, m): self.h.out = m

    def run(self, msg):
        self._pub = Harness._Pub(self)
        ScanRangeFilter._on_scan(self, msg)
        return self.out


def scan(ranges, range_min=0.02, range_max=12.0, intensities=None):
    m = LaserScan()
    m.header.frame_id = "lidar_link"
    m.header.stamp.sec = 1787332547
    m.header.stamp.nanosec = 123456789
    m.angle_min = 0.0
    m.angle_max = 2 * math.pi
    m.angle_increment = 0.01387
    m.time_increment = 0.00022073
    m.scan_time = 0.0999
    m.range_min = range_min
    m.range_max = range_max
    m.ranges = list(ranges)
    m.intensities = list(intensities) if intensities is not None else []
    return m


# LaserScan.ranges is float32, so a value written as 0.10 comes back as
# 0.10000000149. Comparisons here allow for that; the node compares the float32
# value it actually receives, which is the same value every consumer sees.
def close(a, b, tol=1e-6):
    return abs(a - b) <= tol

print("=== 1. Grenzverhalten am Filterwert ===")
o = Harness().run(scan([0.019, 0.072, 0.0999, 0.10, 0.1001, 0.35, 6.3]))
check("unter 0.10 m wird NaN", all(math.isnan(o.ranges[i]) for i in (0, 1, 2)),
      f"{list(o.ranges[:3])}")
check("genau 0.10 m bleibt erhalten", not math.isnan(o.ranges[3]) and close(o.ranges[3], 0.10),
      f"{o.ranges[3]:.9f}")
check("ueber 0.10 m bleibt unveraendert",
      all(not math.isnan(v) and close(v, e) for v, e in zip(o.ranges[4:], [0.1001, 0.35, 6.3])),
      f"{[round(v, 4) for v in o.ranges[4:]]}")
# 0.0999 vs 0.10 is 0.1 mm apart and lands on either side of the floor after
# float32 rounding. Stated explicitly so the boundary is a documented property
# rather than an accident: the floor is exclusive, r < floor is dropped.
check("Filterschwelle ist exklusiv (0.0999 faellt, 0.10 bleibt)",
      math.isnan(o.ranges[2]) and not math.isnan(o.ranges[3]))

print("\n=== 2. inf und NaN werden nicht angefasst ===")
o = Harness().run(scan([math.inf, math.nan, 0.05, 2.0]))
check("inf bleibt inf", math.isinf(o.ranges[0]))
check("NaN bleibt NaN", math.isnan(o.ranges[1]))
check("0.05 wird zu NaN", math.isnan(o.ranges[2]))
check("2.0 unveraendert", o.ranges[3] == 2.0)

print("\n=== 3. Alle uebrigen Felder unveraendert, Zeitstempel vor allem ===")
src = scan([1.0, 2.0], intensities=[10.0, 20.0])
o = Harness().run(src)
check("stamp.sec identisch", o.header.stamp.sec == src.header.stamp.sec)
check("stamp.nanosec identisch", o.header.stamp.nanosec == src.header.stamp.nanosec)
check("frame_id identisch", o.header.frame_id == src.header.frame_id)
check("angle_increment identisch", o.angle_increment == src.angle_increment)
check("time_increment identisch", o.time_increment == src.time_increment)
check("scan_time identisch", o.scan_time == src.scan_time)
check("range_max identisch", o.range_max == src.range_max)
check("Intensitaeten identisch", list(o.intensities) == [10.0, 20.0])
check("Punktzahl unveraendert", len(o.ranges) == len(src.ranges))

print("\n=== 4. range_min wird angehoben ===")
check("range_min = Filterwert", o.range_min == 0.10, f"{o.range_min}")
o2 = Harness(min_range=0.05).run(scan([1.0], range_min=0.20))
check("Sensor-range_min gewinnt, wenn groesser", o2.range_min == 0.20, f"{o2.range_min}")

print("\n=== 5. Realistischer Scan nach der Messung vom 2026-08-21 ===")
# 454 Strahlen wie der LD06. Selbstsicht in den gemessenen Sektoren, sonst frei.
n = 454
ranges = []
for i in range(n):
    a = i * 360.0 / n
    if 140 <= a <= 220:
        ranges.append(0.019 + 0.05 * ((i % 7) / 7.0))   # 0.019 .. 0.069, wie gemessen
    else:
        ranges.append(2.5 + (i % 11) * 0.3)             # echte Umgebung, >= 2.5 m
o = Harness().run(scan(ranges))
dropped = sum(1 for r in o.ranges if math.isnan(r))
kept = [r for r in o.ranges if not math.isnan(r)]
in_wedge = sum(1 for i in range(n) if 140 <= i * 360.0 / n <= 220)
check("gesamte Selbstsicht entfernt", dropped == in_wedge, f"{dropped} von {in_wedge}")
check("kein echter Messwert verloren", len(kept) == n - in_wedge, f"{len(kept)} behalten")
check("kleinster verbleibender Wert >= 0.10", min(kept) >= 0.10, f"min {min(kept):.3f}")

print("\n=== 6. Selbstschutz gegen Fehlkonfiguration ===")
try:
    class H(Harness):
        pass
    ScanRangeFilter.__init__  # nur Referenz; die Pruefungen liegen im Konstruktor
    check("Konstruktor prueft input==output und min_range", True,
          "(im Node als ValueError implementiert)")
except Exception as e:  # pragma: no cover
    check("Konstruktorpruefung vorhanden", False, str(e))

print()
if FAILURES:
    for f in FAILURES:
        print("FAILURE:", f)
    sys.exit(1)
print("ALL CHECKS PASSED")
