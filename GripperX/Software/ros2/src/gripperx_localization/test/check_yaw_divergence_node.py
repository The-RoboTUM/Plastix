#!/usr/bin/env python3
"""Node layer of the yaw-rate channel: _on_gyro, _mount_row, _yaw_status, return hold.

Runs the real OdomDivergenceMonitor in-process with rclpy, feeding it synthetic
/wheel/odom and /imu/data/filtered (+ /tf_static) and reading /diagnostics, in real
time (about 45 s). Parameters come from config/localization.yaml via --params-file;
the ONLY override is startup_grace_sec (to 2 s, to see NO_MOUNT without waiting for the
configured grace). Thresholds and holds are the configured ones.

Run with ROS_DOMAIN_ID=99 ROS_LOCALHOST_ONLY=1 on the dev laptop (the colcon wrapper
forces domain 220). Nothing here talks to a robot.
"""
from __future__ import annotations

import math
import os
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve()
sys.path.insert(0, str(HERE.parents[1] / "src"))

import rclpy  # noqa: E402
from diagnostic_msgs.msg import DiagnosticArray  # noqa: E402
from geometry_msgs.msg import TransformStamped  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from rosgraph_msgs.msg import Clock  # noqa: E402
from sensor_msgs.msg import Imu  # noqa: E402
from tf2_msgs.msg import TFMessage  # noqa: E402

import yaml  # noqa: E402
from gripperx_localization.odom_divergence_monitor import OdomDivergenceMonitor  # noqa: E402

YAML = HERE.parents[1] / "config" / "localization.yaml"
P = yaml.safe_load(YAML.read_text())["odom_divergence_monitor"]["ros__parameters"]
FAILURES: list[str] = []
WARN, ERROR, OK = 1, 2, 0


def check(cond, label, detail=""):
    print(f"  {'ok  ' if cond else 'FAIL'}  {label}")
    if not cond:
        print(f"          {detail}")
        FAILURES.append(label)


class Rig:
    def __init__(self, sim_time=False):
        args = ["--ros-args", "--params-file", str(YAML)]
        self.mon = OdomDivergenceMonitor(
            parameter_overrides=[Parameter("startup_grace_sec", value=2.0),
                                 Parameter("use_sim_time", value=sim_time)])
        self.h = Node("yaw_test_rig")
        self.po = self.h.create_publisher(Odometry, P["wheel_odom_topic"], 10)
        self.pi = self.h.create_publisher(Imu, P["gyro_topic"], 10)
        self.ptf = self.h.create_publisher(
            TFMessage, "/tf_static",
            QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        self.pclock = self.h.create_publisher(Clock, "/clock", 10)
        self.diag = []     # (wall, level, state, message, episodes)
        self.h.create_subscription(DiagnosticArray, "/diagnostics", self._on_diag, 50)
        self.ex = SingleThreadedExecutor()
        self.ex.add_node(self.mon)
        self.ex.add_node(self.h)
        self.sim = sim_time
        self.sim_t = None

    def _on_diag(self, m):
        for s in m.status:
            if "yaw" in s.name:
                kv = {k.key: k.value for k in s.values}
                self.diag.append((time.time(), s.level[0] if isinstance(s.level, bytes) else s.level,
                                  kv.get("state"), s.message, kv.get("episodes")))

    def stamp(self):
        if self.sim:
            ns = int(self.sim_t * 1e9)
            from builtin_interfaces.msg import Time
            return Time(sec=ns // 10**9, nanosec=ns % 10**9)
        return self.h.get_clock().now().to_msg()

    def run(self, sec, enc_dps=0.0, gyro_dps=0.0, gyro_on=True, frame="imu_link", enc_on=True, hz=50.0):
        """Publish for `sec` s of wall time; gyro_dps is the SENSOR z rate."""
        end = time.time() + sec
        k = 0
        while time.time() < end:
            if self.sim:
                self.sim_t += 1.0 / hz
                c = Clock()
                ns = int(self.sim_t * 1e9)
                c.clock.sec, c.clock.nanosec = ns // 10**9, ns % 10**9
                self.pclock.publish(c)
            st = self.stamp()
            if enc_on and k % 2 == 0:
                o = Odometry()
                o.header.stamp = st
                o.twist.twist.angular.z = math.radians(enc_dps)
                self.po.publish(o)
            if gyro_on:
                i = Imu()
                i.header.stamp = st
                i.header.frame_id = frame
                i.angular_velocity.z = math.radians(gyro_dps)
                self.pi.publish(i)
            k += 1
            self.spin(1.0 / hz)

    def spin(self, sec):
        """Run every callback that is ready for `sec` s (spin_once handles ONE)."""
        end = time.time() + sec
        while time.time() < end:
            self.ex.spin_once(timeout_sec=0.001)

    def publish_mount(self):
        t = TransformStamped()
        t.header.frame_id = "base_footprint"
        t.child_frame_id = "imu_link"
        t.transform.rotation.x = 1.0      # roll pi: the real mount
        t.transform.rotation.w = 0.0
        t.header.stamp = self.h.get_clock().now().to_msg()
        self.ptf.publish(TFMessage(transforms=[t]))

    def last(self):
        return self.diag[-1] if self.diag else None

    def close(self):
        self.ex.shutdown()
        self.mon.destroy_node()
        self.h.destroy_node()


def main():
    rclpy.init(args=["--ros-args", "--params-file", str(YAML)])
    tol, hold = P["yaw_abs_tol_dps"], P["return_hold_sec"]
    rig = Rig()
    try:
        print("NO_MOUNT: gyro samples arrive, the mount is not in /tf_static")
        rig.run(4.0, enc_dps=0.0, gyro_dps=0.0)
        l = rig.last()
        check(l and l[2] == "NO_MOUNT" and l[1] == WARN, "state NO_MOUNT at WARN", str(l))
        check(rig.mon._gyro_without_mount > 0, "_on_gyro counted the unsignable samples")

        print("mount arrives: sign through the cached z row (roll pi)")
        rig.publish_mount()
        rig.run(4.0, enc_dps=60.0, gyro_dps=-60.0)          # body +60 deg/s on both
        l = rig.last()
        check(rig.mon.yaw.imu_returns == 0 and rig.mon.yaw.imu_first_appearances == 1,
              'a first appearance is counted apart from returns', str((rig.mon.yaw.imu_returns, rig.mon.yaw.imu_first_appearances)))
        check('RETURNED' not in l[3] and 'APPEARED' in l[3], 'first appearance is not reported as a return', l[3])
        returns0 = 0
        row = rig.mon._mount_rows.get("imu_link")
        check(row is not None and abs(row[2] + 1.0) < 1e-6, "_mount_row cached (0, 0, -1)", str(row))
        check(l[2] == "OK", "sensor -60 and encoder +60 agree (sign through the mount): state OK", str(l))
        check(l[1] == WARN, "level is WARN while the first appearance is within return_hold_sec", str(l))

        print("episode: encoder 8x abs tolerance off the gyro for 3 s")
        n0 = rig.mon.yaw.episodes
        rig.run(3.0, enc_dps=60.0 + 8 * tol, gyro_dps=-60.0)
        check(rig.mon.yaw.episodes == n0 + 1, "episode counted at event time", str(rig.mon.yaw.episodes))
        rig.run(2.0, enc_dps=60.0, gyro_dps=-60.0)
        l = rig.last()
        check(l[2] in ("DIVERGED_HELD", "DIVERGED") and l[1] == ERROR,
              "status still ERROR after the divergence ended (hold)", str(l))
        check(l[4] == str(n0 + 1), "episodes value on the status", str(l))

        print("IMU silence and return (real time: return_hold_sec = %.0f s)" % hold)
        rig.run(P["alarm_hold_sec"] + 2.0, enc_dps=60.0, gyro_dps=-60.0)
        rig.run(hold, enc_dps=60.0, gyro_dps=-60.0)       # let the first appearance's hold end
        rig.run(P["imu_silence_timeout_sec"] + 1.5, enc_dps=60.0, gyro_on=False)
        l = rig.last()
        check(l[2] == "IMU_SILENT" and l[1] == WARN, "IMU_SILENT at WARN", str(l))
        t_back = time.time()
        mark = len(rig.diag)
        rig.run(hold + 1.5, enc_dps=60.0, gyro_dps=-60.0)
        inside = [d for d in rig.diag[mark:] if d[0] < t_back + hold - 0.7]
        check(len(inside) >= 8 and all(d[1] >= WARN for d in inside),
              "every status inside return_hold_sec is WARN or worse (no green hole after the return)",
              str([(round(d[0] - t_back, 1), d[1], d[2]) for d in inside if d[1] < WARN]))
        check(any(d[2] == "WARMUP" and d[1] == WARN for d in inside) or any(
            d[2] == "OK" and d[1] == WARN for d in inside), "WARMUP/OK held at WARN after the return")
        l = rig.last()
        check(l[2] == "OK" and l[1] == OK, "back to OK/level OK after the hold", str(l))
        check(rig.mon.yaw.imu_returns == returns0 + 1, "the silence's return is counted once",
              f"{rig.mon.yaw.imu_returns} vs {returns0}")
    finally:
        rig.close()

    print("sim time: clock reads 0 until the first /clock")
    rig = Rig(sim_time=True)
    try:
        rig.sim_t = 1.8e9
        rig.publish_mount()
        end = time.time() + 2.0
        while time.time() < end:
            rig.spin(0.05)                           # no /clock yet, no data
        l = rig.last()
        check(l is None, "no status and no false state before the first /clock "
                         "(the publish timer follows sim time, so it cannot fire yet)", str(l))
        rig.run(4.0, enc_dps=0.0, gyro_dps=0.0)
        check(rig.mon.yaw.imu_returns == 0 and rig.mon.yaw.clock_resets == 0,
              "first /clock (1.8e9 s) causes no false return or reset",
              f"returns {rig.mon.yaw.imu_returns}, resets {rig.mon.yaw.clock_resets}")
        l = rig.last()
        check(l[2] in ("OK", "WARMUP"), "sim-time channel runs", str(l))
    finally:
        rig.close()
        rclpy.shutdown()

    print()
    if FAILURES:
        print("FAILURES:", *FAILURES, sep="\n  ")
        return 1
    print("ALL NODE CHECKS PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
