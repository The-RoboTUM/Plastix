#!/usr/bin/env python3
"""Read the IMU in base_link axes, to check its mounting by hand. Moves nothing.

Subscribes to an Imu topic and rotates angular velocity and acceleration from
the message frame (imu_link) into base_link with the mounting rotation, then
prints them. Two modes:

  live (default)   prints ~5 lines/s for the hand-rotation check:
                   rotate the robot CCW seen from above -> base wz > 0;
                   lift the FRONT -> base wy < 0 while lifting and pitch < 0
                   (REP-103: x forward, y left, z up; nose up = negative pitch).
  --stats SECONDS  robot at rest: publish rate, mean and spread of acceleration
                   (imu_link and base_link), gyroscope bias, orientation drift.

Mounting rotation: looked up from TF (base_link <- frame_id) by default, i.e.
whatever the running robot_description says. Until the bringup runs a URDF with
the measured imu_joint, pass it explicitly: --rpy R P Y (the imu_mount_rpy
values in gripperx_geometry/config/geometry.yaml).
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu


def quat_from_rpy(r: float, p: float, y: float):
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def quat_conj(q):
    return (-q[0], -q[1], -q[2], q[3])


def rotate(q, v):
    """Rotate vector v by unit quaternion q (x, y, z, w)."""
    p = quat_mul(quat_mul(q, (v[0], v[1], v[2], 0.0)), quat_conj(q))
    return (p[0], p[1], p[2])


def rpy_from_quat(q):
    x, y, z, w = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


class AxisCheck(Node):
    def __init__(self, args) -> None:
        super().__init__("imu_axis_check")
        self.args = args
        self.q_base_imu = quat_from_rpy(*args.rpy) if args.rpy else None
        self.tf_buffer = None
        if self.q_base_imu is None:
            from tf2_ros import Buffer, TransformListener
            self.tf_buffer = Buffer()
            self._tf_listener = TransformListener(self.tf_buffer, self)
        self.samples = []
        self.last_print = 0.0
        self.t0 = None
        self.create_subscription(Imu, args.topic, self.on_imu, 50)

    def mount(self, frame_id: str):
        if self.q_base_imu is None and self.tf_buffer is not None:
            try:
                t = self.tf_buffer.lookup_transform(self.args.base_frame, frame_id, rclpy.time.Time())
            except Exception as exc:  # noqa: BLE001 - report and keep waiting
                self.get_logger().warn(f"waiting for TF {self.args.base_frame} <- {frame_id}: {exc}",
                                       throttle_duration_sec=2.0)
                return None
            r = t.transform.rotation
            self.q_base_imu = (r.x, r.y, r.z, r.w)
            rpy = rpy_from_quat(self.q_base_imu)
            self.get_logger().info(
                "mounting from TF: rpy = " + " ".join(f"{v:+.4f}" for v in rpy) + " rad")
        return self.q_base_imu

    def on_imu(self, msg: Imu) -> None:
        q_bi = self.mount(msg.header.frame_id)
        if q_bi is None:
            return
        now = time.monotonic()
        if self.t0 is None:
            self.t0 = now
        w_i = (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)
        a_i = (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)
        w_b = rotate(q_bi, w_i)
        a_b = rotate(q_bi, a_i)
        o = msg.orientation
        has_orientation = msg.orientation_covariance[0] != -1.0
        # Orientation of base_link in the sensor's world frame:
        # R_world_base = R_world_imu * R_imu_base.
        q_wb = quat_mul((o.x, o.y, o.z, o.w), quat_conj(q_bi)) if has_orientation else None
        if self.args.stats:
            self.samples.append((now, w_i, a_i, w_b, a_b, q_wb))
            if now - self.t0 >= self.args.stats:
                self.report()
                raise SystemExit(0)
            return
        if now - self.last_print < 0.2:
            return
        self.last_print = now
        line = ("base w [deg/s] x {:+7.2f} y {:+7.2f} z {:+7.2f} | "
                "base a [m/s2] x {:+6.2f} y {:+6.2f} z {:+6.2f}").format(
                    *(math.degrees(v) for v in w_b), *a_b)
        if q_wb is not None:
            r, p, y = (math.degrees(v) for v in rpy_from_quat(q_wb))
            line += f" | base roll {r:+7.2f} pitch {p:+7.2f} yaw {y:+8.2f} deg"
        print(line, flush=True)

    def report(self) -> None:
        s = self.samples
        n = len(s)
        dur = s[-1][0] - s[0][0]

        def mean_sd(idx):
            cols = list(zip(*(row[idx] for row in s)))
            means = [sum(c) / n for c in cols]
            sds = [math.sqrt(sum((v - m) ** 2 for v in c) / max(n - 1, 1)) for c, m in zip(cols, means)]
            return means, sds

        print(f"samples {n} over {dur:.1f} s -> rate {(n - 1) / dur if dur > 0 else 0:.2f} Hz")
        for label, idx, unit, scale in (("accel imu_link ", 2, "m/s2", 1.0),
                                        ("accel base_link", 4, "m/s2", 1.0),
                                        ("gyro  imu_link ", 1, "deg/s", math.degrees(1)),
                                        ("gyro  base_link", 3, "deg/s", math.degrees(1))):
            m, sd = mean_sd(idx)
            print(f"{label} mean " + " ".join(f"{v * scale:+9.4f}" for v in m)
                  + f"  sd " + " ".join(f"{v * scale:8.4f}" for v in sd) + f"  [{unit}]")
        m, _ = mean_sd(4)
        print(f"|accel| base_link mean {math.sqrt(sum(v * v for v in m)):.4f} m/s2")
        qs = [row[5] for row in s if row[5] is not None]
        if qs:
            first = [math.degrees(v) for v in rpy_from_quat(qs[0])]
            last = [math.degrees(v) for v in rpy_from_quat(qs[-1])]
            rpys = [[math.degrees(v) for v in rpy_from_quat(q)] for q in qs]
            spread = [max(c) - min(c) for c in zip(*rpys)]
            print("base rpy first " + " ".join(f"{v:+8.3f}" for v in first)
                  + " | last " + " ".join(f"{v:+8.3f}" for v in last) + " deg")
            print("base rpy drift (last-first) " + " ".join(f"{b - a:+7.3f}" for a, b in zip(first, last))
                  + " | peak-to-peak " + " ".join(f"{v:6.3f}" for v in spread) + " deg")


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--topic", default="/imu/bno085/data")
    ap.add_argument("--base-frame", default="base_link")
    ap.add_argument("--rpy", type=float, nargs=3, metavar=("R", "P", "Y"),
                    help="mounting rotation base_link <- imu frame, rad (default: from TF)")
    ap.add_argument("--stats", type=float, default=0.0, metavar="SECONDS",
                    help="collect for SECONDS at rest and print statistics")
    args, ros_args = ap.parse_known_args(argv if argv is not None else sys.argv[1:])
    rclpy.init(args=[sys.argv[0]] + ros_args)
    node = AxisCheck(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
