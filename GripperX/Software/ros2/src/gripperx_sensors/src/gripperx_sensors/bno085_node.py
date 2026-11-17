#!/usr/bin/env python3
"""BNO085 IMU driver: sensor_msgs/Imu from the SH-2 reports over /dev/i2c-N.

WHAT IT PUBLISHES. One Imu per input packet that carries a gyroscope report:
calibrated angular velocity (rad/s), accelerometer with gravity included (m/s^2,
the ROS convention: +9.81 on the axis pointing up at rest), and the orientation
from the report chosen by `orientation_report`. All in the sensor's own axes,
stamped `frame_id` (imu_link), whose mounting pose the URDF carries.

WHY THE GAME ROTATION VECTOR BY DEFAULT. It fuses accelerometer and gyroscope
only. The IMU sits among the drive motors, and absolute IMU yaw is not fused at
all (user decision D2, 2026-10-22: yaw rate only), so a magnetic heading would
buy nothing and import the motors' fields. `rotation_vector` (magnetometer included) stays selectable.

WHERE IT GOES. The topic comes from the parameter file (/imu/bno085/data), which
is what localization_input_node reads; it republishes the conditioned samples on
/imu/data/filtered for the EKF. Which of them the EKF fuses is set in
gripperx_localization's localization.yaml (imu0_config), not by this driver.

I2C ON THE PI 5. The BNO08x stretches the clock. The Pi 1-4 Broadcom controller
mishandles that; the Pi 5's header I2C is on RP1 (Synopsys DesignWare
controller), which is not that controller. The driver does not rely on the bus
being clean anyway: every header is validated, malformed packets are counted
and dropped, and after `max_consecutive_errors` failures or `stale_timeout_s`
without a sample it reopens the bus and soft-resets the sensor (the SH-2 command
on channel 1; the RST line, HWR-42, is pulsed only once per hung episode and on
~/hw_reset, whose response reports whether the chip was seen to reboot).

NO RESET ON START BY DEFAULT. At a non-power-up reset the sensor persists its
RAM dynamic calibration data to flash (BNO08X datasheet 1000-3927 v1.17 section
3.4). With `reset_on_start` false (default) a normal start does not reset; it
switches off every report the package may have enabled that this run does not
use (a previous run's orientation report, the query tool's raw accelerometer)
and enables the ones it does - see bno085_device. The reset is kept for
recovery, where the sensor state is unknown.

EXIT STATUS. If the reader thread dies the process exits non-zero, so a
supervisor (systemd Restart=on-failure) can restart it.

TIMESTAMP. Host clock at the moment the packet was read. The sensor's own report
delay (< 1 report interval) is not subtracted.
"""

from __future__ import annotations

import sys
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger

from gripperx_sensors import bno08x_shtp as sh2
from gripperx_sensors.bno085_device import HUNG_HINT, HW_RESET_VERDICT_WAIT_S, Bno085Device
from gripperx_sensors.bno085_reset_line import make_reset_line


def _diag3(variance: float) -> list[float]:
    return [variance, 0.0, 0.0, 0.0, variance, 0.0, 0.0, 0.0, variance]


class Bno085Node(Node):
    def __init__(self) -> None:
        super().__init__("bno085_imu")

        self.declare_parameter("i2c_bus", 1)
        self.declare_parameter("i2c_address", 0x4A)
        self.declare_parameter("imu_topic", "/imu/bno085/data")
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("orientation_report", "game_rotation_vector")
        self.declare_parameter("report_interval_us", 20000)
        self.declare_parameter("orientation_stddev_rad", -1.0)
        self.declare_parameter("angular_velocity_stddev_rad_s", -1.0)
        self.declare_parameter("linear_acceleration_stddev_m_s2", -1.0)
        self.declare_parameter("max_consecutive_errors", 20)
        self.declare_parameter("stale_timeout_s", 1.0)
        self.declare_parameter("idle_poll_s", 0.002)
        self.declare_parameter("stats_period_s", 10.0)
        self.declare_parameter("reset_on_start", False)
        self.declare_parameter("recovery_backoff_initial_s", 1.0)
        self.declare_parameter("recovery_backoff_max_s", 300.0)
        self.declare_parameter("max_recovery_resets_per_hour", 6)
        self.declare_parameter("healthy_period_s", 60.0)
        self.declare_parameter("hung_recoveries", 3)
        self.declare_parameter("hw_reset_gpio_chip", "")
        self.declare_parameter("hw_reset_gpio_line", -1)
        self.declare_parameter("hw_reset_pulse_s", 0.0)
        self.declare_parameter("hw_reset_settle_s", 0.0)

        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.bus_no = int(p("i2c_bus"))
        self.address = int(p("i2c_address"))
        self.frame_id = str(p("frame_id"))
        self.interval_us = int(p("report_interval_us"))
        if self.interval_us < 2500:
            raise ValueError(f"report_interval_us {self.interval_us} below 2500 (400 Hz)")
        report = str(p("orientation_report"))
        if report != "none" and report not in sh2.ORIENTATION_REPORTS:
            raise ValueError(
                f"orientation_report {report!r}: expected one of "
                f"{sorted(sh2.ORIENTATION_REPORTS)} or 'none'"
            )
        self.orientation_id = sh2.ORIENTATION_REPORTS.get(report)
        self.reset_on_start = bool(p("reset_on_start"))

        # A stddev <= 0 means "not known": sensor_msgs/Imu says covariance all
        # zero = unknown, and orientation_covariance[0] = -1 = no orientation.
        self.orientation_cov = self._cov(float(p("orientation_stddev_rad")))
        self.gyro_cov = self._cov(float(p("angular_velocity_stddev_rad_s")))
        self.accel_cov = self._cov(float(p("linear_acceleration_stddev_m_s2")))
        if self.orientation_id is None:
            self.orientation_cov = [-1.0] + [0.0] * 8

        topic = str(p("imu_topic"))
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(Imu, topic, qos)

        self.diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.failed = False
        self._stop = threading.Event()
        self.dev = Bno085Device(
            sh2.I2cDev(self.bus_no, self.address),
            orientation_id=self.orientation_id,
            interval_us=self.interval_us,
            reset_on_start=self.reset_on_start,
            max_errors=int(p("max_consecutive_errors")),
            stale_timeout=float(p("stale_timeout_s")),
            idle_poll=float(p("idle_poll_s")),
            on_sample=self._publish,
            log=self._log,
            stop=self._stop,
            backoff_initial_s=float(p("recovery_backoff_initial_s")),
            backoff_max_s=float(p("recovery_backoff_max_s")),
            max_recovery_resets_per_hour=int(p("max_recovery_resets_per_hour")),
            healthy_period_s=float(p("healthy_period_s")),
            hung_recoveries=int(p("hung_recoveries")),
            reset_line=make_reset_line(str(p("hw_reset_gpio_chip")),
                                       int(p("hw_reset_gpio_line")),
                                       float(p("hw_reset_pulse_s")),
                                       float(p("hw_reset_settle_s")), self._log,
                                       sleep=self._stop.wait),
        )
        # Not a motion command: it only resets the IMU chip, so it has no extra gating.
        self.create_service(Trigger, "~/hw_reset", self._on_hw_reset)
        self._last_stats = self.dev.stats()
        self._last_stats_t = time.monotonic()

        self.get_logger().info(
            f"BNO085 on /dev/i2c-{self.bus_no} addr 0x{self.address:02X}: "
            f"orientation={report}, interval={self.interval_us} us, topic={topic}, "
            f"frame={self.frame_id}, reset_on_start={self.reset_on_start}")
        self.create_timer(float(p("stats_period_s")), self._log_stats)
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    @staticmethod
    def _cov(stddev: float) -> list[float]:
        return _diag3(stddev * stddev) if stddev > 0.0 else [0.0] * 9

    def _log(self, level: str, text: str) -> None:
        # rclpy fixes a severity per calling source line and raises
        # "Logger severity cannot be changed between calls" if one line logs at
        # two levels, so every level has its own call here.
        lg = self.get_logger()
        if level == "debug":
            lg.debug(text)
        elif level == "info":
            lg.info(text)
        elif level == "warn":
            lg.warning(text)
        elif level == "error":
            lg.error(text)
        else:
            lg.fatal(text)

    def _run(self) -> None:
        try:
            self.dev.run(keep_going=rclpy.ok)
        except Exception as exc:  # noqa: BLE001 - a dead reader must end the process
            if rclpy.ok() and not self._stop.is_set():
                self.failed = True
                self.get_logger().fatal(f"BNO085 reader thread died: {exc!r}")
                rclpy.try_shutdown()
        finally:
            # Closed here, by the only thread that uses the bus, never under it.
            try:
                self.dev.bus.close()
            except OSError:
                pass

    def _publish(self, gyro, accel, quat) -> bool:
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        if quat is not None:
            q = quat
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = q
        msg.orientation_covariance = list(self.orientation_cov)
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = gyro
        msg.angular_velocity_covariance = list(self.gyro_cov)
        a = accel
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = a
        msg.linear_acceleration_covariance = list(self.accel_cov)
        try:
            self.pub.publish(msg)
        except Exception:  # noqa: BLE001 - only swallowed while shutting down
            # SIGINT invalidates the context from the main thread while this
            # reader thread may be mid-publish.
            if rclpy.ok() and not self._stop.is_set():
                raise
            self._stop.set()
            return False
        return True

    def _log_stats(self) -> None:
        now = time.monotonic()
        cur = self.dev.stats()
        dt = now - self._last_stats_t
        rate = (cur["published"] - self._last_stats["published"]) / dt if dt > 0 else 0.0
        delta = {k: cur[k] - self._last_stats[k] for k in cur if k != "published"}
        self._last_stats, self._last_stats_t = cur, now
        self._publish_diagnostics(rate)
        self.get_logger().info(
            f"rate {rate:.1f} Hz | last {dt:.0f} s: "
            + ", ".join(f"{k} {v}" for k, v in delta.items())
            + f" | accuracy accel/gyro/orient {self.dev.status['accel']}/"
              f"{self.dev.status['gyro']}/{self.dev.status['quat']} (0..3)"
            + (" | HUNG: power cycle required" if self.dev.hung else ""))

    def _on_hw_reset(self, request, response):
        # Blocks this executor for at most the verdict wait; the reader is its own thread.
        response.success, response.message = self.dev.hw_reset(wait_s=HW_RESET_VERDICT_WAIT_S)
        return response

    def _publish_diagnostics(self, rate: float) -> None:
        st = DiagnosticStatus()
        st.name = "bno085_imu"
        st.hardware_id = f"bno085:i2c-{self.bus_no}:0x{self.address:02X}"
        if self.dev.hung:
            st.level, st.message = DiagnosticStatus.ERROR, HUNG_HINT
        elif rate <= 0.0:
            st.level, st.message = DiagnosticStatus.WARN, "no IMU samples"
        else:
            st.level, st.message = DiagnosticStatus.OK, "ok"
        cause = self.dev.reset_cause
        st.values = [KeyValue(key="hung", value=str(self.dev.hung).lower()),
                     KeyValue(key="rate_hz", value=f"{rate:.1f}"),
                     KeyValue(key="reset_cause", value="unknown" if cause is None else str(cause))]
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [st]
        self.diag_pub.publish(msg)

    def destroy_node(self):
        self._stop.set()
        # Every wait in the reader is on _stop; what remains is at most one
        # blocking i2c transfer. The reader closes the bus itself on its way out.
        self._thread.join(timeout=5.0)
        if self._thread.is_alive():
            self.get_logger().error("BNO085 reader thread did not stop within 5 s")
        return super().destroy_node()


def main(args=None) -> int:
    rclpy.init(args=args)
    node = Bno085Node()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 1 if node.failed else 0


if __name__ == "__main__":
    sys.exit(main())
