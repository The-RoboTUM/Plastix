"""A dead reader thread must end the process with a non-zero status, so that a
supervisor's Restart=on-failure fires. Needs rclpy (a sourced ROS 2)."""

import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
rclpy = pytest.importorskip("rclpy")
pytest.importorskip("sensor_msgs.msg")
from gripperx_sensors import bno085_node  # noqa: E402


def test_reader_death_exits_non_zero(monkeypatch):
    def die(self, keep_going=None):
        raise RuntimeError("simulated reader failure")

    monkeypatch.setattr(bno085_node.Bno085Device, "run", die)
    assert bno085_node.main(args=["--ros-args", "-p", "stats_period_s:=100.0"]) == 1


def test_every_device_log_level_goes_through_the_real_rclpy_logger():
    # rclpy raises "Logger severity cannot be changed between calls" when one
    # source line logs at two severities; the robot's reader died of exactly
    # that on its first restart. Drive the device's restart, recovery, error
    # and busy paths through the node's real log callback.
    from rclpy.node import Node
    from gripperx_sensors import bno08x_shtp as sh2
    from gripperx_sensors.bno085_device import Bno085Device
    from test_bno085_device import FakeTime, TimedI2c

    rclpy.init()
    try:
        node = Node("bno085_log_test")
        log = lambda level, text: bno085_node.Bno085Node._log(node, level, text)  # noqa: E731
        for level in ("debug", "info", "warn", "error", "fatal", "warn", "info", "error"):
            log(level, f"level {level}")
        ft = FakeTime()
        bus = TimedI2c(ft)
        dev = Bno085Device(bus, orientation_id=0x08, interval_us=20000, clock=ft.clock,
                           sleep=ft.sleep, log=log, idle_poll=0.1, backoff_max_s=5.0)
        dev.run(keep_going=lambda: ft.t < 2 * 3600)  # warn, info, error (suppressed)
        assert dev.stats()["resets_suppressed"] > 0

        class Busy(TimedI2c):
            def open(self):
                super().open()
                raise (sh2.BusBusyError("held") if self.opens % 2
                       else sh2.LockPermissionError("lock file denied"))

        busy = Busy(ft)
        dev2 = Bno085Device(busy, orientation_id=0x08, interval_us=20000, clock=ft.clock,
                            sleep=lambda s: ft.sleep(40.0), log=log)
        dev2.run(keep_going=lambda: busy.opens < 4)
        assert dev2.stats()["bus_busy"] == 2 and dev2.stats()["lock_denied"] == 2
        node.destroy_node()
    finally:
        rclpy.try_shutdown()
