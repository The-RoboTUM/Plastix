"""Software interface for the switchable LD06 power branch (HWR-23).

Exposes the LiDAR power relay on a Raspberry Pi header GPIO as ROS services:

  * ``/lidar/set_power``   ``std_srvs/srv/SetBool``   ``data: true`` = LiDAR powered
  * ``/lidar/power_cycle`` ``std_srvs/srv/Trigger``   off -> wait -> on
  * ``/lidar/power_state`` ``std_msgs/Bool`` (latched) — observability only

Scope boundary: this node switches on request ONLY. It deliberately contains no
/scan watchdog and no automatic power cycling — that mechanism is NFR-9 / OP-10
and is out of scope here.

FAIL-SAFE POLARITY (measured on the machine, INVERTED vs. WIRING_PLAN.md):
the relay is wired normally-closed. Coil de-energized (GPIO low / released) =
LiDAR ON; coil energized (GPIO driven high) = LiDAR OFF. The BSS138 gate
pull-down (R3, 10k) holds the gate low whenever the line is unclaimed or
high-impedance, so an absent or crashed node means LiDAR ON. That is the
fail-safe state and every code path here preserves it:
  * startup requests the line as an output already carrying the ON value,
  * clean shutdown drives ON and only then releases the line.
"""

from __future__ import annotations

import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger


class GpiodLineOutput:
    """One GPIO output line, driven through the libgpiod **v1** Python bindings.

    All kernel/GPIO specifics of this node live in this class so the backend can
    be swapped (e.g. for a dependency-free ctypes wrapper around the GPIO v2
    character-device ioctls) without touching the node logic. The v1 API is what
    Ubuntu 24.04 ships (python3-libgpiod 1.6.3); it is NOT the v2 Python API —
    ``gpiod.Chip(path, OPEN_BY_PATH)`` + ``line.request(type=...)``, not
    ``gpiod.request_lines()``.
    """

    def __init__(self, chip_path: str, line_offset: int, consumer: str, initial_value: int):
        import gpiod  # imported here so the dependency stays inside the backend

        self._gpiod = gpiod
        self._chip = gpiod.Chip(chip_path, gpiod.Chip.OPEN_BY_PATH)
        self._line = self._chip.get_line(line_offset)
        # default_vals makes the request itself carry the safe level: the line is
        # never briefly driven to the opposite state between request and first
        # set_value().
        self._line.request(
            consumer=consumer,
            type=gpiod.LINE_REQ_DIR_OUT,
            default_vals=[int(initial_value)],
        )

    def set_value(self, value: int) -> None:
        self._line.set_value(int(value))

    def get_value(self) -> int:
        return int(self._line.get_value())

    def release(self) -> None:
        if self._line is not None:
            self._line.release()
            self._line = None
        if self._chip is not None:
            self._chip.close()
            self._chip = None


class LidarPowerNode(Node):
    # Class-level default so destroy_node() is safe even if __init__ aborted
    # before the line was claimed.
    _gpio = None

    def __init__(self):
        super().__init__("lidar_power_node")

        self.declare_parameter("gpio_chip_path", "/dev/gpiochip4")
        self.declare_parameter("gpio_line_offset", 23)
        self.declare_parameter("gpio_consumer_name", "gripperx_lidar_power")
        self.declare_parameter("invert_logic", True)
        self.declare_parameter("power_cycle_off_duration_sec", 2.0)

        self._chip_path = self.get_parameter("gpio_chip_path").value
        self._line_offset = int(self.get_parameter("gpio_line_offset").value)
        self._consumer = self.get_parameter("gpio_consumer_name").value
        self._invert = bool(self.get_parameter("invert_logic").value)
        self._off_duration = float(self.get_parameter("power_cycle_off_duration_sec").value)

        if self._off_duration < 0.0:
            raise ValueError("power_cycle_off_duration_sec must not be negative")

        # Latched so a late subscriber (diagnostics, operator shell) learns the
        # current power state without having to wait for a change.
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._state_pub = self.create_publisher(Bool, "/lidar/power_state", state_qos)

        # Startup state is ON and is deliberately NOT a parameter: HWR-23 fixes
        # "default ON at boot", and a config able to boot the LiDAR off would be
        # a way to lose the fail-safe by editing yaml.
        self._powered = True
        self._gpio = GpiodLineOutput(
            chip_path=self._chip_path,
            line_offset=self._line_offset,
            consumer=self._consumer,
            initial_value=self._line_value_for(self._powered),
        )
        self._publish_state()
        self.get_logger().info(
            f"LiDAR power branch claimed: {self._chip_path} line {self._line_offset} "
            f"(invert_logic={self._invert}) -> LiDAR ON, line value "
            f"{self._read_line_value()}"
        )

        # One (implicit) mutually exclusive callback group: set_power cannot
        # interleave with the blocking off-phase of power_cycle.
        self._set_power_srv = self.create_service(SetBool, "/lidar/set_power", self._on_set_power)
        self._power_cycle_srv = self.create_service(Trigger, "/lidar/power_cycle", self._on_power_cycle)

    # ── logical state <-> line level ─────────────────────────────────────────
    def _line_value_for(self, powered: bool) -> int:
        """Line level that produces the requested logical power state."""
        if self._invert:
            return 0 if powered else 1
        return 1 if powered else 0

    def _read_line_value(self) -> int:
        try:
            return self._gpio.get_value()
        except Exception as exc:  # read-back is diagnostic only, never fatal
            self.get_logger().warn(f"GPIO read-back failed: {exc}")
            return -1

    def _publish_state(self) -> None:
        msg = Bool()
        msg.data = self._powered
        self._state_pub.publish(msg)

    def _apply(self, powered: bool) -> None:
        value = self._line_value_for(powered)
        self._gpio.set_value(value)
        self._powered = powered
        self._publish_state()
        self.get_logger().info(
            f"LiDAR power {'ON' if powered else 'OFF'} "
            f"(line {self._line_offset} driven {value}, read-back {self._read_line_value()})"
        )

    # ── services ─────────────────────────────────────────────────────────────
    def _on_set_power(self, request, response):
        try:
            self._apply(bool(request.data))
        except Exception as exc:
            response.success = False
            response.message = f"GPIO write failed: {exc}"
            self.get_logger().error(response.message)
            return response
        response.success = True
        response.message = f"LiDAR power {'ON' if self._powered else 'OFF'}"
        return response

    def _on_power_cycle(self, request, response):
        del request
        try:
            self._apply(False)
            # Blocking is intentional and safe: the callback group serializes, so
            # nothing can flip the line mid-cycle, and the off-phase must be a
            # real wall-clock interval for the LD06 to lose its supply rail.
            time.sleep(self._off_duration)
            self._apply(True)
        except Exception as exc:
            response.success = False
            response.message = f"power cycle failed: {exc}"
            self.get_logger().error(response.message)
            return response
        response.success = True
        response.message = f"LiDAR power cycled (off for {self._off_duration:.2f} s)"
        return response

    # ── shutdown ─────────────────────────────────────────────────────────────
    def destroy_node(self):
        # Drive ON before releasing: releasing alone would already yield ON via
        # the gate pull-down, but driving first keeps the state defined for the
        # whole shutdown and makes the log say what the hardware does.
        try:
            if self._gpio is not None:
                self._gpio.set_value(self._line_value_for(True))
                self.get_logger().info("Shutdown: LiDAR power driven ON, releasing GPIO line")
                self._gpio.release()
                self._gpio = None
        except Exception as exc:
            self.get_logger().error(f"Shutdown GPIO handling failed: {exc}")
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LidarPowerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # rclpy.init() installs handlers for SIGINT *and* SIGTERM
        # (SignalHandlerOptions.ALL for the default context). Both shut the
        # context down from under the executor, and under Jazzy rclpy.spin()
        # then raises ExternalShutdownException -- NOT KeyboardInterrupt. Not
        # catching it made every SIGTERM end in a traceback and exit code 1.
        # The GPIO fail-safe below survived that only because it sits in
        # `finally`; the node still looked like it had crashed, and the non-zero
        # exit propagates into the launch parent's exit status, which makes a
        # clean `systemctl stop` look like a failed unit (SR-12).
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
