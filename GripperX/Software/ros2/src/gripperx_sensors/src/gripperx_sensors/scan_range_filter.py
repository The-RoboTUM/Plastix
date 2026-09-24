#!/usr/bin/env python3
"""Drop laser returns closer than a floor, because the robot sees itself.

WHY THIS EXISTS. The LD06 reports from 2 cm (range_min 0.0200) and the arm and
gripper sit permanently in the scan plane behind the sensor. Measured on the real
robot over 25 scans, standing still:

    sector      < 0.10 m   0.10-0.35 m   > 0.35 m
    140-150        284          0           14
    150-160        291          0            0
    200-210        307          0            0
    210-220        117          0          186
    220-230          0          0          302

Every self-return is below 0.10 m and NOTHING lies between 0.10 and 0.35 m, so a
0.10 m floor removes the robot's own structure and discards no real reading. The
costmaps already have obstacle_min_range/raytrace_min_range, but slam_toolbox has
no equivalent parameter -- it declares max_laser_range and nothing else -- so
without this node the map is built against the robot's own arm.

WHY NaN AND NOT inf, WHICH IS THE DECISION THAT MATTERS. In LaserScan, `inf`
means "the beam flew off into nothing", and a costmap ray-traces along it,
CLEARING every cell out to range_max. Using inf here would tell the stack that
the sector behind the robot is empty out to 12 m on the strength of a reading
that was actually the gripper. NaN means "no valid measurement" and is ignored
rather than believed. Behind the arm we do not know what is there, and NaN is the
honest encoding of that.

WHY A NODE AND NOT laser_filters. laser_filters is not installed on this laptop
and is not in .rosdeps_local; whether it is on the robot could not be checked
(the Pi is powered down, and the next real-robot slot is the last one). A node
with no dependency beyond rclpy and sensor_msgs cannot fail to be there.

FAILURE MODE, DELIBERATE. This node is wired between the driver and everything
else: the driver publishes /scan_raw, this republishes /scan. If it is not
running there is NO /scan at all, which every consumer reports immediately. The
alternative -- a separate /scan_filtered that only some consumers read -- fails
silently by leaving the others on unfiltered data, and a silent failure on the
last test day is worse than a loud one.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


class ScanRangeFilter(Node):
    def __init__(self) -> None:
        super().__init__("scan_range_filter")

        self.declare_parameter("input_topic", "/scan_raw")
        self.declare_parameter("output_topic", "/scan")
        # 0.10 m: above every measured self-return (max 0.072 m by CAD, 0.019 m
        # observed) and below every real reading seen in the same scans, the
        # nearest of which was 0.35 m. Not a round number picked for looks -- the
        # gap it sits in was measured and is empty.
        self.declare_parameter("min_range", 0.10)
        # Report how much is being dropped, once per this many scans. 0 disables.
        self.declare_parameter("report_every_n_scans", 100)

        self.min_range = float(self.get_parameter("min_range").value)
        if not math.isfinite(self.min_range) or self.min_range < 0.0:
            raise ValueError(f"min_range must be finite and >= 0, got {self.min_range}")
        in_topic = str(self.get_parameter("input_topic").value)
        out_topic = str(self.get_parameter("output_topic").value)
        if in_topic == out_topic:
            raise ValueError(
                f"input_topic and output_topic are both {in_topic!r}; this node would "
                "subscribe to its own output and feed back on itself"
            )
        self._report_every = int(self.get_parameter("report_every_n_scans").value)

        # The LD06 driver publishes with plain create_publisher(topic, 10), i.e.
        # RELIABLE / VOLATILE / KEEP_LAST 10. Subscribing RELIABLE matches it. A
        # BEST_EFFORT subscription would also receive from a RELIABLE publisher,
        # but the reverse does not hold, so publishing RELIABLE keeps every
        # existing consumer working exactly as it did on the raw topic.
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self._pub = self.create_publisher(LaserScan, out_topic, qos)
        self._sub = self.create_subscription(LaserScan, in_topic, self._on_scan, qos)

        self._scans = 0
        self._dropped_total = 0
        self.get_logger().info(
            f"{in_topic} -> {out_topic}, dropping returns below {self.min_range:.3f} m "
            f"(set to NaN, range_min raised)"
        )

    def _on_scan(self, msg: LaserScan) -> None:
        floor = max(self.min_range, msg.range_min)
        ranges = list(msg.ranges)
        dropped = 0
        for i, r in enumerate(ranges):
            # Only finite values below the floor are dropped. inf and NaN already
            # mean "no return" and are left exactly as the driver sent them.
            if math.isfinite(r) and r < floor:
                ranges[i] = math.nan
                dropped += 1

        out = LaserScan()
        # Everything not deliberately changed is copied verbatim. The stamp above
        # all: shifting it would break TF lookups for every consumer.
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_max = msg.range_max
        out.intensities = msg.intensities
        out.ranges = ranges
        # Raised so a consumer that honours range_min agrees with what the data
        # now contains. Both are needed: the field alone would leave the values
        # in place for consumers that ignore it, and the values alone would leave
        # the field claiming a reach the data no longer has.
        out.range_min = floor
        self._pub.publish(out)

        self._scans += 1
        self._dropped_total += dropped
        if self._report_every > 0 and self._scans % self._report_every == 0:
            self.get_logger().info(
                f"{self._scans} scans, {self._dropped_total} returns dropped "
                f"({self._dropped_total / self._scans:.1f} per scan)"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanRangeFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
