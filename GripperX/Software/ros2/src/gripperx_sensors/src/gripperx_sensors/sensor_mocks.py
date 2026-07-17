"""Publish /scan, /imu/data, and /gps/fix for bench testing without hardware."""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan, NavSatFix


class SensorMocks(Node):
    def __init__(self) -> None:
        super().__init__("sensor_mocks")

        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("gps_topic", "/gps/fix")
        self.declare_parameter("scan_frame_id", "lidar_link")
        self.declare_parameter("imu_frame_id", "imu_link")
        self.declare_parameter("gps_frame_id", "gps_link")
        self.declare_parameter("scan_rate_hz", 10.0)
        self.declare_parameter("imu_rate_hz", 50.0)
        self.declare_parameter("gps_rate_hz", 5.0)
        self.declare_parameter("scan_sample_count", 720)
        self.declare_parameter("scan_range_max", 12.0)

        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.imu_topic = str(self.get_parameter("imu_topic").value)
        self.gps_topic = str(self.get_parameter("gps_topic").value)
        self.scan_frame_id = str(self.get_parameter("scan_frame_id").value)
        self.imu_frame_id = str(self.get_parameter("imu_frame_id").value)
        self.gps_frame_id = str(self.get_parameter("gps_frame_id").value)
        self.scan_sample_count = int(self.get_parameter("scan_sample_count").value)
        self.scan_range_max = float(self.get_parameter("scan_range_max").value)

        scan_rate_hz = float(self.get_parameter("scan_rate_hz").value)
        imu_rate_hz = float(self.get_parameter("imu_rate_hz").value)
        gps_rate_hz = float(self.get_parameter("gps_rate_hz").value)

        self.scan_pub = self.create_publisher(LaserScan, self.scan_topic, 10)
        self.imu_pub = self.create_publisher(Imu, self.imu_topic, 10)
        self.gps_pub = self.create_publisher(NavSatFix, self.gps_topic, 10)

        self.create_timer(1.0 / scan_rate_hz, self.publish_scan)
        self.create_timer(1.0 / imu_rate_hz, self.publish_imu)
        self.create_timer(1.0 / gps_rate_hz, self.publish_gps)

        self.get_logger().info(
            "Sensor mocks ready. scan=%s imu=%s gps=%s"
            % (self.scan_topic, self.imu_topic, self.gps_topic)
        )

    def publish_scan(self) -> None:
        message = LaserScan()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.scan_frame_id
        message.angle_min = -math.pi
        message.angle_max = math.pi
        message.angle_increment = (message.angle_max - message.angle_min) / max(
            self.scan_sample_count - 1, 1
        )
        message.time_increment = 0.0
        message.scan_time = 1.0 / max(float(self.get_parameter("scan_rate_hz").value), 1.0)
        message.range_min = 0.1
        message.range_max = self.scan_range_max
        message.ranges = [self.scan_range_max] * self.scan_sample_count
        message.intensities = [1.0] * self.scan_sample_count
        self.scan_pub.publish(message)

    def publish_imu(self) -> None:
        message = Imu()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.imu_frame_id
        message.orientation.w = 1.0
        message.orientation_covariance[0] = 0.02
        message.orientation_covariance[4] = 0.02
        message.orientation_covariance[8] = 0.04
        message.angular_velocity_covariance[8] = 0.002
        message.linear_acceleration.z = 9.81
        message.linear_acceleration_covariance[0] = 0.04
        message.linear_acceleration_covariance[4] = 0.04
        message.linear_acceleration_covariance[8] = 0.06
        self.imu_pub.publish(message)

    def publish_gps(self) -> None:
        message = NavSatFix()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.gps_frame_id
        message.status.status = 0
        message.latitude = -12.0464
        message.longitude = -77.0428
        message.altitude = 150.0
        message.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        message.position_covariance[0] = 1.0
        message.position_covariance[4] = 1.0
        message.position_covariance[8] = 4.0
        self.gps_pub.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SensorMocks()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
