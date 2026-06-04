import csv
from copy import deepcopy
from datetime import datetime
from pathlib import Path
from typing import Optional

import rclpy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState


def yaw_from_quaternion(message) -> float:
    siny_cosp = 2.0 * ((message.w * message.z) + (message.x * message.y))
    cosy_cosp = 1.0 - (2.0 * ((message.y * message.y) + (message.z * message.z)))
    return math.atan2(siny_cosp, cosy_cosp)


class DebugLogNode(Node):
    def __init__(self) -> None:
        super().__init__("debug_log_node")

        self.declare_parameter("ground_truth_topic", "/ground_truth/odom")
        self.declare_parameter("wheel_odom_topic", "/wheel/odom")
        self.declare_parameter("laser_odom_topic", "/laser/odom")
        self.declare_parameter("filtered_odom_topic", "/odometry/filtered")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("joint_command_topic", "/swerve_cmd_joint_states")
        self.declare_parameter("log_period", 0.1)
        self.declare_parameter("output_directory", "~/ros2_ws/ITQ_robot/debug_logs")

        self.ground_truth_topic = str(self.get_parameter("ground_truth_topic").value)
        self.wheel_odom_topic = str(self.get_parameter("wheel_odom_topic").value)
        self.laser_odom_topic = str(self.get_parameter("laser_odom_topic").value)
        self.filtered_odom_topic = str(self.get_parameter("filtered_odom_topic").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.joint_command_topic = str(self.get_parameter("joint_command_topic").value)
        self.log_period = float(self.get_parameter("log_period").value)
        self.output_directory = Path(str(self.get_parameter("output_directory").value)).expanduser()
        self.output_directory.mkdir(parents=True, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = self.output_directory / f"debug_log_{timestamp}.csv"
        self.csv_file = self.csv_path.open("w", newline="", encoding="ascii")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(
            [
                "time_s",
                "gt_x",
                "gt_y",
                "gt_yaw",
                "wheel_x",
                "wheel_y",
                "wheel_yaw",
                "laser_x",
                "laser_y",
                "laser_yaw",
                "ekf_x",
                "ekf_y",
                "ekf_yaw",
                "cmd_vx",
                "cmd_vy",
                "cmd_wz",
                "front_left_steer",
                "front_right_steer",
                "rear_left_steer",
                "rear_right_steer",
                "front_left_wheel_vel",
                "front_right_wheel_vel",
                "rear_left_wheel_vel",
                "rear_right_wheel_vel",
            ]
        )

        self.latest_ground_truth: Optional[Odometry] = None
        self.latest_wheel_odom: Optional[Odometry] = None
        self.latest_laser_odom: Optional[Odometry] = None
        self.latest_filtered_odom: Optional[Odometry] = None
        self.latest_cmd_vel: Optional[Twist] = None
        self.latest_joint_command: Optional[JointState] = None

        self.create_subscription(Odometry, self.ground_truth_topic, self.ground_truth_callback, 20)
        self.create_subscription(Odometry, self.wheel_odom_topic, self.wheel_odom_callback, 20)
        self.create_subscription(Odometry, self.laser_odom_topic, self.laser_odom_callback, 20)
        self.create_subscription(Odometry, self.filtered_odom_topic, self.filtered_odom_callback, 20)
        self.create_subscription(Twist, self.cmd_vel_topic, self.cmd_vel_callback, 20)
        self.create_subscription(JointState, self.joint_command_topic, self.joint_command_callback, 20)
        self.timer = self.create_timer(self.log_period, self.log_sample)

        self.get_logger().info(f"Debug logger writing to {self.csv_path}")

    def ground_truth_callback(self, message: Odometry) -> None:
        self.latest_ground_truth = deepcopy(message)

    def wheel_odom_callback(self, message: Odometry) -> None:
        self.latest_wheel_odom = deepcopy(message)

    def laser_odom_callback(self, message: Odometry) -> None:
        self.latest_laser_odom = deepcopy(message)

    def filtered_odom_callback(self, message: Odometry) -> None:
        self.latest_filtered_odom = deepcopy(message)

    def cmd_vel_callback(self, message: Twist) -> None:
        self.latest_cmd_vel = deepcopy(message)

    def joint_command_callback(self, message: JointState) -> None:
        self.latest_joint_command = deepcopy(message)

    def log_sample(self) -> None:
        if self.latest_ground_truth is None or self.latest_filtered_odom is None:
            return

        stamp = self.latest_filtered_odom.header.stamp
        time_s = float(stamp.sec) + (float(stamp.nanosec) * 1e-9)

        gt_pose = self.latest_ground_truth.pose.pose
        ekf_pose = self.latest_filtered_odom.pose.pose
        wheel_pose = self.latest_wheel_odom.pose.pose if self.latest_wheel_odom is not None else None
        laser_pose = self.latest_laser_odom.pose.pose if self.latest_laser_odom is not None else None
        cmd_vel = self.latest_cmd_vel if self.latest_cmd_vel is not None else None

        front_left_steer = ""
        front_right_steer = ""
        rear_left_steer = ""
        rear_right_steer = ""
        front_left_wheel_vel = ""
        front_right_wheel_vel = ""
        rear_left_wheel_vel = ""
        rear_right_wheel_vel = ""

        if self.latest_joint_command is not None:
            joint_indices = {name: index for index, name in enumerate(self.latest_joint_command.name)}
            if "f_left_steer" in joint_indices:
                front_left_steer = self.latest_joint_command.position[joint_indices["f_left_steer"]]
            if "f_right_steer" in joint_indices:
                front_right_steer = self.latest_joint_command.position[joint_indices["f_right_steer"]]
            if "b_leftsteer" in joint_indices:
                rear_left_steer = self.latest_joint_command.position[joint_indices["b_leftsteer"]]
            if "b_rightsteer" in joint_indices:
                rear_right_steer = self.latest_joint_command.position[joint_indices["b_rightsteer"]]
            if "f_leftwheel" in joint_indices:
                front_left_wheel_vel = self.latest_joint_command.velocity[joint_indices["f_leftwheel"]]
            if "f_rightwheel" in joint_indices:
                front_right_wheel_vel = self.latest_joint_command.velocity[joint_indices["f_rightwheel"]]
            if "b_leftwheel" in joint_indices:
                rear_left_wheel_vel = self.latest_joint_command.velocity[joint_indices["b_leftwheel"]]
            if "b_rightwheel" in joint_indices:
                rear_right_wheel_vel = self.latest_joint_command.velocity[joint_indices["b_rightwheel"]]

        self.csv_writer.writerow(
            [
                time_s,
                gt_pose.position.x,
                gt_pose.position.y,
                yaw_from_quaternion(gt_pose.orientation),
                wheel_pose.position.x if wheel_pose is not None else "",
                wheel_pose.position.y if wheel_pose is not None else "",
                yaw_from_quaternion(wheel_pose.orientation) if wheel_pose is not None else "",
                laser_pose.position.x if laser_pose is not None else "",
                laser_pose.position.y if laser_pose is not None else "",
                yaw_from_quaternion(laser_pose.orientation) if laser_pose is not None else "",
                ekf_pose.position.x,
                ekf_pose.position.y,
                yaw_from_quaternion(ekf_pose.orientation),
                cmd_vel.linear.x if cmd_vel is not None else "",
                cmd_vel.linear.y if cmd_vel is not None else "",
                cmd_vel.angular.z if cmd_vel is not None else "",
                front_left_steer,
                front_right_steer,
                rear_left_steer,
                rear_right_steer,
                front_left_wheel_vel,
                front_right_wheel_vel,
                rear_left_wheel_vel,
                rear_right_wheel_vel,
            ]
        )
        self.csv_file.flush()

    def destroy_node(self) -> bool:
        try:
            self.csv_file.close()
        finally:
            return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DebugLogNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
