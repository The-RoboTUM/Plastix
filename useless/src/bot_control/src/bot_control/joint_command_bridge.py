from typing import Dict, List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class JointCommandBridge(Node):
    def __init__(self) -> None:
        super().__init__("joint_command_bridge")

        self.declare_parameter("command_topic", "/swerve_cmd_joint_states")
        self.declare_parameter("steering_command_topic", "/steering_position_controller/commands")
        self.declare_parameter("wheel_command_topic", "/wheel_velocity_controller/commands")
        self.declare_parameter(
            "steering_joint_names",
            ["f_left_steer", "f_right_steer", "b_leftsteer", "b_rightsteer"],
        )
        self.declare_parameter(
            "wheel_joint_names",
            ["f_leftwheel", "f_rightwheel", "b_leftwheel", "b_rightwheel"],
        )
        self.declare_parameter(
            "wheel_command_multipliers",
            [1.0, -1.0, 1.0, -1.0],
        )

        self.command_topic = str(self.get_parameter("command_topic").value)
        self.steering_command_topic = str(self.get_parameter("steering_command_topic").value)
        self.wheel_command_topic = str(self.get_parameter("wheel_command_topic").value)
        self.steering_joint_names: List[str] = list(self.get_parameter("steering_joint_names").value)
        self.wheel_joint_names: List[str] = list(self.get_parameter("wheel_joint_names").value)
        self.wheel_command_multipliers: List[float] = [
            float(value) for value in self.get_parameter("wheel_command_multipliers").value
        ]

        if len(self.wheel_joint_names) != len(self.wheel_command_multipliers):
            raise ValueError(
                "wheel_command_multipliers must have the same length as wheel_joint_names"
            )

        self.steering_publisher = self.create_publisher(Float64MultiArray, self.steering_command_topic, 10)
        self.wheel_publisher = self.create_publisher(Float64MultiArray, self.wheel_command_topic, 10)
        self.subscription = self.create_subscription(
            JointState,
            self.command_topic,
            self.command_callback,
            20,
        )

        self.get_logger().info(
            "Joint command bridge ready. %s -> [%s, %s]"
            % (self.command_topic, self.steering_command_topic, self.wheel_command_topic)
        )

    def command_callback(self, message: JointState) -> None:
        name_to_index: Dict[str, int] = {name: index for index, name in enumerate(message.name)}

        steering_msg = Float64MultiArray()
        wheel_msg = Float64MultiArray()

        steering_msg.data = []
        for joint_name in self.steering_joint_names:
            index = name_to_index.get(joint_name)
            if index is None or index >= len(message.position):
                self.get_logger().warning(f"Missing steering joint command for {joint_name}")
                return
            steering_msg.data.append(float(message.position[index]))

        wheel_msg.data = []
        for joint_name, multiplier in zip(self.wheel_joint_names, self.wheel_command_multipliers):
            index = name_to_index.get(joint_name)
            if index is None or index >= len(message.velocity):
                self.get_logger().warning(f"Missing wheel joint velocity command for {joint_name}")
                return
            wheel_msg.data.append(float(message.velocity[index]) * multiplier)

        self.steering_publisher.publish(steering_msg)
        self.wheel_publisher.publish(wheel_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JointCommandBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
