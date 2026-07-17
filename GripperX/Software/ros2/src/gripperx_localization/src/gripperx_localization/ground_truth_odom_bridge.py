import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class GroundTruthOdomBridge(Node):
    """DT-4/DT-7 transitional bridge for the digital twin.

    Reason: `gripperx_localization/launch/localization.launch.py` starts an
    `ekf_filter_node` from the ROS system package `robot_localization` in
    every configuration (enable_laser_odometry x enable_gps). That package
    isn't installed on this laptop (`ros-jazzy-robot-localization` is
    missing, and installing it needs root/`sudo` password, which the agent
    doesn't have) -> the EKF chain is blocked.

    DT-7 (status: resolved) mandates using the perfect ground-truth odometry
    (`/ground_truth/odom`, noise-free from Gazebo) for M1-M3 anyway, to test
    Nav2/SLAM in isolation from odometry-fidelity questions. This node
    implements exactly that: it passes `/ground_truth/odom` through 1:1 as
    `odom_frame` ("odom" instead of "ground_truth") + TF
    `odom->base_footprint`, so that `slam_toolbox` (and later Nav2) get a
    continuous TF chain without needing `robot_localization`.

    Once `ros-jazzy-robot-localization` is installed, M2/M3 should be
    switched over to the real Stack-B EKF chain (`localization.launch.py`);
    this node is deliberately not a replacement for EKF fusion (wheel odom +
    laser_scan_matcher + IMU), only the ground-truth special case from DT-7.
    """

    def __init__(self) -> None:
        super().__init__("ground_truth_odom_bridge")

        self.declare_parameter("input_topic", "/ground_truth/odom")
        self.declare_parameter("output_topic", "/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("publish_tf", True)

        input_topic = self.get_parameter("input_topic").value
        self._output_topic = self.get_parameter("output_topic").value
        self._odom_frame = self.get_parameter("odom_frame").value
        self._base_frame = self.get_parameter("base_frame").value
        self._publish_tf = self.get_parameter("publish_tf").value

        self._tf_broadcaster = TransformBroadcaster(self)
        self._odom_pub = self.create_publisher(Odometry, self._output_topic, 10)
        self.create_subscription(Odometry, input_topic, self._on_ground_truth, 20)

    def _on_ground_truth(self, msg: Odometry) -> None:
        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self._odom_frame
        out.child_frame_id = self._base_frame
        out.pose = msg.pose
        out.twist = msg.twist
        self._odom_pub.publish(out)

        if not self._publish_tf:
            return

        tf = TransformStamped()
        tf.header.stamp = msg.header.stamp
        tf.header.frame_id = self._odom_frame
        tf.child_frame_id = self._base_frame
        tf.transform.translation.x = msg.pose.pose.position.x
        tf.transform.translation.y = msg.pose.pose.position.y
        tf.transform.translation.z = msg.pose.pose.position.z
        tf.transform.rotation = msg.pose.pose.orientation
        self._tf_broadcaster.sendTransform(tf)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GroundTruthOdomBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
