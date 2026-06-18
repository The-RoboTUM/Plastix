import os
import sys

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles, QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseArray, Pose


class DetectorNode(Node):
    """Subscribes to camera frames, runs YOLO detection + AprilTag/normalized
    localization + tracking, and publishes 2D trash positions.

    Topics published (both geometry_msgs/PoseArray, z = 0):
      * ``~/detections``  every processed frame: all current detection positions.
      * ``~/confirmed``   only when trash gets confirmed: the full set of
        confirmed (settled) trash positions. Latched (transient-local) so a
        subscriber that joins late still receives the latest confirmed map.

    Position units depend on the ``tags`` parameter: world coordinates when an
    AprilTag CSV is supplied, otherwise normalized image coordinates in [0, 1]
    with (0, 0) = bottom-left.
    """

    def __init__(self):
        super().__init__('detector_node')

        # Path to the detect-and-localize repo that holds the shared pipeline.
        default_repo = '/home/victor-tipkemper/projects/robotics/PlastiX/eve/Software/detect-and-localize'
        self.declare_parameter('detect_localize_path', default_repo)
        self.declare_parameter('model', 'data/models/indoor_11s.pt')
        self.declare_parameter('thresh', 0.6)
        self.declare_parameter('tags', '')  # empty -> normalized image coordinates
        self.declare_parameter('yolo_frameskip', 4)
        self.declare_parameter('max_lost', 100)
        self.declare_parameter('confirm_frames', 10)
        self.declare_parameter('dist_thresh', 0.05)
        self.declare_parameter('move_thresh', 0.04)
        self.declare_parameter('input_topic', 'camera/image_raw/compressed')
        self.declare_parameter('output_frame', 'map')

        repo_path = self.get_parameter('detect_localize_path').value
        self.output_frame = self.get_parameter('output_frame').value
        input_topic = self.get_parameter('input_topic').value

        # Make the shared pipeline importable, then build it.
        if repo_path not in sys.path:
            sys.path.insert(0, repo_path)
        try:
            from src.pipeline import DetectAndLocalize
        except ImportError as exc:
            self.get_logger().error(
                f"Could not import the pipeline from '{repo_path}'. Set the "
                f"'detect_localize_path' parameter to the detect-and-localize repo. ({exc})"
            )
            raise

        model = self._resolve(repo_path, self.get_parameter('model').value)
        tags = self.get_parameter('tags').value or None
        if tags:
            tags = self._resolve(repo_path, tags)

        self.pipeline = DetectAndLocalize(
            model=model,
            thresh=self.get_parameter('thresh').value,
            tags=tags,
            yolo_frameskip=self.get_parameter('yolo_frameskip').value,
            max_lost=self.get_parameter('max_lost').value,
            confirm_frames=self.get_parameter('confirm_frames').value,
            dist_thresh=self.get_parameter('dist_thresh').value,
            move_thresh=self.get_parameter('move_thresh').value,
        )

        # Live detections: reliable so standard tools/consumers receive them.
        self.detections_pub = self.create_publisher(PoseArray, '~/detections', 10)
        # Confirmed trash: latched so late joiners get the current map.
        latched_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.confirmed_pub = self.create_publisher(PoseArray, '~/confirmed', latched_qos)

        self.subscription = self.create_subscription(
            CompressedImage, input_topic, self.image_callback,
            QoSPresetProfiles.SENSOR_DATA.value,
        )

        self.get_logger().info(
            f"Detector ready. Subscribing '{input_topic}', model '{model}', "
            f"{'world coords (tags)' if tags else 'normalized image coords'}, "
            f"output frame '{self.output_frame}'."
        )

    @staticmethod
    def _resolve(repo_path, path):
        """Resolve a possibly-relative path against the repo root."""
        return path if os.path.isabs(path) else os.path.join(repo_path, path)

    def _to_pose_array(self, positions, stamp):
        msg = PoseArray()
        msg.header.stamp = stamp
        msg.header.frame_id = self.output_frame
        for pos in positions:
            pose = Pose()
            pose.position.x = float(pos[0])
            pose.position.y = float(pos[1])
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            msg.poses.append(pose)
        return msg

    def image_callback(self, msg):
        # Decode the JPEG CompressedImage directly with OpenCV. This avoids
        # cv_bridge, whose native extension is built against a different NumPy
        # ABI than the venv and crashes when the two are mixed.
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warn('Failed to decode frame')
            return

        result = self.pipeline.process(frame, draw=False)
        stamp = msg.header.stamp

        self.detections_pub.publish(
            self._to_pose_array(result['detections_world'], stamp)
        )

        if result['new_confirmed']:
            confirmed_positions = [t['pos'] for t in result['confirmed']]
            self.confirmed_pub.publish(self._to_pose_array(confirmed_positions, stamp))
            for t in result['new_confirmed']:
                pos = t['pos']
                self.get_logger().info(
                    f"Confirmed rubbish #{t['id']} at {self.pipeline.coord_label} "
                    f"({pos[0]:.3f}, {pos[1]:.3f})"
                )


def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
