#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float64, Float32MultiArray
import cv2
import numpy as np
import sys
import os
    
DAL_SRC = os.path.expanduser(
    '~/PlastiX/eve/Software/detect-and-localize/src'
)
sys.path.insert(0, DAL_SRC)

from yolo_detector import YOLODetector
from tracking import GarbageTracker


class DetectionNode(Node):
    def __init__(self):
        super().__init__('detection_node')

        self.declare_parameter('model_path',
            os.path.expanduser('~/PlastiX/eve/Software/detect-and-localize/data/models/thousand_11s.pt'))
        self.declare_parameter('conf_thresh', 0.6)
        self.declare_parameter('yolo_frameskip', 2)
        self.declare_parameter('use_compressed', True)

        model_path     = self.get_parameter('model_path').value
        conf_thresh    = self.get_parameter('conf_thresh').value
        self.frameskip = self.get_parameter('yolo_frameskip').value
        use_compressed = self.get_parameter('use_compressed').value

        self.get_logger().info(f'Loading model from {model_path}')
        self.yolo    = YOLODetector(model_path, conf_thresh)
        self.tracker = GarbageTracker(max_lost=100, confirm_frames=10, dist_thresh=0.2)

        self.frame_n   = 0
        self.last_dets = []
        self.altitude  = 30.0

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        if use_compressed:
            self.create_subscription(
                CompressedImage, 'camera/image_raw/compressed',
                self.compressed_cb, qos)
            self.get_logger().info('Subscribed to camera/image_raw/compressed')
        else:
            self.create_subscription(
                Image, 'camera/image_raw',
                self.raw_cb, qos)
            self.get_logger().info('Subscribed to camera/image_raw')

        self.create_subscription(Float64, '/Height_EVE', self.height_cb, 10)

        self.pub = self.create_publisher(Float32MultiArray, '/trash_coords', 10)
        self.get_logger().info('Detection node ready')

    def height_cb(self, msg):
        self.altitude = msg.data

    def compressed_cb(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame  = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is not None:
            self._process(frame)

    def raw_cb(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame  = np_arr.reshape((msg.height, msg.width, 3))
        self._process(frame)

    def _process(self, frame):
        h, w = frame.shape[:2]

        if self.frameskip == 0 or self.frame_n % (self.frameskip + 1) == 0:
            self.last_dets, _ = self.yolo.detect(frame, draw=False)
        self.frame_n += 1

        detections_world = []
        for det in self.last_dets:
            if det['class'] != 'rubbish':
                continue
            xmin, ymin, xmax, ymax = det['bbox']
            cx = (xmin + xmax) / 2
            cy = (ymin + ymax) / 2
            nx = cx / w
            ny = 1.0 - cy / h
            detections_world.append(np.array([nx, ny]))

        confirmed, new_conf = self.tracker.update(detections_world)

        for t in new_conf:
            pos = t['pos']
            self.get_logger().info(
                f"Confirmed trash #{t['id']} at ({pos[0]:.3f}, {pos[1]:.3f})"
            )

        if confirmed:
            flat = [float(v) for t in confirmed for v in t['pos']]
            out  = Float32MultiArray()
            out.data = flat
            self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
