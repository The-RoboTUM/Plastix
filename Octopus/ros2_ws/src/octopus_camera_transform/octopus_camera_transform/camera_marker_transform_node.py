#!/usr/bin/env python3

import json
import time

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseArray
from std_msgs.msg import String


class CameraMarkerTransformNode(Node):
    def __init__(self):
        super().__init__("camera_marker_transform_node")

        self.declare_parameter("image_topic", "/camera/image_raw/compressed")
        self.declare_parameter("detector_topic", "/detector_node/confirmed")
        self.declare_parameter("output_topic", "/octopus/detections_world")

        self.declare_parameter("field_width_m", 5.0)
        self.declare_parameter("field_height_m", 3.0)

        self.declare_parameter("marker_id_origin", 61)
        self.declare_parameter("marker_id_x", 65)
        self.declare_parameter("marker_id_xy", 57)
        self.declare_parameter("marker_id_y", 11)
        self.declare_parameter("marker_dictionary", "APRILTAG_36h11")

        self.declare_parameter("source_id", "camera_marker_transform")
        self.declare_parameter("default_class_name", "trash")
        self.declare_parameter("default_confidence", 1.0)
        self.declare_parameter("coverage_confidence", 0.8)

        self.declare_parameter("publish_coverage_on_image", True)
        self.declare_parameter("coverage_publish_period_sec", 1.0)
        self.declare_parameter("clamp_coverage_to_field", True)

        self.image_topic = self.get_parameter("image_topic").value
        self.detector_topic = self.get_parameter("detector_topic").value
        self.output_topic = self.get_parameter("output_topic").value

        self.field_width_m = float(self.get_parameter("field_width_m").value)
        self.field_height_m = float(self.get_parameter("field_height_m").value)

        self.marker_id_origin = int(self.get_parameter("marker_id_origin").value)
        self.marker_id_x = int(self.get_parameter("marker_id_x").value)
        self.marker_id_xy = int(self.get_parameter("marker_id_xy").value)
        self.marker_id_y = int(self.get_parameter("marker_id_y").value)
        self.marker_dictionary = self.get_parameter("marker_dictionary").value

        self.source_id = self.get_parameter("source_id").value
        self.default_class_name = self.get_parameter("default_class_name").value
        self.default_confidence = float(self.get_parameter("default_confidence").value)
        self.coverage_confidence = float(self.get_parameter("coverage_confidence").value)

        self.publish_coverage_on_image = bool(
            self.get_parameter("publish_coverage_on_image").value
        )
        self.coverage_publish_period_sec = float(
            self.get_parameter("coverage_publish_period_sec").value
        )
        self.clamp_coverage_to_field = bool(
            self.get_parameter("clamp_coverage_to_field").value
        )

        self.homography = None
        self.image_width = None
        self.image_height = None
        self.last_coverage_publish_time = 0.0
        self.last_marker_debug_time = 0.0

        self.publisher = self.create_publisher(
            String,
            self.output_topic,
            10,
        )

        self.image_sub = self.create_subscription(
            CompressedImage,
            self.image_topic,
            self.image_callback,
            QoSPresetProfiles.SENSOR_DATA.value,
        )

        self.detector_sub = self.create_subscription(
            PoseArray,
            self.detector_topic,
            self.detector_callback,
            10,
        )

        self.aruco_available = hasattr(cv2, "aruco")
        if self.aruco_available:
            self.aruco_dict = self.create_marker_dictionary(self.marker_dictionary)

            if hasattr(cv2.aruco, "ArucoDetector"):
                self.aruco_params = cv2.aruco.DetectorParameters()
                self.aruco_detector = cv2.aruco.ArucoDetector(
                    self.aruco_dict,
                    self.aruco_params,
                )
            else:
                self.aruco_params = cv2.aruco.DetectorParameters_create()
                self.aruco_detector = None
        else:
            self.get_logger().error(
                "cv2.aruco is not available. Install OpenCV with ArUco support."
            )

        self.get_logger().info("Camera marker transform node started")
        self.get_logger().info(f"Image topic: {self.image_topic}")
        self.get_logger().info(f"Detector topic: {self.detector_topic}")
        self.get_logger().info(f"Output topic: {self.output_topic}")
        self.get_logger().info(f"Marker dictionary: {self.marker_dictionary}")
        self.get_logger().info(
            "Expected marker IDs: "
            f"{self.marker_id_origin}=origin, "
            f"{self.marker_id_x}=x corner, "
            f"{self.marker_id_xy}=xy corner, "
            f"{self.marker_id_y}=y corner"
        )

    def create_marker_dictionary(self, dictionary_name):
        name = str(dictionary_name)

        aliases = {
            "tag36h11": "DICT_APRILTAG_36h11",
            "TAG36H11": "DICT_APRILTAG_36h11",
            "APRILTAG_36h11": "DICT_APRILTAG_36h11",
            "APRILTAG_36H11": "DICT_APRILTAG_36h11",
            "DICT_APRILTAG_36h11": "DICT_APRILTAG_36h11",
            "aruco_4x4_50": "DICT_4X4_50",
            "ARUCO_4X4_50": "DICT_4X4_50",
            "DICT_4X4_50": "DICT_4X4_50",
        }

        attr_name = aliases.get(name, name)

        if not hasattr(cv2.aruco, attr_name):
            available = sorted(
                dictionary for dictionary in dir(cv2.aruco)
                if dictionary.startswith("DICT_")
            )
            raise RuntimeError(
                f"OpenCV marker dictionary '{attr_name}' is not available. "
                f"Available dictionaries: {available}"
            )

        self.get_logger().info(f"Using marker dictionary: {attr_name}")
        return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, attr_name))

    def image_callback(self, msg: CompressedImage):
        if not self.aruco_available:
            return

        image = self.decode_compressed_image(msg)
        if image is None:
            return

        self.image_height, self.image_width = image.shape[:2]

        marker_centers = self.detect_marker_centers(image)

        now = time.time()
        if now - self.last_marker_debug_time >= 2.0:
            detected_ids = sorted(marker_centers.keys())
            self.get_logger().info(f"Detected marker IDs: {detected_ids}")
            self.last_marker_debug_time = now

        ok = self.update_homography(marker_centers)

        if not ok:
            return

        now = time.time()
        if (
            self.publish_coverage_on_image
            and now - self.last_coverage_publish_time >= self.coverage_publish_period_sec
        ):
            self.publish_octopus_message(
                detections=[],
                timestamp=self.header_stamp_to_float(msg),
                include_coverage=True,
            )
            self.last_coverage_publish_time = now

    def detector_callback(self, msg: PoseArray):
        if self.homography is None:
            self.get_logger().warn(
                "No valid homography yet. Cannot transform detector PoseArray."
            )
            return

        if self.image_width is None or self.image_height is None:
            self.get_logger().warn(
                "Image size unknown. Cannot transform detector PoseArray."
            )
            return

        timestamp = self.header_stamp_to_float(msg)

        detections = []
        for pose in msg.poses:
            u = float(pose.position.x)
            v = float(pose.position.y)

            if u < 0.0 or u > 1.0 or v < 0.0 or v > 1.0:
                self.get_logger().warn(
                    f"Skipping detector point outside normalized range: u={u}, v={v}"
                )
                continue

            pixel_x = u * float(self.image_width - 1)
            pixel_y = v * float(self.image_height - 1)

            map_point = self.image_to_map(pixel_x, pixel_y)
            if map_point is None:
                continue

            x, y = map_point

            if not self.point_inside_field(x, y):
                self.get_logger().warn(
                    f"Skipping transformed detection outside field: x={x:.3f}, y={y:.3f}"
                )
                continue

            detections.append(
                {
                    "class_name": self.default_class_name,
                    "x": x,
                    "y": y,
                    "confidence": self.default_confidence,
                }
            )

        if not detections:
            self.get_logger().warn(
                "Detector PoseArray received, but no valid detections were transformed."
            )
            return

        self.publish_octopus_message(
            detections=detections,
            timestamp=timestamp,
            include_coverage=True,
        )

        self.get_logger().info(
            f"Transformed and published {len(detections)} detector point(s)"
        )

    def decode_compressed_image(self, msg: CompressedImage):
        try:
            data = np.frombuffer(msg.data, dtype=np.uint8)
            image = cv2.imdecode(data, cv2.IMREAD_COLOR)
            if image is None:
                self.get_logger().warn("Could not decode compressed image.")
            return image
        except Exception as exc:
            self.get_logger().error(f"Image decode failed: {exc}")
            return None

    def detect_marker_centers(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        if self.aruco_detector is not None:
            corners, ids, _ = self.aruco_detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray,
                self.aruco_dict,
                parameters=self.aruco_params,
            )

        marker_centers = {}

        if ids is None:
            return marker_centers

        ids = ids.flatten()

        for marker_id, marker_corners in zip(ids, corners):
            pts = marker_corners.reshape(-1, 2)
            center = pts.mean(axis=0)
            marker_centers[int(marker_id)] = (float(center[0]), float(center[1]))

        return marker_centers

    def update_homography(self, marker_centers):
        required_ids = [
            self.marker_id_origin,
            self.marker_id_x,
            self.marker_id_xy,
            self.marker_id_y,
        ]

        missing = [marker_id for marker_id in required_ids if marker_id not in marker_centers]
        if missing:
            return False

        image_points = np.array(
            [
                marker_centers[self.marker_id_origin],
                marker_centers[self.marker_id_x],
                marker_centers[self.marker_id_xy],
                marker_centers[self.marker_id_y],
            ],
            dtype=np.float32,
        )

        map_points = np.array(
            [
                [0.0, 0.0],
                [self.field_width_m, 0.0],
                [self.field_width_m, self.field_height_m],
                [0.0, self.field_height_m],
            ],
            dtype=np.float32,
        )

        homography, _ = cv2.findHomography(image_points, map_points)

        if homography is None:
            self.get_logger().warn("Could not compute homography.")
            return False

        self.homography = homography

        self.get_logger().info(
            "Updated image-to-map homography from field markers."
        )
        return True

    def image_to_map(self, pixel_x, pixel_y):
        if self.homography is None:
            return None

        pts = np.array([[[pixel_x, pixel_y]]], dtype=np.float32)
        transformed = cv2.perspectiveTransform(pts, self.homography)
        x = float(transformed[0][0][0])
        y = float(transformed[0][0][1])

        return x, y

    def get_coverage_polygon(self):
        if self.homography is None or self.image_width is None or self.image_height is None:
            return None

        image_corners = [
            (0.0, 0.0),
            (float(self.image_width - 1), 0.0),
            (float(self.image_width - 1), float(self.image_height - 1)),
            (0.0, float(self.image_height - 1)),
        ]

        polygon = []
        for px, py in image_corners:
            map_point = self.image_to_map(px, py)
            if map_point is None:
                return None

            x, y = map_point

            if self.clamp_coverage_to_field:
                x = max(0.0, min(self.field_width_m, x))
                y = max(0.0, min(self.field_height_m, y))

            polygon.append([x, y])

        return polygon

    def publish_octopus_message(self, detections, timestamp, include_coverage):
        payload = {
            "source_id": self.source_id,
            "frame_id": "map",
            "timestamp": timestamp,
            "detections": detections,
        }

        if include_coverage:
            coverage_polygon = self.get_coverage_polygon()
            if coverage_polygon is not None:
                payload["coverage_confidence"] = self.coverage_confidence
                payload["coverage_polygon"] = coverage_polygon

        msg = String()
        msg.data = json.dumps(payload)
        self.publisher.publish(msg)

    def point_inside_field(self, x, y):
        return (
            x >= 0.0
            and x <= self.field_width_m
            and y >= 0.0
            and y <= self.field_height_m
        )

    @staticmethod
    def header_stamp_to_float(msg):
        sec = int(msg.header.stamp.sec)
        nanosec = int(msg.header.stamp.nanosec)

        if sec == 0 and nanosec == 0:
            return time.time()

        return float(sec) + float(nanosec) * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = CameraMarkerTransformNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
