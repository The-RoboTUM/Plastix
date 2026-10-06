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

        self.declare_parameter("status_topic", "/octopus/camera_transform/status")
        self.declare_parameter(
            "debug_image_topic",
            "/octopus/camera_transform/debug_image/compressed",
        )

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

        self.declare_parameter("status_publish_period_sec", 1.0)
        self.declare_parameter("log_period_sec", 5.0)
        self.declare_parameter("homography_stale_warn_sec", 2.0)
        self.declare_parameter("homography_stale_drop_sec", 5.0)
        self.declare_parameter("publish_debug_image", False)
        self.declare_parameter("debug_image_jpeg_quality", 80)

        self.image_topic = self.get_parameter("image_topic").value
        self.detector_topic = self.get_parameter("detector_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.status_topic = self.get_parameter("status_topic").value
        self.debug_image_topic = self.get_parameter("debug_image_topic").value

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

        self.status_publish_period_sec = float(
            self.get_parameter("status_publish_period_sec").value
        )
        self.log_period_sec = float(self.get_parameter("log_period_sec").value)
        self.homography_stale_warn_sec = float(
            self.get_parameter("homography_stale_warn_sec").value
        )
        self.homography_stale_drop_sec = float(
            self.get_parameter("homography_stale_drop_sec").value
        )
        self.publish_debug_image = bool(
            self.get_parameter("publish_debug_image").value
        )
        self.debug_image_jpeg_quality = int(
            self.get_parameter("debug_image_jpeg_quality").value
        )

        self.required_marker_ids = [
            self.marker_id_origin,
            self.marker_id_x,
            self.marker_id_xy,
            self.marker_id_y,
        ]

        self.homography = None
        self.inverse_homography = None
        self.image_width = None
        self.image_height = None

        self.last_homography_update_time = None
        self.last_coverage_publish_time = 0.0
        self.last_log_time = 0.0
        self.last_warn_times = {}

        self.detected_marker_ids = []
        self.missing_marker_ids = self.required_marker_ids.copy()
        self.last_input_detection_count = 0
        self.last_transformed_detection_count = 0

        self.publisher = self.create_publisher(String, self.output_topic, 10)
        self.status_publisher = self.create_publisher(String, self.status_topic, 10)
        self.debug_image_publisher = self.create_publisher(
            CompressedImage,
            self.debug_image_topic,
            2,
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

        self.status_timer = self.create_timer(
            self.status_publish_period_sec,
            self.publish_status,
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
        self.get_logger().info(f"Status topic: {self.status_topic}")
        self.get_logger().info(f"Debug image topic: {self.debug_image_topic}")
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

        marker_centers, marker_corners = self.detect_markers(image)

        self.detected_marker_ids = sorted(marker_centers.keys())
        self.missing_marker_ids = [
            marker_id
            for marker_id in self.required_marker_ids
            if marker_id not in marker_centers
        ]

        homography_updated = self.update_homography(marker_centers)

        self.log_transform_state_throttled(homography_updated)

        if self.publish_debug_image:
            self.publish_debug_marker_image(msg, image, marker_corners)

        if not self.homography_is_usable():
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
        self.last_input_detection_count = len(msg.poses)
        self.last_transformed_detection_count = 0

        if not self.homography_is_usable():
            self.warn_throttled(
                "homography_not_usable",
                "No fresh homography. Cannot transform detector PoseArray.",
                period_sec=2.0,
            )
            return

        if self.image_width is None or self.image_height is None:
            self.warn_throttled(
                "image_size_unknown",
                "Image size unknown. Cannot transform detector PoseArray.",
                period_sec=2.0,
            )
            return

        timestamp = self.header_stamp_to_float(msg)

        detections = []
        for pose in msg.poses:
            u = float(pose.position.x)
            v = float(pose.position.y)

            if u < 0.0 or u > 1.0 or v < 0.0 or v > 1.0:
                self.warn_throttled(
                    "normalized_range",
                    f"Skipping detector point outside normalized range: u={u}, v={v}",
                    period_sec=2.0,
                )
                continue

            pixel_x = u * float(self.image_width - 1)
            pixel_y = v * float(self.image_height - 1)

            map_point = self.image_to_map(pixel_x, pixel_y)
            if map_point is None:
                continue

            x, y = map_point

            if not self.point_inside_field(x, y):
                self.warn_throttled(
                    "outside_field",
                    f"Skipping transformed detection outside field: x={x:.3f}, y={y:.3f}",
                    period_sec=2.0,
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

        self.last_transformed_detection_count = len(detections)

        if not detections:
            self.warn_throttled(
                "no_valid_detections",
                "Detector PoseArray received, but no valid detections were transformed.",
                period_sec=2.0,
            )
            self.publish_status()
            return

        self.publish_octopus_message(
            detections=detections,
            timestamp=timestamp,
            include_coverage=True,
        )

        self.get_logger().info(
            f"Transformed and published {len(detections)} detector point(s)"
        )
        self.publish_status()

    def decode_compressed_image(self, msg: CompressedImage):
        try:
            data = np.frombuffer(msg.data, dtype=np.uint8)
            image = cv2.imdecode(data, cv2.IMREAD_COLOR)
            if image is None:
                self.warn_throttled(
                    "decode_failed",
                    "Could not decode compressed image.",
                    period_sec=2.0,
                )
            return image
        except Exception as exc:
            self.get_logger().error(f"Image decode failed: {exc}")
            return None

    def detect_markers(self, image):
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
        marker_corners = {}

        if ids is None:
            return marker_centers, marker_corners

        for marker_id, marker_corners_array in zip(ids.flatten(), corners):
            marker_id = int(marker_id)
            pts = marker_corners_array.reshape(-1, 2).astype(np.float32)
            center = pts.mean(axis=0)
            marker_centers[marker_id] = center
            marker_corners[marker_id] = pts

        return marker_centers, marker_corners

    def update_homography(self, marker_centers):
        if any(marker_id not in marker_centers for marker_id in self.required_marker_ids):
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

        homography = cv2.getPerspectiveTransform(image_points, map_points)

        if homography is None or not np.isfinite(homography).all():
            self.warn_throttled(
                "invalid_homography",
                "Computed homography is invalid.",
                period_sec=2.0,
            )
            return False

        try:
            inverse_homography = np.linalg.inv(homography)
        except np.linalg.LinAlgError:
            self.warn_throttled(
                "invalid_inverse_homography",
                "Computed homography is not invertible.",
                period_sec=2.0,
            )
            return False

        self.homography = homography
        self.inverse_homography = inverse_homography
        self.last_homography_update_time = time.time()
        return True

    def homography_age_sec(self):
        if self.last_homography_update_time is None:
            return None
        return max(0.0, time.time() - self.last_homography_update_time)

    def homography_is_usable(self):
        if self.homography is None:
            return False

        age = self.homography_age_sec()
        if age is None:
            return False

        if age > self.homography_stale_drop_sec:
            self.warn_throttled(
                "homography_stale_drop",
                f"Homography is stale for {age:.2f}s. Dropping detector transform.",
                period_sec=2.0,
            )
            return False

        if age > self.homography_stale_warn_sec:
            self.warn_throttled(
                "homography_stale_warn",
                f"Homography is stale for {age:.2f}s. Transform still allowed.",
                period_sec=2.0,
            )

        return True

    def log_transform_state_throttled(self, homography_updated):
        now = time.time()
        if now - self.last_log_time < self.log_period_sec:
            return

        age = self.homography_age_sec()
        age_text = "none" if age is None else f"{age:.2f}s"

        self.get_logger().info(
            "Camera transform status: "
            f"markers={self.detected_marker_ids}, "
            f"missing={self.missing_marker_ids}, "
            f"has_homography={self.homography is not None}, "
            f"homography_age={age_text}, "
            f"updated_now={homography_updated}"
        )
        self.last_log_time = now

    def warn_throttled(self, key, message, period_sec=2.0):
        now = time.time()
        last_time = self.last_warn_times.get(key, 0.0)
        if now - last_time >= period_sec:
            self.get_logger().warn(message)
            self.last_warn_times[key] = now

    def image_to_map(self, pixel_x, pixel_y):
        if self.homography is None:
            return None

        point = np.array([[[float(pixel_x), float(pixel_y)]]], dtype=np.float32)
        transformed = cv2.perspectiveTransform(point, self.homography)

        if transformed is None:
            return None

        x = float(transformed[0, 0, 0])
        y = float(transformed[0, 0, 1])

        if not np.isfinite(x) or not np.isfinite(y):
            return None

        return x, y

    def map_to_image(self, x, y):
        if self.inverse_homography is None:
            return None

        point = np.array([[[float(x), float(y)]]], dtype=np.float32)
        transformed = cv2.perspectiveTransform(point, self.inverse_homography)

        if transformed is None:
            return None

        px = float(transformed[0, 0, 0])
        py = float(transformed[0, 0, 1])

        if not np.isfinite(px) or not np.isfinite(py):
            return None

        return int(round(px)), int(round(py))

    def point_inside_field(self, x, y):
        eps = 1e-6
        return (
            -eps <= x <= self.field_width_m + eps
            and -eps <= y <= self.field_height_m + eps
        )

    def compute_coverage_polygon(self):
        if self.homography is None or self.image_width is None or self.image_height is None:
            return []

        image_corners = np.array(
            [
                [[0.0, 0.0]],
                [[float(self.image_width - 1), 0.0]],
                [[float(self.image_width - 1), float(self.image_height - 1)]],
                [[0.0, float(self.image_height - 1)]],
            ],
            dtype=np.float32,
        )

        transformed = cv2.perspectiveTransform(image_corners, self.homography)
        polygon = []

        for pt in transformed.reshape(-1, 2):
            x = float(pt[0])
            y = float(pt[1])

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
            payload["coverage_confidence"] = self.coverage_confidence
            payload["coverage_polygon"] = self.compute_coverage_polygon()

        msg = String()
        msg.data = json.dumps(payload)
        self.publisher.publish(msg)

    def publish_status(self):
        age = self.homography_age_sec()
        has_homography = self.homography is not None
        is_stale_warn = age is not None and age > self.homography_stale_warn_sec
        is_stale_drop = age is not None and age > self.homography_stale_drop_sec

        if not has_homography:
            state = "not_ready"
        elif is_stale_drop:
            state = "stale_drop"
        elif is_stale_warn:
            state = "stale_warning"
        else:
            state = "ok"

        payload = {
            "mode": "apriltag_field_homography",
            "state": state,
            "has_homography": has_homography,
            "is_stale": bool(is_stale_warn),
            "is_transform_allowed": bool(has_homography and not is_stale_drop),
            "homography_age_sec": age,
            "detected_marker_ids": self.detected_marker_ids,
            "missing_marker_ids": self.missing_marker_ids,
            "required_marker_ids": self.required_marker_ids,
            "field_width_m": self.field_width_m,
            "field_height_m": self.field_height_m,
            "image_width": self.image_width,
            "image_height": self.image_height,
            "coverage_confidence": self.coverage_confidence,
            "last_input_detection_count": self.last_input_detection_count,
            "last_transformed_detection_count": self.last_transformed_detection_count,
            "output_topic": self.output_topic,
            "detector_topic": self.detector_topic,
            "image_topic": self.image_topic,
        }

        msg = String()
        msg.data = json.dumps(payload)
        self.status_publisher.publish(msg)

    def publish_debug_marker_image(self, source_msg, image, marker_corners):
        debug = image.copy()

        for marker_id, pts in marker_corners.items():
            pts_int = pts.astype(int)
            cv2.polylines(debug, [pts_int], isClosed=True, color=(0, 255, 0), thickness=2)
            center = pts.mean(axis=0).astype(int)
            cv2.putText(
                debug,
                str(marker_id),
                (int(center[0]), int(center[1])),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

        if self.inverse_homography is not None:
            field_points = [
                self.map_to_image(0.0, 0.0),
                self.map_to_image(self.field_width_m, 0.0),
                self.map_to_image(self.field_width_m, self.field_height_m),
                self.map_to_image(0.0, self.field_height_m),
            ]
            if all(point is not None for point in field_points):
                polygon = np.array(field_points, dtype=np.int32)
                cv2.polylines(debug, [polygon], isClosed=True, color=(0, 165, 255), thickness=2)

        age = self.homography_age_sec()
        age_text = "none" if age is None else f"{age:.2f}s"
        status_text = (
            f"markers {len(self.detected_marker_ids)}/{len(self.required_marker_ids)} "
            f"missing {self.missing_marker_ids} age {age_text}"
        )
        cv2.putText(
            debug,
            status_text,
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )

        success, encoded = cv2.imencode(
            ".jpg",
            debug,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.debug_image_jpeg_quality],
        )
        if not success:
            self.warn_throttled(
                "debug_encode_failed",
                "Could not encode debug marker image.",
                period_sec=2.0,
            )
            return

        msg = CompressedImage()
        msg.header = source_msg.header
        msg.format = "jpeg"
        msg.data = encoded.tobytes()
        self.debug_image_publisher.publish(msg)

    @staticmethod
    def header_stamp_to_float(msg):
        stamp = msg.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            return time.time()
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = CameraMarkerTransformNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
