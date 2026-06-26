#!/usr/bin/env python3

import json
import math
import time
from typing import List, Optional

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray
from px4_msgs.msg import VehicleLocalPosition, VehicleOdometry
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from std_msgs.msg import String


def is_finite_list(values) -> bool:
    return values is not None and all(math.isfinite(float(v)) for v in values)


def quat_wxyz_to_rotmat(q: List[float]) -> np.ndarray:
    """
    PX4 VehicleOdometry quaternion order is q(w, x, y, z).
    It represents rotation from FRD body frame to NED reference frame.
    """
    w, x, y, z = [float(v) for v in q]

    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm <= 1e-9:
        raise ValueError("Quaternion norm is too small")

    w, x, y, z = w / norm, x / norm, y / norm, z / norm

    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


def rpy_to_rotmat(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """
    Returns R = Rz(yaw) * Ry(pitch) * Rx(roll).
    Used as camera optical frame -> drone body FRD frame.
    """
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)

    return rz @ ry @ rx


class FlightCameraTransformNode(Node):
    def __init__(self):
        super().__init__("flight_camera_transform_node")

        self.declare_parameter("detector_topic", "/detector_node/confirmed")
        self.declare_parameter("output_topic", "/octopus/detections_world")
        self.declare_parameter("odometry_topic", "/fmu/out/vehicle_odometry")
        self.declare_parameter("local_position_topic", "/fmu/out/vehicle_local_position")
        self.declare_parameter("status_topic", "/octopus/flight_camera_transform/status")

        self.declare_parameter("status_period_sec", 1.0)
        self.declare_parameter("pose_stale_sec", 0.5)

        # Safety switch: keep false until pose + camera calibration are verified.
        self.declare_parameter("projection_enabled", False)

        # Detector gives normalized image coordinates in [0, 1].
        # Current detector convention: bottom-left origin.
        self.declare_parameter("normalized_v_origin", "bottom_left")

        # Camera intrinsics placeholder. Replace with calibration values later.
        self.declare_parameter("image_width", 640.0)
        self.declare_parameter("image_height", 480.0)
        self.declare_parameter("fx", 359.3292231592479)
        self.declare_parameter("fy", 359.2290038414162)
        self.declare_parameter("cx", 312.8204647201454)
        self.declare_parameter("cy", 237.947360594595)

        # OpenCV plumb_bob distortion coefficients from calibration.
        self.declare_parameter("k1", -0.057128411511179616)
        self.declare_parameter("k2", 0.0028040539388385884)
        self.declare_parameter("p1", 0.00015933624483912515)
        self.declare_parameter("p2", -0.001408459710522939)
        self.declare_parameter("k3", 0.0)

        # Rotation from camera optical frame to drone body FRD frame.
        # Must be calibrated for the real mount.
        self.declare_parameter("camera_to_body_roll_rad", 0.0)
        self.declare_parameter("camera_to_body_pitch_rad", 0.0)
        self.declare_parameter("camera_to_body_yaw_rad", 1.5707963267948966)

        # Camera optical center relative to PX4 body origin in FRD body frame.
        # x forward, y right, z down.
        self.declare_parameter("camera_x_body_m", 0.113)
        self.declare_parameter("camera_y_body_m", 0.0)
        self.declare_parameter("camera_z_body_m", 0.022)

        # NED ground plane. PX4 local z is down. If drone is above start plane,
        # z is usually negative and ground_z_ned is often 0.
        self.declare_parameter("ground_z_ned", 0.0)
        self.declare_parameter("use_dist_bottom_if_valid", True)

        # Safety: for real map placement, local x/y validity should be true.
        # Set false only for bench testing.
        self.declare_parameter("require_local_xy_valid", True)
        self.declare_parameter("require_local_z_valid", True)

        self.detector_topic = self.get_parameter("detector_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.odometry_topic = self.get_parameter("odometry_topic").value
        self.local_position_topic = self.get_parameter("local_position_topic").value
        self.status_topic = self.get_parameter("status_topic").value

        self.status_period_sec = float(self.get_parameter("status_period_sec").value)
        self.pose_stale_sec = float(self.get_parameter("pose_stale_sec").value)

        self.last_odometry: Optional[VehicleOdometry] = None
        self.last_odometry_receive_time: Optional[float] = None
        self.last_local_position: Optional[VehicleLocalPosition] = None
        self.last_local_position_receive_time: Optional[float] = None

        self.last_input_detection_count = 0
        self.last_transformed_detection_count = 0
        self.last_projection_error = None
        self.last_output_points = []

        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.output_pub = self.create_publisher(PoseArray, self.output_topic, 10)

        sensor_qos = QoSPresetProfiles.SENSOR_DATA.value

        self.odom_sub = self.create_subscription(
            VehicleOdometry,
            self.odometry_topic,
            self.odometry_callback,
            sensor_qos,
        )

        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition,
            self.local_position_topic,
            self.local_position_callback,
            sensor_qos,
        )

        self.detector_sub = self.create_subscription(
            PoseArray,
            self.detector_topic,
            self.detection_callback,
            10,
        )

        self.timer = self.create_timer(self.status_period_sec, self.publish_status)

        self.get_logger().info("Flight camera transform node started")
        self.get_logger().info(f"Detector topic: {self.detector_topic}")
        self.get_logger().info(f"Odometry topic: {self.odometry_topic}")
        self.get_logger().info(f"Local position topic: {self.local_position_topic}")
        self.get_logger().info(f"Output topic: {self.output_topic}")
        self.get_logger().info(f"Status topic: {self.status_topic}")

    def odometry_callback(self, msg: VehicleOdometry):
        self.last_odometry = msg
        self.last_odometry_receive_time = time.time()

    def local_position_callback(self, msg: VehicleLocalPosition):
        self.last_local_position = msg
        self.last_local_position_receive_time = time.time()

    def age_sec(self, receive_time: Optional[float]) -> Optional[float]:
        if receive_time is None:
            return None
        return max(0.0, time.time() - receive_time)

    def finite_list(self, values):
        return [float(v) if math.isfinite(float(v)) else None for v in values]

    def get_pose_state(self):
        odom_age = self.age_sec(self.last_odometry_receive_time)
        local_age = self.age_sec(self.last_local_position_receive_time)

        odom_fresh = odom_age is not None and odom_age <= self.pose_stale_sec
        local_fresh = local_age is not None and local_age <= self.pose_stale_sec

        odom_position = None
        odom_q = None
        odom_pose_frame = None

        if self.last_odometry is not None:
            odom_position = self.finite_list(self.last_odometry.position)
            odom_q = self.finite_list(self.last_odometry.q)
            odom_pose_frame = int(self.last_odometry.pose_frame)

        local = self.last_local_position

        local_xy_valid = bool(local.xy_valid) if local is not None else False
        local_z_valid = bool(local.z_valid) if local is not None else False
        local_dist_bottom_valid = bool(local.dist_bottom_valid) if local is not None else False

        local_position_ned = None
        local_heading = None
        local_dist_bottom = None

        if local is not None:
            local_position_ned = [
                float(local.x) if math.isfinite(float(local.x)) else None,
                float(local.y) if math.isfinite(float(local.y)) else None,
                float(local.z) if math.isfinite(float(local.z)) else None,
            ]
            local_heading = float(local.heading) if math.isfinite(float(local.heading)) else None
            local_dist_bottom = (
                float(local.dist_bottom)
                if math.isfinite(float(local.dist_bottom))
                else None
            )

        odom_position_valid = is_finite_list(odom_position) and len(odom_position) == 3
        odom_quaternion_valid = is_finite_list(odom_q) and len(odom_q) == 4

        require_xy = bool(self.get_parameter("require_local_xy_valid").value)
        require_z = bool(self.get_parameter("require_local_z_valid").value)

        pose_ready = bool(odom_fresh and odom_position_valid and odom_quaternion_valid)
        local_valid_enough = bool(
            local_fresh
            and (local_xy_valid or not require_xy)
            and (local_z_valid or not require_z)
        )

        projection_enabled = bool(self.get_parameter("projection_enabled").value)
        transform_ready = bool(pose_ready and local_valid_enough and projection_enabled)

        if transform_ready:
            state = "ready"
            reason = "projection enabled and fresh pose/local validity available"
        elif pose_ready and local_valid_enough:
            state = "pose_ready_projection_disabled"
            reason = "pose is ready, but projection_enabled is false"
        elif pose_ready:
            state = "pose_only"
            reason = "odometry is fresh, but local position validity is incomplete"
        else:
            state = "not_ready"
            reason = "waiting for fresh valid odometry"

        return {
            "state": state,
            "transform_ready": transform_ready,
            "pose_ready": pose_ready,
            "local_valid_enough": local_valid_enough,
            "reason": reason,
            "odometry": {
                "topic": self.odometry_topic,
                "fresh": odom_fresh,
                "age_sec": odom_age,
                "pose_frame": odom_pose_frame,
                "position": odom_position,
                "q": odom_q,
                "position_valid": odom_position_valid,
                "quaternion_valid": odom_quaternion_valid,
            },
            "local_position": {
                "topic": self.local_position_topic,
                "fresh": local_fresh,
                "age_sec": local_age,
                "xy_valid": local_xy_valid,
                "z_valid": local_z_valid,
                "dist_bottom_valid": local_dist_bottom_valid,
                "position_ned": local_position_ned,
                "heading": local_heading,
                "dist_bottom": local_dist_bottom,
            },
        }

    def normalized_to_pixel(self, u_norm: float, v_norm: float):
        image_width = float(self.get_parameter("image_width").value)
        image_height = float(self.get_parameter("image_height").value)
        v_origin = str(self.get_parameter("normalized_v_origin").value)

        px = u_norm * image_width

        if v_origin == "bottom_left":
            py = (1.0 - v_norm) * image_height
        elif v_origin == "top_left":
            py = v_norm * image_height
        else:
            raise ValueError(f"Unsupported normalized_v_origin: {v_origin}")

        return px, py

    def detection_to_world_ned(self, u_norm: float, v_norm: float):
        if self.last_odometry is None:
            raise ValueError("No odometry available")

        fx = float(self.get_parameter("fx").value)
        fy = float(self.get_parameter("fy").value)
        cx = float(self.get_parameter("cx").value)
        cy = float(self.get_parameter("cy").value)

        if fx <= 0.0 or fy <= 0.0:
            raise ValueError("Invalid camera intrinsics: fx/fy must be positive")

        px, py = self.normalized_to_pixel(u_norm, v_norm)

        k1 = float(self.get_parameter("k1").value)
        k2 = float(self.get_parameter("k2").value)
        p1 = float(self.get_parameter("p1").value)
        p2 = float(self.get_parameter("p2").value)
        k3 = float(self.get_parameter("k3").value)

        camera_matrix = np.array(
            [
                [fx, 0.0, cx],
                [0.0, fy, cy],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
        distortion = np.array([k1, k2, p1, p2, k3], dtype=np.float64)

        pixel = np.array([[[px, py]]], dtype=np.float64)

        # OpenCV returns normalized undistorted optical coordinates.
        # x = right, y = down, z = forward in camera optical frame.
        undistorted = cv2.undistortPoints(pixel, camera_matrix, distortion)
        x_cam = float(undistorted[0, 0, 0])
        y_cam = float(undistorted[0, 0, 1])

        ray_cam = np.array([x_cam, y_cam, 1.0], dtype=float)
        ray_cam = ray_cam / np.linalg.norm(ray_cam)

        roll = float(self.get_parameter("camera_to_body_roll_rad").value)
        pitch = float(self.get_parameter("camera_to_body_pitch_rad").value)
        yaw = float(self.get_parameter("camera_to_body_yaw_rad").value)

        r_body_cam = rpy_to_rotmat(roll, pitch, yaw)
        r_ned_body = quat_wxyz_to_rotmat(self.last_odometry.q)

        ray_ned = r_ned_body @ r_body_cam @ ray_cam
        ray_ned = ray_ned / np.linalg.norm(ray_ned)

        p_body_ned = np.array([float(v) for v in self.last_odometry.position], dtype=float)

        p_cam_body = np.array(
            [
                float(self.get_parameter("camera_x_body_m").value),
                float(self.get_parameter("camera_y_body_m").value),
                float(self.get_parameter("camera_z_body_m").value),
            ],
            dtype=float,
        )

        # Camera optical center in NED frame.
        p_cam_ned = p_body_ned + r_ned_body @ p_cam_body

        ground_z_ned = float(self.get_parameter("ground_z_ned").value)

        local = self.last_local_position
        use_dist_bottom = bool(self.get_parameter("use_dist_bottom_if_valid").value)

        if (
            use_dist_bottom
            and local is not None
            and bool(local.dist_bottom_valid)
            and math.isfinite(float(local.dist_bottom))
        ):
            ground_z_ned = p_cam_ned[2] + float(local.dist_bottom)

        if abs(ray_ned[2]) < 1e-6:
            raise ValueError("Camera ray is nearly parallel to ground plane")

        t = (ground_z_ned - p_cam_ned[2]) / ray_ned[2]

        if t <= 0.0:
            raise ValueError(
                f"Ground intersection is behind camera: t={t:.3f}, ray_ned_z={ray_ned[2]:.3f}"
            )

        hit_ned = p_cam_ned + t * ray_ned
        return hit_ned

    def detection_callback(self, msg: PoseArray):
        self.last_input_detection_count = len(msg.poses)
        self.last_transformed_detection_count = 0
        self.last_output_points = []
        self.last_projection_error = None

        pose_state = self.get_pose_state()

        if not pose_state["transform_ready"]:
            return

        out = PoseArray()
        out.header = msg.header
        out.header.frame_id = "map"

        transformed = []

        for detection_pose in msg.poses:
            u = float(detection_pose.position.x)
            v = float(detection_pose.position.y)

            try:
                hit_ned = self.detection_to_world_ned(u, v)
            except Exception as exc:
                self.last_projection_error = str(exc)
                continue

            pose = Pose()
            # Octopus map convention for now:
            # x = PX4 local North, y = PX4 local East, z = 0 ground/map plane.
            pose.position.x = float(hit_ned[0])
            pose.position.y = float(hit_ned[1])
            pose.position.z = 0.0
            pose.orientation.w = 1.0

            out.poses.append(pose)
            transformed.append([float(hit_ned[0]), float(hit_ned[1]), float(hit_ned[2])])

        self.last_transformed_detection_count = len(out.poses)
        self.last_output_points = transformed[:10]

        if len(out.poses) > 0:
            self.output_pub.publish(out)

    def publish_status(self):
        pose_state = self.get_pose_state()

        payload = {
            "mode": "flight_pose_ground_plane",
            "state": pose_state["state"],
            "transform_ready": pose_state["transform_ready"],
            "pose_ready": pose_state["pose_ready"],
            "local_valid_enough": pose_state["local_valid_enough"],
            "reason": pose_state["reason"],
            "projection_enabled": bool(self.get_parameter("projection_enabled").value),
            "pose_stale_sec": self.pose_stale_sec,
            "detector_topic": self.detector_topic,
            "output_topic": self.output_topic,
            "last_input_detection_count": self.last_input_detection_count,
            "last_transformed_detection_count": self.last_transformed_detection_count,
            "last_projection_error": self.last_projection_error,
            "last_output_points_ned": self.last_output_points,
            "camera_model": {
                "normalized_v_origin": str(self.get_parameter("normalized_v_origin").value),
                "image_width": float(self.get_parameter("image_width").value),
                "image_height": float(self.get_parameter("image_height").value),
                "fx": float(self.get_parameter("fx").value),
                "fy": float(self.get_parameter("fy").value),
                "cx": float(self.get_parameter("cx").value),
                "cy": float(self.get_parameter("cy").value),
                "distortion_coefficients": {
                    "k1": float(self.get_parameter("k1").value),
                    "k2": float(self.get_parameter("k2").value),
                    "p1": float(self.get_parameter("p1").value),
                    "p2": float(self.get_parameter("p2").value),
                    "k3": float(self.get_parameter("k3").value),
                },
                "camera_to_body_rpy_rad": [
                    float(self.get_parameter("camera_to_body_roll_rad").value),
                    float(self.get_parameter("camera_to_body_pitch_rad").value),
                    float(self.get_parameter("camera_to_body_yaw_rad").value),
                ],
                "camera_translation_body_m": [
                    float(self.get_parameter("camera_x_body_m").value),
                    float(self.get_parameter("camera_y_body_m").value),
                    float(self.get_parameter("camera_z_body_m").value),
                ],
                "ground_z_ned": float(self.get_parameter("ground_z_ned").value),
                "use_dist_bottom_if_valid": bool(self.get_parameter("use_dist_bottom_if_valid").value),
            },
            "odometry": pose_state["odometry"],
            "local_position": pose_state["local_position"],
            "output_note": "Flight projection node. Keep projection_enabled=false until camera intrinsics/extrinsics are verified.",
            "backend_received_at": None,
        }

        msg = String()
        msg.data = json.dumps(payload)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FlightCameraTransformNode()
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
