#!/usr/bin/env python3

import base64
import json
import time
import urllib.error
import urllib.request

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

try:
    import cv2
except ImportError:  # pragma: no cover - the bridge still runs, just uncropped
    cv2 = None

# Sides of the frame the operator can cut away, as a fraction of the full frame.
# Mirrors CAMERA_CROP_MAX_SIDE in the dashboard backend (api.py) and live_data.js.
CROP_SIDES = ("top", "right", "bottom", "left")
CROP_MAX_SIDE = 0.45


class CameraDebugBackendBridgeNode(Node):
    def __init__(self):
        super().__init__("camera_debug_backend_bridge_node")

        self.declare_parameter("image_topic", "/detector_node/debug_image/compressed")
        self.declare_parameter("detections_topic", "/detector_node/detections_debug")
        self.declare_parameter(
            "frame_backend_url",
            "http://127.0.0.1:8000/api/camera_debug/frame",
        )
        self.declare_parameter(
            "detections_backend_url",
            "http://127.0.0.1:8000/api/camera_debug/detections",
        )
        self.declare_parameter(
            "crop_config_url",
            "http://127.0.0.1:8000/api/camera_debug/crop",
        )
        self.declare_parameter("request_timeout_sec", 1.0)
        self.declare_parameter("image_post_period_sec", 0.5)
        self.declare_parameter("detections_post_period_sec", 0.2)
        self.declare_parameter("log_period_sec", 5.0)
        self.declare_parameter("crop_poll_period_sec", 2.0)
        # The detector already encoded at quality 80, so cropping is a SECOND lossy
        # generation — but at 85 that generation costs ~49 dB PSNR (invisible) while
        # a cropped frame still stays well below the full frame's size. Going higher
        # is counterproductive: at 95 the cropped frame gets LARGER than the
        # uncropped one it replaces.
        self.declare_parameter("crop_jpeg_quality", 85)

        self.image_topic = self.get_parameter("image_topic").value
        self.detections_topic = self.get_parameter("detections_topic").value
        self.frame_backend_url = self.get_parameter("frame_backend_url").value
        self.detections_backend_url = self.get_parameter("detections_backend_url").value
        self.crop_config_url = self.get_parameter("crop_config_url").value
        self.request_timeout_sec = float(self.get_parameter("request_timeout_sec").value)
        self.image_post_period_sec = float(self.get_parameter("image_post_period_sec").value)
        self.detections_post_period_sec = float(
            self.get_parameter("detections_post_period_sec").value
        )
        self.log_period_sec = float(self.get_parameter("log_period_sec").value)
        self.crop_poll_period_sec = float(self.get_parameter("crop_poll_period_sec").value)
        self.crop_jpeg_quality = int(self.get_parameter("crop_jpeg_quality").value)

        # Crop the operator asked for, refreshed from the backend. Until the first
        # successful poll this is "no crop", so a frame is never silently cut.
        self.crop = {side: 0.0 for side in CROP_SIDES}
        self.crop_revision = None
        self.last_crop_poll_time = 0.0
        self.crop_warned_no_cv2 = False
        self.cropped_image_count = 0

        self.last_image_post_time = 0.0
        self.last_detections_post_time = 0.0
        self.last_error_log_time = 0.0
        self.last_success_log_time = 0.0
        self.image_count = 0
        self.detections_count = 0

        self.image_subscription = self.create_subscription(
            CompressedImage,
            self.image_topic,
            self.image_callback,
            10,
        )
        self.detections_subscription = self.create_subscription(
            String,
            self.detections_topic,
            self.detections_callback,
            10,
        )

        self.get_logger().info("Camera debug backend bridge started")
        self.get_logger().info(f"Image topic: {self.image_topic}")
        self.get_logger().info(f"Detections topic: {self.detections_topic}")
        self.get_logger().info(f"Frame backend URL: {self.frame_backend_url}")
        self.get_logger().info(f"Detections backend URL: {self.detections_backend_url}")
        self.get_logger().info(f"Crop config URL: {self.crop_config_url}")
        if cv2 is None:
            self.get_logger().warn(
                "cv2 is not available - frames are forwarded uncropped and the "
                "dashboard falls back to cropping them in the browser"
            )

    @staticmethod
    def stamp_to_float(stamp):
        try:
            value = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        except Exception:
            value = 0.0
        return value if value > 0.0 else time.time()

    def image_callback(self, msg: CompressedImage):
        now = time.time()
        if now - self.last_image_post_time < self.image_post_period_sec:
            return
        self.last_image_post_time = now

        self.refresh_crop(now)

        jpeg_bytes = bytes(msg.data)
        crop_result = self.crop_jpeg(jpeg_bytes)

        payload = {
            "format": "jpeg",
            "stamp": self.stamp_to_float(msg.header.stamp),
            "frame_id": msg.header.frame_id or "camera",
            "data_base64": base64.b64encode(crop_result["data"]).decode("ascii"),
            # What the dashboard needs to line its overlay up with this frame:
            # the crop that was actually applied here (None when the frame is the
            # untouched full frame), and both frame sizes.
            "crop": crop_result["crop"],
            "source_size": crop_result["source_size"],
            "cropped_size": crop_result["cropped_size"],
        }

        try:
            self.post_json(self.frame_backend_url, payload)
        except Exception as exc:
            self.log_error_throttled(f"Failed to POST camera debug frame: {exc}")
            return

        self.image_count += 1
        if crop_result["crop"]:
            self.cropped_image_count += 1
        self.log_success_throttled()

    # -------------------------------------------------------------------------
    # Camera crop
    #
    # The operator picks the crop in the dashboard, the backend stores it, and the
    # cut happens HERE - so only the wanted part of the frame ever goes over the
    # wire. The detections topic is left alone: the detector still runs on the
    # full frame, and the dashboard drops the detections that fall in the
    # cut-away edges (it needs the full-frame u/v to decide that).
    # -------------------------------------------------------------------------

    def refresh_crop(self, now):
        """Poll the operator's crop from the backend, at most every crop_poll_period_sec."""
        if now - self.last_crop_poll_time < self.crop_poll_period_sec:
            return
        self.last_crop_poll_time = now

        try:
            request = urllib.request.Request(self.crop_config_url, method="GET")
            with urllib.request.urlopen(request, timeout=self.request_timeout_sec) as response:
                body = json.loads(response.read().decode("utf-8"))
        except (urllib.error.URLError, OSError, ValueError) as exc:
            # Backend down or unreachable: keep the last known crop rather than
            # silently reverting to full frames mid-flight.
            self.log_error_throttled(f"Failed to read camera crop config: {exc}")
            return

        config = body.get("crop") or {}
        crop = {}
        for side in CROP_SIDES:
            try:
                value = float(config.get(side, 0.0))
            except (TypeError, ValueError):
                value = 0.0
            crop[side] = max(0.0, min(value, CROP_MAX_SIDE))

        revision = config.get("revision")
        if crop != self.crop:
            self.crop = crop
            self.crop_revision = revision
            if any(value > 0.0 for value in crop.values()):
                self.get_logger().info(
                    "Camera crop set to "
                    f"top={crop['top']:.2f} right={crop['right']:.2f} "
                    f"bottom={crop['bottom']:.2f} left={crop['left']:.2f} "
                    "- only the cropped region is sent to the dashboard"
                )
            else:
                self.get_logger().info("Camera crop cleared - sending full frames again")
        else:
            self.crop_revision = revision

    def crop_jpeg(self, jpeg_bytes):
        """Cut the configured edges off a JPEG frame and re-encode it.

        Returns the bytes to send plus the crop that was really applied. On any
        problem (no crop configured, no cv2, undecodable frame, degenerate crop)
        the original bytes are returned with crop=None, and the dashboard falls
        back to cropping the frame in the browser.
        """
        unchanged = {
            "data": jpeg_bytes,
            "crop": None,
            "source_size": None,
            "cropped_size": None,
        }

        crop = dict(self.crop)
        if not any(value > 0.0 for value in crop.values()):
            return unchanged

        if cv2 is None:
            if not self.crop_warned_no_cv2:
                self.get_logger().warn(
                    "A camera crop is configured but cv2 is missing - forwarding full frames"
                )
                self.crop_warned_no_cv2 = True
            return unchanged

        try:
            buffer = np.frombuffer(jpeg_bytes, dtype=np.uint8)
            image = cv2.imdecode(buffer, cv2.IMREAD_COLOR)
        except Exception as exc:  # pragma: no cover - defensive
            self.log_error_throttled(f"Failed to decode frame for cropping: {exc}")
            return unchanged

        if image is None or image.size == 0:
            self.log_error_throttled("Failed to decode frame for cropping: empty image")
            return unchanged

        height, width = image.shape[:2]
        x0 = int(round(crop["left"] * width))
        x1 = width - int(round(crop["right"] * width))
        y0 = int(round(crop["top"] * height))
        y1 = height - int(round(crop["bottom"] * height))

        # A crop that would leave nothing usable is ignored rather than sent.
        if x1 - x0 < 8 or y1 - y0 < 8:
            self.log_error_throttled(
                f"Camera crop leaves only {max(0, x1 - x0)}x{max(0, y1 - y0)} px - ignored"
            )
            return unchanged

        cropped = image[y0:y1, x0:x1]

        try:
            ok, encoded = cv2.imencode(
                ".jpg",
                cropped,
                [int(cv2.IMWRITE_JPEG_QUALITY), self.crop_jpeg_quality],
            )
        except Exception as exc:  # pragma: no cover - defensive
            self.log_error_throttled(f"Failed to re-encode cropped frame: {exc}")
            return unchanged

        if not ok:
            self.log_error_throttled("Failed to re-encode cropped frame")
            return unchanged

        return {
            "data": encoded.tobytes(),
            # The crop is reported back as the exact pixel fractions that were cut,
            # not as the requested ones, so the dashboard's overlay matches the
            # frame even after the integer rounding above.
            "crop": {
                "top": y0 / height,
                "right": (width - x1) / width,
                "bottom": (height - y1) / height,
                "left": x0 / width,
            },
            "source_size": {"width": width, "height": height},
            "cropped_size": {"width": int(x1 - x0), "height": int(y1 - y0)},
        }

    def detections_callback(self, msg: String):
        now = time.time()
        if now - self.last_detections_post_time < self.detections_post_period_sec:
            return
        self.last_detections_post_time = now

        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.log_error_throttled(f"Invalid detector debug JSON: {exc}")
            return

        payload["bridge_received_at"] = now

        try:
            self.post_json(self.detections_backend_url, payload)
        except Exception as exc:
            self.log_error_throttled(f"Failed to POST camera debug detections: {exc}")
            return

        self.detections_count += 1
        self.log_success_throttled(payload)

    def post_json(self, url, payload):
        data = json.dumps(payload).encode("utf-8")
        request = urllib.request.Request(
            url,
            data=data,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with urllib.request.urlopen(request, timeout=self.request_timeout_sec) as response:
            if response.status < 200 or response.status >= 300:
                raise RuntimeError(f"Backend returned HTTP {response.status}")

    def log_error_throttled(self, message):
        now = time.time()
        if now - self.last_error_log_time >= self.log_period_sec:
            self.get_logger().warn(message)
            self.last_error_log_time = now

    def log_success_throttled(self, payload=None):
        now = time.time()
        if now - self.last_success_log_time < self.log_period_sec:
            return
        detection_count = 0
        if payload:
            detection_count = len(payload.get("detections", []))
        crop_note = ""
        if any(value > 0.0 for value in self.crop.values()):
            crop_note = (
                f", cropped_images={self.cropped_image_count}"
                f" (top={self.crop['top']:.2f} right={self.crop['right']:.2f}"
                f" bottom={self.crop['bottom']:.2f} left={self.crop['left']:.2f})"
            )
        self.get_logger().info(
            "Forwarded camera debug data "
            f"images={self.image_count}, detections_msgs={self.detections_count}, "
            f"latest_detection_count={detection_count}{crop_note}"
        )
        self.last_success_log_time = now


def main(args=None):
    rclpy.init(args=args)
    node = CameraDebugBackendBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
