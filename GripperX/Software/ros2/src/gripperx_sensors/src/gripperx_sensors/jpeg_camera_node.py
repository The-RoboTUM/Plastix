#!/usr/bin/env python3
"""USB camera driver that passes the camera's own MJPEG frames through undecoded.

Why not usb_cam: ros-jazzy-usb-cam 0.8.1-1noble cannot open a /dev/v4l/by-id
symlink, segfaults in `mjpeg2rgb` (and on shutdown keeps the device busy), and
`raw_mjpeg` publishes JPEG bytes labelled "yuv422" on image_raw.

WHAT IT PUBLISHES. sensor_msgs/CompressedImage (format "jpeg") on image_raw/compressed
and sensor_msgs/CameraInfo on camera_info, same header. No uncompressed image_raw.
HWR-22: MJPEG only over USB. OpenCV is used with CONVERT_RGB=0, so cap.read()
returns the raw JPEG as a (1, N) uint8 array and nothing is decoded on the Pi.

LAZY CAPTURE. With no subscriber on image_raw/compressed or camera_info (checked about
once per second) the VideoCapture is released, so the USB stream and the Pi CPU cost
stop; it is reopened when a subscriber appears. Capture only fills a one-slot buffer;
a separate thread publishes the newest frame, so a slow subscriber costs dropped frames
and never capture rate.

PROFILES. `profile` selects a mode from the table profiles.<name>.{width,height,fps}
in camera_front.yaml. Setting it at runtime reopens the camera; unknown names are
rejected. CameraInfo width/height follow the active mode.

TIMESTAMP. Clock time right after read() returns. It includes USB transfer and
exposure latency, which is unmeasured (TO-VERIFY).

Self-contained (no package-relative imports) so the file also runs as
`python3 jpeg_camera_node.py --ros-args ...`.
"""

import os
import signal
import sys
import threading
import time

import cv2
import rclpy
import yaml
from rclpy._rclpy_pybind11 import RCLError
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage

DEFAULT_DEVICE = (
    "/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam_1080P_Low_Light_UC684-video-index0"
)


def is_jpeg(buf):
    """True if the buffer starts with the JPEG start-of-image marker ffd8."""
    return len(buf) >= 2 and bytes(buf[:2]) == b"\xff\xd8"


def resolve_device(path):
    """Real /dev/videoN path behind a symlink such as the by-id one."""
    return os.path.realpath(path)


def load_camera_info(url, width, height):
    """CameraInfo from a file:// ROS calibration YAML; width/height only if url is empty."""
    info = CameraInfo()
    info.width = int(width)
    info.height = int(height)
    if not url:
        return info
    path = url[len("file://"):] if url.startswith("file://") else url
    with open(path) as f:
        data = yaml.safe_load(f)
    info.width = int(data.get("image_width", width))
    info.height = int(data.get("image_height", height))
    info.distortion_model = data.get("distortion_model", "")
    info.d = [float(v) for v in data["distortion_coefficients"]["data"]]
    info.k = [float(v) for v in data["camera_matrix"]["data"]]
    info.r = [float(v) for v in data["rectification_matrix"]["data"]]
    info.p = [float(v) for v in data["projection_matrix"]["data"]]
    return info


class JpegCamera(Node):
    def __init__(self, open_fn=None, parameter_overrides=None):
        """open_fn: callable returning a cv2.VideoCapture-like object (tests inject a fake);
        it is called with no arguments and reads the active mode from self.width/height/fps.
        """
        super().__init__("camera_front", parameter_overrides=parameter_overrides)
        self._open_fn = open_fn or self._open
        self.declare_parameter("video_device", DEFAULT_DEVICE)
        self.declare_parameter("profile", "view")
        self.declare_parameter("profile_names", ["view", "detail"])
        self.declare_parameter("frame_id", "camera_link")  # mount pose TO-VERIFY
        self.declare_parameter("camera_info_url", "")
        # profiles.<name>.{width,height,fps}: the table lives in camera_front.yaml.
        self.profiles = {}
        for name in self.get_parameter("profile_names").value:
            self.declare_parameter("profiles.%s.width" % name, 0)
            self.declare_parameter("profiles.%s.height" % name, 0)
            self.declare_parameter("profiles.%s.fps" % name, 0.0)
            mode = {
                "width": int(self.get_parameter("profiles.%s.width" % name).value),
                "height": int(self.get_parameter("profiles.%s.height" % name).value),
                "fps": float(self.get_parameter("profiles.%s.fps" % name).value),
            }
            if mode["width"] <= 0 or mode["height"] <= 0 or mode["fps"] <= 0.0:
                raise ValueError("profile %r is not defined in the parameters" % name)
            self.profiles[name] = mode
        self.frame_id = self.get_parameter("frame_id").value
        self.device = resolve_device(self.get_parameter("video_device").value)
        self._info_url = self.get_parameter("camera_info_url").value
        self._lock = threading.Lock()
        self.profile = self.get_parameter("profile").value
        if self.profile not in self.profiles:
            raise ValueError("profile %r is not one of %s" % (self.profile, sorted(self.profiles)))
        self._apply_mode(self.profile)
        self._reopen = False
        qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
        self.pub_img = self.create_publisher(CompressedImage, "image_raw/compressed", qos)
        self.pub_info = self.create_publisher(CameraInfo, "camera_info", qos)
        self.add_on_set_parameters_callback(self._on_params)
        self.failed = False
        self.capturing = False
        self.mode_mismatch = False
        self.actual_size = (0, 0)
        self.dropped = 0  # frames replaced in the slot before the publisher took them
        self._slot = None  # (stamp, data, info): newest captured frame, not yet published
        self._slot_cv = threading.Condition()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._pub_thread = threading.Thread(target=self._publish_loop, daemon=True)

    def _apply_mode(self, profile):
        mode = self.profiles[profile]
        self.width, self.height, self.fps = mode["width"], mode["height"], mode["fps"]

    def _on_params(self, params):
        # Validate the WHOLE batch first, then apply: a rejected batch changes nothing.
        profile = None
        for p in params:
            if p.name == "profile":
                if not isinstance(p.value, str) or p.value not in self.profiles:
                    return SetParametersResult(
                        successful=False,
                        reason="unknown profile %r, valid: %s" % (p.value, sorted(self.profiles)),
                    )
                profile = p.value
            elif (
                p.name.startswith("profiles.")
                or p.name in ("profile_names", "video_device", "frame_id", "camera_info_url")
            ):
                return SetParametersResult(
                    successful=False, reason="%s can only be set at start" % p.name
                )
        if profile is not None:
            with self._lock:
                if profile != self.profile:
                    self.profile = profile
                    self._apply_mode(profile)
                    self._reopen = True
                    self.get_logger().info(
                        "profile -> %s (%dx%d @ %.1f fps), reopening"
                        % (profile, self.width, self.height, self.fps)
                    )
        return SetParametersResult(successful=True)

    def _actual_mode(self, cap):
        """(fourcc, width, height) the capture reports; (None, 0, 0) if it cannot say."""
        try:
            code = int(cap.get(cv2.CAP_PROP_FOURCC))
            fourcc = "".join(chr((code >> (8 * i)) & 0xFF) for i in range(4))
            return fourcc, int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(
                cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        except Exception:  # noqa: BLE001 -- a capture without get(): unknown
            return None, 0, 0

    def _make_info(self, width, height):
        info = load_camera_info(self._info_url, width, height)
        if (info.width, info.height) != (width, height):
            self.get_logger().warn(
                "calibration is for %dx%d but the active mode is %dx%d: publishing it "
                "uncalibrated (size only)" % (info.width, info.height, width, height),
                throttle_duration_sec=60.0,
            )
            info = load_camera_info("", width, height)
        return info

    def _info_for_open(self, cap):
        """CameraInfo for what the camera ACTUALLY delivers. The camera may ignore the
        requested mode; then this logs an ERROR and publishes the actual size, never the
        requested one. The frames are not refused (they are still valid JPEG); consumers
        that need the requested size must check CameraInfo. Size 0 = capture cannot say."""
        with self._lock:
            want = (self.width, self.height)
        fourcc, w, h = self._actual_mode(cap)
        self.actual_size = (w, h)
        self.mode_mismatch = (fourcc is not None and fourcc != "MJPG") or (w, h) != want
        if self.mode_mismatch:
            self.get_logger().error(
                "camera did not take the requested mode: asked MJPG %dx%d, got %s %dx%d; "
                "CameraInfo carries the actual size" % (want + (fourcc, w, h))
            )
        return self._make_info(w, h)

    def _has_subscribers(self):
        return (
            self.pub_img.get_subscription_count() > 0
            or self.pub_info.get_subscription_count() > 0
        )

    def start(self):
        self._thread.start()
        self._pub_thread.start()

    def _open(self):
        cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
        if cap.isOpened():
            code = int(cap.get(cv2.CAP_PROP_FOURCC))
            self.get_logger().info(
                "opened %s: fourcc %s, %dx%d @ %.1f fps"
                % (
                    self.device,
                    "".join(chr((code >> (8 * i)) & 0xFF) for i in range(4)),
                    cap.get(cv2.CAP_PROP_FRAME_WIDTH),
                    cap.get(cv2.CAP_PROP_FRAME_HEIGHT),
                    cap.get(cv2.CAP_PROP_FPS),
                )
            )
        return cap

    def _run(self):
        """Capture thread. Never calls publish(): it only fills the one-slot buffer."""
        cap = None
        first = True
        info = None
        t_check = 0.0
        wanted = False
        while not self._stop.is_set():
            now = time.monotonic()
            if now - t_check >= 1.0 or cap is None:
                t_check = now
                wanted = self._has_subscribers()
            if not wanted:
                if cap is not None:
                    cap.release()
                    cap = None
                    self.capturing = False
                    self.get_logger().info("no subscribers: camera released, USB stream stopped")
                self._stop.wait(1.0)  # idle: look for a subscriber about once per second
                continue
            with self._lock:
                reopen, self._reopen = self._reopen, False
            if reopen and cap is not None:
                cap.release()
                cap = None
            if cap is None:
                if not self.capturing:
                    self.get_logger().info("subscriber present: opening camera")
                cap = self._open_fn()
                # Only an opened capture has a mode to compare (a missing device would
                # otherwise log a mismatch ERROR on every retry).
                info = self._info_for_open(cap) if cap.isOpened() else None
                self.capturing = True
                first = True
            ok, buf = cap.read() if cap.isOpened() else (False, None)
            stamp = self.get_clock().now().to_msg()
            if not ok or buf is None:
                self.get_logger().warn(
                    "read failed on %s, reopening in 1 s" % self.device, throttle_duration_sec=10.0
                )
                cap.release()
                cap = None
                self.capturing = False
                self._stop.wait(1.0)
                continue
            data = buf.tobytes()
            if first:
                first = False
                if not is_jpeg(data):
                    self.get_logger().error(
                        "first frame is not JPEG (camera ignored MJPG); exiting"
                    )
                    self.failed = True
                    break
            with self._slot_cv:
                if self._slot is not None:
                    self.dropped += 1
                self._slot = (stamp, data, info)
                self._slot_cv.notify()
        if cap is not None:
            cap.release()
        self.capturing = False

    def _publish_loop(self):
        """Publish thread: takes the newest frame; a slow publish() only costs dropped frames."""
        count = 0
        t_log = time.monotonic()
        while not self._stop.is_set():
            with self._slot_cv:
                while self._slot is None and not self._stop.is_set():
                    self._slot_cv.wait(0.2)
                slot, self._slot = self._slot, None
            if slot is None:
                continue
            stamp, data, info = slot
            msg = CompressedImage()
            msg.header.stamp = stamp
            msg.header.frame_id = self.frame_id
            msg.format = "jpeg"
            msg.data = data
            info.header = msg.header
            if self._stop.is_set() or not self.context.ok():
                break
            try:
                self.pub_img.publish(msg)
                self.pub_info.publish(info)
            except RCLError:
                if self._stop.is_set() or not self.context.ok():
                    break  # shutting down
                raise
            count += 1
            now = time.monotonic()
            if now - t_log >= 60.0:
                self.get_logger().info(
                    "%.1f fps published over the last minute (%d dropped in total)"
                    % (count / (now - t_log), self.dropped)
                )
                count, t_log = 0, now

    def stop(self):
        """Stop the threads and free the device (read() blocks <= one frame)."""
        self._stop.set()
        with self._slot_cv:
            self._slot_cv.notify_all()
        for t in (self._thread, self._pub_thread):
            if t.is_alive():
                t.join(timeout=2.0)


def _raise_interrupt(signum, frame):
    raise KeyboardInterrupt


def main(args=None):
    # SIGINT must not invalidate the context while the capture thread publishes.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, _raise_interrupt)
    signal.signal(signal.SIGTERM, _raise_interrupt)
    node = JpegCamera()
    node.start()
    try:
        while not node.failed:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(1 if node.failed else 0)


if __name__ == "__main__":
    main()
