"""Pure parts of the JPEG camera node, without a camera."""

import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
from gripperx_sensors import jpeg_camera_node as cam  # noqa: E402

CALIB = """image_width: 640
image_height: 480
camera_name: c
distortion_model: plumb_bob
camera_matrix: {rows: 3, cols: 3, data: [500, 0, 320, 0, 500, 240, 0, 0, 1]}
distortion_coefficients: {rows: 1, cols: 5, data: [0.1, -0.2, 0, 0, 0]}
rectification_matrix: {rows: 3, cols: 3, data: [1, 0, 0, 0, 1, 0, 0, 0, 1]}
projection_matrix: {rows: 3, cols: 4, data: [500, 0, 320, 0, 0, 500, 240, 0, 0, 0, 1, 0]}
"""


def test_is_jpeg():
    assert cam.is_jpeg(b"\xff\xd8\xff\xc0rest")
    assert cam.is_jpeg(np.array([0xFF, 0xD8, 1], dtype=np.uint8))
    assert not cam.is_jpeg(b"\x00\x01")
    assert not cam.is_jpeg(b"\xff")
    assert not cam.is_jpeg(b"")


def test_resolve_device_follows_symlink(tmp_path):
    real = tmp_path / "video0"
    real.write_text("")
    link = tmp_path / "by-id-link"
    link.symlink_to(real)
    assert cam.resolve_device(str(link)) == str(real.resolve())


def test_camera_info_empty_url():
    info = cam.load_camera_info("", 1280, 720)
    assert (info.width, info.height) == (1280, 720)
    assert info.distortion_model == "" and len(info.k) == 9 and not any(info.k)


def test_camera_info_from_file(tmp_path):
    f = tmp_path / "c.yaml"
    f.write_text(CALIB)
    info = cam.load_camera_info("file://" + str(f), 1280, 720)
    assert (info.width, info.height) == (640, 480)
    assert info.distortion_model == "plumb_bob"
    assert list(info.k)[:3] == [500.0, 0.0, 320.0]
    assert list(info.d)[:2] == [0.1, -0.2]
    assert len(info.p) == 12 and len(info.r) == 9


CONFIG = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "config", "camera_front.yaml"
)


def active_config():
    """ros__parameters of the ACTIVE config file, flattened to dotted names."""
    import yaml

    data = yaml.safe_load(open(CONFIG))
    params = next(iter(data.values()))["ros__parameters"]
    flat = {}

    def walk(prefix, value):
        if isinstance(value, dict):
            for k, v in value.items():
                walk(prefix + k + ".", v)
        else:
            flat[prefix[:-1]] = value

    walk("", params)
    flat.pop("video_device")  # not a real device here
    return flat


def overrides(**extra):
    from rclpy.parameter import Parameter

    flat = active_config()
    flat.update(extra)
    return [Parameter(k, value=v) for k, v in flat.items()]


class FakeCap:
    def __init__(self, delay=0.01, mode=(b"MJPG", 320, 240)):
        self.released = False
        self.delay = delay
        self.mode = mode  # what get() reports: (fourcc, width, height)

    def get(self, prop):
        import cv2

        fourcc, w, h = self.mode
        if prop == cv2.CAP_PROP_FOURCC:
            return int.from_bytes(fourcc, "little")
        return {cv2.CAP_PROP_FRAME_WIDTH: w, cv2.CAP_PROP_FRAME_HEIGHT: h}.get(prop, 0)

    def isOpened(self):
        return not self.released

    def read(self):
        time.sleep(self.delay)
        return True, np.frombuffer(b"\xff\xd8\xff\xc0fake", dtype=np.uint8).reshape(1, -1)

    def release(self):
        self.released = True


class Rig:
    """Node with a fake camera that records every open (with the mode asked for)."""

    def __init__(self, delay=0.01, ignore_mode=None, **extra):
        import rclpy

        rclpy.init(signal_handler_options=cam.SignalHandlerOptions.NO)
        self.opens = []
        self.caps = []
        self.node = cam.JpegCamera(open_fn=self._open, parameter_overrides=overrides(**extra))
        self.delay = delay
        self.ignore_mode = ignore_mode  # fixed (fourcc, w, h) the fake camera reports

    def _open(self):
        mode = self.ignore_mode or (b"MJPG", self.node.width, self.node.height)
        cap = FakeCap(self.delay, mode)
        self.caps.append(cap)
        self.opens.append((self.node.width, self.node.height, self.node.fps))
        return cap

    def spin_until(self, cond, timeout=6.0):
        import rclpy

        end = time.monotonic() + timeout
        while not cond() and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return cond()

    def close(self):
        import rclpy

        self.node.stop()
        self.node.destroy_node()
        rclpy.try_shutdown()


def test_config_profiles_are_the_decided_ones():
    cfg = active_config()
    assert cfg["profile"] == "view"
    assert (cfg["profiles.view.width"], cfg["profiles.view.height"]) == (320, 240)
    assert cfg["profiles.view.fps"] == 15.0
    assert (cfg["profiles.detail.width"], cfg["profiles.detail.height"]) == (640, 480)
    assert cfg["profiles.detail.fps"] == 5.0


def test_start_publish_stop_is_clean():
    from sensor_msgs.msg import CameraInfo, CompressedImage

    rig = Rig()
    node = rig.node
    got, infos = [], []
    node.create_subscription(CompressedImage, "image_raw/compressed", got.append, 10)
    node.create_subscription(CameraInfo, "camera_info", infos.append, 10)
    node.start()
    assert rig.spin_until(lambda: len(got) >= 3 and infos)
    node.stop()
    assert not node._thread.is_alive() and not node._pub_thread.is_alive()
    assert rig.caps[0].released
    assert not node.failed
    assert got[0].format == "jpeg" and cam.is_jpeg(got[0].data)
    assert (infos[0].width, infos[0].height) == (320, 240)
    rig.close()


def test_lazy_open_and_close_by_subscriber_count():
    from sensor_msgs.msg import CompressedImage

    rig = Rig()
    node = rig.node
    node.start()
    time.sleep(2.2)
    assert rig.opens == [] and not node.capturing  # no subscriber: camera never opened
    sub = node.create_subscription(CompressedImage, "image_raw/compressed", lambda m: None, 10)
    assert rig.spin_until(lambda: len(rig.opens) == 1)
    assert node.capturing and not rig.caps[0].released
    node.destroy_subscription(sub)
    assert rig.spin_until(lambda: rig.caps[0].released)
    assert rig.spin_until(lambda: not node.capturing)
    sub = node.create_subscription(CompressedImage, "image_raw/compressed", lambda m: None, 10)
    assert rig.spin_until(lambda: len(rig.opens) == 2)  # reopened on the next subscriber
    rig.close()


def test_capture_not_blocked_by_slow_publish():
    from sensor_msgs.msg import CompressedImage

    rig = Rig(delay=0.005)
    node = rig.node

    class SlowPub:
        def __init__(self, real):
            self.real, self.n = real, 0

        def publish(self, msg):
            self.n += 1
            time.sleep(0.3)
            self.real.publish(msg)

        def get_subscription_count(self):
            return self.real.get_subscription_count()

    slow = SlowPub(node.pub_img)
    node.pub_img = slow
    node.create_subscription(CompressedImage, "image_raw/compressed", lambda m: None, 10)
    node.start()
    t0 = time.monotonic()
    assert rig.spin_until(lambda: slow.n >= 3, timeout=10)
    elapsed = time.monotonic() - t0
    # Capture kept running at ~200 fps; frames the publisher could not take were dropped.
    assert node.dropped > 20 * slow.n // 3
    assert elapsed < 6
    rig.close()


def test_profile_switch_reopens_and_rejects_unknown():
    import rclpy
    from rclpy.parameter import Parameter
    from sensor_msgs.msg import CameraInfo

    rig = Rig()
    node = rig.node
    infos = []
    node.create_subscription(CameraInfo, "camera_info", infos.append, 10)
    node.start()
    assert rig.spin_until(lambda: len(rig.opens) == 1 and infos)
    assert rig.opens[0] == (320, 240, 15.0)
    bad = node.set_parameters([Parameter("profile", value="bogus")])[0]
    assert not bad.successful and "bogus" in bad.reason
    assert node.profile == "view" and len(rig.opens) == 1
    assert node.set_parameters([Parameter("profile", value="detail")])[0].successful
    assert rig.spin_until(lambda: len(rig.opens) == 2)
    assert rig.opens[1] == (640, 480, 5.0)
    assert rig.caps[0].released
    del infos[:]
    assert rig.spin_until(lambda: any(i.width == 640 and i.height == 480 for i in infos))
    assert not node.set_parameters([Parameter("profiles.view.width", value=1)])[0].successful
    rig.close()


def test_mixed_batch_is_atomic():
    from rclpy.parameter import Parameter

    rig = Rig()
    node = rig.node
    node.start()
    for batch in (
        [Parameter("profile", value="detail"), Parameter("frame_id", value="x")],
        [Parameter("profile", value="detail"), Parameter("camera_info_url", value="x")],
        [Parameter("profile", value="detail"), Parameter("video_device", value="/dev/null")],
        [Parameter("profile", value="detail"), Parameter("profile", value="bogus")],
    ):
        res = node.set_parameters_atomically(batch)
        assert not res.successful
        # nothing of the rejected batch was applied
        assert node.profile == "view" and not node._reopen
        assert (node.width, node.height) == (320, 240)
    ok = node.set_parameters_atomically([Parameter("profile", value="detail")])
    assert ok.successful and node.profile == "detail"
    rig.close()


def test_camera_ignoring_the_mode_is_reported_with_actual_size():
    from sensor_msgs.msg import CameraInfo

    rig = Rig(ignore_mode=(b"MJPG", 1280, 720))  # asked 320x240, camera delivers 720p
    node = rig.node
    infos = []
    node.create_subscription(CameraInfo, "camera_info", infos.append, 10)
    node.start()
    assert rig.spin_until(lambda: infos)
    assert node.mode_mismatch and node.actual_size == (1280, 720)
    assert (infos[0].width, infos[0].height) == (1280, 720)  # never the requested 320x240
    rig.close()


def test_camera_with_wrong_fourcc_is_a_mismatch():
    rig = Rig(ignore_mode=(b"YUYV", 320, 240))
    rig.node.create_subscription(
        __import__("sensor_msgs.msg", fromlist=["CameraInfo"]).CameraInfo,
        "camera_info", lambda m: None, 10)
    rig.node.start()
    assert rig.spin_until(lambda: rig.opens)
    assert rig.node.mode_mismatch
    rig.close()


def test_matching_mode_is_no_mismatch():
    from sensor_msgs.msg import CameraInfo

    rig = Rig()
    rig.node.create_subscription(CameraInfo, "camera_info", lambda m: None, 10)
    rig.node.start()
    assert rig.spin_until(lambda: rig.opens)
    time.sleep(0.1)
    assert not rig.node.mode_mismatch
    rig.close()


def test_missing_device_does_not_log_a_mode_mismatch():
    """Minor B: a capture that did not open has no mode to compare."""
    from sensor_msgs.msg import CameraInfo

    rig = Rig()
    node = rig.node
    state = {"n": 0}

    class Closed(FakeCap):
        def isOpened(self):
            return False

    def open_fn():
        state["n"] += 1
        return Closed() if state["n"] <= 2 else FakeCap(0.01, (b"MJPG", 320, 240))

    node._open_fn = open_fn
    calls = []
    orig = node._info_for_open
    node._info_for_open = lambda cap: calls.append(cap) or orig(cap)
    node.create_subscription(CameraInfo, "camera_info", lambda m: None, 10)
    node.start()
    assert rig.spin_until(lambda: state["n"] >= 3 and node.capturing, timeout=10)
    assert len(calls) == 1 and calls[0].isOpened()  # only the open capture
    assert not node.mode_mismatch
    rig.close()
