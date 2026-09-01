import json
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles, QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import String


class DetectorNode(Node):
    """Subscribes to camera frames, runs YOLO detection + AprilTag/normalized
    localization + tracking, and publishes 2D trash positions.

    Topics published (both geometry_msgs/PoseArray, z = 0):
      * ``~/detections``  every processed frame: all current detection positions.
      * ``~/confirmed``   only when trash gets confirmed: the full set of
        confirmed (settled) trash positions. Latched (transient-local) so a
        subscriber that joins late still receives the latest confirmed map.
      * ``~/debug_image/compressed`` current camera frame annotated with boxes.
      * ``~/detections_debug`` JSON payload with bbox/confidence/class/id/u/v.

    Position units depend on the ``tags`` parameter: world coordinates when an
    AprilTag CSV is supplied, otherwise normalized image coordinates in [0, 1]
    with (0, 0) = bottom-left.
    """

    def __init__(self):
        super().__init__('detector_node')

        # Path to the detect-and-localize folder that holds the shared pipeline.
        # It lives in the same repo (Octopus/detect-and-localize); this default only
        # covers the documented clone location, so SETUP.md passes the parameter
        # explicitly and any other clone path must do the same.
        default_repo = os.path.expanduser('~/PlastiX/Octopus/detect-and-localize')
        self.declare_parameter('detect_localize_path', default_repo)
        self.declare_parameter('model', 'data/models/thousand_11s.pt')
        self.declare_parameter('thresh', 0.6)
        self.declare_parameter('tags', '')  # empty -> normalized image coordinates
        self.declare_parameter('yolo_frameskip', 6)
        self.declare_parameter('max_lost', 10)
        self.declare_parameter('confirm_frames', 10)
        self.declare_parameter('dist_thresh', 0.05)
        self.declare_parameter('move_thresh', 0.04)
        self.declare_parameter('input_topic', 'camera/image_raw/compressed')
        self.declare_parameter('output_frame', 'map')
        self.declare_parameter('show_ui', False)

        # Republish the latest confirmed set periodically so downstream mapping
        # stays live. Set <= 0 to disable periodic republishing.
        self.declare_parameter('confirmed_republish_period_sec', 1.0)

        # JPEG quality of ~/debug_image/compressed, the frame the dashboard shows.
        # The default keeps the previous bandwidth. Raise it (92-98) when the feed
        # looks blocky: the camera is only 640x480, so the dashboard magnifies every
        # pixel and the 8x8 compression blocks become clearly visible.
        self.declare_parameter('debug_image_jpeg_quality', 80)

        repo_path = self.get_parameter('detect_localize_path').value
        self.output_frame = self.get_parameter('output_frame').value
        input_topic = self.get_parameter('input_topic').value
        self.show_ui = self.get_parameter('show_ui').value
        self.debug_image_jpeg_quality = max(
            1, min(100, int(self.get_parameter('debug_image_jpeg_quality').value))
        )

        # Make the shared pipeline importable, then build it.
        if repo_path not in sys.path:
            sys.path.insert(0, repo_path)
        try:
            from src.pipeline import DetectAndLocalize
            from src.visualization import Visualizer
            from src.fps_meter import FPSMeter
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

        self.visualizer = Visualizer() if self.show_ui else None
        self._fps_meter = FPSMeter(avg_len=100) if self.show_ui else None
        self._world_viewport = None  # (center, scale) computed once from tag positions

        # Live detections: reliable so standard tools/consumers receive them.
        self.detections_pub = self.create_publisher(PoseArray, '~/detections', 10)
        # Confirmed trash: latched so late joiners get the current map.
        latched_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.confirmed_pub = self.create_publisher(PoseArray, '~/confirmed', latched_qos)

        # Additional debug outputs for the dashboard Detection Inspector.
        # Do not change ~/confirmed: the flight transform depends on it.
        self.debug_image_pub = self.create_publisher(
            CompressedImage, '~/debug_image/compressed', 10
        )
        self.detections_debug_pub = self.create_publisher(String, '~/detections_debug', 10)
        self._next_debug_detection_id = 1
        self._temporary_debug_ids = {}

        self._latest_confirmed_pose_array = None
        self.confirmed_republish_period_sec = float(
            self.get_parameter('confirmed_republish_period_sec').value
        )
        self.confirmed_republish_timer = None
        if self.confirmed_republish_period_sec > 0.0:
            self.confirmed_republish_timer = self.create_timer(
                self.confirmed_republish_period_sec,
                self._republish_confirmed_detections,
            )

        self.subscription = self.create_subscription(
            CompressedImage, input_topic, self.image_callback,
            QoSPresetProfiles.SENSOR_DATA.value,
        )

        self.get_logger().info(
            f"Detector ready. Subscribing '{input_topic}', model '{model}', "
            f"{'world coords (tags)' if tags else 'normalized image coords'}, "
            f"output frame '{self.output_frame}'."
        )

        if self.show_ui:
            cv2.namedWindow('Frame', cv2.WINDOW_NORMAL)
            cv2.namedWindow('World Map', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('Frame', 960, 540)
            cv2.resizeWindow('World Map', 600, 600)
            blank = np.ones((540, 960, 3), np.uint8) * 60
            cv2.putText(blank, 'Waiting for stream...', (280, 270),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (180, 180, 180), 2)
            cv2.imshow('Frame', blank)
            cv2.imshow('World Map', np.ones((600, 600, 3), np.uint8) * 60)
            for _ in range(10):
                cv2.waitKey(50)

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


    def _republish_confirmed_detections(self):
        if self._latest_confirmed_pose_array is None:
            return
        if not self._latest_confirmed_pose_array.poses:
            return

        self._latest_confirmed_pose_array.header.stamp = self.get_clock().now().to_msg()
        self.confirmed_pub.publish(self._latest_confirmed_pose_array)

    @staticmethod
    def _stamp_to_float(stamp):
        try:
            value = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        except Exception:
            value = 0.0
        return value if value > 0.0 else time.time()

    @staticmethod
    def _as_list(value):
        if value is None:
            return []
        if isinstance(value, dict):
            return list(value.values())
        if isinstance(value, (list, tuple)):
            return list(value)
        return [value]

    @staticmethod
    def _field(item, *names, default=None):
        if item is None:
            return default
        if isinstance(item, dict):
            for name in names:
                if name in item:
                    return item[name]
            return default
        for name in names:
            if hasattr(item, name):
                return getattr(item, name)
        return default

    @staticmethod
    def _plain_float(value):
        try:
            number = float(value)
        except Exception:
            return None
        if not np.isfinite(number):
            return None
        return number

    def _bbox_to_dict(self, raw_bbox, frame_shape):
        if raw_bbox is None:
            return None

        frame_h, frame_w = frame_shape[:2]
        x1 = y1 = x2 = y2 = None

        if isinstance(raw_bbox, dict):
            x1 = self._plain_float(self._field(raw_bbox, "x1", "xmin", "left"))
            y1 = self._plain_float(self._field(raw_bbox, "y1", "ymin", "top"))
            x2 = self._plain_float(self._field(raw_bbox, "x2", "xmax", "right"))
            y2 = self._plain_float(self._field(raw_bbox, "y2", "ymax", "bottom"))

            width = self._plain_float(self._field(raw_bbox, "width", "w"))
            height = self._plain_float(self._field(raw_bbox, "height", "h"))
            if x2 is None and x1 is not None and width is not None:
                x2 = x1 + width
            if y2 is None and y1 is not None and height is not None:
                y2 = y1 + height
        else:
            try:
                arr = np.array(raw_bbox, dtype=float).reshape(-1).tolist()
            except Exception:
                arr = []
            if len(arr) >= 4:
                x1, y1, x2, y2 = arr[:4]

        if None in (x1, y1, x2, y2):
            return None

        x1 = max(0.0, min(float(frame_w - 1), float(x1)))
        x2 = max(0.0, min(float(frame_w - 1), float(x2)))
        y1 = max(0.0, min(float(frame_h - 1), float(y1)))
        y2 = max(0.0, min(float(frame_h - 1), float(y2)))

        if x2 < x1:
            x1, x2 = x2, x1
        if y2 < y1:
            y1, y2 = y2, y1

        return {
            "x1": int(round(x1)),
            "y1": int(round(y1)),
            "x2": int(round(x2)),
            "y2": int(round(y2)),
            "width": int(round(x2 - x1)),
            "height": int(round(y2 - y1)),
        }

    def _candidate_bbox(self, item, frame_shape):
        raw_bbox = self._field(
            item,
            "bbox", "box", "xyxy", "rect", "rectangle", "tlbr",
            default=None,
        )
        if raw_bbox is None:
            raw_bbox = self._field(item, "bounds", "bounding_box", default=None)
        return self._bbox_to_dict(raw_bbox, frame_shape)

    def _candidate_uv(self, item, bbox, frame_shape):
        frame_h, frame_w = frame_shape[:2]

        if bbox:
            cx = 0.5 * (bbox["x1"] + bbox["x2"])
            cy = 0.5 * (bbox["y1"] + bbox["y2"])
            return cx / max(1, frame_w), 1.0 - (cy / max(1, frame_h))

        u = self._plain_float(self._field(
            item, "u", "norm_x", "normalized_x", "cx_norm", default=None
        ))
        v = self._plain_float(self._field(
            item, "v", "norm_y", "normalized_y", "cy_norm", default=None
        ))
        if u is not None and v is not None:
            return u, v

        pos = self._field(item, "pos", "position", "center", "uv", default=None)
        if isinstance(pos, dict):
            u = self._plain_float(self._field(pos, "u", "x", 0, default=None))
            v = self._plain_float(self._field(pos, "v", "y", 1, default=None))
        else:
            try:
                arr = np.array(pos, dtype=float).reshape(-1).tolist()
            except Exception:
                arr = []
            if len(arr) >= 2:
                u = self._plain_float(arr[0])
                v = self._plain_float(arr[1])

        if u is None or v is None:
            return None, None
        return u, v

    def _temporary_debug_id(self, class_name, u, v, bbox, index):
        if bbox:
            key = (
                class_name,
                int(bbox["x1"] / 10),
                int(bbox["y1"] / 10),
                int(bbox["x2"] / 10),
                int(bbox["y2"] / 10),
            )
        elif u is not None and v is not None:
            key = (class_name, round(float(u), 2), round(float(v), 2))
        else:
            key = (class_name, "fallback", int(index))

        if key not in self._temporary_debug_ids:
            self._temporary_debug_ids[key] = self._next_debug_detection_id
            self._next_debug_detection_id += 1

        # Keep this debug-only dictionary bounded.
        if len(self._temporary_debug_ids) > 200:
            self._temporary_debug_ids = dict(list(self._temporary_debug_ids.items())[-100:])

        return self._temporary_debug_ids[key]

    def _debug_detection_from_candidate(self, item, index, frame_shape, status="detected"):
        bbox = self._candidate_bbox(item, frame_shape)
        u, v = self._candidate_uv(item, bbox, frame_shape)

        class_name = self._field(
            item,
            "class_name", "class", "label", "name", "category",
            default="rubbish",
        )
        class_name = str(class_name or "rubbish")

        confidence = self._plain_float(self._field(
            item,
            "confidence", "conf", "score", "probability",
            default=None,
        ))

        raw_id = self._field(
            item,
            "id", "track_id", "detection_id", "object_id",
            default=None,
        )
        if raw_id is None:
            raw_id = self._temporary_debug_id(class_name, u, v, bbox, index)

        try:
            detection_id = int(raw_id)
        except Exception:
            detection_id = str(raw_id)

        item_status = self._field(item, "status", "state", default=status)

        return {
            "id": detection_id,
            "class_name": class_name,
            "confidence": confidence,
            "u": self._plain_float(u),
            "v": self._plain_float(v),
            "bbox": bbox,
            "status": str(item_status or status),
            # TODO: propagate detection ID through flight_camera_transform_node.
            # TODO: join camera debug detections with map/world detections in dashboard.
        }

    def _merge_distance(self):
        """Radius (in normalized u/v units) for matching a confirmed track to the
        raw detection it belongs to. The tracker uses the same threshold to decide
        whether a detection updates an existing track, so anything further apart is
        a different object as far as the tracker is concerned.

        Only meaningful in normalized mode: with AprilTags the track position is in
        world coordinates while the detection u/v stays in image space, so no match
        is found and confirmed tracks simply do not appear on the camera feed."""
        tracker = getattr(self.pipeline, "tracker", None)
        return float(getattr(tracker, "dist_thresh", 0.05) or 0.05)

    @staticmethod
    def _nearest_detection(track_det, candidates, radius, claimed=()):
        """Closest unclaimed candidate within `radius` of a confirmed track, or None."""
        tu, tv = track_det.get("u"), track_det.get("v")
        if tu is None or tv is None:
            return None

        best = None
        best_dist = radius
        for candidate in candidates:
            if id(candidate) in claimed:
                continue
            cu, cv = candidate.get("u"), candidate.get("v")
            if cu is None or cv is None:
                continue
            dist = float(np.hypot(cu - tu, cv - tv))
            if dist <= best_dist:
                best = candidate
                best_dist = dist
        return best

    def _build_debug_detections(self, result, frame_shape):
        confirmed_items = self._as_list(result.get("confirmed", []))
        confirmed_debug = [
            self._debug_detection_from_candidate(item, index, frame_shape, "confirmed")
            for index, item in enumerate(confirmed_items)
        ]

        current_items = self._as_list(result.get("detections", None))
        if not current_items:
            current_items = self._as_list(getattr(self.pipeline, "detections", []))

        current_debug = [
            self._debug_detection_from_candidate(item, index, frame_shape, "detected")
            for index, item in enumerate(current_items)
        ]

        if current_debug:
            # A confirmed track and the YOLO detection it came from are the same
            # physical object, but the track carries only a locked position — no
            # bbox, no confidence — and its own tracker id, so emitting both drew a
            # second, confidence-less box next to the real one. Fold each track into
            # the nearest detection and keep the (stable) tracker id. A track with no
            # detection nearby is a leftover the tracker holds for `max_lost` frames;
            # ~/confirmed still carries it to the map, so it is dropped here.
            radius = self._merge_distance()
            claimed = set()
            for det in confirmed_debug:
                nearest = self._nearest_detection(det, current_debug, radius, claimed)
                if nearest is None:
                    continue
                claimed.add(id(nearest))
                nearest["id"] = det["id"]
                nearest["status"] = "confirmed"
            return current_debug

        if confirmed_debug:
            return confirmed_debug

        # Fallback: use the same positions that feed ~/detections. In normalized
        # mode these are u/v. In tag/world mode bbox may be unavailable here.
        fallback = []
        for index, pos in enumerate(result.get("detections_world", [])):
            item = {"pos": pos, "class_name": "rubbish", "status": "detected"}
            fallback.append(self._debug_detection_from_candidate(item, index, frame_shape, "detected"))
        return fallback

    def _publish_debug_outputs(self, image_msg, frame, result, stamp):
        try:
            timestamp = self._stamp_to_float(stamp)
            frame_id = image_msg.header.frame_id or "camera"
            frame_h, frame_w = frame.shape[:2]
            detections = self._build_debug_detections(result, frame.shape)

            # No overlays are burned into the debug frame — the dashboard draws both
            # the bounding box and the label from the detections payload below. The
            # detector only ships the raw camera frame plus the detection data.
            debug_frame = result.get("annotated")
            debug_frame = debug_frame.copy() if debug_frame is not None else frame.copy()

            payload = {
                "source_id": "detector_node",
                "frame_id": frame_id,
                "timestamp": timestamp,
                "image_width": int(frame_w),
                "image_height": int(frame_h),
                "detections": detections,
            }
            debug_json_msg = String()
            debug_json_msg.data = json.dumps(payload, separators=(",", ":"))
            self.detections_debug_pub.publish(debug_json_msg)

            ok, encoded = cv2.imencode(
                ".jpg",
                debug_frame,
                [int(cv2.IMWRITE_JPEG_QUALITY), self.debug_image_jpeg_quality],
            )
            if not ok:
                self.get_logger().warn("Failed to encode detector debug image")
                return

            debug_image_msg = CompressedImage()
            debug_image_msg.header = image_msg.header
            debug_image_msg.format = "jpeg"
            debug_image_msg.data = encoded.tobytes()
            self.debug_image_pub.publish(debug_image_msg)
        except Exception as exc:
            self.get_logger().warn(f"Failed to publish camera debug output: {exc}")


    def image_callback(self, msg):
        # Decode the JPEG CompressedImage directly with OpenCV. This avoids
        # cv_bridge, whose native extension is built against a different NumPy
        # ABI than the venv and crashes when the two are mixed.
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warn('Failed to decode frame')
            return

        result = self.pipeline.process(frame, draw=True)
        stamp = msg.header.stamp

        self.detections_pub.publish(
            self._to_pose_array(result['detections_world'], stamp)
        )

        # Always assign, including the empty case. The tracker already smooths over
        # frames where YOLO missed -- update() returns every confirmed track whose
        # "missed" count is still below max_lost -- so an empty list here means the
        # object is really gone, not merely unseen this frame. Guarding the
        # assignment on a non-empty list instead froze the last sighting into
        # _latest_confirmed_pose_array, and the 1 Hz republish timer then re-sent it
        # with a fresh stamp forever: a picked-up or removed object stayed a live
        # target, and any downstream age check (trash_gps_goal_node's
        # target_ttl_sec) could never fire because last_seen kept refreshing.
        confirmed_positions = [t['pos'] for t in result.get('confirmed', [])]
        self._latest_confirmed_pose_array = self._to_pose_array(confirmed_positions, stamp)

        if result['new_confirmed']:
            if self._latest_confirmed_pose_array is not None:
                self.confirmed_pub.publish(self._latest_confirmed_pose_array)
            for t in result['new_confirmed']:
                pos = t['pos']
                self.get_logger().info(
                    f"Confirmed rubbish #{t['id']} at {self.pipeline.coord_label} "
                    f"({pos[0]:.3f}, {pos[1]:.3f})"
                )

        self._publish_debug_outputs(msg, frame, result, stamp)

        if self.show_ui:
            self._show_ui(result)


    def _show_ui(self, result):
        if self._world_viewport is None:
            if self.pipeline.use_tags and self.pipeline.tags_world:
                self._world_viewport = self.visualizer.compute_viewport(
                    self.pipeline.tags_world
                )
            else:
                self._world_viewport = (
                    np.array([0.5, 0.5]),
                    self.visualizer.canvas_size - 2 * self.visualizer.margin,
                )

        center, scale = self._world_viewport
        canvas = self.visualizer.draw_canvas(
            tag_positions=self.pipeline.tags_world,
            detections_world=result['detections_world'],
            confirmed_tracks=self.pipeline.confirmed_tracks,
            world_center=center,
            scale=scale,
        )

        _, _ = self._fps_meter.update()
        if result['yolo_ran'] and result['annotated'] is not None:
            self._fps_meter.draw_on_frame(
                result['annotated'],
                f"Objects: {len(self.pipeline.detections)}",
            )

        if result['annotated'] is not None:
            cv2.imshow('Frame', result['annotated'])
        cv2.imshow('World Map', canvas)
        cv2.waitKey(1)


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
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
