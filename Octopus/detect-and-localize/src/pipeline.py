import cv2
import numpy as np

from src.apriltag_utils import load_tag_positions, detect_tags, estimate_homography
from src.yolo_detector import YOLODetector
from src.tracking import GarbageTracker


class DetectAndLocalize:
    """Per-frame detection + localization + tracking pipeline.

    The core logic lives here so the CLI (main.py) and the ROS node share a
    single implementation. Each call to ``process()`` runs YOLO (respecting the
    configured frameskip), maps the "rubbish" detections into a 2D coordinate
    space, and feeds them to the tracker.

    Coordinate space:
      * With an AprilTag CSV (``tags``): detections are projected into world
        coordinates via the tag homography. If fewer than 4 tags are visible
        the homography is unavailable and no positions are produced that frame.
      * Without tags: detections use normalized image coordinates where
        (0, 0) = bottom-left and (1, 1) = top-right.
    """

    def __init__(self, model, thresh=0.6, tags=None, yolo_frameskip=0,
                 max_lost=100, confirm_frames=10, dist_thresh=0.05,
                 move_thresh=0.04):
        self.yolo = YOLODetector(model, thresh)
        self.yolo_frameskip = yolo_frameskip
        self.tracker = GarbageTracker(
            max_lost=max_lost,
            confirm_frames=confirm_frames,
            dist_thresh=dist_thresh,
            move_thresh=move_thresh,
        )

        tags = tags or None  # treat empty string as "no tags"
        self.use_tags = tags is not None
        self.tags_world = load_tag_positions(tags) if self.use_tags else {}

        # rolling per-frame state
        self.frame_count = 0
        self.detections = []          # last raw YOLO detections
        self.frame_annotated = None   # last annotated frame
        self.confirmed_tracks = {}    # id -> resting position of confirmed trash

    @property
    def coord_label(self):
        return "world pos" if self.use_tags else "image pos"

    def process(self, frame, draw=True):
        """Run one frame through the pipeline.

        Returns a dict:
          detections_world : list[np.ndarray]  all current detection positions
          confirmed        : list[dict]        every confirmed track
          new_confirmed    : list[dict]        tracks confirmed on this frame
          annotated        : np.ndarray | None annotated frame when ``draw``
          yolo_ran         : bool              whether YOLO ran this frame
        """
        H = None
        if self.use_tags:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            tag_dets = detect_tags(gray)
            H = estimate_homography(tag_dets, self.tags_world)

        # YOLO only every (frameskip + 1) frames; reuse the last detections
        # in between so the tracker still sees a steady stream.
        process_yolo = (self.yolo_frameskip == 0) or \
            (self.frame_count % (self.yolo_frameskip + 1) == 0)

        if process_yolo:
            self.detections, self.frame_annotated = self.yolo.detect(frame, draw=draw)
        elif self.frame_annotated is None:
            self.frame_annotated = frame.copy()

        self.frame_count += 1

        detections_world = self._to_world(frame, H)
        confirmed, new_conf = self.tracker.update(detections_world)
        for t in new_conf:
            self.confirmed_tracks[t["id"]] = t["pos"]

        return {
            "detections_world": detections_world,
            "confirmed": confirmed,
            "new_confirmed": new_conf,
            "annotated": self.frame_annotated if draw else None,
            "yolo_ran": process_yolo,
        }

    def _to_world(self, frame, H):
        """Map the current rubbish detections into the output coordinate space."""
        detections_world = []
        if self.use_tags:
            if H is None:
                return detections_world
            for det in self.detections:
                if det["class"] != "rubbish":
                    continue
                u, v = self._bbox_center(det["bbox"])
                pt = np.array([[[u, v]]], np.float32)
                pt_w = cv2.perspectiveTransform(pt, H)
                detections_world.append(pt_w[0, 0])
        else:
            frame_h, frame_w = frame.shape[:2]
            for det in self.detections:
                if det["class"] != "rubbish":
                    continue
                u, v = self._bbox_center(det["bbox"])
                nx = u / frame_w
                ny = 1.0 - v / frame_h  # flip y so (0,0) is bottom-left
                detections_world.append(np.array([nx, ny]))
        return detections_world

    @staticmethod
    def _bbox_center(bbox):
        xmin, ymin, xmax, ymax = bbox
        return (xmin + xmax) / 2, (ymin + ymax) / 2
