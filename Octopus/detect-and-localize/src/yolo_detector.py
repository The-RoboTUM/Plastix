import cv2
from ultralytics import YOLO

class YOLODetector:
    def __init__(self, model_path, conf_thresh=0.6):
        self.model = YOLO(model_path)
        self.labels = self.model.names
        self.conf_thresh = conf_thresh

    def detect(self, frame, draw=False):
        """
        Run YOLO inference on a frame.
        Returns: list of dicts {class, conf, bbox}, and optionally the annotated frame.
        """
        results = self.model(frame, verbose=False)
        annotated = frame.copy()
        detections = []

        for box in results[0].boxes:
            conf = box.conf.item()
            if conf < self.conf_thresh:
                continue
            cls = int(box.cls.item())
            label = self.labels[cls]
            xyxy = box.xyxy.cpu().numpy().squeeze().astype(int)
            detections.append({
                "class": label,
                "conf": conf,
                "bbox": xyxy
            })

            # Boxes/labels are no longer burned into the frame here. The dashboard
            # draws its own (yellow) box, and detector_node draws a single green
            # label in _draw_debug_overlays, so every detection is framed once
            # (dashboard) and labelled once (detector) without duplication.

        if draw:
            return detections, annotated
        return detections, frame
