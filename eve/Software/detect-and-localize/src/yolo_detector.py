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

            if draw:
                color = (0, 255, 0)
                (xmin, ymin, xmax, ymax) = xyxy
                cv2.rectangle(annotated, (xmin, ymin), (xmax, ymax), color, 2)
                text = f"{label} {conf:.2f}"
                (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                cv2.rectangle(annotated, (xmin, ymin - th - 4), (xmin + tw, ymin), color, -1)
                cv2.putText(annotated, text, (xmin, ymin - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 1)

        if draw:
            return detections, annotated
        return detections, frame

