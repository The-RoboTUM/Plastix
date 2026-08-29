#!/usr/bin/env python3
"""
Standalone test camera feed for the Octopus dashboard — no drone, no ROS.

Replaces the whole ROS pipeline (camera_node + detector_node + backend bridge):
reads an image, video file, or webcam, optionally runs YOLO detection, and POSTs
frames + detections to the dashboard backend's /api/camera_debug endpoints.

Examples
--------
  # feed only (no boxes), a single still image, looped:
  python test_camera_feed.py --source test.jpg

  # feed + fake demo boxes (no ML needed) — good to test overlay + map projection:
  python test_camera_feed.py --source test.jpg --demo

  # feed + REAL YOLO detection on a video clip:
  python test_camera_feed.py --source clip.mp4 --model data/models/best_model_10_08_26.pt

  # feed + REAL YOLO from a webcam:
  python test_camera_feed.py --source 0 --model data/models/best_model_10_08_26.pt

The backend keeps only the newest frame/detections in memory, so this script just
needs to keep POSTing. Open http://127.0.0.1:8000/dashboard.html to see it live.
"""
import argparse
import base64
import json
import sys
import time
import urllib.request

import cv2

IMAGE_EXTS = (".jpg", ".jpeg", ".png", ".bmp", ".webp")


def post_json(url, payload, timeout=2.0):
    data = json.dumps(payload).encode()
    req = urllib.request.Request(
        url, data=data, headers={"Content-Type": "application/json"}, method="POST"
    )
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return resp.read()


def detections_from_yolo(model, frame, conf_thresh, class_name):
    """Run YOLO and convert boxes to the dashboard's detection format.

    The dashboard expects:
      - bbox in image pixels, top-left origin
      - u = center_x / width   (0..1, left->right)
      - v = 1 - center_y / height  (0..1, BOTTOM-left origin; the detector's convention)
    """
    height, width = frame.shape[:2]
    results = model(frame, verbose=False)
    dets = []
    for index, box in enumerate(results[0].boxes):
        conf = float(box.conf.item())
        if conf < conf_thresh:
            continue
        x1, y1, x2, y2 = box.xyxy.cpu().numpy().squeeze().astype(int).tolist()
        cx = 0.5 * (x1 + x2)
        cy = 0.5 * (y1 + y2)
        try:
            label = model.names[int(box.cls.item())]
        except Exception:
            label = class_name
        dets.append(
            {
                "id": index,
                "class_name": label or class_name,
                "confidence": conf,
                "u": cx / width,
                "v": 1.0 - cy / height,
                "bbox": {"x1": x1, "y1": y1, "x2": x2, "y2": y2,
                         "width": x2 - x1, "height": y2 - y1},
                "status": "detected",
            }
        )
    return dets


def demo_detections(frame, class_name):
    """Static fake boxes (no ML). Positions are fractions of the frame."""
    height, width = frame.shape[:2]
    specs = [
        # (center_x_frac, center_y_frac, box_w_frac, box_h_frac, confidence)
        (0.30, 0.40, 0.12, 0.16, 0.88),
        (0.62, 0.55, 0.10, 0.12, 0.74),
        (0.78, 0.30, 0.08, 0.10, 0.55),
    ]
    dets = []
    for index, (fx, fy, fw, fh, conf) in enumerate(specs):
        cx, cy = fx * width, fy * height
        bw, bh = fw * width, fh * height
        x1, y1 = int(cx - bw / 2), int(cy - bh / 2)
        x2, y2 = int(cx + bw / 2), int(cy + bh / 2)
        dets.append(
            {
                "id": index,
                "class_name": class_name,
                "confidence": conf,
                "u": cx / width,
                "v": 1.0 - cy / height,
                "bbox": {"x1": x1, "y1": y1, "x2": x2, "y2": y2,
                         "width": x2 - x1, "height": y2 - y1},
                "status": "detected",
            }
        )
    return dets


def frame_source(source):
    """Yield frames forever. Image -> same frame repeated. Video/webcam -> looped."""
    if source.isdigit():
        source = int(source)

    if isinstance(source, str) and source.lower().endswith(IMAGE_EXTS):
        img = cv2.imread(source)
        if img is None:
            sys.exit(f"Could not read image: {source}")
        while True:
            yield img
        return

    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        sys.exit(f"Could not open video/webcam source: {source}")
    while True:
        ok, frame = cap.read()
        if not ok:
            # end of video file -> rewind and loop
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ok, frame = cap.read()
            if not ok:
                break
        yield frame


def main():
    ap = argparse.ArgumentParser(description="Test camera feed for the Octopus dashboard.")
    ap.add_argument("--source", required=True,
                    help="image path, video path, or webcam index (e.g. 0)")
    ap.add_argument("--backend", default="http://127.0.0.1:8000",
                    help="dashboard backend base URL")
    ap.add_argument("--model", default="",
                    help="YOLO .pt path for real detection; empty = no YOLO")
    ap.add_argument("--demo", action="store_true",
                    help="post static fake detection boxes (no ML needed)")
    ap.add_argument("--conf", type=float, default=0.25, help="YOLO confidence threshold")
    ap.add_argument("--fps", type=float, default=5.0, help="post rate (frames per second)")
    ap.add_argument("--class-name", default="rubbish", help="fallback class label")
    ap.add_argument("--jpeg-quality", type=int, default=80)
    args = ap.parse_args()

    base = args.backend.rstrip("/")
    frame_url = base + "/api/camera_debug/frame"
    det_url = base + "/api/camera_debug/detections"

    model = None
    if args.model:
        from ultralytics import YOLO  # imported only when needed
        print(f"Loading YOLO model: {args.model}")
        model = YOLO(args.model)

    period = 1.0 / max(0.1, args.fps)
    print(f"Posting to {base}  (source={args.source}, "
          f"mode={'yolo' if model else 'demo' if args.demo else 'feed-only'})")
    print("Open http://127.0.0.1:8000/dashboard.html  ·  Ctrl+C to stop")

    n = 0
    for frame in frame_source(args.source):
        ok, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, args.jpeg_quality])
        if not ok:
            continue
        b64 = base64.b64encode(jpg.tobytes()).decode("ascii")

        if model is not None:
            dets = detections_from_yolo(model, frame, args.conf, args.class_name)
        elif args.demo:
            dets = demo_detections(frame, args.class_name)
        else:
            dets = []

        try:
            post_json(frame_url, {"format": "jpeg", "frame_id": "test_cam", "data_base64": b64})
            post_json(det_url, {"frame_id": "test_cam", "detections": dets})
        except Exception as exc:
            print(f"POST failed (is the backend running on {base}?): {exc}")
            time.sleep(1.0)
            continue

        n += 1
        if n % 20 == 0:
            print(f"posted {n} frames · {len(dets)} detections in last frame")
        time.sleep(period)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped.")
