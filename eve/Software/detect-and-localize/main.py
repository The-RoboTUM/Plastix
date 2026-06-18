import argparse
import cv2
import numpy as np
import ncnn
import yaml
import argparse
import os

from src.camera_source import CameraSource
from src.apriltag_utils import load_tag_positions, detect_tags, estimate_homography
from src.yolo_detector import YOLODetector
from src.tracking import GarbageTracker
from src.visualization import Visualizer
from src.fps_meter import FPSMeter

script_dir = os.path.dirname(os.path.abspath(__file__))


def load_preset(preset_name):
    preset_path = os.path.join(script_dir, r"presets", f"{preset_name}.yaml") 
    if not os.path.exists(preset_path):
        print(f"⚠️ Preset '{preset_name}' not found at {preset_path}")
        return {}   # <— wichtig: leeres Dict zurückgeben
    with open(preset_path, 'r') as f:
        return yaml.safe_load(f) or {}

parser = argparse.ArgumentParser()
parser.add_argument('--preset', default='default', help='Name of preset YAML in ./presets/')
parser.add_argument('--model', help='Path to YOLO model file')
parser.add_argument('--source', help='Input source (camera, folder, etc.)')
parser.add_argument('--thresh', type=float, help='Confidence threshold')
parser.add_argument('--tags', default=None, help='.CSV file with AprilTag positions (optional; omit to use normalized image coordinates)')
parser.add_argument('--yolo_frameskip', type=int, help='Number of frames to skip between YOLO inferences')

args = parser.parse_args()

# load preset
preset = load_preset(args.preset)

# override with command line args if necessary
config = {**preset, **{k: v for k, v in vars(args).items() if v is not None}}

path_keys = ["model", "tags"]  # adjust to your YAML

for key in path_keys:
    if key in config and config[key] and not os.path.isabs(config[key]):
        config[key] = os.path.join(script_dir, config[key])

print("🔧 Loaded configuration:")
for k, v in config.items():
    print(f"  {k}: {v}")

yolo = YOLODetector(config["model"], config["thresh"])
yolo_frameskip = config.get("yolo_frameskip", 0)
camera = CameraSource(config["source"])
viz = Visualizer()
tracker = GarbageTracker(max_lost=100, confirm_frames=10, dist_thresh=0.2)

tags_csv = config.get("tags") or None  # treat empty string as no tags
use_tags = tags_csv is not None
if use_tags:
    tags_world = load_tag_positions(tags_csv)
    world_center, scale = viz.compute_viewport(tags_world)
else:
    tags_world = {}
    world_center = np.array([0.5, 0.5])
    scale = viz.canvas_size - 2 * viz.margin
    print("ℹ️  No tags file provided — using normalized image coordinates (0,0)=bottom-left, (1,1)=top-right")

confirmed_tracks = {}
frame_count = 0
frame_annotated = None
detections = []
fps_meter = FPSMeter(avg_len=100)


while True:
    frame = camera.read()
    if frame is None:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tag_dets = detect_tags(gray)
    H = estimate_homography(tag_dets, tags_world)

    # YOLO only every N frames
    process_yolo = (yolo_frameskip == 0) or (frame_count % (yolo_frameskip + 1) == 0)

    if process_yolo:
        detections, frame_annotated = yolo.detect(frame, draw=True)
    elif frame_annotated is None:
        frame_annotated = frame.copy()

    frame_count += 1

    detections_world = []
    if use_tags and H is not None:
        for det in detections:
            xmin, ymin, xmax, ymax = det["bbox"]
            u, v = (xmin + xmax) / 2, (ymin + ymax) / 2
            if det["class"] == "rubbish":
                pt = np.array([[[u, v]]], np.float32)
                pt_w = cv2.perspectiveTransform(pt, H)
                detections_world.append(pt_w[0, 0])
    elif not use_tags:
        frame_h, frame_w = frame.shape[:2]
        for det in detections:
            xmin, ymin, xmax, ymax = det["bbox"]
            u, v = (xmin + xmax) / 2, (ymin + ymax) / 2
            if det["class"] == "rubbish":
                nx = u / frame_w
                ny = 1.0 - v / frame_h  # flip so (0,1)=top-left, (1,0)=bottom-right
                detections_world.append(np.array([nx, ny]))

    confirmed, new_conf = tracker.update(detections_world)
    for t in new_conf:
        confirmed_tracks[t["id"]] = t["pos"]
        tid = t["id"]
        pos = t["pos"]
        label = "image pos" if not use_tags else "world pos"
        print(f"✅ New confirmed rubbish #{tid} at {label} ({pos[0]:.3f}, {pos[1]:.3f})")

    canvas = viz.draw_canvas(tags_world, detections_world, confirmed_tracks, world_center, scale)
    
    fps, avg_fps = fps_meter.update()
    if process_yolo:
        fps_meter.draw_on_frame(frame_annotated, f"Objects: {len(detections)}")

    cv2.imshow("Frame", frame_annotated)
    cv2.imshow("World Map", canvas)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

camera.release()
cv2.destroyAllWindows()
