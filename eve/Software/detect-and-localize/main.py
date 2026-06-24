import argparse
import cv2
import numpy as np
import yaml
import os
import time

from src.camera_source import CameraSource
from src.pipeline import DetectAndLocalize
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

# The detection/localization/tracking logic lives in DetectAndLocalize so it is
# shared with the ROS node; main.py only owns the CLI source + visualization.
pipeline = DetectAndLocalize(
    model=config["model"],
    thresh=config["thresh"],
    tags=config.get("tags") or None,
    yolo_frameskip=config.get("yolo_frameskip", 0),
    max_lost=100,
    confirm_frames=10,
    dist_thresh=0.05,
    move_thresh=0.04,
)

camera = CameraSource(config["source"])
viz = Visualizer()
fps_meter = FPSMeter(avg_len=100)

if pipeline.use_tags:
    world_center, scale = viz.compute_viewport(pipeline.tags_world)
else:
    world_center = np.array([0.5, 0.5])
    scale = viz.canvas_size - 2 * viz.margin
    print("ℹ️  No tags file provided — using normalized image coordinates (0,0)=bottom-left, (1,1)=top-right")


while True:
    frame = camera.read()
    if frame is None:
        os.system('pause') # avoid busy loop if source is temporarily unavailable
        break

    result = pipeline.process(frame, draw=True)

    for t in result["new_confirmed"]:
        pos = t["pos"]
        print(f"✅ New confirmed rubbish #{t['id']} at {pipeline.coord_label} "
              f"({pos[0]:.3f}, {pos[1]:.3f})")

    canvas = viz.draw_canvas(
        pipeline.tags_world, result["detections_world"],
        pipeline.confirmed_tracks, world_center, scale,
    )

    fps, avg_fps = fps_meter.update()
    if result["yolo_ran"]:
        fps_meter.draw_on_frame(result["annotated"], f"Objects: {len(pipeline.detections)}")

    cv2.imshow("Frame", result["annotated"])
    cv2.imshow("World Map", canvas)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

camera.release()
cv2.destroyAllWindows()
