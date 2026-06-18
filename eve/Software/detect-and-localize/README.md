## What this is

Detects and localizes trash from an image, video file, or live camera feed. YOLO models are used for detection.

**Two localization modes:**

- **AprilTag mode** (default): Localization to real-world 2D coordinates via AprilTag markers (`tag16h5` family). At least 4 tags must be visible for the homography to work. Tag layout is configured in a `.csv` file in `data/tags/`. Provide the file with `--tags`.
- **Image coordinate mode**: When no `--tags` file is given, detected trash is reported in normalized image coordinates — `(0, 1)` = top-left, `(1, 0)` = bottom-right, `(0.5, 0.5)` = center. Useful for footage without AprilTags (e.g. outdoor/drone footage).

## Status

Originally intended for **indoor use** with a controlled environment and AprilTag markers. Image coordinate mode extends it to outdoor footage without any tag setup.

## Setup

### 1. Get the data folder

The data folder (models, footage, tag configs) is too large for Git. It is stored separately at Nextcloud:  
`https://nextcloud.itq.de/apps/files/files/2210?dir=/CirQmind%20Plastix%20%28S3%29/Additional%20Content/Eve/code/detect-and-localize`

At the end of this step the directory should contain:
```
data/
presets/
src/
main.py
requirements.txt
README.md
```

### 2. Change to the project directory

```bash
cd /home/robik/PlastiX/eve/Software/detect-and-localize
```

### 3. Create a virtual environment (optional but recommended)

```bash
python3 -m venv .venv
source .venv/bin/activate      # Linux / Mac
# .venv\Scripts\activate       # Windows
```

### 4. Install dependencies

```bash
pip install --upgrade pip
pip install -r requirements.txt
```

> **Note:** `main.py` also imports `ncnn`, which is not in `requirements.txt` and not actually used. If the import fails, install it with `pip install ncnn` or remove the import from `main.py`.

## Running

Presets are YAML files in `presets/` that bundle all arguments. CLI arguments always override preset values.

| Argument | Description |
|---|---|
| `--preset` | Name of a preset YAML in `presets/` (default: `default`) |
| `--model` | Path to YOLO `.pt` model file |
| `--source` | Input source: video file, image folder, `usb<index>` for USB camera, or RTSP URL |
| `--thresh` | Confidence threshold (0.0–1.0) |
| `--tags` | `.csv` file defining the AprilTag positions in world space (optional; omit or pass `""` for image coordinate mode) |
| `--yolo_frameskip` | Run YOLO every N+1 frames (0 = every frame). Useful for performance. |

**Run the default preset (video file demo):**
```bash
python main.py --preset default
```

**Run with a USB camera:**
```bash
python main.py --model data/models/indoor_11s.pt --source usb0 --thresh 0.6 --tags data/tags/tags_whiteboard.csv --yolo_frameskip 2
```

**Run without AprilTags (image coordinate mode):**
```bash
python main.py --preset default --source data/footage/outdoor.mp4 --tags ""
```

Or create a preset without a `tags` key — the mode activates automatically when `tags` is absent.

**Override a single argument from a preset:**
```bash
python main.py --preset default --model data/models/fotogenic_11s.pt
```

Press `Q` to close the OpenCV windows.

## Closing

```bash
deactivate    # deactivate the virtual environment
```

