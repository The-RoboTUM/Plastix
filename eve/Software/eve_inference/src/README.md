## 1. `project_paths.py`

**Purpose:** Centralizes project paths to make scripts portable.

**Features:**
- Computes `ROOT_DIR` dynamically based on its location.
- Defines paths for:
  - Train Dataset
  - Validation Dataset
  - SegFormer weights
  - YOLO weights
  - Class CSV file
  - Config.yaml
  - Output image folders for segmentation, YOLO, and combined results
- Adds `ROOT_DIR` to `sys.path` so all scripts can import packages from the project root.