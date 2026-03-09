# project_paths.py
from pathlib import Path
import sys

ROOT_DIR = Path(__file__).resolve().parent
if str(ROOT_DIR) not in sys.path:
    sys.path.append(str(ROOT_DIR))

# paths
# Training
TRAIN_DATASET = ROOT_DIR /"AI-Training/Dataset/Dataset_Overfitting/train_2/" 
VALID_DATASET = ROOT_DIR /"AI-Training/Dataset/Dataset_Overfitting/valid_2/"
LOGS_FILE = ROOT_DIR /"AI-Training/logs"

SEG_WEIGHTS = ROOT_DIR / "AI-Training/segformer_weights"
YOLO_WEIGHTS = ROOT_DIR / "AI-Inference/yolo_weights/fotogenic_v8s.pt"
CLASSES_FILE = ROOT_DIR / "AI-Inference/_classes.csv"
CONFIG_COORD_FILE = ROOT_DIR / "AI-Inference/Config.yaml"

SEGMENTATION_IMG = ROOT_DIR / "AI-Inference/Images/segmentation"
YOLO_IMG = ROOT_DIR / "AI-Inference/Images/yolo"
COMBINED_IMG = ROOT_DIR / "AI-Inference/Images/combined"