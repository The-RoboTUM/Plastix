This script contains three main scripts:

## 1. `SemanticSegmentationDataset.py`

**Purpose:** Custom PyTorch Dataset for semantic segmentation.

**Features:**
- Loads images (`.jpg`) and masks (`.png`) from a dataset folder.
- Reads class labels from `_classes.csv`.
- Sorts images and masks to maintain consistent pairing.
- Uses `SegformerImageProcessor` to convert images/masks to tensors.
- Compatible with PyTorch `DataLoader`.
- Currently it uses an Overfitting Dataset for testing...-> exchange it with your Dataset!!!
---

## 2. `SegformerFinetuner.py`

**Purpose:** PyTorch Lightning module for training SegFormer.

**Features:**
- Wraps `SegformerForSemanticSegmentation` with dataset-specific labels.
- Implements training, validation, and test steps.
- Computes metrics: **mean IoU** and **accuracy** using the `evaluate` library.
- Supports TensorBoard logging.
- Includes early stopping and checkpoint callbacks.
- Can save trained model weights for inference.
- Accepts independent `DataLoader` instances for training, validation, and testing.

---

## 3. `inference_pipeline.py` (Training Script)

**Purpose:** Main script to train SegFormer on custom datasets.

**Features:**
- Loads `project_paths.py` for dataset and weight paths.
- Initializes `SegformerImageProcessor` for preprocessing.
- Creates training and validation datasets using `SemanticSegmentationDataset`.
- Sets up `DataLoader` for batching and prefetching.
- Instantiates `SegformerFinetuner` with datasets and metrics configuration.
- Configures `PyTorch Lightning Trainer` with GPU support, early stopping, checkpointing, and TensorBoard logging.
- Trains the model and saves the trained weights.

## 4. `Test-Trained-SegFormer.py` (Evaluation Script in Evaluation)

**Purpose:**  
Standalone script to evaluate a trained SegFormer model on a single image.  
Useful for quickly testing the model after training without running the full training pipeline.
