## inference_pipeline.py + Coordinate Transformation (Planned)

**Purpose:**  
Main inference pipeline for real-time semantic segmentation and object detection using **SegFormer** and **YOLO**.  
There is a plan to integrate **coord_calculator** to transform object coordinates from camera space to drone/world/GPS coordinates, but **this functionality is not yet implemented**.  
Currently, the pipeline only calculates bounding box midpoints in image space.

---

### Key Features

1. **Dynamic Path Management**
   - Uses `project_paths.py` to access weights, datasets, and output folders.  
   - Works independently from any working directory.

2. **Semantic Segmentation**
   - Loads a pretrained SegFormer model (`SegformerForSemanticSegmentation`).  
   - Preprocesses images using `SegformerImageProcessor`.  
   - Produces a segmentation map and RGB/colored output for visualization.  
   - Maps each pixel to its class label using `_classes.csv`.

3. **Object Detection (YOLO)**
   - Loads a YOLO model (`ultralytics.YOLO`) with pretrained weights.  
   - Detects objects (trash) in the input image.  
   - Associates each bounding box with the most frequent segmentation class in its ROI.  
   - Optionally saves segmentation and bounding-box images to output folders.

4. **Bounding Box Postprocessing**
   - Calculates **midpoint coordinates** of bounding boxes in **image space**.

5. **Coordinate Transformation (coord_calculator.py) (Planned)**
   - There is a plan to add functions to transform midpoints:
     - Pixel → Camera coordinates  
     - Camera → Drone coordinates  
     - Drone → World coordinates  
     - Drone → GPS coordinates  
   - Currently, this **coordinate transformation is not implemented**.

6. **Parallel ROS2 Integration**
   - Starts a ROS2 node as a separate process (`ros_main`) to communicate with sensors or other nodes.  
   - AI inference runs in the main process, receiving images from a queue and sending back bounding box midpoints.

7. **Real-time Processing**
   - Uses Python `multiprocessing.Queue` to handle input images and output coordinates efficiently.  
   - Designed for continuous inference with a ROS2-based robotics workflow.

---

### Typical Workflow

1. Start the ROS2 node in a separate process.  
2. Load SegFormer and YOLO models once at the start.  
3. For each input image:
   - Generate a semantic segmentation map.  
   - Detect objects and compute bounding boxes with YOLO.  
   - Calculate midpoints of bounding boxes in image space.  
   - **Note:** No transformation to drone/world/GPS coordinates yet.  
4. Optionally, save segmented and bounding-boxed images for visualization.

---
