## interface_node.py

**Purpose:**  
ROS2 interface node that connects the AI inference pipeline with the robotics system.  
It subscribes to image data from a camera topic, sends images to the AI process via a queue, and publishes bounding box coordinates back to ROS2 topics. The test_node.py is only for testing, you can ignore it....

---

### Key Features

1. **ROS2 Subscriber**
   - Subscribes to `/Image_EVE` topic (`sensor_msgs/Image`) to receive camera images.  
   - Converts ROS2 Image messages to **PIL Images** using `CvBridge`.  
   - Pushes images into a multiprocessing queue for AI inference.  

2. **ROS2 Publisher**
   - Publishes bounding box coordinates to `/Camera_Coordinates_Octopus` as a `geometry_msgs/Polygon` message.  
   - Converts midpoint coordinates from AI inference into `Point32` points for ROS2.  

3. **Multiprocessing Integration**
   - Receives images from ROS2 and sends them to the **AI inference process** via `Queue`.  
   - Receives calculated bounding box coordinates from the AI process and publishes them.  
   - Uses **background thread** (`queue_watcher`) to monitor the coordinates queue without blocking the main ROS2 loop.

4. **Threaded Processing**
   - Coordinates publishing runs in a separate thread to avoid blocking ROS2 callbacks.  
   - Non-blocking image insertion ensures that images are not lost if the queue is full.  

5. **Real-time Communication**
   - Works alongside `inference_pipeline.py` for continuous AI inference.  
   - Enables seamless data flow between camera inputs and AI detection results.  

---

### Typical Workflow

1. Initialize ROS2 node with `main(img_queue, c_queue)`.  
2. Subscribe to camera images on `/Image_EVE`.  
3. Convert ROS2 images to PIL and push to `image_queue`.  
4. Wait for midpoints from AI via `coords_queue`.  
5. Publish bounding box coordinates as a `Polygon` to `/Camera_Coordinates_Octopus`.  
6. Operates in parallel with the AI inference pipeline for real-time performance.

