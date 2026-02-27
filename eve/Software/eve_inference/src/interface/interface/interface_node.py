#!/usr/bin/env python3
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Polygon, Point32
from cv_bridge import CvBridge
from PIL import Image as PIL_IMG
from multiprocessing import Queue
import threading
class Interface_Node(Node):

    def __init__(self):
        super().__init__('interface_node')

        # Subscriber <- receive images
        self.img_subscriber = self.create_subscription(
            Image, # message type
            '/Image_EVE', # topic name 
            self.img_to_container,
            10 
        )

        # Publisher -> send coordinates
        self.camera_coordinates_publisher = self.create_publisher(
            Polygon, 
            '/Camera_Coordinates_Octopus', 
            10
        )
        self.bridge = CvBridge() # for converting ros2 Image messages into numpy array

        # Background thread — reacts instantly when coordinates arrive
        self.queue_thread = threading.Thread(target=self.queue_watcher, daemon=True) 
        self.queue_thread.start()

    def img_to_container(self, msg: Image):
        """Receive Image and store them in the queue"""
        # ROS2 Image → NumPy Array
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        pil_image = PIL_IMG.fromarray(cv_image) # convert to PIL Image
        try:
            image_queue.put(pil_image, block=False)  # Non-blocking
        except:
            # Queue Full ...
            self.get_logger().warn("Queue full, Image lost!")

    def queue_watcher(self):
        """Publish Coordinates of the bounding box""" #TODO Should be modified to use the coord_calculator script
        while rclpy.ok():
            coordinates_list = coords_queue.get()  # blocks only this thread
            msg = Polygon()
                
            msg.points = [Point32(x=float(c[0]), y=float(c[1]), z=0.0) for c in coordinates_list] # extract coordinate points
            self.camera_coordinates_publisher.publish(msg)
                
            self.get_logger().info(f"Published: {coordinates_list}") 

def main(img_queue, c_queue):
    
    # Define Queues
    global image_queue
    global coords_queue
    image_queue = img_queue  # for storing images
    coords_queue = c_queue # for storing coordinates
    
    # Start Ros2 Node
    rclpy.init()
    node = Interface_Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



