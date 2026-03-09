#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Float64, Float32MultiArray
from geometry_msgs.msg import Polygon, Point
import cv2
import numpy as np
from PIL import Image as IMG
import os

class Test_Node(Node):

    def __init__(self):
        super().__init__('test_node')

        # Bild laden
        package_path = get_package_share_directory('image_container')
        image_path = os.path.join(package_path,'resource', 'test_image.jpg')        
        img = IMG.open(image_path)
        img = img.convert("RGB")  # sicherstellen dass 3 Kanäle vorhanden sind
        self.img_data = np.asarray(img, dtype=np.uint8)

        # Publisher
        self.img_publisher = self.create_publisher(
            Image,
            '/Image_EVE',
            10
        )

        # subscriber
        self.coord_subscriber = self.create_subscription(
            Polygon, 
            '/Camera_Coordinates_Octopus',
            self.coordinate_handler,
            10
        )

        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Image()

        # Header
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_frame"

        # Image information
        msg.height = self.img_data.shape[0]
        msg.width = self.img_data.shape[1]
        msg.encoding = "rgb8"  
        msg.is_bigendian = False
        msg.step = self.img_data.shape[1] * 3  # width * channels

        # Bilddaten
        msg.data = self.img_data.tobytes()

        self.img_publisher.publish(msg)
        self.get_logger().info("Image Send")
    
    def coordinate_handler(self, msg: Polygon):
        message = msg
        print(message)



def main():
    rclpy.init()
    node = Test_Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()