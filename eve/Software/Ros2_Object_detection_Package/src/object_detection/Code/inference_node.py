#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, Float32MultiArray
from object_detection.inference_pipeline import complete_inference
from object_detection.coordinate_transformation import pixel_to_camera
import yaml
from typing import List, Tuple
import os

class Inference_Node(Node):
    
    def __init__(self):
        from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        super().__init__('inference_node')
        self.get_logger().info('Inference Node initialised')

        self.height: float = 0.0  # Drone height

        # Subscribers
        # self.image_subscription = self.create_subscription(
        #     Image, '/world/baylands/model/x500_mono_cam_down_1/link/camera_link/sensor/imager/image', self.inference_callback, 10
        # )
        self.image_subscription = self.create_subscription(
            Image, '/world/baylands/model/x500_mono_cam_down_1/link/camera_link/sensor/imager/image', self.inference_callback, qos
        )
        self.height_subscription = self.create_subscription(
            Float64, '/Height_EVE', self.height_callback, 10
        )
        
        # Publisher
        self.camera_coordinates_publisher = self.create_publisher(
            Float32MultiArray, '/Camera_Coordinates_Octopus', 10
        )

        # Load config
        script_dir = os.path.dirname(__file__)  # Ordner des Skripts
        configuration_file = os.path.join(script_dir, "Config.yaml")
        
        with open(configuration_file, 'r') as ymlfile:
           self.cfg = yaml.full_load(ymlfile)
        

    def height_callback(self, msg: Float64):
        self.height = msg.data

    def inference_callback(self, msg: Image):
        #Run inference
        #bbox_midpoints = complete_inference(img=msg)

        

        # #Convert pixel to camera coordinates
        # camera_coordinates = pixel_to_camera(
        #     config=self.cfg,
        #     img_width=msg.width,
        #     img_height=msg.height,
        #     bbox_midpoints=bbox_midpoints,
        #     camera_height=self.height
        # )

        # # Flatten list of tuples
        # flat_list = [x for coord in camera_coordinates for x in coord]

        # # Publish
        # msg_out = Float32MultiArray()
        # msg_out.data = flat_list
        # self.camera_coordinates_publisher.publish(msg_out)

        print("Image received")

        #return bbox_midpoints


def main():
    rclpy.init()
    node = Inference_Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
