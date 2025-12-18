import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Pose

"""
The localizer is responsible for two things:
1. Constantly keeping track of where the robot is located both in terms of GPS and local pose
2. Translating GPS targets sent by the brain to local pose
"""

class Localizer(Node):

    def __init__(self):
        super().__init__("robby_localizer")

        #Variables
        self.current_gps = None
        self.current_IMU = None
        self.current_odometer = None

        #Publishers
        self.pose_publisher = self.create_publisher(Pose, "robby_pose", 10)
        self.target_pose_publisher = self.create_publisher(Pose, "robby_target_pose", 10)

        #Subscribers
        self.translate_subscriber = self.create_subscription(NavSatFix, "robby_translate", self.translate_callback, 10)
        self.GPS_subscriber = self.create_subscription(NavSatFix, "robby_GPS", self.GPS_callback, 10)
        #DATA TYPE FOR THESE TWO ARE WRONG
        self.IMU_subscriber = self.create_subscription(Pose, "robby_IMU_data", self.IMU_callback, 10)
        self.odometer_subscriber = self.create_subscription(Pose, "robby_odometer_data", self.odometer_callback, 10)

        #Timer
        self.pose_timer = self.create_timer(0.01, self.pose_callback)

    #Translate function for brain requests
    def translate_callback(self, msg):
        pass

    def GPS_callback(self, msg):
        self.current_gps = msg
    
    def IMU_callback(self, msg):
        self.current_IMU = msg

    def odometer_callback(self, msg):
        self.current_odometer = msg

    #Constant pose publisher
    def pose_callback(self):
        pass

def main(args=None):
    rclpy.init()
    localizer = Localizer()
    rclpy.spin(localizer)
    localizer.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()