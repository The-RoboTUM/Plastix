import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool, Float32MultiArray
from robby_common.enums import Signal, State

"""
The eyes are responsible for the following:
1. Read input from the camera data
2. Use machine learning model to detect trash
3. Determine relative position of trash
"""

class Eyes(Node):

    def __init__(self):
        super().__init__("robby_eyes")

        #Variables
        self.state = State.IDLE

        #Publishers
        self.signal_publisher = self.create_publisher(String, "robby_signal", 10)
        #This publisher is fully a template even the msg type can change
        self.trash_pose_publisher = self.create_subscription(Pose, "robby_trash_pose", 10)

        #Subscribers
        self.state_subscriber = self.create_subscription(String, "robby_state", self.state_callback, 10)
        self.pose_subscriber = self.create_subscription(Pose, "robby_pose", self.pose_callback, 10)
        #The data type for camera images needs to be decided Pose is just a placeholder
        self.camera_data_subscriber = self.create_subscription(Pose, "robby_camera_data", self.camera_callback, 10)

        #Timer
        self.run_timer = self.create_timer(0.0001, self.run)

    def state_callback(self, msg):
        pass

    def pose_callback(self, msg):
        pass

    def camera_callback(self, msg):
        pass

    #Main function of eyes, will act according to current state
    def run(self):
        pass

def main(args=None):
    rclpy.init()
    eyes = Eyes()
    rclpy.spin(eyes)
    eyes.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()