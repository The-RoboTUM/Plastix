import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool, Float32MultiArray
from robby_common.robby_common.enums import Signal, State

"""
The navgigator is responsible for the following:
1. Following the state of the robot to decide what to do
2. Calculate the best path given current pose and target pose
3. Detect obstacles on path
4. Dynamically replan path to avoid obstacles
5. Control motors to follow path
"""

class Navigator(Node):

    def __init__(self):
        super().__init__("robby_navigator")

        #Variables
        self.state = State.IDLE

        #Publishers
        self.signal_publisher = self.create_publisher(String, "robby_signal", 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, "robby_cmd_vel", 10)
        self.vacuum_cmd_publisher = self.create_publisher(Bool, "robby_vacuum_cmd", 10)

        #Subscribers
        self.state_subscriber = self.create_subscription(String, "robby_state", self.state_callback, 10)
        self.pose_subscriber = self.create_subscription(Pose, "robby_pose", None, 10)
        self.target_pose_subscriber = self.create_subscription(Pose, "robby_target_pose", None, 10)
        self.sensor_data_subscriber = self.create_subscription(Float32MultiArray, "robby_sensor_data", None, 10)
        #This subscription is fully a template even the msg type can change
        self.trash_pose_subscriber = self.create_subscription(Pose, "robby_trash_pose", None, 10)

        #Timer
        self.run_timer = self.create_timer(0.0001, self.run)

    def state_callback(self):
        pass

    #Main function of navigator, will act according to current state
    def run(self):
        pass

def main(args=None):
    rclpy.init()
    navigator = Navigator()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()