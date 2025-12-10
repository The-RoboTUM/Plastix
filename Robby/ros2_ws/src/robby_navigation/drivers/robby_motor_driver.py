import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gpiozero import Robot

class Motor_Driver(Node):

    def __init__(self, left_pins, right_pins):
        super().__init__("robby_motor_driver")

        #Variables
        self.robot = Robot(left_pins, right_pins)

        #Subscriber
        self.cmd_vel_subscriber = self.create_subscription(Twist, "robby_cmd_vel", self.cmd_vel_callback, 10)

    def cmd_vel_callback(self, cmd : Twist):
        #Convert Twist to motor speed
        left_speed = cmd.linear.x - cmd.angular.z
        right_speed = cmd.linear.x + cmd.angular.z

        #Set robot speed
        self.robot.value = (left_speed, right_speed)

    def destroy_node(self):
        self.robot.close()
        return super().destroy_node()
    
def main(args=None):
    #Set pins
    left_pins = (17,18)
    right_pins = (22,23)

    rclpy.init()
    driver = Motor_Driver(left_pins, right_pins)
    rclpy.spin(driver)
    driver.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()