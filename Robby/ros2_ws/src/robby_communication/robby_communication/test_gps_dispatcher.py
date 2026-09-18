import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import random

class GPS_Dispatcher(Node):

    def __init__(self):
        super().__init__("test_gps_dispatcher")

        #Publisher
        self.dispatch_publisher = self.create_publisher(NavSatFix, "robby_new_location_", 10)

        #Timer
        self.dispatch_timer = self.create_timer(1., self.gps_dispatch_callback)

    def gps_dispatch_callback(self):
        gps = NavSatFix()

        gps.latitude = random.uniform(0,100)
        gps.longitude = random.uniform(0,100)

        self.dispatch_publisher.publish(gps)
        self.get_logger().info(f"Published gps: ({gps.latitude}, {gps.altitude})")

def main(args = None):
    rclpy.init()
    dispatcher = GPS_Dispatcher()
    rclpy.spin(dispatcher)
    dispatcher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()