import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from collections import deque
from time import sleep
from robby_common.enums import Signal, State

"""
The brain is a state machine that has four states: Idle, Navigation, Recognition, Collection
The switching of the states will be broadcast onto the topic "robby_state_"
The brain will subscribe to the appropriate topics and decide on the state accordingly
The brain will also provide services such as providing new location data

Functions needed:
1. A callback that acts when a new location is received from Octopus and stores it
2. A dispatcher that sets a new target location if there isn't a current one
3. A state changer based on signals received
"""

class Brain(Node):

    def __init__(self):
        super().__init__("robby_brain")

        #Variables
        self.state = State.IDLE
        self.locations = deque()
        self.current_target = None

        #Publishers
        self.state_publisher = self.create_publisher(String, "robby_state", 10)
        self.translate_publisher = self.create_publisher(NavSatFix, "robby_translate", 10)

        #Subscribers
        self.new_location_subscriber = self.create_subscription(NavSatFix, "robby_new_location", self.new_location_callback, 10)
        self.signal_subscriber = self.create_subscription(String, "robby_signal", self.signal_handler, 10)

    #Signal handler
    def signal_handler(self, signal : str):
        match signal:
            #TODO
            case Signal.HALT:
                pass
            case Signal.ARRIVED:
                self.state = State.RECOGNITION
                self.get_logger().info("Arrived at target location")
            case Signal.RECOGNIZED:
                self.state = State.COLLECTION
                self.get_logger().info("Recognized trash")
            case Signal.COLLECTED:
                self.state = State.IDLE
                self.get_logger().info("Collected trash")
                self.current_target = None
            case Signal.FAILED_NAVIGATION | Signal.FAILED_RECOGNITION:
                self.state = State.IDLE
                self.get_logger().info(f"Failed with: {signal}")
                self.current_target = None
        self.state_publisher.publish(self.state)

    #Dispatcher function
    def target_dispatcher(self):
        if len(self.locations) != 0 and self.current_target == None:
            target = self.locations.popleft()
            self.current_target = target
            self.translate_publisher.publish(target)

            #Sleeping is here to give localizer enough time for translation just in case this can be removed
            sleep(0.01)

            self.state = State.NAVIGATION
            self.get_logger().info(f"New target: {self.current_target}")
            self.state_publisher.publish(self.state)

    #Callback to add new locations to queue
    def new_location_callback(self, gps : NavSatFix):
        self.locations.append(gps)
        self.get_logger().info(f"Received new location: {gps}")
        self.target_dispatcher()

def main(args = None):
    rclpy.init()
    brain = Brain()
    rclpy.spin(brain)
    brain.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()