import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from collections import deque

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

class State():
    IDLE = "IDLE"
    NAVIGATION = "NAVIGATION"
    RECOGNITION = "RECOGNITION"
    COLLECTION = "COLLECTION"

class Signal():
    NEW_LOCATION = "NEW_LOCATION"
    ARRIVED = "ARRIVED"
    RECOGNIZED = "RECOGNIZED"
    COLLECTED = "COLLECTED"
    FAILED_NAVIGATION = "FAILED_NAVIGATION"
    FAILED_RECOGNITION = "FAILED_RECOGNITION"
    HALT = "HALT"

class Brain(Node):

    def __init__(self):
        super().__init__("robby_brain")

        #Variables
        self.state = State.IDLE
        self.locations = deque()
        self.current_target = None

        #Publishers
        self.state_publisher = self.create_publisher(String, "robby_state_", 10)
        self.target_publisher = self.create_publisher(NavSatFix, "robby_target_", 10)
        self.signal_publisher = self.create_publisher(String, "robby_update_", 10)

        #Subscribers
        self.new_location_subscriber = self.create_subscription(NavSatFix, "robby_new_location_", self.new_location_callback, 10)
        self.signal_subscriber = self.create_subscription(String, "robby_update_", self.signal_handler, 10)

        #Timers
        self.target_dispatch_timer = self.create_timer(1., self.target_dispatcher)

    #Signal handler
    def signal_handler(self, signal : str):
        match signal:
            case Signal.HALT:
                pass
            case Signal.NEW_LOCATION:
                self.state = State.NAVIGATION
                self.get_logger().info(f"New target: {self.current_target}")
            case Signal.ARRIVED:
                self.state = State.RECOGNITION
                self.get_logger().info("Arrived at target location")
            case Signal.RECOGNIZED:
                self.state = State.COLLECTION
                self.get_logger().info("Recognized trash")
            case Signal.COLLECTED:
                self.state = State.IDLE
                self.get_logger().info("Collected trash")
            case Signal.FAILED_NAVIGATION | Signal.FAILED_RECOGNITION:
                self.state = State.IDLE
                self.get_logger().info(f"Failed with: {signal}")

    #Dispatcher function on a timer (this won't be on a timer on the final version)
    def target_dispatcher(self):
        if len(self.locations) != 0 and self.state == State.IDLE:
            gps = self.locations.popleft()
            self.current_target = gps
            self.target_publisher.publish(gps)

            signal = String()
            signal.data = Signal.NEW_LOCATION
            self.signal_publisher.publish(signal)
            
            self.signal_handler(Signal.NEW_LOCATION)

    #Callback to add new locations to queue
    def new_location_callback(self, gps : NavSatFix):
        self.locations.append(gps)
        self.get_logger().info(f"Received new location: {gps}")


def main(args = None):
    rclpy.init()
    brain = Brain()
    rclpy.spin(brain)
    brain.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()