import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from gpiozero import DistanceSensor

class Sensor_Driver(Node):

    def __init__(self, pins : list):
        super().__init__("robby_sensor_driver")

        #Variable
        self.sensors = []
        for echo, trigger in pins:
            self.sensors.append(DistanceSensor(echo, trigger))

        #Publisher
        self.sensor_data_publisher = self.create_publisher(Float32MultiArray, "robby_sensor_data", 10)

        #Timer
        self.sensor_data_timer = self.create_timer(0.01, self.sensor_data_callback)

    def sensor_data_callback(self):
        #Collect data
        msg = Float32MultiArray()
        msg.data = [sensor.distance * 100 for sensor in self.sensors]
        self.sensor_data_publisher.publish(msg)

    def destroy_node(self):
        for sensor in self.sensors:
            sensor.close()
        return super().destroy_node()

def main(args=None):
    #Set pins
    pins = []

    rclpy.init()
    driver = Sensor_Driver(pins)
    rclpy.spin(driver)
    driver.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()