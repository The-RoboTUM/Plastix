import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import board
import busio
from adafruit_bno08x import BNO_REPORT_ACCELEROMETER, BNO_REPORT_GYROSCOPE, BNO_REPORT_ROTATION_VECTOR
from adafruit_bno08x.i2c import BNO08X_I2C

class BNO085Publisher(Node):

    def __init__(self):
        super().__init__('bno085_publisher')
        
        # Create Publisher for standard ROS IMU message
        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        
        # Timer callback (e.g., 50 Hz)
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Initialize I2C and BNO085
        try:
            self.i2c = busio.I2C(board.SCL, board.SDA)
            self.bno = BNO08X_I2C(self.i2c)
            
            # Enable the features we need
            self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
            self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
            self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
            
            self.get_logger().info("BNO085 IMU successfully initialized!")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize BNO085: {e}")
            raise e

    def timer_callback(self):
        msg = Imu()
        
        # 1. Populate Header
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        
        try:
            # 2. Orientation (Quaternion: x, y, z, w)
            # BNO08x returns (i, j, k, real)
            quat_i, quat_j, quat_k, quat_real = self.bno.quaternion
            msg.orientation.x = float(quat_i)
            msg.orientation.y = float(quat_j)
            msg.orientation.z = float(quat_k)
            msg.orientation.w = float(quat_real)
            
            # 3. Angular Velocity (rad/sec)
            gyro_x, gyro_y, gyro_z = self.bno.gyro
            msg.angular_velocity.x = float(gyro_x)
            msg.angular_velocity.y = float(gyro_y)
            msg.angular_velocity.z = float(gyro_z)
            
            # 4. Linear Acceleration (m/s^2)
            accel_x, accel_y, accel_z = self.bno.acceleration
            msg.linear_acceleration.x = float(accel_x)
            msg.linear_acceleration.y = float(accel_y)
            msg.linear_acceleration.z = float(accel_z)
            
            # (Optional) Covariance matrices can be filled if you have calibrated values.
            # Leaving them as 0 implies unknown/default variance.

            # Publish the message
            self.publisher_.publish(msg)
            
        except Exception as e:
            self.get_logger().warn(f"Error reading BNO085 data: {e}")

def main(args=None):
    rclpy.init(args=args)
    bno085_publisher = BNO085Publisher()
    
    try:
        rclpy.spin(bno085_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        bno085_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()