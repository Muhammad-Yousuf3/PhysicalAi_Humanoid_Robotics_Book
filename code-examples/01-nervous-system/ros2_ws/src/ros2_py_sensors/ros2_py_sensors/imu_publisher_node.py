import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import numpy as np

class ImuPublisherNode(Node):
    def __init__(self):
        super().__init__('imu_publisher_node')
        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        self.timer = self.create_timer(1.0 / 50.0, self.timer_callback) # 50 Hz
        self.get_logger().info('Mock IMU Publisher Node started.')

        # Initialize some dummy IMU data
        self.orientation = np.array([0.0, 0.0, 0.0, 1.0]) # w, x, y, z
        self.angular_velocity = np.array([0.0, 0.0, 0.0])
        self.linear_acceleration = np.array([0.0, 0.0, 9.81]) # Gravity

    def timer_callback(self):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'base_link' # Assuming IMU is on base_link

        # Simulate some minor fluctuations
        self.orientation += np.random.normal(0, 0.001, 4)
        self.orientation /= np.linalg.norm(self.orientation) # Normalize quaternion
        
        imu_msg.orientation.w = self.orientation[3]
        imu_msg.orientation.x = self.orientation[0]
        imu_msg.orientation.y = self.orientation[1]
        imu_msg.orientation.z = self.orientation[2]

        self.angular_velocity = np.random.normal(0, 0.01, 3)
        imu_msg.angular_velocity.x = self.angular_velocity[0]
        imu_msg.angular_velocity.y = self.angular_velocity[1]
        imu_msg.angular_velocity.z = self.angular_velocity[2]

        self.linear_acceleration = np.array([0.0, 0.0, 9.81]) + np.random.normal(0, 0.01, 3)
        imu_msg.linear_acceleration.x = self.linear_acceleration[0]
        imu_msg.linear_acceleration.y = self.linear_acceleration[1]
        imu_msg.linear_acceleration.z = self.linear_acceleration[2]

        # Set covariance matrices (example values)
        imu_msg.orientation_covariance = [0.0025, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0025]
        imu_msg.angular_velocity_covariance = [0.0025, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0025]
        imu_msg.linear_acceleration_covariance = [0.0025, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0025]

        self.publisher_.publish(imu_msg)
        # self.get_logger().info('Publishing mock IMU data')

def main(args=None):
    rclpy.init(args=args)
    imu_publisher_node = ImuPublisherNode()
    rclpy.spin(imu_publisher_node)
    imu_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
