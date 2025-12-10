import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, Imu, PointCloud2
from geometry_msgs.msg import WrenchStamped

class SensorListener(Node):
    def __init__(self):
        super().__init__('sensor_listener')
        self.get_logger().info('Starting sensor listener node.')

        # IMU Subscriber
        self.imu_subscriber = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )
        self.imu_subscriber  # prevent unused variable warning

        # Camera Subscriber (Image Raw)
        self.camera_image_subscriber = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.camera_image_callback,
            10
        )
        self.camera_image_subscriber # prevent unused variable warning

        # Camera Subscriber (Depth Image Raw)
        self.camera_depth_subscriber = self.create_subscription(
            Image,
            '/camera/depth',
            self.camera_depth_callback,
            10
        )
        self.camera_depth_subscriber # prevent unused variable warning

        # LiDAR Subscriber
        self.lidar_subscriber = self.create_subscription(
            PointCloud2,
            '/scan',
            self.lidar_callback,
            10
        )
        self.lidar_subscriber # prevent unused variable warning

        # Force/Torque Sensor Subscriber
        self.ft_subscriber = self.create_subscription(
            WrenchStamped,
            '/ft_sensor/data',
            self.ft_callback,
            10
        )
        self.ft_subscriber # prevent unused variable warning


    def imu_callback(self, msg):
        self.get_logger().info(f'IMU Data: Orientation x={msg.orientation.x:.2f}, y={msg.orientation.y:.2f}, z={msg.orientation.z:.2f}, w={msg.orientation.w:.2f}')
        # In a real example, you'd process this data further

    def camera_image_callback(self, msg):
        self.get_logger().info(f'Camera Image: Received image with dimensions {msg.width}x{msg.height}')
        # In a real example, you'd convert this to an OpenCV image or display it

    def camera_depth_callback(self, msg):
        self.get_logger().info(f'Camera Depth: Received depth image with dimensions {msg.width}x{msg.height}')
        # In a real example, you'd process depth data

    def lidar_callback(self, msg):
        self.get_logger().info(f'LiDAR PointCloud: Received {msg.width * msg.height} points.')
        # In a real example, you'd process the point cloud

    def ft_callback(self, msg):
        self.get_logger().info(f'FT Sensor: Force fx={msg.wrench.force.x:.2f}, fy={msg.wrench.force.y:.2f}, fz={msg.wrench.force.z:.2f}')
        # In a real example, you'd process force/torque data


def main(args=None):
    rclpy.init(args=args)
    sensor_listener = SensorListener()
    rclpy.spin(sensor_listener)
    sensor_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
