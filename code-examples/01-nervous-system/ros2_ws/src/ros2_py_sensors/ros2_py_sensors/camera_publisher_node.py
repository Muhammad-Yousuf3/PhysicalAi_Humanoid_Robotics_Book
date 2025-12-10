import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2

class CameraPublisherNode(Node):
    def __init__(self):
        super().__init__('camera_publisher_node')
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        self.timer = self.create_timer(1.0 / 10.0, self.timer_callback) # 10 Hz
        self.bridge = CvBridge()
        self.get_logger().info('Mock Camera Publisher Node started.')

    def timer_callback(self):
        # Create a dummy image (e.g., a simple colored rectangle or random noise)
        height, width = 480, 640
        image = np.zeros((height, width, 3), dtype=np.uint8)
        
        # Draw a simple moving rectangle for visual effect
        t = self.get_clock().now().nanoseconds / 1e9 # current time in seconds
        x = int(100 * (1 + np.sin(t))) % (width - 100)
        y = int(50 * (1 + np.cos(t))) % (height - 50)
        cv2.rectangle(image, (x, y), (x + 100, y + 50), (0, 255, 0), -1) # Green rectangle

        # Convert OpenCV image to ROS Image message
        msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link' # Assuming a camera_link frame

        self.publisher_.publish(msg)
        # self.get_logger().info('Publishing mock camera image')

def main(args=None):
    rclpy.init(args=args)
    camera_publisher_node = CameraPublisherNode()
    rclpy.spin(camera_publisher_node)
    camera_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
