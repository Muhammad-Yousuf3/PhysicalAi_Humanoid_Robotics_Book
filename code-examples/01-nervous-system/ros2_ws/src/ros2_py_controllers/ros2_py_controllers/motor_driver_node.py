import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver_node')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.subscription # prevent unused variable warning
        self.get_logger().info('Motor Driver Node started. Subscribing to /cmd_vel...')

    def cmd_vel_callback(self, msg):
        self.get_logger().info(f'Received Twist command: Linear.x={msg.linear.x:.2f}, Angular.z={msg.angular.z:.2f}')
        # In a real robot, this is where you would send commands to physical motors
        if msg.linear.x > 0:
            self.get_logger().info('Moving forward...')
        elif msg.linear.x < 0:
            self.get_logger().info('Moving backward...')
        elif msg.angular.z > 0:
            self.get_logger().info('Turning left...')
        elif msg.angular.z < 0:
            self.get_logger().info('Turning right...')
        else:
            self.get_logger().info('Stopping motors.')

def main(args=None):
    rclpy.init(args=args)
    motor_driver_node = MotorDriverNode()
    rclpy.spin(motor_driver_node)
    motor_driver_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
