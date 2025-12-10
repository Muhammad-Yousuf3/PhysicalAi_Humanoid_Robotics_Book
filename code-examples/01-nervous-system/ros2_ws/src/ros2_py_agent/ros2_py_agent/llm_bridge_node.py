import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import re

class LLMBridgeNode(Node):
    def __init__(self):
        super().__init__('llm_bridge_node')
        self.cmd_subscription = self.create_subscription(
            String,
            'llm_commands',
            self.llm_command_callback,
            10
        )
        self.cmd_subscription # prevent unused variable warning

        self.twist_publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info('LLM Bridge Node started. Waiting for commands on /llm_commands...')

    def llm_command_callback(self, msg):
        self.get_logger().info(f'Received LLM command: "{msg.data}"')
        twist_msg = self.process_llm_command(msg.data)
        if twist_msg:
            self.twist_publisher_.publish(twist_msg)
            self.get_logger().info(f'Published Twist: Linear.x={twist_msg.linear.x}, Angular.z={twist_msg.angular.z}')
        else:
            self.get_logger().warn(f'Could not process command: "{msg.data}" into a Twist message.')

    def process_llm_command(self, command_text):
        """
        Mock LLM logic to translate natural language commands into Twist messages.
        In a real scenario, this would involve an actual LLM API call.
        """
        twist = Twist()
        command_text_lower = command_text.lower()

        if "move forward" in command_text_lower:
            distance_match = re.search(r'(\d+(\.\d+)?)\s*meter', command_text_lower)
            if distance_match:
                distance = float(distance_match.group(1))
                twist.linear.x = min(distance, 0.5) # Max linear speed 0.5 m/s
                self.get_logger().info(f"LLM interpreted: Move forward {distance} meter(s).")
            else:
                twist.linear.x = 0.2 # Default forward speed
                self.get_logger().info("LLM interpreted: Move forward.")
        elif "move backward" in command_text_lower:
            twist.linear.x = -0.2
            self.get_logger().info("LLM interpreted: Move backward.")
        elif "turn left" in command_text_lower:
            angle_match = re.search(r'(\d+(\.\d+)?)\s*degree', command_text_lower)
            if angle_match:
                angle_rad = float(angle_match.group(1)) * np.pi / 180.0
                twist.angular.z = min(angle_rad, 0.5) # Max angular speed 0.5 rad/s
                self.get_logger().info(f"LLM interpreted: Turn left {angle_match.group(1)} degree(s).")
            else:
                twist.angular.z = 0.3 # Default left turn speed
                self.get_logger().info("LLM interpreted: Turn left.")
        elif "turn right" in command_text_lower:
            twist.angular.z = -0.3
            self.get_logger().info("LLM interpreted: Turn right.")
        elif "stop" in command_text_lower:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info("LLM interpreted: Stop.")
        else:
            self.get_logger().info("LLM could not interpret command clearly, sending no movement.")
            return None # No Twist message if command not understood

        return twist

def main(args=None):
    rclpy.init(args=args)
    llm_bridge_node = LLMBridgeNode()
    rclpy.spin(llm_bridge_node)
    llm_bridge_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
