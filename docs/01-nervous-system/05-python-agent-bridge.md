# Section 5: Building a Python Agent → ROS Bridge – Connecting AI to Action

The true power of ROS 2 in Physical AI emerges when we can seamlessly connect intelligent AI agents, often developed in Python using frameworks like TensorFlow or PyTorch, with the robot's ROS 2-based control system. This section focuses on building that crucial **Python Agent → ROS bridge** using `rclpy`, the Python client library for ROS 2.

## 5.1. `rclpy` Basics: The Python Interface to ROS 2

`rclpy` provides all the necessary tools to write ROS 2 nodes in Python. It wraps the underlying C-based ROS Client Library (rcl), offering a Pythonic interface for creating:

-   Nodes
-   Publishers and Subscribers
-   Service Servers and Clients
-   Action Servers and Clients
-   Parameters
-   Timers and Executors

The elegance of Python, combined with `rclpy`, allows for rapid prototyping and integration of complex AI algorithms into a ROS 2 system.

## 5.2. Agent-to-Controller Flow

Consider an AI agent (e.g., an LLM-based planner) that generates high-level commands, and a low-level robot controller that executes motor commands. The bridge facilitates this interaction:

1.  **AI Agent (Python)**: Interprets user input (e.g., natural language command), processes sensor data, and decides on a high-level robot action (e.g., "move forward 1 meter").
2.  **ROS Bridge Node (`rclpy`)**: This node, written in Python, receives the high-level AI command.
    -   It translates the AI's intent into a ROS 2-compatible message (e.g., a `geometry_msgs/Twist` message for linear and angular velocity).
    -   It publishes this message to a ROS 2 topic (e.g., `/cmd_vel`) that the robot's low-level controller is subscribed to.
    -   Optionally, it can also subscribe to sensor feedback topics (e.g., `/odom`, `/imu/data`) to provide the AI agent with real-time state information.
3.  **ROS 2 Controller Node**: Subscribes to the ROS 2 command topic (`/cmd_vel`) and translates the generic command into specific signals for the robot's motors.

### Example: LLM to `Twist` Message Bridge

Our `llm_bridge_node.py` exemplifies this pattern. It receives natural language commands, parses them (using simple regex here, but extensible to LLM APIs), and publishes a `geometry_msgs/Twist` message.

**`llm_bridge_node.py` (excerpt from `ros2_py_agent` package):**

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import re
import numpy as np

class LLMBridgeNode(Node):
    def __init__(self):
        super().__init__('llm_bridge_node')
        self.cmd_subscription = self.create_subscription(
            String,
            'llm_commands', # Topic where natural language commands arrive
            self.llm_command_callback,
            10
        )
        self.twist_publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info('LLM Bridge Node started. Waiting for commands on /llm_commands...')

    def llm_command_callback(self, msg):
        self.get_logger().info(f'Received LLM command: "{msg.data}"')
        twist_msg = self._process_llm_command(msg.data)
        if twist_msg:
            self.twist_publisher_.publish(twist_msg)
            self.get_logger().info(f'Published Twist: Linear.x={twist_msg.linear.x}, Angular.z={twist_msg.angular.z}')
        else:
            self.get_logger().warn(f'Could not process command: "{msg.data}" into a Twist message.')

    def _process_llm_command(self, command_text):
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
            else:
                twist.linear.x = 0.2 # Default forward speed
        # ... (other command parsing logic)
        elif "stop" in command_text_lower:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        else:
            return None # Command not understood

        return twist
```

This node acts as an intermediary, abstracting the complexity of natural language understanding from the low-level motor control.

## 5.3. Safety Checks within the Bridge

Integrating AI agents directly with robot hardware introduces significant safety concerns. The Python Agent → ROS bridge is a critical point to implement safety checks to prevent unintended or dangerous actions.

Key safety considerations:

-   **Command Validation**: Filter out or flag commands that are physically impossible, exceed joint limits, or violate environmental constraints.
-   **Rate Limiting**: Ensure commands are not sent at an excessively high rate, which could overload controllers or cause erratic behavior.
-   **Emergency Stop Integration**: The bridge should be aware of and respect emergency stop signals from the robot or external systems.
-   **"Soft" Limits**: Implement software-enforced speed, acceleration, and force limits to prevent damage or injury, even if a command requests an unsafe value.
-   **Confirmation/Feedback Loops**: For critical actions, the bridge might require a confirmation from the AI agent or even a human operator before execution.
-   **Behavior Trees/State Machines**: For more complex behaviors, a behavior tree or state machine can be integrated into the bridge to ensure that the robot transitions between states (e.g., from "idle" to "moving" to "grasping") in a safe and logical sequence.

By carefully designing the Python Agent → ROS bridge with `rclpy`, we can create a powerful yet safe interface that allows AI agents to effectively control humanoid robots, translating abstract intentions into physical realities.
