# Section 4: ROS 2 Services and Actions – Beyond Simple Messaging

While ROS 2 Topics are excellent for streaming continuous data, robotic systems often require more structured communication patterns. This is where **Services** and **Actions** come into play, providing mechanisms for request-response interactions and goal-oriented, long-running tasks with feedback.

## 4.1. ROS 2 Services: Request-Response Communication

**Services** are designed for synchronous, request-response style communication. A client sends a request to a service server, and the server processes the request and sends back a response. This is similar to a function call or a web API request. Services are best suited for:

-   **Immediate, short-duration tasks**: e.g., "get the current robot pose," "toggle a light," "trigger a single camera shot."
-   **Querying data**: Asking for a specific piece of information that is not continuously published.
-   **Configuration changes**: Setting parameters or modes on a node.

### Service Definition (`.srv` file)

Like messages, services are defined using a simple text format in `.srv` files within a ROS 2 package. A service definition consists of two parts, separated by `---`: the request structure and the response structure.

**Example: `my_robot_interfaces/srv/SetLed.srv`**

```
# Request
int64 led_number
int64 state # 0: off, 1: on, 2: blink
---
# Response
bool success
string message
```

### Python Service Server Example

```python
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import SetLed # Assuming this service definition

class LedPanelService(Node):
    def __init__(self):
        super().__init__('led_panel_service')
        self.service = self.create_service(
            SetLed,
            'set_led', # Service name
            self.set_led_callback
        )
        self.get_logger().info('LED Panel Service is ready.')

    def set_led_callback(self, request, response):
        self.get_logger().info(f'Received request: Set LED {request.led_number} to state {request.state}')
        # In a real robot, you would interface with hardware here
        if request.led_number >= 0 and request.led_number < 10: # Example validation
            response.success = True
            response.message = f'LED {request.led_number} set to state {request.state}'
        else:
            response.success = False
            response.message = 'Invalid LED number'
        return response

def main(args=None):
    rclpy.init(args=args)
    node = LedPanelService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Python Service Client Example

```python
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import SetLed
import sys

class LedPanelClient(Node):
    def __init__(self):
        super().__init__('led_panel_client')
        self.client = self.create_client(SetLed, 'set_led')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = SetLed.Request()

    def send_request(self, led_number, state):
        self.req.led_number = led_number
        self.req.state = state
        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    client = LedPanelClient()
    
    if len(sys.argv) != 3:
        client.get_logger().info('Usage: ros2 run my_robot_bringup led_client <led_number> <state>')
        return

    led_number = int(sys.argv[1])
    state = int(sys.argv[2])
    
    response = client.send_request(led_number, state)
    if response.success:
        client.get_logger().info(f'Success: {response.message}')
    else:
        client.get_logger().error(f'Failed: {response.message}')
    
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 4.2. ROS 2 Actions: Goal-Oriented, Long-Running Tasks

**Actions** are a more complex communication type built on top of topics and services, designed for long-running, preemptable tasks that provide continuous feedback. Actions are particularly well-suited for robotic tasks like:

-   **Navigation**: "Go to a specific location." (The robot provides feedback on its current pose and progress).
-   **Manipulation**: "Pick up an object." (Feedback on gripper state, object acquired).
-   **Complex behaviors**: "Perform a dance routine."

An Action consists of three distinct parts:

1.  **Goal**: The request to perform a task (e.g., target pose for navigation).
2.  **Result**: The final outcome of the task (e.g., success/failure of reaching the goal).
3.  **Feedback**: Intermediate updates on the progress of the task (e.g., current distance to target).

Crucially, an action client can cancel a goal that is already in progress (preemption).

### Action Definition (`.action` file)

An action definition uses a similar structure to services, but with three sections separated by `---`.

**Example: `my_robot_interfaces/action/MoveRobot.action`**

```
# Goal
float32 x_target
float32 y_target
---
# Result
bool success
string message
---
# Feedback
float32 distance_remaining
```

### Python Action Server Example

```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from my_robot_interfaces.action import MoveRobot
import time

class MoveRobotActionServer(Node):
    def __init__(self):
        super().__init__('move_robot_action_server')
        self._action_server = ActionServer(
            self,
            MoveRobot,
            'move_robot',
            self.execute_callback
        )
        self.get_logger().info('Move Robot Action Server ready.')

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        feedback_msg = MoveRobot.Feedback()
        
        # Simulate movement
        distance_to_target = ((goal_handle.request.x_target - 0)**2 + (goal_handle.request.y_target - 0)**2)**0.5
        current_distance = distance_to_target
        
        while current_distance > 0.1: # While not close enough to target
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled.')
                return MoveRobot.Result(success=False, message='Goal canceled.')

            current_distance -= 0.5 # Simulate progress
            feedback_msg.distance_remaining = current_distance
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Feedback: {current_distance:.2f} m remaining')
            time.sleep(0.5)

        goal_handle.succeed()
        result = MoveRobot.Result()
        result.success = True
        result.message = 'Robot reached target!'
        self.get_logger().info('Goal succeeded!')
        return result

def main(args=None):
    rclpy.init(args=args)
    node = MoveRobotActionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Python Action Client Example

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from my_robot_interfaces.action import MoveRobot
import sys

class MoveRobotActionClient(Node):
    def __init__(self):
        super().__init__('move_robot_action_client')
        self._action_client = ActionClient(self, MoveRobot, 'move_robot')

    def send_goal(self, x, y):
        goal_msg = MoveRobot.Goal()
        goal_msg.x_target = float(x)
        goal_msg.y_target = float(y)

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info(f'Result: {result.message}')
        else:
            self.get_logger().warn(f'Result: {result.message}')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: Distance remaining = {feedback.distance_remaining:.2f} m')

def main(args=None):
    rclpy.init(args=args)
    action_client = MoveRobotActionClient()
    
    if len(sys.argv) != 3:
        action_client.get_logger().info('Usage: ros2 run my_robot_bringup move_robot_client <x_target> <y_target>')
        return

    action_client.send_goal(sys.argv[1], sys.argv[2])
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
```

By judiciously choosing between Topics, Services, and Actions, developers can build robust and flexible communication architectures for complex robotic behaviors, from simple sensor streams to intricate, goal-driven mission plans.
