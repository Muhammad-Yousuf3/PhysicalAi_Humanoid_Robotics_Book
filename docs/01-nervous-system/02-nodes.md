# Section 2: ROS 2 Nodes – The Building Blocks of Robot Intelligence

In ROS 2, a **Node** is the fundamental unit of computation. Conceptually, a node is simply an executable program that performs a specific task, such as reading data from a sensor, controlling a motor, running an AI algorithm, or providing a user interface. Multiple nodes can run concurrently and communicate with each other to form the complete robotic system.

## 2.1. What is a Node?

A ROS 2 node is essentially a process that uses the ROS 2 client libraries (like `rclpy` for Python or `rclcpp` for C++) to interface with the ROS 2 graph. Each node:

-   Has a unique name within its ROS domain.
-   Can publish data to topics.
-   Can subscribe to data from topics.
-   Can offer services and act as a service client.
-   Can offer actions and act as an action client.
-   Can declare and manage parameters.
-   Can utilize timers for periodic execution.

The modular nature of nodes promotes reusability and simplifies development. For instance, a robot might have separate nodes for:
-   `camera_driver`: Publishes images from the camera.
-   `object_detector`: Subscribes to images, publishes detected objects.
-   `navigation_planner`: Subscribes to map data and robot pose, publishes navigation goals.
-   `motor_controller`: Subscribes to velocity commands, publishes motor states.

## 2.2. Node Creation (Python Example)

Here's a basic Python example demonstrating how to create a ROS 2 node that simply logs a message.

```python
import rclpy
from rclpy.node import Node

class MyFirstNode(Node):
    def __init__(self):
        super().__init__('my_first_node') # Initialize the node with a unique name
        self.get_logger().info('Hello from MyFirstNode!')

def main(args=None):
    rclpy.init(args=args)         # Initialize the ROS client library
    node = MyFirstNode()          # Create the node
    rclpy.spin(node)              # Keep the node alive until it's explicitly shut down
    node.destroy_node()           # Clean up when done
    rclpy.shutdown()              # Shut down the ROS client library

if __name__ == '__main__':
    main()
```

To make this node executable, you would typically add an entry point in your package's `setup.py` file, similar to:
```python
entry_points={
    'console_scripts': [
        'my_first_node = my_package.my_first_node:main'
    ],
},
```

## 2.3. Publishers and Subscribers

Nodes primarily communicate using a publish-subscribe model.

-   **Publisher**: A node that sends messages to a named **Topic**.
-   **Subscriber**: A node that receives messages from a named **Topic**.

When a publisher publishes data to a topic, all nodes subscribed to that topic will receive the data. This is a one-to-many, asynchronous communication pattern, ideal for continuous data streams.

### Python Publisher Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String # Standard message type for a string

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        # Create a publisher to the 'my_topic' topic, using String messages, with a queue size of 10
        self.publisher_ = self.create_publisher(String, 'my_topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello ROS 2! Count: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = SimplePublisher()
    rclpy.spin(node) # Keeps node running, executing callbacks
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Python Subscriber Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        # Create a subscriber to the 'my_topic' topic, expecting String messages
        self.subscription = self.create_subscription(
            String,
            'my_topic',
            self.listener_callback, # Callback function to execute when a message is received
            10 # QoS queue size
        )
        self.subscription # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 2.4. Lifecycle Nodes

For robust and highly available robotic systems, especially those designed for long-term operation or safety-critical applications, ROS 2 introduces **Lifecycle Nodes**. These are managed nodes that expose a state machine, allowing external managers to control their transitions between states (e.g., `unconfigured`, `inactive`, `active`, `finalized`).

**Key States and Transitions:**

-   **`unconfigured`**: Initial state after creation. Node is initialized but not yet configured (e.g., resources not allocated).
    -   `configure()`: Attempts to configure the node. If successful, transitions to `inactive`.
-   **`inactive`**: Node is configured and ready to be activated, but not yet processing data or communicating.
    -   `activate()`: Attempts to activate the node. If successful, transitions to `active`.
    -   `deactivate()`: Transitions back to `unconfigured`.
-   **`active`**: Node is fully operational, publishing/subscribing, and executing its main logic.
    -   `deactivate()`: Attempts to deactivate the node. If successful, transitions to `inactive`.
-   **`error processing`**: A state entered if a transition fails.
-   **`finalized`**: Terminal state after shutdown.

**Benefits of Lifecycle Nodes:**

-   **Deterministic Startup/Shutdown**: Ensures components are initialized and shut down in a controlled, predictable manner.
-   **Resource Management**: Allows resources (e.g., camera feeds, motor connections) to be allocated only when needed (`configure` state) and released when not in use (`deactivate` state).
-   **Fault Tolerance**: Enables graceful handling of errors and allows for recovery or restart of specific components without affecting the entire system.
-   **System-level Control**: Managers can orchestrate the startup of an entire robot system, ensuring dependencies are met and components come online in the correct order.

While standard nodes are sufficient for many tasks, understanding and utilizing lifecycle nodes is crucial for developing professional, resilient ROS 2 applications.
