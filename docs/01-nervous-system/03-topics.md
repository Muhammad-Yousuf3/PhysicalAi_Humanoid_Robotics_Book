# Section 3: ROS 2 Topics – The Asynchronous Data Streams

**Topics** are the primary mechanism for asynchronous, many-to-many communication in ROS 2. They represent named channels over which nodes can publish messages and other nodes can subscribe to receive those messages. This pattern is ideal for continuous data streams like sensor readings, joint states, or velocity commands, where the sender doesn't necessarily need to know who is receiving the data, and receivers don't need to acknowledge every message.

## 3.1. How Topics Work

1.  **Publisher**: A node creates a publisher for a specific topic and message type. It then continuously or periodically sends messages to this topic.
2.  **Subscriber**: A node creates a subscriber for the same topic and message type. It registers a callback function that is executed every time a new message arrives on that topic.
3.  **DDS in Action**: Underneath, the DDS middleware ensures that messages published to a topic are efficiently routed to all active subscribers with compatible Quality of Service (QoS) settings. There is no central server managing topics; communication is direct between publishers and subscribers (peer-to-peer).

## 3.2. Message Definitions

Every message transmitted over a ROS 2 topic must conform to a predefined structure, known as a **Message Definition**. These definitions are typically stored in `.msg` files within a ROS 2 package and describe the data types and names of the fields that comprise a message.

### Anatomy of a `.msg` file

A `.msg` file is a plain text file where each line defines a field:
-   `field_type field_name`

Supported `field_type`s include:
-   **Built-in primitive types**: `bool`, `byte`, `char`, `float32`, `float64`, `int8`, `uint8`, `int16`, `uint16`, `int32`, `uint32`, `int64`, `uint64`, `string`, `wstring` (wide string), `time`, `duration`.
-   **Arrays**: Fixed-size (`float32[4]`) or variable-size (`string[]`).
-   **Other message types**: You can embed other message types (e.g., `std_msgs/Header`, `geometry_msgs/Pose`).

Comments are denoted by `#`.

**Example: `custom_interfaces/msg/RobotStatus.msg`**

Let's imagine we want a message to report the status of a robot.

```
# custom_interfaces/msg/RobotStatus.msg
std_msgs/Header header          # Standard header for timestamp and frame_id
string robot_name               # Name of the robot
float32 battery_percentage      # Current battery level (0.0 to 1.0)
string current_mode             # Operational mode (e.g., "idle", "exploring", "charging")
string[] active_sensors         # List of currently active sensor names
bool has_error                  # True if robot is in an error state
string error_message            # Detailed error message if has_error is true
```

After defining this `.msg` file, you would update your `CMakeLists.txt` and `package.xml` in your `custom_interfaces` package to generate the necessary code for Python and C++ to use this message type.

## 3.3. Real-World Robot Examples of Topics

In a humanoid robot, many essential pieces of information are communicated via topics:

### 1. `sensor_msgs/Image` (`/camera/image_raw`)

-   **Description**: Raw image data from a camera.
-   **Publisher**: Camera driver node (e.g., `usb_cam_node`, `isaac_sim_camera`). *In our examples, this is `ros2_py_sensors/camera_publisher_node`.*
-   **Subscriber**: Object detection node, visual odometry node, display node (RViz2).
-   **QoS**: Often `BEST_EFFORT` for high frequency and `KEEP_LAST` with `depth=1` to prioritize recency over guaranteed delivery.

### 2. `sensor_msgs/Imu` (`/imu/data`)

-   **Description**: Data from an Inertial Measurement Unit (IMU), including orientation, angular velocity, and linear acceleration.
-   **Publisher**: IMU driver node. *In our examples, this is `ros2_py_sensors/imu_publisher_node`.*
-   **Subscriber**: State estimation node (e.g., `robot_localization`), balance controller.
-   **QoS**: `BEST_EFFORT` for high frequency.

### 3. `sensor_msgs/JointState` (`/joint_states`)

-   **Description**: Current position, velocity, and effort of the robot's joints.
-   **Publisher**: Joint state publisher (reads URDF and joint sensors), low-level motor controllers.
-   **Subscriber**: Kinematics solver, motion planning node (e.g., MoveIt), RViz2 for visualization.
-   **QoS**: `BEST_EFFORT` for high frequency.

### 4. `geometry_msgs/Twist` (`/cmd_vel`)

-   **Description**: Command for linear and angular velocity, commonly used for mobile robot navigation.
-   **Publisher**: Navigation stack, teleoperation node, AI agent.
-   **Subscriber**: Base controller, motor driver node.
-   **QoS**: `RELIABLE` with `KEEP_LAST` and `depth=1` to ensure commands are not lost and only the latest command is acted upon.

### 5. `tf2_msgs/TFMessage` (`/tf`, `/tf_static`)

-   **Description**: Transforms (positions and orientations) between coordinate frames in a robot's kinematic tree. Essential for knowing where everything is relative to the robot.
-   **Publisher**: `robot_state_publisher` (reads URDF and joint states), `static_tf_publisher`, SLAM algorithms.
-   **Subscriber**: Any node that needs to know the spatial relationship between sensors, actuators, or the environment.
-   **QoS**: `BEST_EFFORT` for `/tf` (dynamic transforms), `RELIABLE` with `TRANSIENT_LOCAL` for `/tf_static` (static transforms).

By understanding and effectively utilizing ROS 2 topics and message definitions, developers can construct a robust and efficient communication infrastructure for even the most complex humanoid robots.
