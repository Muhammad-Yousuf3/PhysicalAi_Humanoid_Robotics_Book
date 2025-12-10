# Data Model: The Robotic Nervous System (ROS 2)

**Feature Branch**: `001-ros2-nervous-system`
**Created**: 2025-12-09

This document outlines the key entities and their relationships within the "Robotic Nervous System" module.

## Key Entities:

### 1. HumanoidURDF

-   **Description**: XML description of the robot's kinematics, visuals, collisions, and dynamic properties. This serves as the digital blueprint of the humanoid robot.
-   **Fields**:
    -   `links`: Rigid bodies (e.g., `head`, `torso`, `upper_arm`, `lower_arm`, `hand`, `upper_leg`, `lower_leg`, `foot`). Each link has mass, inertia, and geometric properties.
    -   `joints`: Connections between links, defining degrees of freedom (e.g., `revolute`, `prismatic`, `fixed`). Each joint has a type, parent/child link, axis of rotation, limits (position, velocity, effort), and dynamics (friction, damping).
    -   `sensors`: Definition of sensors attached to specific links (e.g., `camera`, `imu`). Includes sensor type, placement, and associated ROS 2 topics.
    -   `actuators`: (Implicitly defined by joints and controllers) mechanisms that apply force/torque to joints.
    -   `visual`: Geometric representation for rendering (e.g., mesh, primitive shapes).
    -   `collision`: Geometric representation for physics simulation and collision detection.
-   **Relationships**:
    -   Hierarchical structure (tree-like): `parent_link` to `child_link` via `joint`.
    -   Referenced by `RosNode` (e.g., `robot_state_publisher` reads URDF).
-   **Validation Rules**:
    -   Must be well-formed XML.
    -   Must pass `check_urdf` validation (kinematic chain, joint limits, unique names).
    -   Links and joints must have unique names.
    -   Inertial properties must be physically plausible.
    -   Collision geometries should accurately represent physical interaction surfaces.

### 2. RosNode

-   **Description**: A Python class inheriting from `rclpy.node.Node`, representing an autonomous computational unit within the ROS 2 graph.
-   **Fields**:
    -   `name`: Unique identifier for the node instance.
    -   `publishers`: Dictionary of `rclpy.publisher.Publisher` objects, each associated with a topic and message type.
    -   `subscribers`: Dictionary of `rclpy.subscriber.Subscriber` objects, each associated with a topic, message type, and callback function.
    -   `services`: Dictionary of `rclpy.service.Service` objects (server side) or `rclpy.client.Client` objects (client side).
    -   `actions`: Dictionary of `rclpy.action.ActionServer` objects (server side) or `rclpy.action.ActionClient` objects (client side).
    -   `parameters`: Dynamic configuration values.
    -   `timers`: Callbacks executed at fixed intervals.
-   **Relationships**:
    -   Interacts with other `RosNode` instances via `topics`, `services`, and `actions`.
    -   Implements logic for `SensorData` processing, control algorithms, and AI agent bridging.
-   **Validation Rules**:
    -   Node names must be unique within the ROS 2 domain.
    -   Message types used by publishers/subscribers must match.
    -   Service/Action types must match between client and server.
    -   Must gracefully handle shutdown and lifecycle transitions if a LifecycleNode.

### 3. LaunchFile

-   **Description**: A Python script (`.launch.py`) used to define and orchestrate the startup of a collection of ROS 2 nodes, executables, and other launch files.
-   **Fields**:
    -   `nodes`: List of `Node` declarations, specifying package, executable, name, parameters, and remappings.
    -   `actions`: List of launch actions (e.g., `IncludeLaunchDescription`, `ExecuteProcess`).
    -   `parameters`: Global or node-specific parameters.
    -   `remappings`: Mapping of topic/service names.
-   **Relationships**:
    -   Starts and manages the lifecycle of multiple `RosNode` instances.
    -   Can include other `LaunchFile`s for modularity.
    -   Often launches `robot_state_publisher` which reads `HumanoidURDF`.
-   **Validation Rules**:
    -   Must be valid Python syntax.
    -   Must correctly specify package and executable names.
    -   Dependencies between nodes should be handled (e.g., `remappings` or explicit `ComposableNode` definitions).

### 4. SensorData

-   **Description**: Standardized ROS 2 messages representing data from various sensors.
-   **Types (examples)**:
    -   `sensor_msgs/Image`: For camera data (pixel array, encoding, frame_id).
    -   `sensor_msgs/Imu`: For Inertial Measurement Unit data (orientation, angular velocity, linear acceleration, covariances).
    -   `sensor_msgs/JointState`: For reporting joint positions, velocities, and efforts.
    -   `geometry_msgs/Twist`: For command velocities (linear and angular components).
-   **Fields**: Varies by message type, but typically includes:
    -   `Header`: Contains `timestamp` and `frame_id`.
    -   Sensor-specific data (e.g., `data` for `Image`, `orientation` for `Imu`).
-   **Relationships**:
    -   Published by `RosNode`s (e.g., `camera_publisher_node`).
    -   Subscribed by other `RosNode`s (e.g., `AI Planner Node`, `High-Level Controller Node`).
-   **Validation Rules**:
    -   Data types must conform to the specified ROS 2 message definition.
    -   `frame_id` in header should correspond to a valid TF frame in the robot's URDF.
    -   Timestamp should be monotonically increasing.
