# API Contract: ROS 2 - Unity Communication (ROS-TCP-Connector)

**Feature Branch**: `002-digital-twin-sim`
**Created**: 2025-12-09

This document describes the conceptual API contracts for communication between ROS 2 and Unity, primarily using the `ROS-TCP-Connector`.

## 1. JointState Synchronization (ROS Topic)

This is a bidirectional synchronization. ROS 2 will publish `JointState` messages for the robot's current joint positions, and Unity will subscribe to visualize these. Conversely, Unity might publish `JointCommand` messages (or similar) to control the robot in ROS 2.

### 1.1. ROS 2 to Unity (Joint State Publisher)

#### Message Type: `sensor_msgs/JointState`

```yaml
# sensor_msgs/JointState
std_msgs/Header header
  time stamp
  string frame_id
string[] name            # Joint names
float64[] position       # Joint positions (radians or meters)
float64[] velocity       # Joint velocities (radians/s or meters/s)
float64[] effort         # Joint efforts (Nm or N)
```

**Description**:
-   **Header**: Timestamp of the joint state data. `frame_id` might be empty or refer to the base frame.
-   **name**: An array of strings representing the names of the joints.
-   **position**: An array of floats representing the position of each joint.
-   **velocity**: An array of floats representing the velocity of each joint.
-   **effort**: An array of floats representing the effort (force/torque) applied to each joint.

**Communication Direction**: ROS 2 (e.g., from Gazebo `joint_state_publisher` or a controller) -> Unity.
**Purpose**: Update the visual model in Unity to match the physical state in Gazebo/ROS.

### 1.2. Unity to ROS 2 (Joint Command Subscriber)

#### Message Type: `std_msgs/Float64MultiArray` (Conceptual, or a custom JointCommand message)

```yaml
# std_msgs/Float64MultiArray (example for simplicity)
std_msgs/MultiArrayLayout layout
  std_msgs/MultiArrayDimension[] dim
    string label        # "joint_names", "joint_positions"
    uint32 size         # number of elements in this dimension
    uint32 stride       # stride for this dimension
uint32 data_offset      # offset in to data where first element is found
float64[] data          # array of actual data
```

**Description**:
-   **layout**: Describes the array dimensions and strides.
-   **data**: An array of floats representing joint commands (e.g., target positions, velocities, or efforts). The `layout.dim` can be used to associate these with joint names.

**Communication Direction**: Unity (e.g., from human interaction in the digital twin) -> ROS 2 (e.g., to a controller node).
**Purpose**: Allow user input in Unity to command the robot in the ROS 2 simulation.

## 2. Sensor Data Streams (ROS Topics)

Simulated sensor data from Gazebo will be published to ROS 2 topics, and Unity can optionally subscribe to these for debugging or advanced visualization.

### 2.1. LiDAR Data

#### Message Type: `sensor_msgs/PointCloud2`

```yaml
# sensor_msgs/PointCloud2
std_msgs/Header header
uint32 height
uint32 width
sensor_msgs/PointField[] fields # Describes how to interpret the data
bool is_bigendian
uint32 point_step # Length of a point in bytes
uint32 row_step   # Length of a row in bytes
uint8[] data      # Actual point data
bool is_dense     # True if no invalid points
```

**Description**:
-   **header**: Timestamp and frame of reference for the LiDAR scan.
-   **height, width**: Dimensions of the point cloud.
-   **fields**: Defines the structure of each point (e.g., x, y, z, intensity, RGB).
-   **data**: Raw byte array containing the point cloud data.

**Communication Direction**: Gazebo (via ROS 2 plugin) -> ROS 2 -> Unity (optional subscription).
**Purpose**: Provide simulated environmental perception data.

### 2.2. Depth Camera Data

#### Message Type: `sensor_msgs/Image`

```yaml
# sensor_msgs/Image
std_msgs/Header header
uint32 height
uint32 width
string encoding       # e.g., "mono8", "rgb8", "32FC1" (for depth)
uint8 is_bigendian
uint32 step          # Full row length in bytes
uint8[] data         # Raw image data
```

**Description**:
-   **header**: Timestamp and frame of reference for the image.
-   **height, width**: Image dimensions.
-   **encoding**: Specifies the pixel format (e.g., `32FC1` for float depth values, `mono16` for unsigned short depth).
-   **data**: Raw byte array of image pixels.

**Communication Direction**: Gazebo (via ROS 2 plugin) -> ROS 2 -> Unity (optional subscription).
**Purpose**: Provide simulated depth information for perception algorithms.

### 2.3. IMU Data

#### Message Type: `sensor_msgs/Imu`

```yaml
# sensor_msgs/Imu
std_msgs/Header header
geometry_msgs/Quaternion orientation
  float64 x, y, z, w
float64[9] orientation_covariance # Row major about x, y, z axes
geometry_msgs/Vector3 angular_velocity
  float64 x, y, z
float64[9] angular_velocity_covariance # Row major about x, y, z axes
geometry_msgs/Vector3 linear_acceleration
  float64 x, y, z
float64[9] linear_acceleration_covariance # Row major about x, y, z axes
```

**Description**:
-   **header**: Timestamp and frame of reference for the IMU data.
-   **orientation**: Quaternion representing the orientation.
-   **angular_velocity**: Vector representing angular velocity.
-   **linear_acceleration**: Vector representing linear acceleration.
-   **covariances**: Uncertainty associated with each measurement.

**Communication Direction**: Gazebo (via ROS 2 plugin) -> ROS 2 -> Unity (optional subscription).
**Purpose**: Provide simulated proprioceptive data for robot state estimation.

## 3. Digital Twin Workflow Commands (ROS Services / Actions - Conceptual)

These are conceptual interactions that enable controlling the digital twin's state or requesting specific actions.

### 3.1. Reset Simulation (ROS Service)

#### Service Type: `std_srvs/Empty` (example for simplicity)

```yaml
# std_srvs/Empty.srv
---
```

**Description**:
-   **Request**: Empty.
-   **Response**: Empty.

**Communication Direction**: ROS 2 (e.g., from a test script or AI Agent) -> Gazebo (via ROS 2 plugin).
**Purpose**: Reset the Gazebo simulation to its initial state.

### 3.2. Spawn Model (ROS Service)

#### Service Type: `gazebo_msgs/SpawnEntity` (conceptual, might use `spawn_entity` service)

```yaml
# gazebo_msgs/SpawnEntity.srv
string name
string xml
string robot_namespace
geometry_msgs/Pose initial_pose
---
bool success
string status_message
```

**Description**:
-   **name**: Name of the model to spawn.
-   **xml**: XML description of the model (SDF or URDF).
-   **initial_pose**: Desired pose of the spawned model.

**Communication Direction**: ROS 2 -> Gazebo.
**Purpose**: Dynamically add models to the Gazebo simulation.

## Versioning Strategy

-   ROS 2 message definitions adhere to their official specifications.
-   Any custom message definitions (`.action`, `.srv`, `.msg` files) will be versioned within their respective ROS 2 packages.
-   Communication protocols used by `ROS-TCP-Connector` are implicitly versioned with the connector itself.

## Idempotency, Timeouts, Retries

-   **ROS Topics**: Publish-subscribe communication is inherently not idempotent. Data streams (sensor data, joint states) are continuous.
-   **ROS Services**: Services like `ResetSimulation` or `SpawnModel` should be designed to be idempotent where logical (e.g., spawning an entity with the same name might overwrite or return an error). Timeouts and retries for service calls are handled by the ROS 2 client library.
-   **ROS Actions**: Actions provide built-in mechanisms for goal tracking, feedback, results, and preemption, effectively managing long-running tasks and their idempotency considerations.

## Error Taxonomy with Status Codes

-   **ROS 2 Standard Errors**: ROS 2 nodes will use standard logging levels (DEBUG, INFO, WARN, ERROR, FATAL) for internal issues.
-   **ROS Service/Action Specific Errors**: Custom services/actions can define their own status codes and messages (e.g., `success` boolean and `status_message` string in `SpawnEntity.srv`).
    -   `SIMULATION_ERROR`: Generic error during Gazebo interaction.
    -   `RENDERING_ERROR`: Issues with Unity rendering or asset loading.
    -   `COMMUNICATION_ERROR`: Problem with `ROS-TCP-Connector` or other bridges.
    -   `INVALID_INPUT_ERROR`: Malformed commands or data from ROS to simulation.
