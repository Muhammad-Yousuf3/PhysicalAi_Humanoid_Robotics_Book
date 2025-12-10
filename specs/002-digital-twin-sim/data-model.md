# Data Model: The Digital Twin (Gazebo & Unity)

**Feature Branch**: `002-digital-twin-sim`
**Created**: 2025-12-09

This document outlines the key entities and their relationships within the "Digital Twin (Gazebo & Unity)" module.

## Key Entities:

### 1. GazeboWorld

-   **Description**: An XML-based file (`.world`) that defines the complete simulation environment for Gazebo. It includes physics parameters, lights, static models (e.g., ground plane, buildings), and the robots to be spawned.
-   **Fields**:
    -   `physics`: Configuration of the physics engine (e.g., `gravity`, `ode`, `bullet`, `dart` settings).
    -   `lights`: Definitions of scene lighting (e.g., `directional`, `point`, `spot` lights with their properties).
    -   `model`: Instances of static or dynamic models (e.g., furniture, obstacles, humanoid robot). Each model references its SDF file.
    -   `scene`: Visual properties of the simulation world (e.g., `sky`, `fog`, `ambient` light).
    -   `gui`: Configuration for the Gazebo graphical user interface (e.g., camera view).
-   **Relationships**:
    -   Contains one or more `HumanoidSDF` instances.
    -   The `physics` element defines the environment for all contained models.
-   **Validation Rules**:
    -   Must be a well-formed XML file conforming to the SDF specification.
    -   Referenced models (SDF files) must exist and be valid.
    -   Physics parameters must be tuned for stability and realism.

### 2. HumanoidSDF

-   **Description**: An XML-based file (`.sdf`) that defines the full description of a humanoid robot compatible with Gazebo. It extends URDF by including elements for physics, collisions, sensors, and actuators that are directly understood by the Gazebo simulator.
-   **Fields**:
    -   `model`: Container for the robot definition.
    -   `link`: Rigid bodies composing the robot (similar to URDF, but with additional Gazebo-specific properties).
    -   `joint`: Connections between links (similar to URDF, but with additional Gazebo-specific properties like `axis`, `limit`, `dynamics`).
    -   `collision`: Defines the collision geometry of links.
    -   `visual`: Defines the visual geometry of links.
    -   `inertial`: Mass properties (mass, inertia matrix) for each link.
    -   `plugin`: Gazebo-specific plugins for sensors (e.g., `gazebo_ros_camera`, `gazebo_ros_imu`, `gazebo_ros_laser`), actuators, and controllers (`gazebo_ros_control`).
    -   `sensor`: Defines the properties of simulated sensors.
-   **Relationships**:
    -   Derived from or complements the `HumanoidURDF` from Module 1.
    -   Spawns within a `GazeboWorld`.
    -   `plugin` elements publish `SensorData` to ROS 2 topics.
-   **Validation Rules**:
    -   Must be a well-formed XML file conforming to the SDF specification.
    -   All referenced meshes/visual files must exist.
    -   Inertial properties must be accurate for stable physics simulation.
    -   Plugin configurations must be correct for ROS 2 integration.

### 3. UnityScene

-   **Description**: A Unity asset file (`.unity`) that defines a 3D environment for high-fidelity rendering and human-robot interaction. It contains 3D models, lighting, cameras, UI elements, and scripts.
-   **Fields**:
    -   `GameObjects`: All entities in the scene (e.g., `Humanoid` model, `Environment` models, `Cameras`, `Lights`).
    -   `Components`: Scripts, physics colliders, renderers, etc., attached to GameObjects.
    -   `Lighting`: Configuration for global illumination, skybox, reflections.
    -   `Cameras`: Viewpoints for rendering, including projection settings.
    -   `Scripts`: Custom C# code for robot control, ROS 2 communication, and UI.
-   **Relationships**:
    -   Contains the visual representation of the humanoid robot (`HumanoidModel`).
    -   Connects to ROS 2 via `RosTcpEndpoint`.
    -   Can interact with `SensorData` received from ROS 2 for visualization.
-   **Validation Rules**:
    -   Must be compatible with the chosen Unity Render Pipeline (HDRP/URP).
    -   Frame rate should meet performance targets (e.g., >30 FPS).
    -   Visuals should match design specifications (Neon Cyber AI theme).

### 4. RosTcpEndpoint

-   **Description**: A conceptual ROS 2 node (often implemented by a client library or bridge within Unity) that facilitates bidirectional TCP communication between ROS 2 and Unity. It acts as the server for Unity's client.
-   **Fields**:
    -   `ip_address`: IP address of the machine running the ROS 2 endpoint.
    -   `port`: TCP port for communication.
    -   `message_types`: List of ROS 2 message types being exchanged.
    -   `connections`: Status of active TCP connections.
-   **Relationships**:
    -   Exchanges `SensorData` and control commands with `UnityScene`.
    -   Acts as a bridge between the ROS 2 graph and the Unity application.
-   **Validation Rules**:
    -   Must maintain stable, low-latency connection between ROS 2 and Unity.
    -   Correctly serialize and deserialize ROS 2 messages for TCP transport.
    -   Handle connection/disconnection events gracefully.

### 5. SensorData (Expanded for Digital Twin)

-   **Description**: Standardized ROS 2 messages representing simulated data from various sensors within the digital twin.
-   **Types (examples)**:
    -   `sensor_msgs/Image`: For RGB, depth, and segmentation images from simulated cameras.
    -   `sensor_msgs/PointCloud2`: For LiDAR scan data.
    -   `sensor_msgs/Imu`: For Inertial Measurement Unit data.
    -   `geometry_msgs/WrenchStamped`: For Force/Torque sensor data.
-   **Fields**: Varies by message type, but typically includes:
    -   `Header`: Contains `timestamp` and `frame_id`.
    -   Sensor-specific data (e.g., `data` for `Image`, `points` for `PointCloud2`).
-   **Relationships**:
    -   Published by `HumanoidSDF` plugins in `GazeboWorld`.
    -   Subscribed by ROS 2 perception nodes and optionally visualized in `UnityScene`.
-   **Validation Rules**:
    -   Data types must conform to specified ROS 2 message definitions.
    -   `frame_id` should correctly map to the robot's kinematic tree.
    -   Data should reflect simulated physical conditions (e.g., depth values should correspond to distances to objects).
    -   Should include appropriate noise models.
