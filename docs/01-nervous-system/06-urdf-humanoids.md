# Section 6: URDF for Humanoids – Building the Digital Body

The **Unified Robot Description Format (URDF)** is an XML-based file format used in ROS to describe the physical structure of a robot. It defines the robot's kinematic and dynamic properties, including its physical links (rigid bodies), joints (connections between links), and can also include sensors and actuators. For humanoid robots, URDF is crucial for modeling their complex anatomy, enabling simulation, visualization, and kinematic calculations.

## 6.1. Core URDF Elements

A URDF file typically starts with a `<robot>` tag and contains several key elements:

-   **`<link>`**: Represents a rigid body segment of the robot. Each link has properties like:
    -   **`<visual>`**: Defines the graphical appearance of the link (e.g., mesh, primitive shape, color). Used for rendering in visualization tools like RViz2.
    -   **`<collision>`**: Defines the geometry used for collision detection in physics simulators. This can be different from the visual geometry to simplify calculations.
    -   **`<inertial>`**: Specifies the physical properties of the link, including `mass`, `origin` (center of mass), and `inertia` tensor. These are critical for realistic physics simulation.

-   **`<joint>`**: Defines the connection between two links (a `parent` link and a `child` link) and specifies their relative motion. Key attributes include:
    -   `name`: Unique identifier for the joint.
    -   `type`: Type of motion allowed (e.g., `revolute`, `continuous`, `prismatic`, `fixed`, `floating`, `planar`).
    -   `parent link`: The name of the parent link.
    -   `child link`: The name of the child link.
    -   **`<origin>`**: Specifies the `xyz` (position) and `rpy` (roll, pitch, yaw orientation) of the joint relative to the parent link.
    -   **`<axis>`**: For `revolute` and `prismatic` joints, defines the axis of rotation or translation.
    -   **`<limit>`**: Specifies the upper/lower position limits, velocity limits, and effort limits for the joint.
    -   **`<dynamics>`**: Defines friction and damping coefficients.

## 6.2. Example: A Simple Humanoid Torso

Let's expand on our basic URDF example to illustrate these concepts with a simple humanoid torso.

```xml
<?xml version="1.0"?>
<robot name="humanoid_torso">

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

  <!-- Torso Link -->
  <link name="torso_link">
    <visual>
      <geometry>
        <box size="0.15 0.1 0.3"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.15 0.1 0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0.15"/> <!-- Center of mass for torso link -->
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Base to Torso Joint (fixed for simplicity in this example) -->
  <joint name="base_to_torso_fixed_joint" type="fixed">
    <parent link="base_link"/>
    <child link="torso_link"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/> <!-- Torso starts 0.05m above base -->
  </joint>

  <!-- Head Link -->
  <link name="head_link">
    <visual>
      <geometry>
        <sphere radius="0.08"/>
      </geometry>
      <material name="green">
        <color rgba="0 0.8 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.08"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Torso to Head Joint (revolute, allowing head pitch) -->
  <joint name="head_pitch_joint" type="revolute">
    <parent link="torso_link"/>
    <child link="head_link"/>
    <origin xyz="0 0 0.18" rpy="0 0 0"/> <!-- Head attached to top of torso -->
    <axis xyz="0 1 0"/> <!-- Rotation around Y-axis -->
    <limit lower="-0.5" upper="0.5" effort="10.0" velocity="1.0"/>
  </joint>

</robot>
```
*   **Links**: `base_link`, `torso_link`, `head_link` - each with `visual`, `collision`, and `inertial` properties.
*   **Joints**: `base_to_torso_fixed_joint` (a `fixed` joint, meaning no relative motion) and `head_pitch_joint` (a `revolute` joint, allowing rotation around the Y-axis, simulating head nodding). Note the `origin` for each joint, which defines its position relative to its parent link.

## 6.3. Sensors and Actuators in URDF

While URDF primarily describes the mechanical structure, you can incorporate sensors and actuators conceptually, often by adding their physical housing as a link and then relying on external ROS 2 components (like `ros2_control` or dedicated driver nodes) to define their functionality.

-   **Sensors**: A common practice is to define a new link for the sensor (e.g., a camera body) and attach it to an existing robot link with a `fixed` joint. The output of this sensor is then typically handled by a ROS 2 driver node that publishes to a topic.
    -   *Example*: A camera attached to the `head_link`. In our examples, `ros2_py_sensors/camera_publisher_node` and `ros2_py_sensors/imu_publisher_node` will publish mock data for these sensors.

-   **Actuators**: Actuators (motors) are implicitly linked to joints as they drive the motion. `ros2_control` is the framework used to connect controllers to these joints. The URDF `limit` and `dynamics` tags are important for defining actuator capabilities.

## 6.4. Visual vs. Collision Models

It's crucial to understand the distinction between visual and collision geometries:

-   **Visual Model**: What the robot looks like. It can be a detailed mesh (`<mesh filename="package://my_robot/meshes/head.dae"/>`) for aesthetic purposes.
-   **Collision Model**: What the robot "feels" like to a physics engine. It should be simplified (e.g., primitive shapes like `<box>`, `<sphere>`, `<cylinder>`) to reduce computational load for collision detection, while still accurately representing the robot's physical extent for interaction with the environment.

Having distinct visual and collision models helps to optimize simulation performance while maintaining high-fidelity visualization.

## 6.5. Validating Your URDF

After creating or modifying a URDF file, it's essential to validate it to ensure structural correctness. ROS provides the `check_urdf` tool:

```bash
check_urdf path/to/your/robot.urdf
```

This tool will report any errors in the XML syntax, kinematic chain, or other structural issues, helping you debug your robot model.

By mastering URDF, you gain the ability to accurately represent your humanoid robot in a digital space, which is a foundational step for simulation, control, and interaction with the broader ROS 2 ecosystem.
