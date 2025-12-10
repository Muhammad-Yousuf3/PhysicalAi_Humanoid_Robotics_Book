# Quickstart: The Digital Twin (Gazebo & Unity)

**Feature Branch**: `002-digital-twin-sim`
**Created**: 2025-12-09

This document will provide a quick start guide for setting up and running the core components of the Digital Twin module, integrating Gazebo and Unity.

## 1. Setup ROS 2 and Gazebo Environment

*(Details on installing ROS 2 Humble and Gazebo Ignition on Ubuntu 22.04 will go here)*

## 2. Clone the Repository

```bash
git clone <repository_url>
cd PhysicalAi_Humanoid_Robotics_Book/code-examples/02-digital-twin-sim/gazebo_ws
```

## 3. Build the Gazebo ROS 2 Workspace

```bash
colcon build --symlink-install
source install/setup.bash
```

## 4. Launch Gazebo with Humanoid Model

*(Details on launching Gazebo with the humanoid_gazebo.urdf and sensor plugins will go here)*

```bash
ros2 launch humanoid_description display_humanoid.launch.py # This should also launch Gazebo
```

## 5. Setup Unity Project

*(Details on importing Unity project, ROS-TCP-Connector, and humanoid model will go here)*

-   Open `PhysicalAi_Humanoid_Robotics_Book/code-examples/02-digital-twin-sim/unity_project` in Unity Hub.
-   Ensure `ROS-TCP-Connector` is installed and configured.
-   Import `Humanoid.fbx` or `Humanoid.gltf` into the Unity project.

## 6. Run Unity Scene and Connect to ROS 2

*(Details on running the Unity scene and establishing connection via ROS-TCP-Connector will go here)*

-   Open `LabScene.unity` in Unity.
-   Press Play in Unity editor. The Unity robot should connect to ROS 2.
-   Verify joint states are synchronizing.
