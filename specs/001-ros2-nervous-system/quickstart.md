# Quickstart: The Robotic Nervous System (ROS 2)

**Feature Branch**: `001-ros2-nervous-system`
**Created**: 2025-12-09

This document will provide a quick start guide for setting up and running the core components of the ROS 2 Robotic Nervous System module.

## 1. Setup ROS 2 Environment

*(Details on installing ROS 2 Humble on Ubuntu 22.04 will go here)*

## 2. Clone the Repository

```bash
git clone <repository_url>
cd PhysicalAi_Humanoid_Robotics_Book/code-examples/01-nervous-system/ros2_ws
```

## 3. Build the ROS 2 Workspace

```bash
colcon build --symlink-install
```

## 4. Source the Workspace

```bash
source install/setup.bash
```

## 5. Launch the Mini Humanoid System

*(Details on launching the example system with URDF, RVIz2, and basic nodes will go here)*

```bash
ros2 launch my_robot_bringup robot_launch.py
```

## 6. Run a Simple AI Agent Command

*(Details on how to send a command to the LLM bridge node will go here)*

```bash
ros2 run ros2_py_agent llm_bridge_node --ros-args -p command:="move forward"
```
