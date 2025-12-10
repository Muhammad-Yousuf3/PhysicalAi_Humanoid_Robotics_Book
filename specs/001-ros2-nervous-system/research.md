# Research Plan: The Robotic Nervous System (ROS 2)

**Feature Branch**: `001-ros2-nervous-system`
**Created**: 2025-12-09

This document outlines the key research areas to be covered concurrently with the writing process for Module 1. Findings and decisions will be documented here iteratively.

## Identified Research Nodes:

-   **ROS middleware behavior**:
    -   Detailed workings of the DDS, discovery protocols, and transport layers.
    -   Key QoS policies and their impact on different communication scenarios (e.g., sensor data vs. command signals).
    -   Differences and nuances between popular DDS implementations (Cyclone DDS, FastDDS).
-   **rclpy Python API functions**:
    -   Comprehensive exploration of `rclpy` API for:
        -   Node creation and management.
        -   Publisher and subscriber patterns for various message types.
        -   Service client and server implementation.
        -   Action client and server implementation, including feedback and preemption.
        -   Parameter handling (getting, setting, declaring).
        -   Timer callbacks and multi-threaded executors.
-   **URDF specification**:
    -   Official URDF XML schema and element definitions (e.g., `<link>`, `<joint>`, `<inertial>`, `<visual>`, `<collision>`, `<sensor>`, `<actuator>`).
    -   Best practices for creating well-structured and reusable URDF models.
    -   Integration of URDF with other tools (RViz2, Gazebo).
    -   Differences between URDF and XACRO.
-   **Humanoid modeling best practices**:
    -   Guidelines for creating kinematically and dynamically accurate URDF models for humanoid robots.
    -   Correct specification of inertial properties (mass, center of mass, inertia matrix).
    -   Defining realistic joint limits and types (revolute, prismatic, fixed).
    -   Integration of sensors (camera, IMU) and actuators into the URDF model.
    -   Strategies for visual and collision model creation.
