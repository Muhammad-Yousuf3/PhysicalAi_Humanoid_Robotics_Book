# Research Plan: The Digital Twin (Gazebo & Unity)

**Feature Branch**: `002-digital-twin-sim`
**Created**: 2025-12-09

This document outlines the key research areas to be covered concurrently with the writing process for Module 2. Findings and decisions will be documented here iteratively.

## Identified Research Nodes:

-   **Gazebo Physics Engines Comparison**:
    -   Detailed comparison of ODE, Bullet, and DART physics engines within Gazebo.
    -   Focus on performance, stability, accuracy, and specific features relevant to humanoid robotics (e.g., contact dynamics, joint limits).
    -   Investigate their integration with Gazebo Ignition (Harmonic).
-   **Simulation Resolution Tradeoffs**:
    -   Research the impact of simulation time steps, update rates, and collision detection granularity on the accuracy and real-time performance of Gazebo simulations.
    -   Optimal settings for humanoid robot stability and interaction fidelity.
-   **Unity Rendering Pipelines (URP vs HDRP)**:
    -   In-depth study of Universal Render Pipeline (URP) and High Definition Render Pipeline (HDRP).
    -   Assessment of their capabilities for achieving high-fidelity visuals suitable for digital twin applications, considering performance implications.
    -   Best practices for material creation, lighting, and post-processing effects in HDRP.
-   **ROS–Unity Bridge Architectures**:
    -   Comprehensive analysis of `ROS-TCP-Connector`, its internal workings, data serialization/deserialization, and performance characteristics.
    -   Exploration of alternative custom Python-based bridges (e.g., using `rclpy` with standard TCP/IP sockets) for scenarios requiring high customization or specific optimizations.
    -   Comparison of latency, throughput, and ease of use.
-   **Sensor Modeling from Robotics Textbooks and Academic Sources**:
    -   Review of theoretical models for LiDAR (raycasting, beam characteristics, return intensity).
    -   Study of depth camera principles (structured light, time-of-flight, stereo vision) and GPU-based depth map generation.
    -   Analysis of IMU noise characteristics (Gaussian noise, bias, drift, scale factor errors) and integration with physics-derived data.
    -   Identification of peer-reviewed papers for advanced noise models and calibration techniques.
-   **Humanoid Asset Pipeline (URDF to SDF, FBX/glTF for Unity)**:
    -   Research best practices for converting or adapting URDF models to SDF for Gazebo, including adding Gazebo-specific plugins for sensors and controllers.
    -   Guidelines for exporting 3D models (meshes) from CAD software to FBX or glTF formats for optimal import and rendering in Unity.
    -   Considerations for maintaining coordinate systems, scales, and PBR material definitions across platforms.
