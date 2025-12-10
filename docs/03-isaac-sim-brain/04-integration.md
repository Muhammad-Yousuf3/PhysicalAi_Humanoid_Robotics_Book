---
title: "Integration & Sim-to-Real"
slug: /isaac-sim-brain/integration
---

# 3.5 Integration: The Feedback Loop

We have defined the components: Isaac Sim (World), Isaac ROS (Eyes), and Nav2 (Mind). Now, we must close the loop.

## The Architecture of a Digital Twin

In a fully integrated simulation, data flows in a continuous cycle:

1.  **Isaac Sim** renders a frame and publishes it to the `/camera/rgb` topic.
2.  **Isaac ROS VSLAM** receives the image, updates the robot's pose (`map -> odom`), and publishes it to `/tf`.
3.  **Nav2** receives the pose and the map, plans a path, and outputs velocity commands to `/cmd_vel`.
4.  **Isaac Sim** (via the ROS Bridge) listens to `/cmd_vel` and applies forces to the robot's virtual wheels/joints, moving it in the simulation.
5.  **Repeat**.

![System Architecture Diagram](placeholder:FIG-3.4-system-architecture)
*Figure 3.4: The Complete Architecture. The ROS Bridge acts as the translator between the 'ground truth' physics of Isaac Sim (USD) and the 'perceived reality' of the robot software (ROS 2).*

## The Reality Gap: Sim-to-Real

If your code works perfectly in simulation, will it work on the real robot? **Maybe.**

This uncertainty is called the **Reality Gap**. It exists because simulation is an idealization. To bridge it, we must model the imperfections of reality:

*   **Latency**: In Sim, data transfer is instant. In reality, USB cables and WiFi introduce delays. We must simulate this latency to ensure our controllers are stable.
*   **Sensor Noise**: Real cameras have motion blur and grain. Real LiDAR has scatter. We use "Domain Randomization" (discussed in Section 3.2) to train our perception systems to ignore this noise.
*   **Physics Divergence**: The friction coefficient of your virtual floor is perfectly uniform. The real floor has dust, bumps, and carpet.

**Best Practice**: Always assume your simulation is "too easy." Make your simulation *harder* than reality (more noise, more latency, lower friction) to ensure robust performance when you deploy to hardware.
