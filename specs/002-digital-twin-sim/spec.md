# Feature Specification: The Digital Twin (Gazebo & Unity)

**Feature Branch**: `002-digital-twin-sim`
**Created**: 2025-12-08
**Status**: Draft
**Input**: User description (Module 2: The Digital Twin)

## User Scenarios & Testing *(mandatory)*

### User Story 1 - High-Fidelity Physics Simulation (Priority: P1)

A student sets up a Gazebo (Harmonic/Garden) environment to simulate the physical dynamics of a humanoid robot, ensuring it behaves realistically under gravity and contact forces.

**Why this priority**: Without a physics-accurate simulation, control algorithms cannot be safely developed or tested.

**Independent Test**: The humanoid model stands stably in the Gazebo simulation without exploding or jittering, and reacts to applied forces (pushes) realistically.

**Acceptance Scenarios**:
1.  **Given** a fresh Gazebo installation, **When** the `humanoid_world` launch file is executed, **Then** the robot spawns in the center of the lab environment.
2.  **Given** the simulation running, **When** gravity is verified (dropping an object), **Then** the object falls at ~9.81 m/s².
3.  **Given** the robot model, **When** mass/inertia properties are inspected, **Then** they match the specified physical design.

---

### User Story 2 - Sensor Suite Simulation (Priority: P1)

A student equips the simulated robot with essential sensors (LiDAR, Camera, IMU) to generate data for the perception system.

**Why this priority**: Perception algorithms need synthetic data that mimics real-world sensor outputs.

**Independent Test**: ROS 2 topics for camera images, LiDAR scans, and IMU data are publishing active, valid data from the Gazebo simulation.

**Acceptance Scenarios**:
1.  **Given** the robot with a camera plugin, **When** the `/camera/image_raw` topic is viewed, **Then** a clear view of the Gazebo world is visible.
2.  **Given** the robot with a LiDAR, **When** viewing `/scan` in RViz, **Then** laser points correctly outline the obstacles in the environment.
3.  **Given** the robot with an IMU, **When** the robot tilts, **Then** the `/imu/data` orientation quaternions update accordingly.

---

### User Story 3 - Unity Visualization & ROS Bridge (Priority: P2)

A student creates a visually high-fidelity "Digital Twin" in Unity that mirrors the state of the robot in ROS 2, using the ROS TCP Connector.

**Why this priority**: Unity provides superior visualization and user interaction capabilities compared to standard engineering tools, essential for "Digital Twin" interfaces.

**Independent Test**: Moving the robot joints in ROS 2 (or Gazebo) causes the corresponding Digital Twin in Unity to move in real-time synchronization.

**Acceptance Scenarios**:
1.  **Given** a running ROS 2 system and Unity scene, **When** the ROS TCP connection is initiated, **Then** the "Connected" status is displayed in Unity.
2.  **Given** joint state updates from ROS, **When** received by Unity, **Then** the Unity articulation body updates its pose to match.
3.  **Given** a complex scene (Living Room), **When** loaded in Unity, **Then** the frame rate remains above 30 FPS on an RTX workstation.

---

### User Story 4 - The Digital Twin Workflow (Priority: P3)

A student follows the full lifecycle: Design (URDF) → Simulate (Gazebo) → Test (Physics/Sensors) → Deploy (Unity Viz) → Refine.

**Why this priority**: Teaches the methodological approach to modern robotics development.

**Independent Test**: A change made in the URDF design is successfully propagated through the pipeline to appear in both Gazebo and Unity.

**Acceptance Scenarios**:
1.  **Given** a modification to the robot's arm length in URDF, **When** the simulation assets are rebuilt, **Then** the new length is reflected in Gazebo.
2.  **Given** the updated asset, **When** imported into Unity, **Then** the visualization matches the new design.

### Edge Cases

-   **High Computational Load**: Running Gazebo and Unity simultaneously on lower-end hardware. (Handling: Provide "Low Quality" simulation presets and instruction to run on separate machines if needed).
-   **Physics Instability**: "Exploding" models due to bad inertia matrices. (Handling: Guide on verifying mass/inertia properties and tuning joint damping).
-   **Asset Mismatch**: Unity assets drifting from Gazebo state. (Handling: Strict versioning or script to auto-export URDF to both).

## Requirements *(mandatory)*

### Functional Requirements

-   **FR-001**: The content MUST provide validated installation steps for Gazebo (Harmonic or Garden) on Ubuntu 22.04.
-   **FR-002**: The system MUST demonstrate the conversion of the humanoid URDF (from Module 1) into SDF format for Gazebo.
-   **FR-003**: The simulation MUST support simulated sensors: RGB Camera, LiDAR (2D/3D), IMU, and Force/Torque sensors on feet.
-   **FR-004**: The content MUST provide a guide on tuning physics parameters: PID gains, joint damping/friction, and mass distribution.
-   **FR-005**: The package MUST include ROS 2 launch files that start Gazebo with the robot and all sensor bridges enabled.
-   **FR-006**: The Unity section MUST guide importing humanoid assets via Unity Robotics Hub.
-   **FR-007**: The system MUST utilize `ROS-TCP-Connector` to establish bidirectional communication between ROS 2 and Unity.
-   **FR-008**: The content MUST provide three sample simulation environments: a Lab (empty/controlled), a Living Room (cluttered), and an Obstacle Course (dynamic).
-   **FR-009**: The Digital Twin documentation MUST define the "Design → Simulate → Test → Deploy → Refine" iteration loop.

### Key Entities

-   **GazeboWorld**: SDF file defining the physics environment (lighting, gravity, static models).
-   **HumanoidSDF**: Simulation-ready robot description with `<plugin>` tags for sensors and control.
-   **UnityScene**: Interactive 3D environment containing the visualized robot.
-   **RosTcpEndpoint**: The ROS node acting as the server for Unity's client connection.

## Success Criteria *(mandatory)*

### Measurable Outcomes

-   **SC-001**: The simulated robot can stand and maintain balance (passively or with simple PD control) in Gazebo for >1 minute without falling.
-   **SC-002**: All simulated sensors (Camera, LiDAR, IMU) publish valid data at a minimum of 10 Hz.
-   **SC-003**: The Unity simulation runs at >30 FPS on an NVIDIA RTX 3060 (or equivalent) while connected to ROS 2.
-   **SC-004**: Latency between a ROS 2 command and Unity visual update is visible to the naked eye as "near real-time" (sub-100ms subjective feel).
-   **SC-005**: 100% of the provided sample environments load in Gazebo without missing texture or mesh errors.