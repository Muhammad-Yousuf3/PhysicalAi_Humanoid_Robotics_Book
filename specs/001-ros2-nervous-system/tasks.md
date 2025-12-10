# Tasks for 001-ros2-nervous-system: The Robotic Nervous System (ROS 2)

**Feature Branch**: `001-ros2-nervous-system`
**Created**: 2025-12-09

This document outlines the actionable, dependency-ordered tasks for implementing "Module 1: The Robotic Nervous System (ROS 2)". Tasks are organized into phases, with an emphasis on independent user story completion.

## Implementation Strategy

The implementation will follow a Minimum Viable Product (MVP) approach, prioritizing User Story 1 and 2 (P1) which are foundational for any ROS 2 robotic system. Subsequent user stories (P2) will build upon these foundations. Tasks are broken down to enable independent development and testing.

## Phase 1: Setup

**(General project setup tasks for the module content)**

- [x] T001 Create module directory structure in Docusaurus: docs/01-nervous-system/
- [x] T002 Create image directory for module: static/img/01-nervous-system/
- [x] T003 Create code examples base directory: code-examples/01-nervous-system/ros2_ws/src/
- [x] T004 Define and create initial Docusaurus Markdown files for section structure (00-intro.md, 01-ros2-architecture.md, etc.): docs/01-nervous-system/
- [x] T005 Update `sidebars.ts` to include the new module's navigation structure: sidebars.ts
- [x] T006 Research and draft the ROS 2 Humble installation guide for Ubuntu 22.04: docs/01-nervous-system/00-setup-ros2.md
- [x] T007 Design Neon Cyber AI visual templates for diagrams (e.g., Excalidraw files): static/img/01-nervous-system/

## Phase 2: Foundational

**(Blocking prerequisites for all user stories)**

- [x] T008 Create base ROS 2 workspace structure: code-examples/01-nervous-system/ros2_ws/
- [x] T009 Create `my_robot_bringup` package for launch files: code-examples/01-nervous-system/ros2_ws/src/my_robot_bringup/
- [x] T010 Create `package.xml` for `my_robot_bringup`: code-examples/01-nervous-system/ros2_ws/src/my_robot_bringup/package.xml
- [x] T011 Create `ros2_py_agent` package: code-examples/01-nervous-system/ros2_ws/src/ros2_py_agent/
- [x] T012 Create `package.xml` for `ros2_py_agent`: code-examples/01-nervous-system/ros2_ws/src/ros2_py_agent/package.xml
- [x] T013 Create `setup.py` for `ros2_py_agent`: code-examples/01-nervous-system/ros2_ws/src/ros2_py_agent/setup.py
- [x] T014 Create `ros2_py_controllers` package: code-examples/01-nervous-system/ros2_ws/src/ros2_py_controllers/
- [x] T015 Create `package.xml` for `ros2_py_controllers`: code-examples/01-nervous-system/ros2_ws/src/ros2_py_controllers/package.xml
- [x] T016 Create `setup.py` for `ros2_py_controllers`: code-examples/01-nervous-system/ros2_ws/src/ros2_py_controllers/setup.py
- [x] T017 Create `ros2_py_sensors` package: code-examples/01-nervous-system/ros2_ws/src/ros2_py_sensors/
- [x] T018 Create `package.xml` for `ros2_py_sensors`: code-examples/01-nervous-system/ros2_ws/src/ros2_py_sensors/package.xml
- [x] T019 Create `setup.py` for `ros2_py_sensors`: code-examples/01-nervous-system/ros2_ws/src/ros2_py_sensors/setup.py
- [x] T020 Create `urdf_tutorial` package: code-examples/01-nervous-system/ros2_ws/src/urdf_tutorial/
- [x] T021 Create `package.xml` for `urdf_tutorial`: code-examples/01-nervous-system/ros2_ws/src/urdf_tutorial/package.xml

## Phase 3: User Story 1 - Environment & Workspace Setup (P1)

**Story Goal**: A student needs to set up a robust ROS 2 development environment to begin building the robot's nervous system.
**Independent Test**: Student can run a custom publisher/subscriber example in a newly created workspace on Ubuntu 22.04.

- [x] T022 [P] [US1] Implement basic ROS 2 publisher node in Python: code-examples/01-nervous-system/ros2_ws/src/my_robot_bringup/my_robot_bringup/simple_publisher.py
- [x] T023 [P] [US1] Implement basic ROS 2 subscriber node in Python: code-examples/01-nervous-system/ros2_ws/src/my_robot_bringup/my_robot_bringup/simple_subscriber.py
- [x] T024 [US1] Create a simple launch file to run publisher and subscriber: code-examples/01-nervous-system/ros2_ws/src/my_robot_bringup/launch/simple_talker_listener.launch.py
- [x] T025 [US1] Add content for "Introduction: Why ROS 2 is the nervous system": docs/01-nervous-system/00-intro.md
- [x] T026 [US1] Add content for "Section 1: ROS 2 Architecture (DDS, discovery, QoS)": docs/01-nervous-system/01-ros2-architecture.md
- [x] T027 [US1] Add content for "Section 2: Nodes" (including lifecycle nodes, publishers/subscribers): docs/01-nervous-system/02-nodes.md
- [x] T028 [US1] Add content for "Section 3: Topics" (including message definitions, real-world examples): docs/01-nervous-system/03-topics.md

## Phase 4: User Story 2 - Digital Body Construction (URDF) (P1)

**Story Goal**: A student designs the physical structure of the humanoid robot using URDF to visualize the "body" that the nervous system will control.
**Independent Test**: The generated URDF file loads in RViz2 without errors, displaying the correct kinematic chain (head, torso, arms, legs).

- [x] T029 [US2] Create `setup.py` for `urdf_tutorial` package: code-examples/01-nervous-system/ros2_ws/src/urdf_tutorial/setup.py
- [x] T030 [P] [US2] Draft initial humanoid URDF model with links and joints: code-examples/01-nervous-system/ros2_ws/src/urdf_tutorial/urdf/humanoid.urdf
- [x] T031 [P] [US2] Create launch file to display URDF in RViz2: code-examples/01-nervous-system/ros2_ws/src/urdf_tutorial/launch/display_humanoid.launch.py
- [x] T032 [US2] Add content for "Section 6: URDF for Humanoids" (links, joints, inertial properties, visual/collision models): docs/01-nervous-system/06-urdf-humanoids.md

## Phase 5: User Story 3 - Sensor Integration & Simulation (P2)

**Story Goal**: A student adds sensory inputs (Vision, IMU) to the robot in simulation (Gazebo) to provide data for the nervous system.
**Independent Test**: Gazebo launches with the robot model, and topics for camera images and IMU data are publishing active data.

- [x] T033 [P] [US3] Implement mock camera publisher node: code-examples/01-nervous-system/ros2_ws/src/ros2_py_sensors/ros2_py_sensors/camera_publisher_node.py
- [x] T034 [P] [US3] Implement mock IMU publisher node: code-examples/01-nervous-system/ros2_ws/src/ros2_py_sensors/ros2_py_sensors/imu_publisher_node.py
- [x] T035 [US3] Integrate camera and IMU sensors into humanoid URDF model: code-examples/01-nervous-system/ros2_ws/src/urdf_tutorial/urdf/humanoid.urdf
- [x] T036 [US3] Update `display_humanoid.launch.py` to include sensor publishers: code-examples/01-nervous-system/ros2_ws/src/urdf_tutorial/launch/display_humanoid.launch.py
- [x] T037 [US3] Add content for "Section 4: Services + Actions" (request-response vs long-running tasks): docs/01-nervous-system/04-services-actions.md
- [x] T038 [US3] Add content for sensor integration examples within relevant docs: docs/01-nervous-system/

## Phase 6: User Story 4 - The Cognitive Bridge (LLM-to-ROS) (P2)

**Story Goal**: A student interfaces a Large Language Model with ROS 2 to allow natural language commands to drive the robot.
**Independent Test**: A Python node accepts a text string ("Move forward"), queries an LLM (mocked or real), and publishes the correct velocity command.

- [x] T039 [P] [US4] Implement LLM bridge node to translate natural language to ROS 2 Twist messages: code-examples/01-nervous-system/ros2_ws/src/ros2_py_agent/ros2_py_agent/llm_bridge_node.py
- [x] T040 [P] [US4] Implement motor driver subscriber node for Twist messages: code-examples/01-nervous-system/ros2_ws/src/ros2_py_controllers/ros2_py_controllers/motor_driver_node.py
- [x] T041 [US4] Update `robot_launch.py` to include LLM bridge and motor driver nodes: code-examples/01-nervous-system/ros2_ws/src/my_robot_bringup/launch/robot_launch.py
- [x] T042 [US4] Add content for "Section 5: Building a Python Agent → ROS bridge" (rclpy basics, agent-to-controller flow, safety checks): docs/01-nervous-system/05-python-agent-bridge.md

## Final Phase: Polish & Cross-Cutting Concerns

- [x] T043 Integrate all module concepts into a mini system example: docs/01-nervous-system/07-mini-system-example.md
- [x] T044 Generate Mermaid diagrams (Execution graph, Control feedback loop, etc.) and save them as Excalidraw files: static/img/01-nervous-system/
- [x] T045 Ensure all code examples are runnable and follow best practices: code-examples/01-nervous-system/ros2_ws/
- [x] T046 Validate thematic alignment (Neon Cyber AI visuals) across all diagrams and content: static/img/01-nervous-system/, docs/01-nervous-system/
- [x] T047 Perform APA citation validation for all references in docs/01-nervous-system/08-references.md
- [x] T048 Write and consolidate APA-style references: docs/01-nervous-system/08-references.md
- [x] T049 Review and refine all module content for clarity, accuracy, and consistency: docs/01-nervous-system/
- [x] T050 Create a health check script for ROS 2 environment setup and connectivity: code-examples/01-nervous-system/ros2_ws/src/my_robot_bringup/scripts/health_check.py

## Dependencies

This section outlines the completion order for user stories, ensuring foundational elements are in place before advanced concepts are tackled.

-   **Phase 1 (Setup) -> Phase 2 (Foundational)**: Foundational tasks depend on the initial setup of the module's directories and basic structures.
-   **Phase 2 (Foundational) -> Phase 3 (US1)**: User Story 1 (Environment & Workspace Setup) depends on the basic ROS 2 workspace and package structures being established.
-   **Phase 3 (US1) -> Phase 4 (US2)**: User Story 2 (Digital Body Construction) depends on a working ROS 2 environment.
-   **Phase 4 (US2) -> Phase 5 (US3)**: User Story 3 (Sensor Integration) requires the URDF model from US2.
-   **Phase 4 (US2) & Phase 5 (US3) -> Phase 6 (US4)**: User Story 4 (Cognitive Bridge) depends on both the robot body (URDF) and sensor data.
-   **All User Story Phases -> Final Phase (Polish)**: The final polish and cross-cutting concerns depend on all functional user stories being implemented.

## Parallel Execution Opportunities

Many tasks can be executed in parallel, especially within user story phases, as long as direct file dependencies are respected.

-   **Within Phase 3 (US1)**: T022 and T023 (publisher/subscriber nodes) can be implemented concurrently.
-   **Within Phase 4 (US2)**: T030 (URDF draft) and T031 (RViz2 launch file) can be developed in parallel initially, with T031 needing the URDF file once it's available.
-   **Within Phase 5 (US3)**: T033 (camera publisher) and T034 (IMU publisher) can be implemented in parallel.
-   **Within Phase 6 (US4)**: T039 (LLM bridge) and T040 (motor driver) can be developed concurrently.
-   **Cross-Phase Opportunities**: Research activities (from the Research Approach) can run concurrently with almost all other phases. Diagram creation (T044) can also start early and be refined iteratively. Content writing for different sections can be parallelized where topics are independent.

## Independent Test Criteria for Each Story

-   **User Story 1 - Environment & Workspace Setup**:
    -   `colcon build` completes without errors for the simple publisher/subscriber example.
    -   `ros2 run my_robot_bringup simple_publisher` and `ros2 run my_robot_bringup simple_subscriber` demonstrate successful message exchange.
    -   `ros2 topic list` shows the custom topic.
-   **User Story 2 - Digital Body Construction (URDF)**:
    -   `check_urdf humanoid.urdf` reports no errors.
    -   `ros2 launch urdf_tutorial display_humanoid.launch.py` successfully launches RViz2 with the humanoid model correctly displayed.
    -   Joints can be manipulated in RViz2 via a joint state publisher.
-   **User Story 3 - Sensor Integration & Simulation**:
    -   `ros2 launch urdf_tutorial display_humanoid.launch.py` (updated) launches Gazebo with the robot model.
    -   `ros2 topic list` shows `/camera/image_raw` and `/imu/data` topics publishing data.
    -   Camera feed is visible in RViz2, and IMU data changes when the robot moves in simulation.
-   **User Story 4 - The Cognitive Bridge (LLM-to-ROS)**:
    -   Running `llm_bridge_node` and sending a natural language command (e.g., "move forward") results in the `motor_driver_node` subscribing to a `Twist` message.
    -   The `Twist` message content corresponds correctly to the natural language command.

## Suggested MVP Scope

The MVP for this module focuses on getting a functional ROS 2 environment, a digital representation of the robot, and basic communication:

-   **Phase 1: Setup** (All tasks)
-   **Phase 2: Foundational** (All tasks)
-   **Phase 3: User Story 1 - Environment & Workspace Setup** (All tasks)
-   **Phase 4: User Story 2 - Digital Body Construction (URDF)** (All tasks)

This MVP provides the core infrastructure and initial content necessary to understand ROS 2 fundamentals and robot modeling, enabling subsequent development of sensor integration and AI bridging.
