# Feature Specification: Vision-Language-Action (VLA)

**Feature Branch**: `004-vla-capstone`
**Created**: 2025-12-08
**Status**: Draft
**Input**: User description (Module 4: Vision-Language-Action)

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Voice Command & Interpretation (Priority: P1)

A user provides a natural language voice command to the humanoid robot, which then accurately transcribes the command and converts it into an internal representation for task planning.

**Why this priority**: The first step in VLA is understanding the user's intent through natural language.

**Independent Test**: User speaks "Go to the kitchen and get me a cup." The Whisper node accurately transcribes it, and a ROS 2 topic publishes the text and a confidence score.

**Acceptance Scenarios**:
1.  **Given** a clear voice command in English, **When** spoken to the robot's microphone, **Then** the Whisper node publishes the correct transcription to a `/command/text` topic within 2 seconds.
2.  **Given** a noisy environment, **When** a command is spoken, **Then** the transcription still maintains >80% accuracy.
3.  **Given** an ambiguous command, **When** processed, **Then** the system requests clarification via text-to-speech feedback to the user.

---

### User Story 2 - Cognitive Task Planning (Priority: P1)

The robot's "brain" (LLM) receives a transcribed natural language command and generates a sequence of robot-executable actions, considering safety constraints and available capabilities.

**Why this priority**: The core intelligence of VLA is the ability to break down high-level goals into executable steps.

**Independent Test**: The LLM planner receives "Pick up the red block" and outputs a structured plan (e.g., YAML or JSON) of sequential ROS 2 actions (navigate, perceive, approach, grasp, retract).

**Acceptance Scenarios**:
1.  **Given** a simple command like "wave hello", **When** processed by the LLM planner, **Then** a plan with a sequence of joint movements (or a predefined ROS action) is generated.
2.  **Given** a potentially unsafe command, **When** evaluated by the safety filters, **Then** the LLM generates a warning and/or proposes a safer alternative.
3.  **Given** a complex multi-step command ("find the cup, bring it here, then put it on the table"), **When** processed, **Then** the LLM generates a logically coherent multi-stage plan.

---

### User Story 3 - Integrated Perception for Planning (Priority: P2)

The robot utilizes its visual perception system (object detection, depth estimation, scene graph) to provide relevant context to the LLM planner, enabling grounded decision-making.

**Why this priority**: LLM planning needs to be grounded in the robot's understanding of its immediate environment.

**Independent Test**: The robot detects a "red mug" in its field of view, and this object, along with its 3D pose, is added to the scene graph accessible by the LLM planner.

**Acceptance Scenarios**:
1.  **Given** a known object in the scene, **When** the robot scans the area, **Then** the object detection module publishes its class, confidence, and 3D pose to a ROS topic.
2.  **Given** multiple objects, **When** queried, **Then** the scene graph accurately describes spatial relationships (e.g., "cup is on the table").
3.  **Given** the object detection output, **When** the LLM plans, **Then** it uses the detected object's name and location from the scene graph.

---

### User Story 4 - Autonomous Action Execution (Capstone) (Priority: P3)

The robot autonomously executes the LLM-generated plan, coordinating navigation, manipulation, and continuous perception to complete the high-level task.

**Why this priority**: This is the culmination of all previous modules, demonstrating a fully capable Physical AI humanoid.

**Independent Test**: The robot receives the plan "Pick up the red block from the table," navigates to the table, detects the block, grasps it, and reports success.

**Acceptance Scenarios**:
1.  **Given** an LLM-generated plan, **When** execution starts, **Then** the robot initiates navigation to the target area.
2.  **Given** the robot is at the target, **When** the object is within reach, **Then** the manipulation controller successfully grasps the object.
3.  **Given** a task completion, **When** the robot finishes, **Then** it reports success via text-to-speech and logs the outcome.

### Edge Cases

-   **Ambiguous Objects**: "Pick up the block" when multiple blocks are present. (Handling: Prompt for clarification, or define a default (e.g., nearest)).
-   **Unreachable Goals**: Object is too far or in a constrained space. (Handling: LLM re-plans or reports impossibility to user).
-   **Manipulation Failure**: Object slips from grasp. (Handling: Retry grasp, re-plan, or ask for human assistance).
-   **Safety Violation**: LLM proposes an action that violates predefined safety rules. (Handling: Immediate abort, human override, or re-planning with safety constraints enforced).

## Requirements *(mandatory)*

### Functional Requirements

-   **FR-001**: The content MUST provide a comprehensive overview of VLA architecture: Speech → LLM → Plan → ROS 2 actions → Execution.
-   **FR-002**: The system MUST integrate Whisper for accurate speech-to-text conversion and publish the result to a ROS 2 topic.
-   **FR-003**: The content MUST detail prompt engineering techniques for robotics, enabling LLMs to generate valid and safe task plans.
-   **FR-004**: The system MUST implement safety filters to validate LLM-generated actions before execution, preventing unsafe movements.
-   **FR-005**: The system MUST integrate vision models (object detection, depth estimation) running on Jetson Orin to populate a dynamic scene graph.
-   **FR-006**: The content MUST provide a ROS 2 action mapping library that translates LLM-generated abstract plans into executable navigation and manipulation commands.
-   **FR-007**: The Capstone project MUST integrate voice command, LLM planning, VSLAM navigation, object detection, and grasping/manipulation.
-   **FR-008**: The system MUST provide logging and reporting mechanisms for Capstone task completion and failures.

### Key Entities

-   **VoiceCommand**: Raw audio input converted to text.
-   **LLMPlanner**: The Large Language Model instance, possibly via API, that generates task plans.
-   **RobotAction**: A ROS 2 action server/client representing a low-level capability (e.g., `NavigateToPose`, `GraspObject`).
-   **SceneGraph**: A dynamic representation of perceived objects and their spatial relationships.

## Success Criteria *(mandatory)*

### Measurable Outcomes

-   **SC-001**: End-to-end latency (voice command to robot initial movement) is <5 seconds for simple commands.
-   **SC-002**: LLM-generated plans for simple tasks (e.g., "pick up block") are valid and executable >95% of the time.
-   **SC-003**: Object detection accuracy for known objects in a controlled environment is >90% mAP.
-   **SC-004**: Capstone project successfully completes a predefined multi-step task (e.g., "fetch cup from table") >80% of the time.
-   **SC-005**: All safety critical LLM plans are flagged or corrected by safety filters before execution.