# Data Model: Module 4 (Vision-Language-Action)

**Type**: Educational Content Structure
**Scope**: Chapter 4 of "Physical AI & Humanoid Robotics"

## 1. Primary Entity: The Book Chapter

### Metadata
*   **Title**: "The Autonomous Humanoid: Vision, Language, and Action"
*   **Module ID**: 004
*   **Target Audience**: Robotics students, AI researchers.
*   **Prerequisites**: Modules 1, 2, 3.

### Components

| Component | Description | Format |
| :--- | :--- | :--- |
| **Learning Objectives** | 3-5 goals (e.g., "Explain VLA", "Design a prompt"). | Bullet points |
| **Concept: Voice** | How Whisper works for robots. | Text + Block Diagram |
| **Concept: Reasoning** | How LLMs plan (Chain of Thought). | Text + Prompt Example |
| **Concept: Grounding** | Connecting text to physical objects (Scene Graph). | Text + JSON Example |
| **Capstone Walkthrough** | Step-by-step trace of a "Tidy Up" task. | Text + Simulation Screenshots |
| **Safety Analysis** | Risks of LLM hallucinations in robotics. | Text |

## 2. Key Conceptual Entities (The "Subjects")

### Entity: The VLA Pipeline
*   **Definition**: The flow of data from User -> Robot -> World.
*   **Stages**:
    1.  **Instruction**: "Voice" (Audio -> Text).
    2.  **Perception**: "Vision" (Pixels -> Scene Graph).
    3.  **Reasoning**: "Language" (Text + Graph -> Plan).
    4.  **Execution**: "Action" (Plan -> ROS 2 Topics).

### Entity: The Scene Graph
*   **Definition**: A structured text representation of the world state.
*   **Structure**:
    *   **Objects**: List of detected items (ID, Class, Pose, Attributes).
    *   **Relations**: Spatial links ("Cup ON Table").
*   **Role**: The "Context" provided to the LLM.

### Entity: The Action Primitive
*   **Definition**: A pre-defined, safe robot capability exposed to the LLM.
*   **Examples**: `navigate_to(landmark)`, `pick_object(id)`, `place_object(location)`.
*   **Role**: The "Vocabulary" of the robot.

## 3. Relationships

*   **User -> Robot**: Voice Command.
*   **Robot -> World**: Action Primitives (ROS Actions).
*   **LLM -> Action Primitive**: The LLM *selects* and *sequences* primitives based on the User Command and Scene Graph.
