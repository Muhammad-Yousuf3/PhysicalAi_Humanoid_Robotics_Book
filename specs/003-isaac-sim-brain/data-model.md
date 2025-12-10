# Data Model: Module 3 (The AI-Robot Brain)

**Type**: Educational Content Structure
**Scope**: Chapter 3 of "Physical AI & Humanoid Robotics"

## 1. Primary Entity: The Book Chapter

### Metadata
*   **Title**: "The AI-Robot Brain: Simulation, Perception, and Planning"
*   **Module ID**: 003
*   **Target Audience**: Robotics students, AI researchers (Grade 10-12 readability).
*   **Prerequisites**: Module 1 (Nervous System), Module 2 (Digital Twin - Intro).

### Components

| Component | Description | Format |
| :--- | :--- | :--- |
| **Learning Objectives** | 3-5 clear goals for the reader. | Bullet points |
| **Concept Introduction** | High-level overview of the "Brain" architecture. | Text + Diagram |
| **Deep Dive: Simulation** | Explaining Isaac Sim's role (Ground Truth). | Text + Screenshots |
| **Deep Dive: Perception** | Explaining VSLAM (Localization). | Text + Graph Diagram |
| **Deep Dive: Planning** | Explaining Nav2 (Decision Making). | Text + Tree Diagram |
| **Synthesis** | How they fit together (The "Loop"). | Text + Flowchart |
| **Review** | Knowledge check. | Quiz/Q&A |

## 2. Key Conceptual Entities (The "Subjects")

These are the technical concepts explained *within* the chapter.

### Entity: The Simulation World (USD Stage)
*   **Definition**: A container for the robot, environment, and physics rules.
*   **Properties**: Geometry, Physics Materials, Lights, Sensors.
*   **Role**: Provides "Synthetic Data" (Images + Labels) to the AI.

### Entity: The VSLAM Graph
*   **Definition**: A mathematical graph where nodes = robot poses, edges = constraints.
*   **Operations**: Optimization (Relaxation), Loop Closure (Snap).
*   **Role**: Answers "Where am I?" and "What does the world look like?".

### Entity: The Behavior Tree
*   **Definition**: A hierarchical control structure for robot autonomy.
*   **Nodes**:
    *   **Root**: The entry point.
    *   **Control Flow**: Sequence, Fallback.
    *   **Action**: Atomic tasks (Move, Look, Stop).
*   **Role**: Answers "What should I do next?".

## 3. Relationships

*   **Simulation -> Perception**: The Sim produces *Synthetic Sensor Data* (RGB-D) consumed by Perception.
*   **Perception -> Planning**: Perception produces a *Map* and *Pose* consumed by Planning.
*   **Planning -> Simulation**: Planning produces *Velocity Commands* that move the robot in Sim.
