# Research & Technical Decisions: Module 4 (Vision-Language-Action)

**Status**: Research Complete
**Date**: 2025-12-10
**Author**: Gemini Agent

## 1. Technical Context & Unknowns

### Context
Module 4 is the "Capstone" of the book, integrating perception, planning, and control into a cohesive "Physical AI" system. The focus is on **Vision-Language-Action (VLA)** models.
**Constraint**: Content must be conceptual and safe. No executable code for physical robots.

### Resolved Unknowns

| Unknown | Resolution | Source/Rationale |
| :--- | :--- | :--- |
| **VLA Architecture** | **Modular (LLM + VLM + Policies)**. Use a VLM for perception (scene graph), an LLM for planning, and standard ROS 2 actions for execution. | "Modular VLA models prioritize flexibility... and safety." End-to-end is too opaque for an educational intro. |
| **Voice Command** | **OpenAI Whisper**. Treat it as a "Sensor" that publishes text to a topic. | Industry standard for speech-to-text. Easy to explain as a ROS node. |
| **Planning Concept** | **Code as Policies / SayCan**. The LLM outputs a sequence of API calls (e.g., `pick(red_block)`) based on "Affordances". | Bridges the gap between "text" and "action" safely. |
| **Capstone Project** | **"The Tidying Robot"**. A simulated robot cleans a table by moving objects to bins based on voice commands. | Classic problem, easy to visualize, covers all distinct phases. |

## 2. Research Findings

### 2.1 Modular vs. End-to-End VLA
*   **End-to-End (e.g., RT-2)**: Pixels + Text -> Motor Actions. Good for performance, bad for explainability/safety in a book.
*   **Modular**:
    1.  **Vision**: Object Detection/VLM -> Scene Graph ("Cup at (x,y)").
    2.  **Language**: LLM -> High-level Plan ("Move to Cup").
    3.  **Action**: Behavior Tree/Controller -> Motor commands.
*   **Decision**: Use **Modular** for the book. It allows us to reuse Module 1 (ROS), Module 2 (Sim), and Module 3 (Nav2/Perception).

### 2.2 LLM as a Planner (The "Cognitive" Layer)
*   **Concept**: The LLM is the "Orchestrator".
*   **Input**: User prompt ("Clean the table") + Scene Graph (List of objects).
*   **Output**: A list of sequential tasks.
*   **Safety**: The LLM *cannot* generate motor currents. It can only generate *requests* to the Nav2/MoveIt stacks, which have their own safety layers.

### 2.3 Voice-to-Action
*   **Pipeline**: Microphone -> Whisper Node -> `/command/text` -> LLM Node.
*   **Latency**: Conceptual discussion on "Streaming" vs "Batch" transcription.

## 3. Decisions & Trade-offs

### Decision 1: The "Scene Graph" as the Interface
*   **Choice**: The VLM/Detector does not feed pixels to the planner. It feeds a text-based "Scene Graph" (JSON).
*   **Rationale**: This creates a clean text-to-text interface for the LLM, making it easier to explain "Reasoning".
*   **Trade-off**: Loses some nuance of visual understanding, but sufficient for "Pick up the red block".

### Decision 2: Simulation-Only Capstone
*   **Choice**: The final project is described entirely within Isaac Sim.
*   **Rationale**: Zero risk of physical damage. Allows students to "fail" safely (e.g., robot drops the cup).

### Decision 3: "Code as Policies" Pseudo-code
*   **Choice**: Show the LLM generating Python-like pseudo-code (`robot.walk_to("kitchen")`).
*   **Rationale**: Intuitive for programmers.
*   **Constraint Check**: Ensure this pseudo-code is clearly marked as *conceptual* and not runnable without a specific (unprovided) API wrapper.

## 4. Safety Strategy
*   **Human-in-the-loop**: The design will include a "Confirmation Step" where the robot reads back the plan via TTS before executing.
*   **Sanitization**: The LLM prompt will include system instructions to "Reject unsafe commands" (e.g., "Run into the wall").
