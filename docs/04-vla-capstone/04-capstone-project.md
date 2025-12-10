---
title: "Capstone: The Autonomous Tidying Robot"
slug: /vla-capstone/project
---

# 4.5 Capstone Project: The Autonomous Tidying Robot

We have built the Ear, the Brain, and the Eyes. Now we put them together in a final simulation project: **The Autonomous Tidying Robot**.

## The Scenario
Our robot stands in a simulated kitchen (Isaac Sim). There is a table with random objects (red block, blue cup) and a trash bin.
**Goal**: The user gives a natural language command, and the robot executes it.

## Execution Trace: "Put the red block in the trash"

Let's trace a single command through the entire VLA pipeline:

### 1. User Input (Voice)
*   **Audio**: User speaks into the microphone.
*   **Whisper Node**: Transcribes audio to text: `"Put the red block in the trash."`
*   **Topic**: Published to `/command/text`.

### 2. Perception (Vision)
*   **Camera**: Captures the scene.
*   **Detector Node**: Identifies objects and estimates poses.
*   **Scene Graph**:
    ```json
    {"objects": [{"id": "block_1", "class": "red_block", "pos": [1,0,0]}, {"id": "bin_1", "class": "trash_bin", "pos": [2,2,0]}]}
    ```

### 3. Reasoning (Planning)
*   **LLM Node**: Receives the Command + Scene Graph.
*   **Prompt**: "You are a robot. Map 'red block' to 'block_1' and 'trash' to 'bin_1'. Output Python code."
*   **Output Plan**:
    ```python
    robot.navigate_to([1,0,0])  # Go to block
    robot.pick("block_1")       # Pick block
    robot.navigate_to([2,2,0])  # Go to bin
    robot.place("bin_1")        # Place in bin
    ```

### 4. Execution (Action)
*   **Executor Node**: Validates the code (Safety Check).
*   **Nav2**: Receives `navigate_to` goals. Moves the robot base.
*   **MoveIt**: Receives `pick` goal. Computes arm trajectory.
*   **Sim**: Physics engine simulates the grasp and drop.

## The Reality Gap: VLA Edition

In simulation, this works 99% of the time. In the real world, VLA systems face unique challenges:

1.  **Latency**: LLMs are slow. A 5-second delay between "Stop!" and the robot stopping is dangerous. *Solution: Run safety stops locally, not in the cloud.*
2.  **Ambiguity**: "The cup" might refer to one of five cups. *Solution: Visual grounding must handle ambiguity (e.g., pointing).*
3.  **Hallucination**: The LLM might try to `pick` a heavy table. *Solution: Physics-based feasibility checks.*

:::danger Hallucination Warning
Simulators forgive physics violations; reality does not. If an LLM tells a real robot to move its arm through a wall, the robot might try—and break itself. Always implement **Kinematic Constraints** at the lowest control level, independent of the AI.
:::
