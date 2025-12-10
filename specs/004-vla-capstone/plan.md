# Implementation Plan - Module 4: Vision-Language-Action (VLA)

**Feature**: `004-vla-capstone`
**Status**: Planned
**Date**: 2025-12-10

## 1. Technical Context

Module 4 is the culmination of the "Physical AI" journey. It integrates the "Nervous System" (ROS 2), "Digital Twin" (Sim), and "Brain" (Nav2/Perception) with a new "Cognitive Layer" powered by Large Language Models (LLMs) and Vision Language Models (VLMs).

**Critical Constraint**: The content must be **conceptual, safe, and academic**. It must NOT provide executable code that connects an LLM directly to physical hardware actuators. All examples are simulation-based or pseudo-code.

**Core Components**:
*   **The Ear**: OpenAI Whisper (Voice -> Text).
*   **The Prefrontal Cortex**: LLM (Text + Context -> Plan).
*   **The Eyes**: VLM/Object Detector (Pixels -> Scene Graph).
*   **The Body**: ROS 2 Action Servers (Plan -> Motion).

## 2. Constitution Check

| Principle | Assessment | Status |
| :--- | :--- | :--- |
| **Scientific Accuracy** | Relies on established "SayCan" / "Code as Policies" frameworks. | ✅ PASS |
| **Clarity & Accessibility** | Uses the "Cognitive Pipeline" analogy to explain complex VLA models. | ✅ PASS |
| **Safety** | Explicitly prohibits direct motor control by LLMs. Emphasizes "Action Primitives". | ✅ PASS |
| **No Plagiarism** | Concepts are synthesized from public research (Google DeepMind, OpenAI) but written originally. | ✅ PASS |

## 3. High-Level Architecture (The VLA Loop)

1.  **User Input**: "Please tidy up the table." (Audio)
2.  **Transcription**: Whisper Node -> "Please tidy up the table." (Text)
3.  **Perception**: Camera -> Object Detector -> Scene Graph (`{"table": ["red_block", "blue_cup"]}`).
4.  **Planning (The LLM)**:
    *   *Prompt*: System Instructions + Scene Graph + User Command.
    *   *Output*: Sequence of Action Primitives (e.g., `pick("red_block")`, `place("bin")`).
5.  **Execution**: Robot executes the primitives safely using the Nav2/MoveIt stack.

## 4. Section-by-Section Outline

### 4.1 Introduction: The Convergence
*   How we moved from "Hard-coded Logic" (Module 1) to "Behavior Trees" (Module 3) to "Cognitive Planning" (Module 4).
*   The definition of VLA: Intersecting Vision, Language, and Action.

### 4.2 Voice-to-Action (The Interface)
*   **Concept**: Treating Voice as a ROS Topic.
*   **Technology**: OpenAI Whisper.
*   **Flow**: Microphone Buffer -> Whisper Inference -> Text Topic.
*   *Safety Note*: "Why we don't act on raw audio."

### 4.3 Cognitive Planning (The Brain)
*   **Concept**: LLMs as Reasoning Engines, not just chatbots.
*   **Mechanism**: "In-Context Learning" and "Few-Shot Prompting".
*   **Structure**:
    *   **System Prompt**: Defining the robot's persona and API (Action Primitives).
    *   **Context**: The Scene Graph (what the robot sees).
    *   **Task**: The User's command.
*   **Output**: Structured JSON or Python-like pseudo-code.

### 4.4 Capstone Project: The Autonomous Humanoid
*   **Scenario**: A "Tidying" task in a simulated kitchen.
*   **Step-by-Step Walkthrough**:
    1.  **Command**: "Put the red block in the trash."
    2.  **See**: Robot scans, finds "red_block" at (x,y) and "trash_bin" at (u,v).
    3.  **Think**: LLM generates plan: `[navigate(x,y), pick(red_block), navigate(u,v), place(trash_bin)]`.
    4.  **Act**: Robot executes the plan in Isaac Sim.

### 4.5 Research & Testing Notes
*   **Testing VLA**: How to "Unit Test" a prompt (does it produce valid JSON?).
*   **Hallucination**: Dealing with the robot trying to invent actions (e.g., `robot.fly()`).
*   **Validation**: Using a "Syntax Checker" node before execution.

## 5. Research Approach

*   **Literature**: Reference "Do As I Can, Not As I Say" (SayCan) and "Code as Policies".
*   **Simplification**: Abstract away the complex tensor math of the LLM; focus on the *Inputs* (Prompt) and *Outputs* (Plan).

## 6. Decisions & Rationale

*   **Decision**: Use "Scene Graph" abstraction.
    *   *Rationale*: Passing raw pixels to an LLM is complex/costly. Passing a text list of objects is robust and easy to teach.
*   **Decision**: Modular Architecture.
    *   *Rationale*: Easier to debug. If the robot fails to pick, is it the Vision (didn't see it), the Brain (didn't plan it), or the Body (IK failed)?
*   **Decision**: Pythonic Pseudo-code Plans.
    *   *Rationale*: "Code as Policies" is the modern standard for LLM robotics.

## 7. Validation Strategy

*   **Concept Check**: Does the Prompt Logic hold up? (If I paste the sample prompt into ChatGPT, does it give a valid plan?)
*   **Safety Check**: Are all examples explicitly "Simulated"?
*   **Flow Check**: Does the data flow (Audio -> Text -> Plan -> Action) make sense?

## 8. Workflow Phases

1.  **Phase 1: Concepts (Voice & Planning)**
    *   Draft Whisper section.
    *   Draft Prompt Engineering section.
2.  **Phase 2: The Capstone (Integration)**
    *   Write the "Tidying Robot" walkthrough.
    *   Design the Scene Graph schema.
3.  **Phase 3: Review**
    *   Safety & Academic tone check.
