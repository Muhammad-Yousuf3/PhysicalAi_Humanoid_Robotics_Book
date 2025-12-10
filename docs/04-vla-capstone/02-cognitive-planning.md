---
title: "The Brain: Cognitive Planning"
slug: /vla-capstone/planning
---

# 4.3 The Brain: Cognitive Planning

The core of VLA is the ability to *reason*. When we tell a robot "Make me breakfast," it must break that high-level goal into thousands of micro-actions (open fridge, find eggs, grasp egg, etc.). This is **Cognitive Planning**.

## The LLM as a Planner

Large Language Models are excellent at breaking down problems. We treat the LLM as the "Prefrontal Cortex"—the executive function. It doesn't move the motors directly; it issues orders to the lower-level systems (Nav2, MoveIt).

## Prompt Engineering for Robots

How do we teach an LLM to control a robot? We don't retrain it; we use **In-Context Learning** via a carefully crafted **System Prompt**.

### The System Prompt
The system prompt defines the robot's "Persona" and its "API". It looks like this:

> You are a helper robot. You can ONLY use the following Python functions:
> - `navigate_to(location_name)`
> - `pick_object(object_name)`
> - `place_object(location_name)`
> - `say(text)`
>
> DO NOT invent new functions.
> DO NOT ask for clarification unless necessary.
>
> Current Scene: `{"table": ["apple", "cup"], "kitchen": ["fridge"]}`

### Code as Policies
This approach is often called **Code as Policies** (Google DeepMind). Instead of outputting a sentence ("I will go to the kitchen"), the LLM outputs executable Python code:

```python
robot.say("Going to the kitchen")
robot.navigate_to("kitchen")
robot.pick_object("apple")
```

![Prompt Engineering Example Diagram](placeholder:FIG-4.2-prompt-engineering)
*Figure 4.2: The Prompt Engineering Flow. The 'System Prompt' defines the rules and API. The 'User Prompt' provides the task. The LLM combines these to generate a structured 'Plan' (JSON or Python code).*

## Safety: The Hallucination Problem

LLMs are prone to **Hallucination**. A robot might confidently output `robot.fly_to_moon()`. Since our API doesn't have a `fly` function, the code would crash.

**Safety Filters**:
1.  **Syntax Check**: Before execution, we parse the LLM's code to ensure it is valid Python.
2.  **API Check**: We verify that every function called exists in our allowed "Action Vocabulary."
3.  **Semantic Check**: We use a secondary, smaller model (or rules) to check for dangerous sequences (e.g., `pick_up` before `navigate_to`).

:::danger Hallucination Warning
Never assume LLM output is safe. Always wrap LLM execution in a "sandbox" that validates every command against a whitelist of allowed actions.
:::
