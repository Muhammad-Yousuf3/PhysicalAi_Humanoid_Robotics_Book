---
title: "The Eyes: Scene Graphs & Grounding"
slug: /vla-capstone/scene-graph
---

# 4.4 The Eyes: Scene Graphs & Grounding

An LLM knows what an "apple" is in the abstract (a fruit, red, sweet). But it doesn't know where *this specific apple* is in the room. **Grounding** is the process of linking abstract words to physical reality.

## From Pixels to Text: The Scene Graph

We cannot feed raw 4K video into an LLM (it's too slow and expensive). Instead, we use an **Object Detection** model (like YOLO or DETR) to scan the image and convert it into a structured text representation called a **Scene Graph**.

A Scene Graph is a JSON list of everything the robot sees, including properties and relationships.

### JSON Schema

```json
{
  "timestamp": "12:00:01",
  "objects": [
    {
      "id": "obj_01",
      "class": "apple",
      "position": {"x": 1.2, "y": 0.5, "z": 0.8},
      "attributes": ["red", "round"],
      "confidence": 0.95
    },
    {
      "id": "obj_02",
      "class": "table",
      "position": {"x": 1.0, "y": 0.5, "z": 0.0},
      "attributes": ["wooden"]
    }
  ],
  "relations": [
    {"subject": "obj_01", "predicate": "on", "object": "obj_02"}
  ]
}
```

This JSON is what we feed into the LLM's context window. Now, when the user says "Pick up the apple," the LLM looks at the JSON, finds `obj_01`, and generates the command `pick_object("obj_01")`.

![Scene Graph Visualization](placeholder:FIG-4.3-scene-graph-viz)
*Figure 4.3: Scene Graph Visualization. On the left, the raw camera view with bounding boxes. On the right, the generated JSON structure showing objects and their spatial relationships (e.g., 'Apple' -> ON -> 'Table').*

## Dynamic Updates

The world changes. If the robot picks up the apple, the Scene Graph must update immediately (`"apple"` is now `"in_gripper"`). This requires a fast, continuously running vision loop that keeps the "Cognitive Planner" synchronized with reality.
