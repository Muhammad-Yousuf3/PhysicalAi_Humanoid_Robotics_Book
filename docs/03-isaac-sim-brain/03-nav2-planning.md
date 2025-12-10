---
title: "The Mind: Nav2 & Behavior Trees"
slug: /isaac-sim-brain/planning
---

# 3.4 The Mind: Nav2 & Behavior Trees

Now that the robot knows *where* it is (VSLAM) and has a map of the world, it needs to decide *how* to move. This is the job of the **ROS 2 Navigation Stack (Nav2)**.

## Global vs. Local Planning

Navigation is split into two layers:
1.  **Global Planner**: The "Strategic" layer. It looks at the static map and calculates the optimal path from A to B (e.g., using A* or Dijkstra). It's like Google Maps planning a route across a city.
2.  **Local Planner (Controller)**: The "Tactical" layer. It looks at the immediate surroundings (using local sensor data) to follow the global path while avoiding dynamic obstacles (like a person walking by). It generates the actual velocity commands (`cmd_vel`).

Both planners rely on **Costmaps**. A costmap is a grid where "safe" areas are 0 and "obstacles" are 255. The robot adds an "inflation layer" around obstacles to ensure it doesn't clip a wall with its shoulder.

## Behavior Trees: The Logic of Autonomy

In older robots, logic was often a messy "Finite State Machine" (if this, then that). Nav2 uses **Behavior Trees (BTs)**, a hierarchical structure common in video game AI.

A Behavior Tree is a tree of nodes that "tick" (execute) at a set frequency.
*   **Action Nodes**: The leaves of the tree. They do things (e.g., `ComputePathToPose`, `FollowPath`, `Spin`).
*   **Control Nodes**: The branches that decide which leaf to tick.
    *   **Sequence (->)**: Runs children in order. If one fails, the whole sequence fails. (e.g., "Plan Path" -> "Follow Path").
    *   **Fallback (?)**: Runs children in order. If one fails, it tries the next one. (e.g., "Follow Path" failed? -> "Try Recovery Behavior").

![Nav2 Behavior Tree Diagram](placeholder:FIG-3.3-nav2-behavior-tree)
*Figure 3.3: A Simplified Nav2 Behavior Tree. The root node ticks a 'Recovery' Fallback node. The primary branch is a Sequence: 'Compute Path' -> 'Follow Path'. If 'Follow Path' fails (e.g., blocked), the Fallback node triggers the 'Spin' recovery action to clear the costmap.*

This modular structure allows us to build complex, robust behaviors. We can easily add a "Wait" action or a "Back up" recovery without rewriting the entire navigation logic.
