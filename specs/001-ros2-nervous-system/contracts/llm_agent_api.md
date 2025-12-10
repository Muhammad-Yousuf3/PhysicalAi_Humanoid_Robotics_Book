# API Contract: LLM Agent to High-Level Controller (ROS 2 Action)

**Feature Branch**: `001-ros2-nervous-system`
**Created**: 2025-12-09

This document describes the conceptual API contract for the interaction between the LLM Agent and the High-Level Controller Node using a ROS 2 Action.

## Action Name: `execute_robot_command`

This action allows the LLM Agent to send high-level, goal-oriented commands to the robot's controller, which will then execute a sequence of actions.

### 1. Goal

```yaml
# execute_robot_command.action
# Goal: High-level command for the robot
string command_text      # e.g., "move forward 1 meter", "turn left 90 degrees"
geometry_msgs/Pose target_pose # Optional: Target pose if command implies a specific location
---
# Result: Outcome of the command execution
bool success             # True if command completed successfully
string message           # A human-readable message about the outcome (e.g., "Command executed", "Error: Obstacle detected")
---
# Feedback: Intermediate progress updates
float32 progress_percentage # Percentage complete (0.0 - 100.0)
string current_status    # Current status (e.g., "Moving towards target", "Detecting obstacles")
```

### 2. Service: `get_robot_status` (Request-Response)

This service allows the LLM Agent to query the current status of the robot.

#### Request

```yaml
# get_robot_status.srv
# Request: Empty, or specific query types
string query_type        # e.g., "battery_level", "current_location", "sensor_health"
---
# Response: Current status of the robot
string status_message    # Human-readable status (e.g., "Battery: 85%", "Location: [1.0, 0.5, 0.0]")
float32 battery_level    # Current battery level (0.0 - 1.0)
geometry_msgs/Pose current_pose # Current pose of the robot
string[] active_sensors  # List of currently active sensors
```

### 3. Topic: `robot_notifications` (Publisher/Subscriber)

This topic allows the robot to asynchronously send important notifications or events to the LLM Agent or other monitoring systems.

#### Message

```yaml
# RobotNotification.msg
# Message: Asynchronous notification from the robot
std_msgs/Header header
string type              # e.g., "warning", "error", "info", "event"
string message           # Detailed message (e.g., "Obstacle detected ahead", "Low battery")
int32 error_code         # Optional: Specific error code if type is "error"
```
