```mermaid
graph LR
    A[AI Agent (Python)] -- High-Level Command --> B(ROS 2 Controller Node)
    B -- Low-Level Actuation Cmd --> C[Robot Actuators]
    C -- Physical Movement --> D[Robot Environment]
    D -- Sensor Data --> E[Robot Sensors]
    E -- Raw Sensor Readings --> F(ROS 2 Sensor Node)
    F -- Processed Sensor Data --> A
    F -- Processed Sensor Data --> B
```