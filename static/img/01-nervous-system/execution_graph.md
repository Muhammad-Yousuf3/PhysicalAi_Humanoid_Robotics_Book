```mermaid
graph TD
    subgraph AI Layer
        A[LLM Agent] -- Requests/Commands --> B(AI Planner Node)
    end

    subgraph ROS 2 Nervous System
        B -- Action Goal (Move) --> C(High-Level Controller Node)
        C -- Joint Commands --> D(Low-Level Motor Driver Node)
        D -- Motor State --> E[Actuators/Motors]
        F[Sensors (Camera, IMU)] -- Sensor Data --> G(Sensor Processing Node)
        G -- Processed Data --> B
        G -- Processed Data --> C
        D -- Joint State --> C
        C -- Joint State --> G
    end

    subgraph Robot Hardware
        E --- Robot Body
        F --- Robot Body
    end

    style A fill:#f9f,stroke:#333,stroke-width:2px
    style B fill:#bbf,stroke:#333,stroke-width:2px
    style C fill:#bbf,stroke:#333,stroke-width:2px
    style D fill:#bbf,stroke:#333,stroke-width:2px
    style E fill:#ccf,stroke:#333,stroke-width:2px
    style F fill:#ccf,stroke:#333,stroke-width:2px
    style G fill:#bbf,stroke:#333,stroke-width:2px
```