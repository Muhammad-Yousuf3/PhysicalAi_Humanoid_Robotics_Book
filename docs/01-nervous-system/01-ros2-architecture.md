# Section 1: ROS 2 Architecture - DDS, Discovery, and QoS

The power and flexibility of ROS 2 stem from its sophisticated underlying architecture, which is a significant evolution from its predecessor, ROS 1. At its heart, ROS 2 leverages a **Data Distribution Service (DDS)** for its inter-process communication, enabling a highly distributed, robust, and real-time capable system.

## 1.1. The Data Distribution Service (DDS)

DDS is an international standard for publish-subscribe middleware designed for real-time applications. It provides the low-level communication layer that ROS 2 nodes use to exchange data. Key characteristics of DDS that benefit ROS 2 include:

-   **Decentralized Architecture**: Unlike ROS 1's central `roscore`, DDS operates in a peer-to-peer fashion. Nodes can directly discover and communicate with each other without a central broker, which eliminates single points of failure and enhances scalability.
-   **Quality of Service (QoS) Policies**: DDS offers a rich set of QoS policies that allow developers to fine-tune communication behavior. This is crucial for robotics, where different data streams have varying requirements (e.g., sensor data vs. critical commands).
-   **Pluggable Implementations**: ROS 2 is not tied to a single DDS vendor. It supports various DDS implementations (e.g., Cyclone DDS, FastDDS, Connext DDS), allowing users to choose the one that best fits their needs regarding performance, features, and licensing. Cyclone DDS is typically the default in recent ROS 2 distributions.

## 1.2. Discovery Mechanism

In a decentralized DDS environment, nodes need a way to find each other and establish communication. ROS 2's discovery process handles this automatically:

1.  **Advertisement**: When a ROS 2 node starts, it advertises its presence and the topics, services, or actions it publishes or subscribes to.
2.  **Matching**: Other nodes listen for these advertisements. When a publisher and a subscriber for the same topic (with compatible QoS settings) are discovered, DDS automatically establishes a direct communication channel between them.
3.  **Dynamic**: Discovery is dynamic, meaning nodes can join or leave the network at any time, and communication channels will be established or torn down accordingly without requiring manual reconfiguration or restarting a central server.

This transparent discovery mechanism allows developers to focus on the logic of their nodes rather than the intricacies of network setup.

## 1.3. Quality of Service (QoS) Policies

QoS policies are a cornerstone of ROS 2's flexibility and reliability. They allow developers to specify the desired behavior of their communication channels, tailoring them to the specific needs of different data types or control loops. Here are some of the most important QoS policies in ROS 2:

-   **History**:
    -   `KEEP_LAST`: Only stores the last N samples.
    -   `KEEP_ALL`: Stores all samples up to a certain limit.
    -   *Use Case*: `KEEP_LAST` is good for frequently updated sensor data (e.g., camera frames), where only the latest reading is relevant. `KEEP_ALL` is useful for log messages or critical data that shouldn't be missed.

-   **Depth**:
    -   Used in conjunction with `KEEP_LAST` History policy. Defines the size of the history buffer.
    -   *Use Case*: A depth of 1 with `KEEP_LAST` means only the most recent message is kept. A larger depth buffers more messages.

-   **Reliability**:
    -   `RELIABLE`: Guarantees that every message sent will be received by all matching subscribers. This involves retransmission if messages are lost.
    -   `BEST_EFFORT`: Messages are sent without guarantee of delivery. If a message is lost, it is not retransmitted.
    -   *Use Case*: `RELIABLE` is essential for critical command signals, parameters, or service calls where message loss is unacceptable. `BEST_EFFORT` is suitable for high-bandwidth, streaming data (e.g., video, lidar scans) where occasional frame drops are acceptable for lower latency.

-   **Durability**:
    -   `TRANSIENT_LOCAL`: The publisher retains a history of published messages and delivers them to late-joining subscribers.
    -   `VOLATILE`: No history is retained for late-joining subscribers; they only receive messages published after they connect.
    -   *Use Case*: `TRANSIENT_LOCAL` is useful for publishing static configuration data or map data that a new subscriber needs immediately upon connection. `VOLATILE` is the default and suitable for most dynamic data streams.

-   **Liveliness**:
    -   Determines how the system detects if a publisher or subscriber is still active.
    -   `AUTOMATIC`: Liveliness is asserted automatically by the DDS implementation.
    -   `MANUAL_BY_TOPIC`: Application must manually assert liveliness.
    -   *Use Case*: Ensures that nodes are actively participating in communication, which can be critical for fault detection in robotic systems.

### 1.4. ROS 2 Communication Patterns Summary

| Communication Type | DDS QoS Considerations | Best Use Case                                |
| :----------------- | :--------------------- | :------------------------------------------- |
| **Topics**         | History, Depth, Reliability, Durability, Liveliness | Continuous data streams (sensors, telemetry) |
| **Services**       | Reliability (implicitly), Liveliness | Request-response for immediate actions/queries |
| **Actions**        | Reliability (implicitly), Liveliness | Goal-oriented, long-running tasks with feedback |

By carefully selecting QoS policies, developers can create highly optimized and robust robotic applications that meet stringent real-time requirements and ensure reliable operation in complex environments.
