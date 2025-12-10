# Section 6 — Mini Example

This section provides a consolidated mini-example that demonstrates the core concepts of the digital twin workflow, integrating Gazebo physics simulation with Unity high-fidelity rendering.

## 1. Build a Small Humanoid Scene

### Gazebo Physics Playground

The `lab_world.world` and `humanoid_gazebo.urdf` created earlier form the basis of our physics playground. This environment allows for basic testing of robot dynamics, stability, and sensor data generation.

**Steps**:

1.  **Navigate to your ROS 2 workspace**:
    ```bash
    cd code-examples/02-digital-twin-sim/gazebo_ws
    ```
2.  **Build the workspace (if you haven't already)**:
    ```bash
    colcon build --symlink-install
    ```
3.  **Source the setup files**:
    ```bash
    source install/setup.bash
    ```
4.  **Launch Gazebo with the humanoid robot**:
    ```bash
    ros2 launch humanoid_description display_humanoid.launch.py
    ```
    This command will open Gazebo, load the `lab_world.world`, and spawn your `humanoid_gazebo.urdf`. You should see the humanoid robot standing in the center of the world.

### Unity Rendering Arena

The `LabScene.unity` created earlier serves as the high-fidelity rendering arena. Here, the visual representation of the humanoid robot, synchronized with Gazebo, will be displayed.

**Steps**:

1.  **Open the Unity Project**:
    -   Launch Unity Hub.
    -   Add the existing project `code-examples/02-digital-twin-sim/unity_project`.
    -   Open the project in the Unity editor.
2.  **Configure ROS-TCP-Connector**:
    -   Ensure the `ROS-TCP-Connector` is correctly configured to connect to your ROS 2 system (typically `localhost` unless specified otherwise).
3.  **Open LabScene.unity**:
    -   Navigate to `Assets/Scenes/` in the Project window and open `LabScene.unity`.
4.  **Run the Scene**:
    -   Press the "Play" button in the Unity editor. The Unity robot model should connect to ROS 2 and begin synchronizing its joint states with the Gazebo simulation.

## 2. Simulate LiDAR + Depth + IMU

With the Gazebo simulation running, you can observe the sensor data being published to ROS 2 topics.

**Steps**:

1.  **Start the ROS 2 Sensor Listener**:
    In a new terminal, navigate to your `code-examples/02-digital-twin-sim/` directory and run the Python script you created earlier:
    ```bash
    cd code-examples/02-digital-twin-sim/
    python3 ros2_sensor_listener.py
    ```
    You should see output indicating that the script is receiving data from the `/imu/data`, `/camera/image_raw`, `/camera/depth`, `/scan`, and `/ft_sensor/data` topics.

2.  **Visualize in RViz 2**:
    For a visual confirmation of sensor data:
    ```bash
    rviz2
    ```
    -   Add a `RobotModel` display and point it to your `robot_description` topic.
    -   Add an `Image` display for `/camera/image_raw` and `/camera/depth`.
    -   Add a `PointCloud2` display for `/scan`.
    -   Add an `IMU` display for `/imu/data`.
    -   Add a `WrenchStamped` display for `/ft_sensor/data`.
    You should be able to see the robot model, camera feed, LiDAR point cloud, and IMU/FT sensor data in real-time.

## 3. Export Data for Agent Training

One of the primary benefits of simulation is the ability to generate vast amounts of labeled data for training AI agents.

**Steps**:

1.  **Record ROS 2 Bag File**:
    While your Gazebo simulation and sensor listener are running, open a new terminal and use `ros2 bag` to record all relevant topics:
    ```bash
    ros2 bag record -a -o digital_twin_sensor_data
    ```
    (Note: `-a` records all topics, consider specifying topics for large simulations).
    Let the simulation run for a period to collect sufficient data, then stop the recording (`Ctrl+C`).

2.  **Inspect Recorded Data**:
    You can then inspect the contents of the bag file:
    ```bash
    ros2 bag info digital_twin_sensor_data
    ros2 bag play digital_twin_sensor_data
    ```
    This recorded data can then be processed offline to extract features, labels, and augment datasets for training perception, control, or learning-based AI algorithms.

This mini-example provides a hands-on demonstration of how the Gazebo-Unity digital twin can be set up, operated, and used to generate valuable data for robotics research and development.