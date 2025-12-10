import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    urdf_tutorial_share_dir = get_package_share_directory('urdf_tutorial')
    urdf_path = os.path.join(urdf_tutorial_share_dir, 'urdf', 'humanoid.urdf')

    rviz_config_dir = os.path.join(urdf_tutorial_share_dir, 'rviz', 'urdf.rviz')

    return LaunchDescription([
        # Robot State Publisher reads URDF and publishes /tf
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_path).read()}],
            arguments=[urdf_path]
        ),

        # Joint State Publisher GUI for manipulating URDF joints in RViz2
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        # RViz2 for visualizing the robot model and sensor data
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_dir]
        ),

        # Mock Camera Publisher
        Node(
            package='ros2_py_sensors',
            executable='camera_publisher_node',
            name='camera_publisher',
            output='screen'
        ),

        # Mock IMU Publisher
        Node(
            package='ros2_py_sensors',
            executable='imu_publisher_node',
            name='imu_publisher',
            output='screen'
        ),

        # LLM Bridge Node (AI agent proxy)
        Node(
            package='ros2_py_agent',
            executable='llm_bridge_node',
            name='llm_bridge_node',
            output='screen'
        ),

        # Motor Driver Node (Robot controller proxy)
        Node(
            package='ros2_py_controllers',
            executable='motor_driver_node',
            name='motor_driver_node',
            output='screen'
        ),
    ])
