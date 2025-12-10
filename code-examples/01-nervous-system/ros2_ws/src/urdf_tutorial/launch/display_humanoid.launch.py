import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the launch directory
    urdf_tutorial_share_dir = get_package_share_directory('urdf_tutorial')
    urdf_path = os.path.join(urdf_tutorial_share_dir, 'urdf', 'humanoid.urdf')

    # RViz configuration file
    rviz_config_dir = os.path.join(urdf_tutorial_share_dir, 'rviz', 'urdf.rviz')

    return LaunchDescription([
        # Launch robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_path).read()}],
            arguments=[urdf_path]
        ),

        # Launch joint_state_publisher_gui
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        # Launch RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_dir]
        ),

        # Launch mock camera publisher
        Node(
            package='ros2_py_sensors',
            executable='camera_publisher_node',
            name='camera_publisher',
            output='screen'
        ),

        # Launch mock IMU publisher
        Node(
            package='ros2_py_sensors',
            executable='imu_publisher_node',
            name='imu_publisher',
            output='screen'
        ),
    ])
