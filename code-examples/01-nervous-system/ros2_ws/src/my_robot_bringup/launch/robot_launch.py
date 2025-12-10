from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_py_agent',
            executable='llm_bridge_node',
            name='llm_bridge_node',
            output='screen'
        ),
        Node(
            package='ros2_py_controllers',
            executable='motor_driver_node',
            name='motor_driver_node',
            output='screen'
        ),
    ])
