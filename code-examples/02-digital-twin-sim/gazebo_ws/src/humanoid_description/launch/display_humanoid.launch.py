import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file_name = LaunchConfiguration('world_file_name', default='lab_world.world')
    
    pkg_name = 'humanoid_description'
    world_package_name = 'gazebo_worlds'

    # Get the launch directory
    humanoid_description_pkg_share = get_package_share_directory(pkg_name)
    gazebo_worlds_pkg_share = get_package_share_directory(world_package_name)


    # Path to the world file
    world_path = PathJoinSubstitution([
        gazebo_worlds_pkg_share,
        'worlds',
        world_file_name
    ])

    # Path to the URDF file
    urdf_path = os.path.join(
        humanoid_description_pkg_share,
        'urdf',
        'humanoid_gazebo.urdf'
    )
    
    # Read the URDF file
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        launch_arguments={'world': world_path}.items(),
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
        arguments=[urdf_path]
    )

    # Spawn the robot into Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'humanoid',
                   '-robot_namespace', '/',
                   '-x', '0', '-y', '0', '-z', '0.5'], # Spawn slightly above ground
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'world_file_name',
            default_value='lab_world.world',
            description='Gazebo world file name'),
        
        gazebo,
        robot_state_publisher_node,
        spawn_entity,
    ])
