from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    declared_arguments = []
    
    # Argument for selecting the world file
    declared_arguments.append(
        DeclareLaunchArgument(
            'world',
            default_value='flat_world.world',
            description='Gazebo world to load (e.g., flat_world.world, uneven_terrain.world, high_friction_world.world)'
        )
    )

    # Find Gazebo ROS package share directory
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Path to the worlds directory in this package
    stability_test_worlds_dir = PathJoinSubstitution([
        FindPackageShare('capstone_stability_test'),
        'worlds'
    ])

    # Gazebo launch file
    gazebo_launch_file = PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'gazebo.launch.py'])
    
    # Launch Gazebo with the selected world
    gazebo_server = IncludeLaunchDescription(
        gazebo_launch_file,
        launch_arguments={'world': PathJoinSubstitution([stability_test_worlds_dir, LaunchConfiguration('world')])}.items()
    )

    # Get URDF via xacro
    robot_description_content = PathJoinSubstitution([
        FindPackageShare('humanoid_description'),
        'urdf',
        'humanoid.urdf'
    ])
    
    # Spawn robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'humanoid',
                   '-file', robot_description_content,
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '0.5', # Spawn slightly above ground
                   '-Y', '0.0'],
        output='screen'
    )

    return LaunchDescription(declared_arguments + [
        gazebo_server,
        spawn_entity,
    ])