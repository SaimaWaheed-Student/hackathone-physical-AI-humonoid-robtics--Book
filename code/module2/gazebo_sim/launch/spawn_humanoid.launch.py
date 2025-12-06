from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_name',
            default_value='humanoid',
            description='Name of the robot'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        )
    )

    # Get URDF via xacro
    robot_description_content = PathJoinSubstitution([
        FindPackageShare('humanoid_description'), # Assuming humanoid_description is a ROS package
        'urdf',
        'humanoid.urdf'
    ])
    
    # Spawn robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', LaunchConfiguration('robot_name'),
                   '-file', robot_description_content,
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '0.0',
                   '-Y', '0.0'],
        output='screen'
    )

    return LaunchDescription(declared_arguments + [
        spawn_entity,
    ])