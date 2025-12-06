from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')
    
    # Gazebo launch
    gazebo_launch_file = PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'gazebo.launch.py'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=[
                PathJoinSubstitution([
                    FindPackageShare('gazebo_ros'),
                    'worlds',
                    'empty_world.world'
                ]),
                ''
            ],
            description='Gazebo world to load'
        ),
        
        Node(
            package='gazebo_ros',
            executable='gazebo_node.py',
            parameters=[{'world_name': LaunchConfiguration('world')}],
            output='screen'
        ),
    ])