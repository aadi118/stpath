from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Launch description for rover path follower node.

    Usage:
    ros2 launch rover_navigation rover_path_follower_launch.py

    With custom parameters:
    ros2 launch rover_navigation rover_path_follower_launch.py L_min:=3.0 k:=2.0
    """

    # Declare launch arguments with default values
    return LaunchDescription([
        DeclareLaunchArgument(
            'L_min',
            default_value='2.0',
            description='Minimum look-ahead distance in meters'
        ),
        DeclareLaunchArgument(
            'k',
            default_value='1.5',
            description='Cross-track error multiplier for adaptive look-ahead'
        ),
        DeclareLaunchArgument(
            'CTE_threshold',
            default_value='0.3',
            description='Cross-track error threshold to start correction (m)'
        ),
        DeclareLaunchArgument(
            'CTE_tolerance',
            default_value='0.1',
            description='Cross-track error tolerance for correction completion (m)'
        ),
        DeclareLaunchArgument(
            'heading_tol',
            default_value='0.05',
            description='Heading tolerance in radians'
        ),
        DeclareLaunchArgument(
            'forward_velocity',
            default_value='1.0',
            description='Forward velocity in m/s'
        ),
        DeclareLaunchArgument(
            'angular_velocity',
            default_value='0.5',
            description='Angular velocity for turns in rad/s'
        ),
        DeclareLaunchArgument(
            'control_frequency',
            default_value='10.0',
            description='Control loop frequency in Hz'
        ),
        DeclareLaunchArgument(
            'path_start_x',
            default_value='0.0',
            description='Path start X coordinate'
        ),
        DeclareLaunchArgument(
            'path_start_y',
            default_value='0.0',
            description='Path start Y coordinate'
        ),
        DeclareLaunchArgument(
            'path_end_x',
            default_value='20.0',
            description='Path end X coordinate'
        ),
        DeclareLaunchArgument(
            'path_end_y',
            default_value='0.0',
            description='Path end Y coordinate'
        ),

        # Create the rover path follower node
        Node(
            package='rover_navigation',
            executable='rover_path_follower',
            name='rover_path_follower',
            output='screen',
            parameters=[
                {
                    'L_min': LaunchConfiguration('L_min'),
                    'k': LaunchConfiguration('k'),
                    'CTE_threshold': LaunchConfiguration('CTE_threshold'),
                    'CTE_tolerance': LaunchConfiguration('CTE_tolerance'),
                    'heading_tol': LaunchConfiguration('heading_tol'),
                    'forward_velocity': LaunchConfiguration('forward_velocity'),
                    'angular_velocity': LaunchConfiguration('angular_velocity'),
                    'control_frequency': LaunchConfiguration('control_frequency'),
                    'path_start_x': LaunchConfiguration('path_start_x'),
                    'path_start_y': LaunchConfiguration('path_start_y'),
                    'path_end_x': LaunchConfiguration('path_end_x'),
                    'path_end_y': LaunchConfiguration('path_end_y'),
                }
            ]
        ),
    ])
