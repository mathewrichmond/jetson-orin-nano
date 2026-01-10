from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value='',
            description='Path to config file'
        ),
        Node(
            package='odrive_controller',
            executable='odrive_controller_node',
            name='odrive_controller_node',
            parameters=[LaunchConfiguration('config_file')] if LaunchConfiguration('config_file') != '' else [],
            output='screen'
        ),
    ])
