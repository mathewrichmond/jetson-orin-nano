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
            package='irobot_serial',
            executable='irobot_serial_node',
            name='irobot_serial_node',
            parameters=[LaunchConfiguration('config_file')] if LaunchConfiguration('config_file') != '' else [],
            output='screen'
        ),
    ])
