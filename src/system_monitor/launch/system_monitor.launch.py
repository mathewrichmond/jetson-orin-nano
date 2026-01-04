from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value='',
            description='Path to config file'
        ),
        DeclareLaunchArgument(
            'update_rate',
            default_value='1.0',
            description='Update rate in Hz'
        ),
        DeclareLaunchArgument(
            'temp_warning',
            default_value='70.0',
            description='Temperature warning threshold (°C)'
        ),
        DeclareLaunchArgument(
            'temp_critical',
            default_value='85.0',
            description='Temperature critical threshold (°C)'
        ),
        Node(
            package='system_monitor',
            executable='system_monitor_node',
            name='system_monitor',
            output='screen',
            parameters=[
                {'update_rate': LaunchConfiguration('update_rate')},
                {'temp_warning_threshold': LaunchConfiguration('temp_warning')},
                {'temp_critical_threshold': LaunchConfiguration('temp_critical')},
            ],
        ),
    ])

