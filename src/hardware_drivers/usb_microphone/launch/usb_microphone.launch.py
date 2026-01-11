# Third-party
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file", default_value="", description="Path to config file"
            ),
            Node(
                package="usb_microphone",
                executable="usb_microphone_node",
                name="usb_microphone_node",
                parameters=(
                    [LaunchConfiguration("config_file")]
                    if LaunchConfiguration("config_file") != ""
                    else []
                ),
                output="screen",
            ),
        ]
    )
