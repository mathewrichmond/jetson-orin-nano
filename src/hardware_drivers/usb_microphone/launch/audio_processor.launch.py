#!/usr/bin/env python3
"""Launch file for audio processor node"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="usb_microphone",
            executable="audio_processor_node",
            name="audio_processor_node",
            parameters=[
                {
                    "audio_topic": "/microphone/audio",
                    "sample_rate": 16000,
                    "channels": 2,
                    "format": "S16_LE",
                    "volume_history_size": 100,
                    "waveform_samples": 100,
                }
            ],
            output="screen",
        ),
    ])
