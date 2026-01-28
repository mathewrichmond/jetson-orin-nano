#!/usr/bin/env python3
"""
Audio Pipeline Module Launch File
Launches audio feature extractor, speech recognition, and orchestrator
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for audio pipeline module"""
    
    # Launch arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='audio',
        description='Namespace for audio pipeline nodes'
    )
    
    recognition_engine_arg = DeclareLaunchArgument(
        'recognition_engine',
        default_value='placeholder',
        description='Speech recognition engine (placeholder, whisper, deepspeech)'
    )
    
    enable_stt_arg = DeclareLaunchArgument(
        'enable_stt',
        default_value='true',
        description='Enable speech-to-text recognition'
    )
    
    # Get launch configurations
    namespace = LaunchConfiguration('namespace')
    recognition_engine = LaunchConfiguration('recognition_engine')
    enable_stt = LaunchConfiguration('enable_stt')
    
    # Audio Feature Extractor Node
    feature_extractor = Node(
        package='audio_pipeline',
        executable='audio_feature_extractor_node',
        name='audio_feature_extractor',
        namespace=namespace,
        parameters=[{
            'audio_topic': '/sensor_fusion/audio/raw',
            'output_namespace': '/audio',
            'sample_rate': 16000,
            'channels': 2,
            'mfcc_coefficients': 13,
            'fft_size': 2048,
            'hop_length': 512,
            'enable_vad': True,
            'vad_threshold': 0.5,
        }],
        output='screen'
    )
    
    # Speech Recognition Node
    speech_recognition = Node(
        package='audio_pipeline',
        executable='speech_recognition_node',
        name='speech_recognition',
        namespace=namespace,
        parameters=[{
            'audio_topic': '/sensor_fusion/audio/raw',
            'output_namespace': '/audio',
            'sample_rate': 16000,
            'channels': 2,
            'recognition_engine': recognition_engine,
            'language': 'en-US',
            'model_size': 'base',
            'buffer_duration_sec': 2.0,
            'enable_continuous': True,
            'vad_enabled': True,
        }],
        output='screen',
        condition=IfCondition(enable_stt)
    )
    
    # Audio Pipeline Orchestrator Node
    audio_pipeline = Node(
        package='audio_pipeline',
        executable='audio_pipeline_node',
        name='audio_pipeline',
        namespace='jetson',
        parameters=[{
            'output_namespace': '/jetson',
            'audio_namespace': '/audio',
            'initial_mode': 'FULL',
            'enable_feature_extraction': True,
            'enable_speech_recognition': enable_stt,
            'auto_mode_switching': True,
            'battery_threshold_reduced': 25.0,
            'battery_threshold_sleep': 15.0,
            'temp_threshold_reduce': 75.0,
        }],
        output='screen'
    )
    
    return LaunchDescription([
        namespace_arg,
        recognition_engine_arg,
        enable_stt_arg,
        feature_extractor,
        speech_recognition,
        audio_pipeline,
    ])
