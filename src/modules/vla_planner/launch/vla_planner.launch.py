#!/usr/bin/env python3
"""
VLA Planner Module Launch File
Launches VLA controller, action executor, and planner
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for VLA planner module"""
    
    # Launch arguments
    model_type_arg = DeclareLaunchArgument(
        'model_type',
        default_value='placeholder',
        description='VLA model type (placeholder, openvla, rt1)'
    )
    
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='',
        description='Path to VLA model weights'
    )
    
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='cuda',
        description='Inference device (cuda or cpu)'
    )
    
    inference_rate_arg = DeclareLaunchArgument(
        'inference_rate',
        default_value='10.0',
        description='VLA inference rate in Hz'
    )
    
    # Get launch configurations
    model_type = LaunchConfiguration('model_type')
    model_path = LaunchConfiguration('model_path')
    device = LaunchConfiguration('device')
    inference_rate = LaunchConfiguration('inference_rate')
    
    # VLA Controller Node
    vla_controller = Node(
        package='vla_planner',
        executable='vla_controller_node',
        name='vla_controller',
        namespace='vla',
        parameters=[{
            'model_path': model_path,
            'model_type': model_type,
            'device': device,
            'inference_rate': inference_rate,
            'vision_features_topic': '/sensor_fusion/vlm_features',
            'audio_features_topic': '/audio/features/mfcc',
            'robot_state_topic': '/rpi/chassis/pose_estimate',
            'transcription_topic': '/audio/transcription',
            'action_topic': '/vla/actions',
            'cmd_vel_topic': '/control/cmd_vel',
            'action_horizon': 10,
            'context_window': 5,
            'confidence_threshold': 0.5,
            'request_gpu': True,
            'gpu_priority': 4,
        }],
        output='screen'
    )
    
    # Action Executor Node
    action_executor = Node(
        package='vla_planner',
        executable='action_executor_node',
        name='action_executor',
        namespace='vla',
        parameters=[{
            'action_topic': '/vla/actions',
            'cmd_vel_topic': '/control/cmd_vel',
            'execution_feedback_topic': '/vla/execution_feedback',
            'max_linear_velocity': 0.5,
            'max_angular_velocity': 2.0,
            'action_timeout_sec': 5.0,
            'enable_safety_checks': True,
            'action_buffer_size': 10,
            'execution_rate': 50.0,
        }],
        output='screen'
    )
    
    # Planner Node
    planner = Node(
        package='vla_planner',
        executable='planner_node',
        name='planner',
        namespace='vla',
        parameters=[{
            'task_queue_size': 10,
            'max_retries': 3,
            'task_timeout_sec': 60.0,
            'planning_rate': 1.0,
            'command_topic': '/vla/command',
            'transcription_topic': '/audio/transcription',
            'execution_feedback_topic': '/vla/execution_feedback',
            'plan_status_topic': '/vla/plan_status',
        }],
        output='screen'
    )
    
    return LaunchDescription([
        model_type_arg,
        model_path_arg,
        device_arg,
        inference_rate_arg,
        vla_controller,
        action_executor,
        planner,
    ])
