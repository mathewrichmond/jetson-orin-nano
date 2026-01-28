#!/usr/bin/env python3
"""
Vision Pipeline Module Launch File
Launches visual SLAM, camera calibration, and vision pipeline orchestrator
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for vision pipeline module"""
    
    # Launch arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='vision',
        description='Namespace for vision pipeline nodes'
    )
    
    rtabmap_enabled_arg = DeclareLaunchArgument(
        'rtabmap_enabled',
        default_value='false',
        description='Enable RTAB-Map backend (requires rtabmap_ros installed)'
    )
    
    # Get launch configurations
    namespace = LaunchConfiguration('namespace')
    rtabmap_enabled = LaunchConfiguration('rtabmap_enabled')
    
    # Visual SLAM Node
    visual_slam = Node(
        package='vision_pipeline',
        executable='visual_slam_node',
        name='visual_slam',
        namespace=namespace,
        parameters=[{
            'camera_name': 'camera_front',
            'use_depth': True,
            'use_imu': True,
            'publish_global_pose': True,
            'global_frame_id': 'map',
            'odom_frame_id': 'odom',
            'base_frame_id': 'base_link',
            'rtabmap_enabled': rtabmap_enabled,
            'rtabmap_min_inliers': 20,
            'rtabmap_max_depth': 4.0,
            'rtabmap_loop_closure_enabled': True,
            'update_rate': 10.0,
        }],
        output='screen'
    )
    
    # Camera Calibration Node
    camera_calibration = Node(
        package='vision_pipeline',
        executable='camera_calibration_node',
        name='camera_calibration',
        namespace=namespace,
        parameters=[{
            'camera_names': ['camera_front', 'camera_rear'],
            'calibration_window_sec': 30.0,
            'min_motion_for_calibration': 0.5,
            'min_rotation_for_calibration': 0.3,
            'calibration_update_rate': 0.1,
        }],
        output='screen'
    )
    
    # Vision Pipeline Orchestrator Node
    vision_pipeline = Node(
        package='vision_pipeline',
        executable='vision_pipeline_node',
        name='vision_pipeline',
        namespace='jetson',
        parameters=[{
            'initial_mode': 'FULL',
            'camera_names': ['camera_front', 'camera_rear'],
            'camera_timeout_sec': 5.0,
            'auto_mode_switching': True,
            'battery_threshold_tracking_only': 20.0,
            'battery_threshold_sleep': 10.0,
            'temp_threshold_reduce': 75.0,
            'request_gpu_on_full': True,
            'request_gpu_off_sleep': True,
            'gpu_request_priority': 3,
            'health_publish_rate': 1.0,
        }],
        output='screen'
    )
    
    return LaunchDescription([
        namespace_arg,
        rtabmap_enabled_arg,
        visual_slam,
        camera_calibration,
        vision_pipeline,
    ])
