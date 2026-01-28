#!/usr/bin/env python3
"""
Chassis Control Module Launch File
Launches IMU processor, chassis controller, and calibration manager
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for chassis control module"""
    
    # Launch arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='rpi',
        description='Namespace for chassis control nodes'
    )
    
    calibration_file_arg = DeclareLaunchArgument(
        'calibration_file',
        default_value='/home/nano/.config/robot/calibration.yaml',
        description='Path to calibration file'
    )
    
    # Get launch configurations
    namespace = LaunchConfiguration('namespace')
    calibration_file = LaunchConfiguration('calibration_file')
    
    # IMU Processor Node
    imu_processor = Node(
        package='chassis_control',
        executable='imu_processor_node',
        name='imu_processor',
        namespace=namespace,
        parameters=[{
            'raw_imu_topic': '/hardware/phat/imu',
            'filtered_imu_topic': '/rpi/imu/filtered',
            'publish_rate': 50.0,
            'kalman_filter_enabled': True,
            'imu_process_noise': 0.01,
            'imu_measurement_noise': 0.1,
        }],
        output='screen'
    )
    
    # Chassis Controller Node
    chassis_controller = Node(
        package='chassis_control',
        executable='chassis_controller_node',
        name='chassis_controller',
        namespace=namespace,
        parameters=[{
            'publish_rate': 50.0,
            'max_linear_velocity': 0.5,
            'max_angular_velocity': 2.0,
            'wheel_diameter_m': 0.072,
            'wheelbase_m': 0.235,
            'keyframe_drift_threshold_m': 0.1,
        }],
        output='screen'
    )
    
    # Calibration Manager Node
    calibration_manager = Node(
        package='chassis_control',
        executable='calibration_manager_node',
        name='calibration_manager',
        namespace=namespace,
        parameters=[{
            'calibration_file': calibration_file,
            'use_last_known_calibration': True,
            'auto_save_interval_sec': 300.0,
            'min_samples_for_update': 10,
            'calibration_learning_rate': 0.01,
        }],
        output='screen'
    )
    
    return LaunchDescription([
        namespace_arg,
        calibration_file_arg,
        imu_processor,
        chassis_controller,
        calibration_manager,
    ])
