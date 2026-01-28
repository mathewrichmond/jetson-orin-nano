#!/usr/bin/env python3
"""
Test launch file for chassis_control module
Launches all three nodes with minimal configuration for testing
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # IMU Processor Node
        Node(
            package='chassis_control',
            executable='imu_processor_node',
            name='imu_processor_node',
            namespace='test',
            parameters=[{
                'raw_imu_topic': '/test/hardware/phat/imu',
                'filtered_imu_topic': '/test/rpi/imu/filtered',
                'publish_rate': 50.0,
                'kalman_filter_enabled': True,
                'health_topic': 'health/imu_processor_node',
                'health_publish_rate': 1.0,
            }],
            output='screen',
        ),

        # Calibration Manager Node
        Node(
            package='chassis_control',
            executable='calibration_manager_node',
            name='calibration_manager_node',
            namespace='test',
            parameters=[{
                'calibration_file': '/tmp/test_calibration.yaml',
                'auto_save_interval_sec': 60.0,
                'use_last_known_calibration': False,  # Don't load existing for test
                'vision_pose_topic': '/test/vision/global_pose',
                'odometry_topic': '/test/rpi/chassis/odometry',
                'calibration_status_topic': '/test/rpi/calibration/status',
                'health_topic': 'health/calibration_manager_node',
            }],
            output='screen',
        ),

        # Chassis Controller Node
        Node(
            package='chassis_control',
            executable='chassis_controller_node',
            name='chassis_controller_node',
            namespace='test',
            parameters=[{
                'cmd_vel_topic': '/test/control/cmd_vel',
                'imu_topic': '/test/rpi/imu/filtered',
                'odometry_output_topic': '/test/rpi/chassis/odometry',
                'pose_estimate_topic': '/test/rpi/chassis/pose_estimate',
                'vision_pose_topic': '/test/vision/global_pose',
                'publish_rate': 50.0,
                'health_topic': 'health/chassis_controller_node',
            }],
            output='screen',
        ),
    ])
