#!/usr/bin/env python3
"""
Power Management Module Launch File
Launches power manager, GPIO controller, and battery monitor
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for power management module"""
    
    # Launch arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='jetson',
        description='Namespace for power management nodes'
    )
    
    mock_mode_arg = DeclareLaunchArgument(
        'mock_mode',
        default_value='false',
        description='Enable mock mode for GPIO (testing without hardware)'
    )
    
    # Get launch configurations
    namespace = LaunchConfiguration('namespace')
    mock_mode = LaunchConfiguration('mock_mode')
    
    # Power Manager Node
    power_manager = Node(
        package='power_management',
        executable='power_manager_node',
        name='power_manager',
        namespace=namespace,
        parameters=[{
            'initial_mode': 'AUTO',
            'battery_full_threshold': 45.0,
            'battery_balanced_threshold': 30.0,
            'battery_economy_threshold': 15.0,
            'battery_critical_threshold': 10.0,
            'temp_warning_threshold': 70.0,
            'temp_critical_threshold': 80.0,
            'temp_throttle_threshold': 75.0,
            'request_timeout_sec': 30.0,
            'publish_rate': 2.0,
        }],
        output='screen'
    )
    
    # GPIO Controller Node (non-composable - uses hardware)
    gpio_controller = Node(
        package='power_management',
        executable='gpio_controller_node',
        name='gpio_controller',
        namespace=namespace,
        parameters=[{
            'gpio_mode': 'BOARD',
            'mock_mode': mock_mode,
            'gpu_power_pin': 15,
            'gpu_enable_pin': 16,
            'power_on_delay_ms': 100,
            'power_off_delay_ms': 50,
            'max_toggle_frequency_hz': 1.0,
        }],
        output='screen'
    )
    
    # Battery Monitor Node
    battery_monitor = Node(
        package='power_management',
        executable='battery_monitor_node',
        name='battery_monitor',
        namespace=namespace,
        parameters=[{
            'voltage_low_threshold': 11.0,
            'voltage_critical_threshold': 10.5,
            'capacity_estimation_enabled': True,
            'runtime_estimation_enabled': True,
            'power_history_window_sec': 300.0,
            'runtime_update_rate': 0.2,
            'health_check_interval_sec': 10.0,
        }],
        output='screen'
    )
    
    return LaunchDescription([
        namespace_arg,
        mock_mode_arg,
        power_manager,
        gpio_controller,
        battery_monitor,
    ])
