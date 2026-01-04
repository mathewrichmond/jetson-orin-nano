#!/usr/bin/env python3
"""
System Monitor Node
Monitors system health including temperature, power, CPU, memory, and disk usage.
Publishes status messages and alerts on threshold violations.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Int32
from sensor_msgs.msg import Temperature
import psutil
import subprocess
import os
import time
from pathlib import Path
from typing import Dict, List, Optional


class SystemMonitorNode(Node):
    """ROS 2 node for system monitoring"""

    def __init__(self):
        super().__init__('system_monitor')

        # Parameters
        self.declare_parameter('update_rate', 1.0)  # Hz
        self.declare_parameter('temp_warning_threshold', 70.0)  # °C
        self.declare_parameter('temp_critical_threshold', 85.0)  # °C
        self.declare_parameter('cpu_warning_threshold', 80.0)  # %
        self.declare_parameter('memory_warning_threshold', 85.0)  # %
        self.declare_parameter('disk_warning_threshold', 85.0)  # %

        self.update_rate = self.get_parameter('update_rate').value
        self.temp_warning = self.get_parameter('temp_warning_threshold').value
        self.temp_critical = self.get_parameter('temp_critical_threshold').value
        self.cpu_warning = self.get_parameter('cpu_warning_threshold').value
        self.memory_warning = self.get_parameter('memory_warning_threshold').value
        self.disk_warning = self.get_parameter('disk_warning_threshold').value

        # Publishers
        self.status_pub = self.create_publisher(String, 'system/status', 10)
        self.cpu_temp_pub = self.create_publisher(Temperature, 'system/temperature/cpu', 10)
        self.gpu_temp_pub = self.create_publisher(Temperature, 'system/temperature/gpu', 10)
        self.cpu_usage_pub = self.create_publisher(Float32, 'system/cpu/usage', 10)
        self.memory_usage_pub = self.create_publisher(Float32, 'system/memory/usage', 10)
        self.disk_usage_pub = self.create_publisher(Float32, 'system/disk/usage', 10)
        self.power_pub = self.create_publisher(Float32, 'system/power', 10)
        self.alert_pub = self.create_publisher(String, 'system/alerts', 10)
        self.camera_health_pub = self.create_publisher(String, 'system/camera/health', 10)

        # Camera health monitoring
        self.camera_status_sub = self.create_subscription(
            String, 'realsense/status', self._camera_status_callback, 10
        )
        self.camera_last_seen: Dict[str, float] = {}
        self.camera_timeout = 5.0  # seconds
        self.expected_cameras = ['camera_front', 'camera_rear']

        # Thermal zone mappings
        self.thermal_zones = self._discover_thermal_zones()

        # Timer
        self.timer = self.create_timer(1.0 / self.update_rate, self.monitor_callback)

        self.get_logger().info('System Monitor Node started')
        self.get_logger().info(f'Monitoring {len(self.thermal_zones)} thermal zones')
        self.get_logger().info('Monitoring RealSense cameras')

    def _discover_thermal_zones(self) -> Dict[str, int]:
        """Discover available thermal zones"""
        zones = {}
        thermal_path = Path('/sys/class/thermal')

        if not thermal_path.exists():
            self.get_logger().warn('Thermal zones not found')
            return zones

        for zone_path in thermal_path.glob('thermal_zone*'):
            zone_num = int(zone_path.name.replace('thermal_zone', ''))
            type_file = zone_path / 'type'
            temp_file = zone_path / 'temp'

            if type_file.exists() and temp_file.exists():
                try:
                    zone_type = type_file.read_text().strip()
                    zones[zone_type] = zone_num
                except Exception as e:
                    self.get_logger().warn(f'Error reading thermal zone {zone_num}: {e}')

        return zones

    def _read_temperature(self, zone_num: int) -> Optional[float]:
        """Read temperature from thermal zone"""
        temp_file = Path(f'/sys/class/thermal/thermal_zone{zone_num}/temp')
        try:
            if temp_file.exists():
                temp_millidegrees = int(temp_file.read_text().strip())
                return temp_millidegrees / 1000.0
        except Exception as e:
            self.get_logger().debug(f'Error reading temperature zone {zone_num}: {e}')
        return None

    def _read_power(self) -> Optional[float]:
        """Read power consumption from INA3221 sensors if available"""
        # Try Jetson power sensors
        power_paths = [
            '/sys/bus/i2c/drivers/ina3221x/1-0040/iio:device0/in_power0_input',
            '/sys/bus/i2c/drivers/ina3221x/1-0041/iio:device0/in_power0_input',
        ]

        total_power = 0.0
        found_sensors = False

        for path in power_paths:
            try:
                if os.path.exists(path):
                    power_mw = float(Path(path).read_text().strip())
                    total_power += power_mw / 1000.0  # Convert mW to W
                    found_sensors = True
            except Exception:
                pass

        # Try reading from tegrastats if available
        if not found_sensors:
            try:
                result = subprocess.run(
                    ['tegrastats', '--interval', '1000', '--stop', '1'],
                    capture_output=True,
                    text=True,
                    timeout=2
                )
                # Parse power from tegrastats output (format varies)
                # This is a simplified parser
                for line in result.stdout.split('\n'):
                    if 'POM_5V_IN' in line or 'power' in line.lower():
                        # Extract power value (simplified)
                        import re
                        matches = re.findall(r'(\d+\.?\d*)\s*W', line)
                        if matches:
                            return float(matches[0])
            except Exception:
                pass

        return total_power if found_sensors else None

    def monitor_callback(self):
        """Main monitoring callback"""
        try:
            # CPU Temperature
            cpu_temp = None
            if 'cpu-thermal' in self.thermal_zones:
                cpu_temp = self._read_temperature(self.thermal_zones['cpu-thermal'])
            elif 'tj-thermal' in self.thermal_zones:
                cpu_temp = self._read_temperature(self.thermal_zones['tj-thermal'])

            if cpu_temp is not None:
                temp_msg = Temperature()
                temp_msg.temperature = cpu_temp
                temp_msg.header.stamp = self.get_clock().now().to_msg()
                self.cpu_temp_pub.publish(temp_msg)

                # Check thresholds
                if cpu_temp >= self.temp_critical:
                    self.publish_alert(f'CRITICAL: CPU temperature {cpu_temp:.1f}°C exceeds critical threshold')
                elif cpu_temp >= self.temp_warning:
                    self.publish_alert(f'WARNING: CPU temperature {cpu_temp:.1f}°C exceeds warning threshold')

            # GPU Temperature
            gpu_temp = None
            if 'gpu-thermal' in self.thermal_zones:
                gpu_temp = self._read_temperature(self.thermal_zones['gpu-thermal'])

            if gpu_temp is not None:
                temp_msg = Temperature()
                temp_msg.temperature = gpu_temp
                temp_msg.header.stamp = self.get_clock().now().to_msg()
                self.gpu_temp_pub.publish(temp_msg)

                # Check thresholds
                if gpu_temp >= self.temp_critical:
                    self.publish_alert(f'CRITICAL: GPU temperature {gpu_temp:.1f}°C exceeds critical threshold')
                elif gpu_temp >= self.temp_warning:
                    self.publish_alert(f'WARNING: GPU temperature {gpu_temp:.1f}°C exceeds warning threshold')

            # CPU Usage
            cpu_percent = psutil.cpu_percent(interval=0.1)
            cpu_msg = Float32()
            cpu_msg.data = float(cpu_percent)
            self.cpu_usage_pub.publish(cpu_msg)

            if cpu_percent >= self.cpu_warning:
                self.publish_alert(f'WARNING: CPU usage {cpu_percent:.1f}% exceeds threshold')

            # Memory Usage
            memory = psutil.virtual_memory()
            memory_percent = memory.percent
            memory_msg = Float32()
            memory_msg.data = float(memory_percent)
            self.memory_usage_pub.publish(memory_msg)

            if memory_percent >= self.memory_warning:
                self.publish_alert(f'WARNING: Memory usage {memory_percent:.1f}% exceeds threshold')

            # Disk Usage
            disk = psutil.disk_usage('/')
            disk_percent = disk.percent
            disk_msg = Float32()
            disk_msg.data = float(disk_percent)
            self.disk_usage_pub.publish(disk_msg)

            if disk_percent >= self.disk_warning:
                self.publish_alert(f'WARNING: Disk usage {disk_percent:.1f}% exceeds threshold')

            # Power Consumption
            power = self._read_power()
            if power is not None:
                power_msg = Float32()
                power_msg.data = power
                self.power_pub.publish(power_msg)

            # Status Summary
            status_msg = String()
            status_parts = []
            if cpu_temp is not None:
                status_parts.append(f'CPU: {cpu_temp:.1f}°C')
            if gpu_temp is not None:
                status_parts.append(f'GPU: {gpu_temp:.1f}°C')
            status_parts.append(f'CPU: {cpu_percent:.1f}%')
            status_parts.append(f'Mem: {memory_percent:.1f}%')
            status_parts.append(f'Disk: {disk_percent:.1f}%')
            if power is not None:
                status_parts.append(f'Power: {power:.2f}W')

            status_msg.data = ' | '.join(status_parts)
            self.status_pub.publish(status_msg)

            # Check camera health
            self._check_camera_health()

        except Exception as e:
            self.get_logger().error(f'Error in monitor callback: {e}')

    def _camera_status_callback(self, msg: String):
        """Callback for camera status messages"""
        # Parse status message to extract camera info
        # Format: [status] message
        current_time = time.time()

        # Update last seen time for cameras
        # This is a simple approach - in production, parse the actual camera names from status
        for camera_name in self.expected_cameras:
            if camera_name.lower() in msg.data.lower():
                self.camera_last_seen[camera_name] = current_time

        # Also check for general camera status
        if 'initialized' in msg.data.lower() or 'started' in msg.data.lower():
            # Mark all expected cameras as seen
            for camera_name in self.expected_cameras:
                if camera_name not in self.camera_last_seen:
                    self.camera_last_seen[camera_name] = current_time

    def _check_camera_health(self):
        """Check health of RealSense cameras"""
        current_time = time.time()
        camera_health_parts = []
        all_healthy = True

        # Check if cameras are publishing (by checking if we've seen status recently)
        # Also check if image topics exist and have recent data
        try:
            # Use ROS 2 topic list to check if camera topics exist
            result = subprocess.run(
                ['ros2', 'topic', 'list'],
                capture_output=True,
                text=True,
                timeout=1
            )

            if result.returncode == 0:
                topics = result.stdout.split('\n')
                for camera_name in self.expected_cameras:
                    color_topic = f'/{camera_name}/color/image_raw'
                    depth_topic = f'/{camera_name}/depth/image_rect_raw'

                    color_exists = any(color_topic in t for t in topics)
                    depth_exists = any(depth_topic in t for t in topics)

                    if color_exists and depth_exists:
                        camera_health_parts.append(f'{camera_name}: OK')
                        self.camera_last_seen[camera_name] = current_time
                    else:
                        camera_health_parts.append(f'{camera_name}: MISSING')
                        all_healthy = False
                        if camera_name not in self.camera_last_seen or \
                           (current_time - self.camera_last_seen.get(camera_name, 0)) > self.camera_timeout:
                            self.publish_alert(f'WARNING: Camera {camera_name} not publishing data')
            else:
                # ROS 2 not available or topics not accessible
                # Check based on last seen times
                for camera_name in self.expected_cameras:
                    last_seen = self.camera_last_seen.get(camera_name, 0)
                    if current_time - last_seen > self.camera_timeout:
                        camera_health_parts.append(f'{camera_name}: TIMEOUT')
                        all_healthy = False
                        self.publish_alert(f'WARNING: Camera {camera_name} timeout (no status for {self.camera_timeout}s)')
                    else:
                        camera_health_parts.append(f'{camera_name}: OK')

        except Exception as e:
            self.get_logger().debug(f'Error checking camera health: {e}')
            # If we can't check, assume cameras are OK if we've seen them recently
            for camera_name in self.expected_cameras:
                last_seen = self.camera_last_seen.get(camera_name, 0)
                if current_time - last_seen < self.camera_timeout:
                    camera_health_parts.append(f'{camera_name}: OK')
                else:
                    camera_health_parts.append(f'{camera_name}: UNKNOWN')

        # Publish camera health status
        if camera_health_parts:
            health_msg = String()
            health_msg.data = ' | '.join(camera_health_parts)
            self.camera_health_pub.publish(health_msg)

    def publish_alert(self, message: str):
        """Publish alert message"""
        alert_msg = String()
        alert_msg.data = f'[{time.strftime("%Y-%m-%d %H:%M:%S")}] {message}'
        self.alert_pub.publish(alert_msg)
        self.get_logger().warn(message)


def main(args=None):
    rclpy.init(args=args)
    node = SystemMonitorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
