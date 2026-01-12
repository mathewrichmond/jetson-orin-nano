#!/usr/bin/env python3
"""
System Monitor Node
Monitors system health: temperature, CPU, memory, disk, power
Publishes status via ROS 2 topics
"""

# Standard library
import re
import subprocess
from pathlib import Path
from typing import Optional

# Third-party
import psutil
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature
from std_msgs.msg import Float32, String


class SystemMonitorNode(Node):
    """ROS 2 node for system monitoring"""

    def __init__(self):
        super().__init__("system_monitor_node")

        # Parameters
        self.declare_parameter("update_rate", 1.0)
        self.declare_parameter("temp_warning_threshold", 70.0)
        self.declare_parameter("temp_critical_threshold", 85.0)
        self.declare_parameter("cpu_warning_threshold", 80.0)
        self.declare_parameter("memory_warning_threshold", 85.0)
        self.declare_parameter("disk_warning_threshold", 85.0)

        self.update_rate = self.get_parameter("update_rate").value
        self.temp_warning = self.get_parameter("temp_warning_threshold").value
        self.temp_critical = self.get_parameter("temp_critical_threshold").value
        self.cpu_warning = self.get_parameter("cpu_warning_threshold").value
        self.memory_warning = self.get_parameter("memory_warning_threshold").value
        self.disk_warning = self.get_parameter("disk_warning_threshold").value

        # Publishers
        self.status_pub = self.create_publisher(String, "status", 10)
        self.cpu_temp_pub = self.create_publisher(Temperature, "temperature/cpu", 10)
        self.gpu_temp_pub = self.create_publisher(Temperature, "temperature/gpu", 10)
        self.cpu_usage_pub = self.create_publisher(Float32, "cpu/usage", 10)
        self.gpu_usage_pub = self.create_publisher(Float32, "gpu/usage", 10)
        self.memory_usage_pub = self.create_publisher(Float32, "memory/usage", 10)
        self.disk_usage_pub = self.create_publisher(Float32, "disk/usage", 10)
        self.power_pub = self.create_publisher(Float32, "power", 10)
        self.alerts_pub = self.create_publisher(String, "alerts", 10)

        # Thermal zone paths
        self.thermal_zones = self._find_thermal_zones()

        # Timer
        timer_period = 1.0 / self.update_rate
        self.timer = self.create_timer(timer_period, self.update)

        self.get_logger().info("System Monitor Node started")
        self.publish_status("initialized", "System monitor started")

    def _find_thermal_zones(self) -> dict:
        """Find available thermal zones"""
        zones = {}
        thermal_path = Path("/sys/class/thermal")

        if not thermal_path.exists():
            self.get_logger().warn("Thermal zones not found")
            return zones

        for zone_path in thermal_path.glob("thermal_zone*"):
            try:
                zone_name = (zone_path / "type").read_text().strip()
                temp_file = zone_path / "temp"
                if temp_file.exists():
                    zones[zone_name] = temp_file
            except Exception as e:
                self.get_logger().debug(f"Error reading thermal zone {zone_path}: {e}")

        self.get_logger().info(f"Found {len(zones)} thermal zones")
        return zones

    def _read_temperature(self, zone_name: str) -> Optional[float]:
        """Read temperature from thermal zone"""
        if zone_name not in self.thermal_zones:
            return None

        try:
            temp_file = self.thermal_zones[zone_name]
            temp_millidegrees = int(temp_file.read_text().strip())
            return temp_millidegrees / 1000.0  # Convert to Celsius
        except Exception as e:
            self.get_logger().debug(f"Error reading {zone_name}: {e}")
            return None

    def _read_cpu_temperature(self) -> Optional[float]:
        """Read CPU temperature"""
        # Try common thermal zone names
        for name in ["cpu-thermal", "CPU-thermal", "cpu", "Tboard"]:
            temp = self._read_temperature(name)
            if temp is not None:
                return temp

        # Try to find any zone with 'cpu' in name
        for zone_name in self.thermal_zones.keys():
            if "cpu" in zone_name.lower():
                temp = self._read_temperature(zone_name)
                if temp is not None:
                    return temp

        return None

    def _read_gpu_temperature(self) -> Optional[float]:
        """Read GPU temperature"""
        # Try common thermal zone names
        for name in ["gpu-thermal", "GPU-thermal", "gpu", "Tdiode_GPU"]:
            temp = self._read_temperature(name)
            if temp is not None:
                return temp

        # Try to find any zone with 'gpu' in name
        for zone_name in self.thermal_zones.keys():
            if "gpu" in zone_name.lower():
                temp = self._read_temperature(zone_name)
                if temp is not None:
                    return temp

        return None

    def _read_gpu_usage(self) -> Optional[float]:
        """Read GPU usage percentage"""
        # Try nvidia-smi first (works on discrete GPUs)
        try:
            result = subprocess.run(
                ["nvidia-smi", "--query-gpu=utilization.gpu", "--format=csv,noheader,nounits"],
                capture_output=True,
                text=True,
                timeout=1.0,
            )
            if result.returncode == 0:
                usage_str = result.stdout.strip()
                # Check if it's a valid number (not "[N/A]")
                if usage_str and usage_str != "[N/A]" and usage_str.replace(".", "").isdigit():
                    return float(usage_str)
        except (subprocess.TimeoutExpired, FileNotFoundError, ValueError) as e:
            self.get_logger().debug(f"nvidia-smi GPU usage read failed: {e}")

        # Fallback: try reading from /sys/devices/gpu.0/load (Jetson)
        try:
            gpu_load_path = Path("/sys/devices/gpu.0/load")
            if gpu_load_path.exists():
                load_value = int(gpu_load_path.read_text().strip())
                # Load is typically 0-1000, convert to percentage
                return min(100.0, load_value / 10.0)
        except (ValueError, IOError) as e:
            self.get_logger().debug(f"GPU load file read failed: {e}")

        # Fallback: parse tegrastats output (Jetson-specific)
        # Use a quick timeout to avoid blocking the update loop
        try:
            # Try to read from a cached tegrastats process or spawn quickly
            process = subprocess.Popen(
                ["timeout", "0.5", "tegrastats", "--interval", "1000"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            stdout, _ = process.communicate(timeout=0.6)

            if stdout:
                # Parse GR3D_FREQ percentage from tegrastats output
                # Format: "GR3D_FREQ 14%" or similar
                match = re.search(r"GR3D_FREQ\s+(\d+)%", stdout)
                if match:
                    return float(match.group(1))
        except (subprocess.TimeoutExpired, FileNotFoundError, ValueError, AttributeError, subprocess.SubprocessError) as e:
            # Fallback: try reading directly with a quick subprocess
            try:
                result = subprocess.run(
                    ["sh", "-c", "timeout 0.5 tegrastats --interval 1000 | head -1"],
                    capture_output=True,
                    text=True,
                    timeout=0.7,
                )
                if result.returncode == 0 and result.stdout:
                    match = re.search(r"GR3D_FREQ\s+(\d+)%", result.stdout)
                    if match:
                        return float(match.group(1))
            except Exception:
                pass
            self.get_logger().debug(f"tegrastats GPU usage read failed: {e}")

        return None

    def _read_power(self) -> Optional[float]:
        """Read power consumption (simplified - returns None if not available)"""
        # Power monitoring would require jetson_stats or INA3221 access
        # For now, return None (can be extended later)
        return None

    def update(self):
        """Update and publish system metrics"""
        try:
            # CPU temperature
            cpu_temp = self._read_cpu_temperature()
            if cpu_temp is not None:
                temp_msg = Temperature()
                temp_msg.header.stamp = self.get_clock().now().to_msg()
                temp_msg.header.frame_id = "cpu"
                temp_msg.temperature = cpu_temp
                temp_msg.variance = 0.0
                self.cpu_temp_pub.publish(temp_msg)

                # Check thresholds
                if cpu_temp >= self.temp_critical:
                    self.publish_alert(f"CRITICAL: CPU temperature {cpu_temp:.1f}°C")
                elif cpu_temp >= self.temp_warning:
                    self.publish_alert(f"WARNING: CPU temperature {cpu_temp:.1f}°C")

            # GPU temperature
            gpu_temp = self._read_gpu_temperature()
            if gpu_temp is not None:
                temp_msg = Temperature()
                temp_msg.header.stamp = self.get_clock().now().to_msg()
                temp_msg.header.frame_id = "gpu"
                temp_msg.temperature = gpu_temp
                temp_msg.variance = 0.0
                self.gpu_temp_pub.publish(temp_msg)

            # CPU usage
            cpu_percent = psutil.cpu_percent(interval=None)
            cpu_msg = Float32()
            cpu_msg.data = float(cpu_percent)
            self.cpu_usage_pub.publish(cpu_msg)

            if cpu_percent >= self.cpu_warning:
                self.publish_alert(f"WARNING: CPU usage {cpu_percent:.1f}%")

            # GPU usage
            gpu_percent = self._read_gpu_usage()
            if gpu_percent is not None:
                gpu_msg = Float32()
                gpu_msg.data = float(gpu_percent)
                self.gpu_usage_pub.publish(gpu_msg)

            # Memory usage
            memory = psutil.virtual_memory()
            memory_percent = memory.percent
            memory_msg = Float32()
            memory_msg.data = float(memory_percent)
            self.memory_usage_pub.publish(memory_msg)

            if memory_percent >= self.memory_warning:
                self.publish_alert(f"WARNING: Memory usage {memory_percent:.1f}%")

            # Disk usage
            disk = psutil.disk_usage("/")
            disk_percent = disk.percent
            disk_msg = Float32()
            disk_msg.data = float(disk_percent)
            self.disk_usage_pub.publish(disk_msg)

            if disk_percent >= self.disk_warning:
                self.publish_alert(f"WARNING: Disk usage {disk_percent:.1f}%")

            # Power (if available)
            power = self._read_power()
            if power is not None:
                power_msg = Float32()
                power_msg.data = power
                self.power_pub.publish(power_msg)

            # Status summary
            status_parts = []
            if cpu_temp is not None:
                status_parts.append(f"CPU: {cpu_temp:.1f}°C")
            if gpu_temp is not None:
                status_parts.append(f"GPU: {gpu_temp:.1f}°C")
            status_parts.append(f"CPU: {cpu_percent:.1f}%")
            if gpu_percent is not None:
                status_parts.append(f"GPU: {gpu_percent:.1f}%")
            status_parts.append(f"Mem: {memory_percent:.1f}%")
            status_parts.append(f"Disk: {disk_percent:.1f}%")

            self.publish_status("operational", " | ".join(status_parts))

        except Exception as e:
            self.get_logger().error(f"Error updating metrics: {e}")
            self.publish_status("error", str(e))

    def publish_status(self, status: str, message: str):
        """Publish status message"""
        msg = String()
        msg.data = f"[{status}] {message}"
        self.status_pub.publish(msg)

    def publish_alert(self, alert: str):
        """Publish alert message"""
        msg = String()
        msg.data = alert
        self.alerts_pub.publish(msg)
        self.get_logger().warn(alert)


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


if __name__ == "__main__":
    main()
