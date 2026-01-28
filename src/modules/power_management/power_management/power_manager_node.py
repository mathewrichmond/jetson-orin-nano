#!/usr/bin/env python3
"""
Power Manager Node
Coordinates GPU power states, battery management, and thermal throttling
Implements power mode state machine based on battery level and system health
"""

# Standard library
from enum import Enum
import time
from typing import Dict, Optional

# Third-party
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from sensor_msgs.msg import BatteryState, Temperature
from custom_msgs.msg import PowerRequest, SystemPerformance

# Local
from isaac_utils import HealthStatusPublisher


class PowerMode(Enum):
    """Power modes for the robot"""
    FULL = "full"               # All systems active
    PERFORMANCE = "performance" # High compute, all features
    BALANCED = "balanced"       # Normal operation
    ECONOMY = "economy"         # Power saving, reduced features
    CRITICAL = "critical"       # Minimal operation, seeking charger
    CHARGING = "charging"       # Docked and charging


class PowerManagerNode(Node):
    """
    ROS 2 node for system-wide power management

    Manages:
    - GPU power states (on/off/sleep)
    - Power mode transitions based on battery
    - Thermal throttling
    - Power request arbitration
    """

    def __init__(self):
        super().__init__("power_manager_node")

        # Parameters
        self.declare_parameter("initial_mode", "balanced")
        self.declare_parameter("auto_mode_switching", True)

        # Battery thresholds (percent)
        self.declare_parameter("battery_full_threshold", 95.0)
        self.declare_parameter("battery_economy_threshold", 30.0)
        self.declare_parameter("battery_critical_threshold", 15.0)

        # Thermal thresholds (celsius)
        self.declare_parameter("cpu_temp_warn", 70.0)
        self.declare_parameter("cpu_temp_critical", 85.0)
        self.declare_parameter("gpu_temp_warn", 75.0)
        self.declare_parameter("gpu_temp_critical", 90.0)

        # Topics
        self.declare_parameter("battery_topic", "/irobot/battery")
        self.declare_parameter("cpu_temp_topic", "/jetson/temperature/cpu")
        self.declare_parameter("gpu_temp_topic", "/jetson/temperature/gpu")
        self.declare_parameter("power_request_topic", "/jetson/power/request")
        self.declare_parameter("power_mode_topic", "/jetson/power/mode")
        self.declare_parameter("gpu_power_control_topic", "/jetson/power/gpu_control")
        self.declare_parameter("system_performance_topic", "/jetson/system/performance")

        # Health monitoring
        self.declare_parameter("health_topic", f"health/{self.get_name()}")
        self.declare_parameter("health_publish_rate", 1.0)

        # Get parameters
        initial_mode_str = str(self.get_parameter("initial_mode").value)
        self.auto_mode_switching = bool(self.get_parameter("auto_mode_switching").value)

        self.battery_full_threshold = float(self.get_parameter("battery_full_threshold").value)
        self.battery_economy_threshold = float(self.get_parameter("battery_economy_threshold").value)
        self.battery_critical_threshold = float(self.get_parameter("battery_critical_threshold").value)

        self.cpu_temp_warn = float(self.get_parameter("cpu_temp_warn").value)
        self.cpu_temp_critical = float(self.get_parameter("cpu_temp_critical").value)
        self.gpu_temp_warn = float(self.get_parameter("gpu_temp_warn").value)
        self.gpu_temp_critical = float(self.get_parameter("gpu_temp_critical").value)

        battery_topic = str(self.get_parameter("battery_topic").value)
        cpu_temp_topic = str(self.get_parameter("cpu_temp_topic").value)
        gpu_temp_topic = str(self.get_parameter("gpu_temp_topic").value)
        power_request_topic = str(self.get_parameter("power_request_topic").value)
        power_mode_topic = str(self.get_parameter("power_mode_topic").value)
        gpu_power_control_topic = str(self.get_parameter("gpu_power_control_topic").value)
        system_performance_topic = str(self.get_parameter("system_performance_topic").value)

        health_topic = str(self.get_parameter("health_topic").value)
        health_rate = float(self.get_parameter("health_publish_rate").value)

        # State
        try:
            self.current_mode = PowerMode(initial_mode_str)
        except ValueError:
            self.get_logger().warn(f"Invalid initial mode '{initial_mode_str}', using BALANCED")
            self.current_mode = PowerMode.BALANCED

        self.battery_level = 100.0  # percent
        self.battery_voltage = 0.0  # volts
        self.battery_current = 0.0  # amps
        self.is_charging = False

        self.cpu_temp = 0.0  # celsius
        self.gpu_temp = 0.0  # celsius

        self.gpu_power_state = "off"  # off, on, sleep
        self.pending_power_requests: Dict[str, PowerRequest] = {}  # {requester: request}

        # Subscribers
        self.battery_sub = self.create_subscription(
            BatteryState,
            battery_topic,
            self._battery_callback,
            10
        )
        self.cpu_temp_sub = self.create_subscription(
            Temperature,
            cpu_temp_topic,
            self._cpu_temp_callback,
            10
        )
        self.gpu_temp_sub = self.create_subscription(
            Temperature,
            gpu_temp_topic,
            self._gpu_temp_callback,
            10
        )
        self.power_request_sub = self.create_subscription(
            PowerRequest,
            power_request_topic,
            self._power_request_callback,
            10
        )

        # Publishers
        self.power_mode_pub = self.create_publisher(
            String,
            power_mode_topic,
            10
        )
        self.gpu_power_control_pub = self.create_publisher(
            String,
            gpu_power_control_topic,
            10
        )
        self.system_performance_pub = self.create_publisher(
            SystemPerformance,
            system_performance_topic,
            10
        )

        # Health monitoring
        self.health_pub = HealthStatusPublisher(
            self,
            health_topic,
            rate_hz=health_rate,
        )

        # Timers
        self.mode_check_timer = self.create_timer(5.0, self._check_mode_transition)
        self.power_arbitration_timer = self.create_timer(1.0, self._arbitrate_power_requests)
        self.health_timer = self.create_timer(1.0 / health_rate, self._publish_health)
        self.performance_timer = self.create_timer(1.0, self._publish_system_performance)

        # Initialize
        self._publish_power_mode()
        self._apply_power_mode(self.current_mode)

        self.get_logger().info(
            f"Power Manager initialized: mode={self.current_mode.value}, "
            f"auto_switching={self.auto_mode_switching}"
        )

    def _battery_callback(self, msg: BatteryState):
        """Receive battery state"""
        self.battery_level = msg.percentage * 100.0  # Convert to percent
        self.battery_voltage = msg.voltage
        self.battery_current = msg.current
        self.is_charging = (msg.current > 0.1)  # Positive current = charging

    def _cpu_temp_callback(self, msg: Temperature):
        """Receive CPU temperature"""
        self.cpu_temp = msg.temperature

    def _gpu_temp_callback(self, msg: Temperature):
        """Receive GPU temperature"""
        self.gpu_temp = msg.temperature

    def _power_request_callback(self, msg: PowerRequest):
        """Receive power request from module"""
        requester = msg.requester_node
        self.pending_power_requests[requester] = msg

        self.get_logger().debug(
            f"Power request from {requester}: {msg.requested_state} "
            f"(priority={msg.priority}, reason={msg.reason})"
        )

    def _check_mode_transition(self):
        """Check if power mode should transition based on system state"""
        if not self.auto_mode_switching:
            return

        new_mode = self.current_mode

        # Charging detection
        if self.is_charging and self.battery_level > self.battery_full_threshold:
            new_mode = PowerMode.FULL
        elif self.is_charging:
            new_mode = PowerMode.CHARGING

        # Battery-based modes (when not charging)
        elif self.battery_level < self.battery_critical_threshold:
            new_mode = PowerMode.CRITICAL
        elif self.battery_level < self.battery_economy_threshold:
            new_mode = PowerMode.ECONOMY
        elif self.battery_level > self.battery_economy_threshold * 1.5:
            new_mode = PowerMode.BALANCED

        # Thermal throttling (overrides battery-based modes)
        if self.cpu_temp > self.cpu_temp_critical or self.gpu_temp > self.gpu_temp_critical:
            if new_mode not in [PowerMode.CRITICAL, PowerMode.ECONOMY]:
                new_mode = PowerMode.ECONOMY
                self.get_logger().warn(
                    f"Thermal throttling: CPU={self.cpu_temp:.1f}°C, GPU={self.gpu_temp:.1f}°C"
                )

        # Apply transition
        if new_mode != self.current_mode:
            self._transition_power_mode(new_mode)

    def _transition_power_mode(self, new_mode: PowerMode):
        """Transition to new power mode"""
        old_mode = self.current_mode
        self.current_mode = new_mode

        self.get_logger().info(
            f"Power mode transition: {old_mode.value} → {new_mode.value} "
            f"(battery={self.battery_level:.1f}%, "
            f"CPU={self.cpu_temp:.1f}°C, GPU={self.gpu_temp:.1f}°C)"
        )

        self._apply_power_mode(new_mode)
        self._publish_power_mode()

    def _apply_power_mode(self, mode: PowerMode):
        """Apply power mode settings"""
        # Determine GPU power state for this mode
        gpu_state_map = {
            PowerMode.FULL: "on",
            PowerMode.PERFORMANCE: "on",
            PowerMode.BALANCED: "on",
            PowerMode.ECONOMY: "sleep",
            PowerMode.CRITICAL: "off",
            PowerMode.CHARGING: "sleep",
        }

        desired_gpu_state = gpu_state_map.get(mode, "off")
        self._set_gpu_power(desired_gpu_state)

    def _set_gpu_power(self, state: str):
        """Set GPU power state"""
        if state == self.gpu_power_state:
            return

        self.get_logger().info(f"Setting GPU power: {self.gpu_power_state} → {state}")
        self.gpu_power_state = state

        # Publish to GPIO controller
        msg = String()
        msg.data = state
        self.gpu_power_control_pub.publish(msg)

    def _arbitrate_power_requests(self):
        """Arbitrate conflicting power requests from modules"""
        if not self.pending_power_requests:
            return

        # Find highest priority request
        highest_priority_request = max(
            self.pending_power_requests.values(),
            key=lambda req: req.priority
        )

        # Apply the request (for GPU power)
        if highest_priority_request.requested_state in ["on", "off", "sleep", "wake"]:
            target_state = highest_priority_request.requested_state
            if target_state == "wake":
                target_state = "on"

            # Only allow power increase if not in critical mode
            if self.current_mode != PowerMode.CRITICAL or target_state == "off":
                self._set_gpu_power(target_state)
            else:
                self.get_logger().warn(
                    f"Power request denied in CRITICAL mode: {target_state} from "
                    f"{highest_priority_request.requester_node}"
                )

        # Clear expired requests
        current_time = self.get_clock().now().nanoseconds * 1e-9
        expired = []
        for requester, req in self.pending_power_requests.items():
            req_time = req.header.stamp.sec + req.header.stamp.nanosec * 1e-9
            if req.timeout_sec > 0 and (current_time - req_time) > req.timeout_sec:
                expired.append(requester)

        for requester in expired:
            del self.pending_power_requests[requester]

    def _publish_power_mode(self):
        """Publish current power mode"""
        msg = String()
        msg.data = self.current_mode.value
        self.power_mode_pub.publish(msg)

    def _publish_system_performance(self):
        """Publish system performance metrics"""
        msg = SystemPerformance()
        msg.header.stamp = self.get_clock().now().to_msg()

        # Temperatures
        msg.temperature_cpu = self.cpu_temp
        msg.temperature_gpu = self.gpu_temp

        # Battery
        msg.battery_level_pct = self.battery_level
        msg.battery_voltage_v = self.battery_voltage
        msg.battery_current_a = self.battery_current

        # TODO: Add CPU/GPU usage, memory, etc. from system monitor
        msg.cpu_usage_pct = 0.0
        msg.gpu_usage_pct = 0.0 if self.gpu_power_state == "off" else 0.0
        msg.memory_usage_pct = 0.0

        self.system_performance_pub.publish(msg)

    def _publish_health(self):
        """Publish health status"""
        # Determine status based on battery and temps
        if self.current_mode == PowerMode.CRITICAL:
            status = "ERROR"
            message = f"Critical battery: {self.battery_level:.1f}%"
        elif self.cpu_temp > self.cpu_temp_warn or self.gpu_temp > self.gpu_temp_warn:
            status = "WARN"
            message = f"High temperature: CPU={self.cpu_temp:.1f}°C, GPU={self.gpu_temp:.1f}°C"
        elif self.current_mode == PowerMode.ECONOMY:
            status = "WARN"
            message = f"Economy mode: battery={self.battery_level:.1f}%"
        else:
            status = "OK"
            message = f"Power {self.current_mode.value}: battery={self.battery_level:.1f}%, GPU={self.gpu_power_state}"

        self.health_pub.publish(status, message)


def main(args=None):
    rclpy.init(args=args)
    node = PowerManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
