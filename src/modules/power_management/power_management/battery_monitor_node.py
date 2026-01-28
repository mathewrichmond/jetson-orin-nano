#!/usr/bin/env python3
"""
Battery Monitor Node
Monitors battery state, estimates runtime, and provides battery health status
Publishes alerts for low battery conditions
"""

# Standard library
from collections import deque
import time
from typing import Optional

# Third-party
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

# Local
from isaac_utils import HealthStatusPublisher


class BatteryMonitorNode(Node):
    """
    ROS 2 node for battery monitoring and health assessment

    Monitors battery state from iRobot Create2 and estimates:
    - Remaining runtime
    - Battery health
    - Charging status
    - Power consumption trends
    """

    def __init__(self):
        super().__init__("battery_monitor_node")

        # Parameters
        self.declare_parameter("battery_topic", "/irobot/battery")
        self.declare_parameter("battery_alerts_topic", "/system/battery/alerts")

        # Battery thresholds
        self.declare_parameter("low_battery_threshold", 20.0)  # percent
        self.declare_parameter("critical_battery_threshold", 10.0)  # percent
        self.declare_parameter("full_battery_threshold", 95.0)  # percent

        # Runtime estimation
        self.declare_parameter("power_consumption_window_sec", 300.0)  # 5 minutes
        self.declare_parameter("default_power_consumption_w", 25.0)  # Watts

        # Health monitoring
        self.declare_parameter("health_topic", f"health/{self.get_name()}")
        self.declare_parameter("health_publish_rate", 1.0)

        # Get parameters
        battery_topic = str(self.get_parameter("battery_topic").value)
        battery_alerts_topic = str(self.get_parameter("battery_alerts_topic").value)

        self.low_battery_threshold = float(self.get_parameter("low_battery_threshold").value)
        self.critical_battery_threshold = float(self.get_parameter("critical_battery_threshold").value)
        self.full_battery_threshold = float(self.get_parameter("full_battery_threshold").value)

        self.power_window = float(self.get_parameter("power_consumption_window_sec").value)
        self.default_power_w = float(self.get_parameter("default_power_consumption_w").value)

        health_topic = str(self.get_parameter("health_topic").value)
        health_rate = float(self.get_parameter("health_publish_rate").value)

        # State
        self.battery_level = 100.0  # percent
        self.battery_voltage = 0.0  # volts
        self.battery_current = 0.0  # amps
        self.battery_capacity = 0.0  # amp-hours
        self.is_charging = False
        self.is_present = False

        self.power_history = deque(maxlen=int(self.power_window))  # (timestamp, power_w)
        self.estimated_runtime_min = 0.0
        self.last_alert_level = ""

        # Subscribers
        self.battery_sub = self.create_subscription(
            BatteryState,
            battery_topic,
            self._battery_callback,
            10
        )

        # Publishers
        self.alerts_pub = self.create_publisher(
            String,
            battery_alerts_topic,
            10
        )

        # Health monitoring
        self.health_pub = HealthStatusPublisher(
            self,
            health_topic,
            rate_hz=health_rate,
        )

        # Timers
        self.runtime_timer = self.create_timer(10.0, self._update_runtime_estimate)
        self.health_timer = self.create_timer(1.0 / health_rate, self._publish_health)
        self.alert_timer = self.create_timer(5.0, self._check_battery_alerts)

        self.get_logger().info(
            f"Battery Monitor initialized: low={self.low_battery_threshold}%, "
            f"critical={self.critical_battery_threshold}%"
        )

    def _battery_callback(self, msg: BatteryState):
        """Receive battery state from iRobot Create2"""
        self.battery_level = msg.percentage * 100.0  # Convert to percent
        self.battery_voltage = msg.voltage
        self.battery_current = msg.current
        self.battery_capacity = msg.capacity
        self.is_charging = (msg.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING)
        self.is_present = (msg.present)

        # Track power consumption
        power_w = abs(msg.voltage * msg.current)  # Watts
        timestamp = self.get_clock().now().nanoseconds * 1e-9
        self.power_history.append((timestamp, power_w))

    def _update_runtime_estimate(self):
        """Estimate remaining runtime based on current power consumption"""
        if not self.is_present or self.is_charging:
            self.estimated_runtime_min = 0.0
            return

        # Calculate average power consumption
        if len(self.power_history) > 10:
            # Use recent power consumption
            recent_power = [p for _, p in self.power_history]
            avg_power_w = sum(recent_power) / len(recent_power)
        else:
            # Use default
            avg_power_w = self.default_power_w

        if avg_power_w < 1.0:
            avg_power_w = self.default_power_w

        # Estimate runtime
        # Runtime (hours) = (Battery capacity * Battery level) / Power consumption
        if self.battery_capacity > 0 and avg_power_w > 0:
            remaining_capacity_ah = self.battery_capacity * (self.battery_level / 100.0)
            runtime_hours = (remaining_capacity_ah * self.battery_voltage) / avg_power_w
            self.estimated_runtime_min = runtime_hours * 60.0
        else:
            self.estimated_runtime_min = 0.0

        self.get_logger().debug(
            f"Runtime estimate: {self.estimated_runtime_min:.1f} min "
            f"(power={avg_power_w:.1f}W, battery={self.battery_level:.1f}%)"
        )

    def _check_battery_alerts(self):
        """Check battery level and publish alerts"""
        if not self.is_present:
            return

        alert_level = ""

        if self.battery_level < self.critical_battery_threshold:
            alert_level = "CRITICAL"
        elif self.battery_level < self.low_battery_threshold:
            alert_level = "LOW"

        # Only publish if alert level changed
        if alert_level and alert_level != self.last_alert_level:
            msg = String()
            msg.data = f"{alert_level}: Battery at {self.battery_level:.1f}% " \
                      f"(~{self.estimated_runtime_min:.0f} min remaining)"
            self.alerts_pub.publish(msg)

            self.get_logger().warn(msg.data)
            self.last_alert_level = alert_level

        # Clear alert if battery recovered
        if alert_level == "" and self.last_alert_level:
            self.last_alert_level = ""
            self.get_logger().info(f"Battery recovered: {self.battery_level:.1f}%")

    def _publish_health(self):
        """Publish health status"""
        if not self.is_present:
            status = "ERROR"
            message = "Battery not detected"
        elif self.battery_level < self.critical_battery_threshold:
            status = "ERROR"
            message = f"Critical battery: {self.battery_level:.1f}% ({self.estimated_runtime_min:.0f} min)"
        elif self.battery_level < self.low_battery_threshold:
            status = "WARN"
            message = f"Low battery: {self.battery_level:.1f}% ({self.estimated_runtime_min:.0f} min)"
        elif self.is_charging:
            status = "OK"
            message = f"Charging: {self.battery_level:.1f}% ({self.battery_voltage:.1f}V)"
        else:
            status = "OK"
            message = f"Battery: {self.battery_level:.1f}% (~{self.estimated_runtime_min:.0f} min)"

        self.health_pub.publish(status, message)


def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
