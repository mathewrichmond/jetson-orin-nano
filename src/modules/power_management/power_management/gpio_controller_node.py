#!/usr/bin/env python3
"""
GPIO Controller Node
Interfaces with Jetson GPIO to control GPU power rails
Executes power state changes requested by power manager
"""

# Standard library
import time
from typing import Optional

# Third-party
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Local
from isaac_utils import HealthStatusPublisher


class GPIOControllerNode(Node):
    """
    ROS 2 node for GPIO-based power control

    Controls Jetson Orin Nano GPU power via GPIO pins.
    Implements safe power sequencing for GPU on/off/sleep transitions.
    """

    def __init__(self):
        super().__init__("gpio_controller_node")

        # Parameters
        self.declare_parameter("gpu_power_pin", 18)  # GPIO pin for GPU power
        self.declare_parameter("gpu_enable_pin", 23)  # GPIO pin for GPU enable
        self.declare_parameter("use_mock_gpio", True)  # Use mock GPIO for testing
        self.declare_parameter("power_on_delay_sec", 0.5)  # Delay after power on
        self.declare_parameter("power_off_delay_sec", 1.0)  # Delay before power off

        # Topics
        self.declare_parameter("gpu_power_control_topic", "/jetson/power/gpu_control")
        self.declare_parameter("gpu_power_status_topic", "/jetson/power/gpu_status")

        # Health monitoring
        self.declare_parameter("health_topic", f"health/{self.get_name()}")
        self.declare_parameter("health_publish_rate", 1.0)

        # Get parameters
        self.gpu_power_pin = int(self.get_parameter("gpu_power_pin").value)
        self.gpu_enable_pin = int(self.get_parameter("gpu_enable_pin").value)
        self.use_mock_gpio = bool(self.get_parameter("use_mock_gpio").value)
        self.power_on_delay = float(self.get_parameter("power_on_delay_sec").value)
        self.power_off_delay = float(self.get_parameter("power_off_delay_sec").value)

        gpu_control_topic = str(self.get_parameter("gpu_power_control_topic").value)
        gpu_status_topic = str(self.get_parameter("gpu_power_status_topic").value)

        health_topic = str(self.get_parameter("health_topic").value)
        health_rate = float(self.get_parameter("health_publish_rate").value)

        # State
        self.current_state = "unknown"  # unknown, off, on, sleep
        self.gpio_initialized = False

        # Initialize GPIO
        self._init_gpio()

        # Subscribers
        self.control_sub = self.create_subscription(
            String,
            gpu_control_topic,
            self._control_callback,
            10
        )

        # Publishers
        self.status_pub = self.create_publisher(
            String,
            gpu_status_topic,
            10
        )

        # Health monitoring
        self.health_pub = HealthStatusPublisher(
            self,
            health_topic,
            rate_hz=health_rate,
        )

        # Timers
        self.status_timer = self.create_timer(1.0, self._publish_status)
        self.health_timer = self.create_timer(1.0 / health_rate, self._publish_health)

        self.get_logger().info(
            f"GPIO Controller initialized: mock={self.use_mock_gpio}, "
            f"pins=[power:{self.gpu_power_pin}, enable:{self.gpu_enable_pin}]"
        )

    def _init_gpio(self):
        """Initialize GPIO pins"""
        if self.use_mock_gpio:
            self.get_logger().info("Using mock GPIO (no hardware control)")
            self.gpio_initialized = True
            self.current_state = "off"  # Assume off initially
            return

        try:
            # Try to import Jetson.GPIO
            import Jetson.GPIO as GPIO

            GPIO.setmode(GPIO.BOARD)
            GPIO.setup(self.gpu_power_pin, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(self.gpu_enable_pin, GPIO.OUT, initial=GPIO.LOW)

            self.gpio = GPIO
            self.gpio_initialized = True
            self.current_state = "off"

            self.get_logger().info(
                f"GPIO initialized: pins {self.gpu_power_pin}, {self.gpu_enable_pin}"
            )
        except ImportError:
            self.get_logger().error(
                "Jetson.GPIO not available, falling back to mock mode"
            )
            self.use_mock_gpio = True
            self.gpio_initialized = True
            self.current_state = "off"
        except Exception as e:
            self.get_logger().error(f"GPIO initialization failed: {e}")
            self.gpio_initialized = False

    def _control_callback(self, msg: String):
        """Receive power control command"""
        requested_state = msg.data.lower()

        if requested_state not in ["on", "off", "sleep"]:
            self.get_logger().warn(f"Invalid GPU power state: {requested_state}")
            return

        if requested_state == self.current_state:
            self.get_logger().debug(f"GPU already in {requested_state} state")
            return

        self.get_logger().info(f"GPU power transition: {self.current_state} → {requested_state}")

        # Execute transition
        success = self._set_gpu_power(requested_state)

        if success:
            self.current_state = requested_state
            self.get_logger().info(f"GPU power state changed to {requested_state}")
        else:
            self.get_logger().error(f"Failed to change GPU power to {requested_state}")

    def _set_gpu_power(self, state: str) -> bool:
        """Set GPU power state via GPIO"""
        if not self.gpio_initialized:
            self.get_logger().error("GPIO not initialized")
            return False

        try:
            if self.use_mock_gpio:
                # Mock implementation
                self.get_logger().debug(f"Mock GPIO: Setting GPU to {state}")
                time.sleep(0.1)  # Simulate delay
                return True

            # Real GPIO control
            if state == "on":
                self._power_on_sequence()
            elif state == "off":
                self._power_off_sequence()
            elif state == "sleep":
                self._power_sleep_sequence()

            return True

        except Exception as e:
            self.get_logger().error(f"GPIO control error: {e}")
            return False

    def _power_on_sequence(self):
        """Power on sequence for GPU"""
        if self.use_mock_gpio:
            return

        # Enable power rail
        self.gpio.output(self.gpu_power_pin, self.gpio.HIGH)
        time.sleep(self.power_on_delay)

        # Enable GPU
        self.gpio.output(self.gpu_enable_pin, self.gpio.HIGH)

        self.get_logger().debug("GPU power on sequence completed")

    def _power_off_sequence(self):
        """Power off sequence for GPU"""
        if self.use_mock_gpio:
            return

        # Disable GPU
        self.gpio.output(self.gpu_enable_pin, self.gpio.LOW)
        time.sleep(self.power_off_delay)

        # Disable power rail
        self.gpio.output(self.gpu_power_pin, self.gpio.LOW)

        self.get_logger().debug("GPU power off sequence completed")

    def _power_sleep_sequence(self):
        """Power sleep sequence for GPU (low power state)"""
        if self.use_mock_gpio:
            return

        # Keep power rail active, disable GPU enable
        self.gpio.output(self.gpu_enable_pin, self.gpio.LOW)

        self.get_logger().debug("GPU power sleep sequence completed")

    def _publish_status(self):
        """Publish current GPU power status"""
        msg = String()
        msg.data = self.current_state
        self.status_pub.publish(msg)

    def _publish_health(self):
        """Publish health status"""
        if self.gpio_initialized:
            status = "OK"
            message = f"GPIO control active: GPU={self.current_state}"
        else:
            status = "ERROR"
            message = "GPIO not initialized"

        self.health_pub.publish(status, message)

    def __del__(self):
        """Cleanup GPIO on node destruction"""
        if hasattr(self, 'gpio') and not self.use_mock_gpio:
            try:
                self.gpio.cleanup()
                self.get_logger().info("GPIO cleanup completed")
            except Exception as e:
                self.get_logger().error(f"GPIO cleanup error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = GPIOControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
