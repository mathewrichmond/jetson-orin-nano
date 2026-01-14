#!/usr/bin/env python3
"""
Hardware Sync Signal Generator Node
Generates periodic sync pulses on GPIO pin synchronized to system clock.
Can be used to trigger RealSense cameras for hardware frame synchronization.
"""

# Standard library
import threading
import time
from typing import Optional

# Third-party
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Try to import GPIO library
GPIO_AVAILABLE = False
GPIO = None
try:
    import Jetson.GPIO as GPIO

    GPIO_AVAILABLE = True
except (ImportError, Exception):
    try:
        import RPi.GPIO as GPIO

        GPIO_AVAILABLE = True
    except (ImportError, Exception):
        GPIO_AVAILABLE = False
        GPIO = None


class HardwareSyncGeneratorNode(Node):
    """Generates hardware sync pulses on GPIO pin"""

    def __init__(self):
        super().__init__("hardware_sync_generator_node")

        # Parameters
        self.declare_parameter("sync_gpio_pin", 21)  # GPIO pin for sync signal (default: 21, unused)
        self.declare_parameter("sync_frequency", 15.0)  # Sync frequency (Hz) - match camera FPS
        self.declare_parameter("pulse_width_ms", 1.0)  # Pulse width in milliseconds
        self.declare_parameter("gpio_mode", "BOARD")  # GPIO numbering mode: "BOARD" or "BCM"
        self.declare_parameter("enable_sync", True)  # Enable/disable sync signal generation
        self.declare_parameter("status_topic", "/hardware_sync/status")

        # Get parameters
        self.sync_pin = int(self.get_parameter("sync_gpio_pin").value or 21)
        self.sync_frequency = float(self.get_parameter("sync_frequency").value or 15.0)
        self.pulse_width_ms = float(self.get_parameter("pulse_width_ms").value or 1.0)
        self.gpio_mode_str = str(self.get_parameter("gpio_mode").value or "BOARD")
        self.enable_sync = bool(self.get_parameter("enable_sync").value or True)

        # Calculate timing
        self.period = 1.0 / self.sync_frequency  # Period in seconds
        self.pulse_width = self.pulse_width_ms / 1000.0  # Pulse width in seconds

        # GPIO state
        self.gpio_initialized = False
        self.sync_thread: Optional[threading.Thread] = None
        self.running = False

        # Status publisher
        status_topic = str(self.get_parameter("status_topic").value or "/hardware_sync/status")
        self.status_pub = self.create_publisher(String, status_topic, 10)

        # Initialize GPIO
        if self.enable_sync:
            self._initialize_gpio()

        self.get_logger().info("Hardware sync generator node started")
        self.get_logger().info(f"Sync frequency: {self.sync_frequency} Hz")
        self.get_logger().info(f"GPIO pin: {self.sync_pin}")
        self.get_logger().info(f"Pulse width: {self.pulse_width_ms} ms")
        self.get_logger().info(f"Enabled: {self.enable_sync}")

    def _initialize_gpio(self):
        """Initialize GPIO pin for sync signal output"""
        if not GPIO_AVAILABLE or GPIO is None:
            self.get_logger().warn("GPIO library not available. Sync signal disabled.")
            self.get_logger().info("Install Jetson.GPIO or RPi.GPIO for hardware sync")
            self.gpio_initialized = False
            return

        try:
            # Set GPIO mode
            gpio_mode = GPIO.BOARD if self.gpio_mode_str == "BOARD" else GPIO.BCM
            GPIO.setmode(gpio_mode)

            # Setup sync pin as output
            GPIO.setup(self.sync_pin, GPIO.OUT)
            GPIO.output(self.sync_pin, GPIO.LOW)  # Start low

            self.gpio_initialized = True
            self.get_logger().info(f"GPIO pin {self.sync_pin} initialized for sync signal")

            # Start sync thread
            self.running = True
            self.sync_thread = threading.Thread(target=self._sync_loop, daemon=True)
            self.sync_thread.start()

            self._publish_status("initialized", f"Sync signal generator active on GPIO {self.sync_pin}")

        except Exception as e:
            self.get_logger().error(f"Failed to initialize GPIO: {e}")
            self.gpio_initialized = False
            self._publish_status("error", f"GPIO initialization failed: {str(e)[:50]}")

    def _sync_loop(self):
        """Generate periodic sync pulses synchronized to system clock"""
        if not self.gpio_initialized:
            return

        self.get_logger().info("Starting sync pulse generation...")

        # Calculate next sync time aligned to system clock
        # Align to whole seconds for better synchronization
        current_time = time.time()
        next_sync = int(current_time) + 1.0  # Start at next second boundary

        pulse_count = 0

        while self.running and rclpy.ok():
            try:
                # Wait until next sync time
                current_time = time.time()
                wait_time = next_sync - current_time

                if wait_time > 0:
                    time.sleep(wait_time)

                # Generate pulse
                if self.gpio_initialized and GPIO is not None:
                    GPIO.output(self.sync_pin, GPIO.HIGH)  # Pulse high
                    time.sleep(self.pulse_width)
                    GPIO.output(self.sync_pin, GPIO.LOW)  # Pulse low

                pulse_count += 1

                # Calculate next sync time
                next_sync += self.period

                # Log periodically
                if pulse_count % int(self.sync_frequency * 10) == 0:  # Every 10 seconds
                    self.get_logger().debug(
                        f"Generated {pulse_count} sync pulses at {self.sync_frequency} Hz"
                    )

            except Exception as e:
                self.get_logger().error(f"Error in sync loop: {e}")
                time.sleep(0.1)

        # Cleanup
        if self.gpio_initialized and GPIO is not None:
            try:
                GPIO.output(self.sync_pin, GPIO.LOW)
            except Exception:
                pass

        self.get_logger().info("Sync pulse generation stopped")

    def _publish_status(self, status: str, message: str = ""):
        """Publish status message"""
        status_msg = String()
        status_msg.data = f"{status}: {message}" if message else status
        self.status_pub.publish(status_msg)

    def destroy_node(self):
        """Cleanup on shutdown"""
        self.running = False
        if self.sync_thread and self.sync_thread.is_alive():
            self.sync_thread.join(timeout=1.0)

        if self.gpio_initialized and GPIO is not None:
            try:
                GPIO.output(self.sync_pin, GPIO.LOW)
                GPIO.cleanup(self.sync_pin)
            except Exception:
                pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HardwareSyncGeneratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
