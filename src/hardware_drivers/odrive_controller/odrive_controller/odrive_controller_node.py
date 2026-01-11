#!/usr/bin/env python3
"""
ODrive Motor Controller Node
Controls ODrive motor controllers via serial/CAN and publishes status
"""

# Standard library
import threading
import time
from typing import Optional

# Third-party
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
from std_msgs.msg import Header, String


class ODriveControllerNode(Node):
    """ROS 2 node for ODrive motor controller"""

    def __init__(self):
        super().__init__("odrive_controller_node")

        # Parameters
        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("timeout", 1.0)
        self.declare_parameter("use_can", False)
        self.declare_parameter("can_interface", "can0")
        self.declare_parameter("can_node_id", 0)
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("enable_accelerometer", True)
        self.declare_parameter("status_topic", "/odrive/status")
        self.declare_parameter("imu_topic", "/odrive/imu")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("imu_frame_id", "odrive_imu")
        self.declare_parameter("retry_delay", 2.0)  # Seconds between connection retry attempts

        self.serial_port = self.get_parameter("serial_port").value
        self.baudrate = self.get_parameter("baudrate").value
        self.timeout = self.get_parameter("timeout").value
        self.use_can = self.get_parameter("use_can").value
        self.can_interface = self.get_parameter("can_interface").value
        self.can_node_id = self.get_parameter("can_node_id").value
        self.publish_rate = self.get_parameter("publish_rate").value
        self.enable_accelerometer = self.get_parameter("enable_accelerometer").value
        self.status_topic = self.get_parameter("status_topic").value
        self.imu_topic = self.get_parameter("imu_topic").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.imu_frame_id = self.get_parameter("imu_frame_id").value
        self.retry_delay = self.get_parameter("retry_delay").value

        # Serial connection
        self.serial_conn: Optional[serial.Serial] = None
        self.connected = False

        # Publishers
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.imu_pub = (
            self.create_publisher(Imu, self.imu_topic, 10) if self.enable_accelerometer else None
        )

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, self.cmd_vel_topic, self.cmd_vel_callback, 10
        )

        # Timer for status updates
        self.status_timer = self.create_timer(
            1.0 / self.publish_rate, self._publish_status_callback
        )

        # Start connection thread
        self.connect_thread = threading.Thread(target=self._connect_odrive, daemon=True)
        self.connect_thread.start()

        self.get_logger().info("ODrive Controller node started")
        self._publish_status_message("initialized", "Node initialized")

    def _connect_odrive(self):
        """Connect to ODrive via serial or CAN"""
        while rclpy.ok() and not self.connected:
            try:
                if self.use_can:
                    self.get_logger().info(f"Connecting to ODrive via CAN: {self.can_interface}")
                    # CAN connection would be implemented here
                    # For now, we'll use serial
                    self.get_logger().warn("CAN mode not yet implemented, using serial")
                    self.use_can = False

                if not self.use_can:
                    self.get_logger().info(f"Connecting to ODrive via serial: {self.serial_port}")
                    self.serial_conn = serial.Serial(
                        port=self.serial_port, baudrate=self.baudrate, timeout=self.timeout
                    )
                    self.connected = True
                    self._publish_status_message("connected", f"Connected to {self.serial_port}")
                    self.get_logger().info("ODrive connected successfully")

            except serial.SerialException as e:
                self.get_logger().warn(f"Failed to connect to ODrive: {e}")
                self._publish_status_message("error", f"Connection failed: {str(e)[:50]}")
                time.sleep(self.retry_delay)
            except Exception as e:
                self.get_logger().error(f"Unexpected error connecting to ODrive: {e}")
                time.sleep(self.retry_delay)

    def cmd_vel_callback(self, msg: Twist):
        """Handle velocity commands"""
        if not self.connected:
            self.get_logger().warn("ODrive not connected, ignoring command")
            return

        try:
            # Convert Twist to ODrive commands
            # ODrive typically uses velocity setpoints
            linear_x = msg.linear.x
            angular_z = msg.angular.z

            # Send commands to ODrive
            # This is a simplified example - actual ODrive protocol would be more complex
            if self.serial_conn and self.serial_conn.is_open:
                # Example: set velocity command (actual protocol depends on ODrive firmware)
                # cmd = f"v 0 {linear_x}\n"
                # self.serial_conn.write(cmd.encode())
                self.get_logger().debug(f"Command: linear={linear_x}, angular={angular_z}")

        except Exception as e:
            self.get_logger().error(f"Error sending command to ODrive: {e}")

    def _publish_status_callback(self):
        """Timer callback to publish ODrive status"""
        if not self.connected:
            return

        try:
            # Read status from ODrive
            # This is a simplified example - actual implementation would parse ODrive status
            status_msg = String()
            if self.serial_conn and self.serial_conn.is_open:
                status_msg.data = "connected: ODrive operational"
            else:
                status_msg.data = "disconnected"
            self.status_pub.publish(status_msg)

            # Publish IMU data if accelerometer is enabled
            if self.enable_accelerometer and self.imu_pub:
                imu_msg = Imu()
                imu_msg.header = Header()
                imu_msg.header.stamp = self.get_clock().now().to_msg()
                imu_msg.header.frame_id = self.imu_frame_id

                # Read accelerometer data from ODrive
                # This is a placeholder - actual implementation would read from ODrive
                # For now, we'll publish zero values
                imu_msg.linear_acceleration.x = 0.0
                imu_msg.linear_acceleration.y = 0.0
                imu_msg.linear_acceleration.z = 0.0

                self.imu_pub.publish(imu_msg)

        except Exception as e:
            self.get_logger().error(f"Error publishing status: {e}")

    def _publish_status_message(self, status: str, message: str = ""):
        """Publish a status message"""
        status_msg = String()
        status_msg.data = f"{status}: {message}" if message else status
        self.status_pub.publish(status_msg)

    def destroy_node(self):
        """Cleanup on shutdown"""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ODriveControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
