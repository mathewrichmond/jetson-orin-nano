#!/usr/bin/env python3
"""
iRobot Developer Kit Serial Node
Communicates with iRobot Create/Roomba via serial connection
"""

import serial
import threading
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Header, String


class iRobotSerialNode(Node):
    """ROS 2 node for iRobot Create/Roomba serial communication"""

    def __init__(self):
        super().__init__('irobot_serial_node')

        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 1.0)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('robot_type', 'create2')  # create2, roomba, etc.

        self.serial_port = self.get_parameter('serial_port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.timeout = self.get_parameter('timeout').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.robot_type = self.get_parameter('robot_type').value

        # Serial connection
        self.serial_conn: Optional[serial.Serial] = None
        self.connected = False

        # Publishers
        self.status_pub = self.create_publisher(String, '/irobot/status', 10)
        self.battery_pub = self.create_publisher(BatteryState, '/irobot/battery', 10)

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Timer for status updates
        self.status_timer = self.create_timer(1.0 / self.publish_rate, self.publish_status)

        # Start connection thread
        self.connect_thread = threading.Thread(target=self._connect_irobot, daemon=True)
        self.connect_thread.start()

        self.get_logger().info('iRobot Serial node started')
        self._publish_status_message('initialized', 'Node initialized')

    def _connect_irobot(self):
        """Connect to iRobot via serial"""
        while rclpy.ok() and not self.connected:
            try:
                self.get_logger().info(f'Connecting to iRobot {self.robot_type} via serial: {self.serial_port}')
                self.serial_conn = serial.Serial(
                    port=self.serial_port,
                    baudrate=self.baudrate,
                    timeout=self.timeout
                )
                self.connected = True
                self._publish_status_message('connected', f'Connected to {self.serial_port}')

                # Send start command (iRobot Create protocol)
                if self.robot_type == 'create2':
                    self._send_command([128])  # Start command
                    time.sleep(0.1)
                    self._send_command([131])  # Safe mode
                    self.get_logger().info('iRobot Create 2 initialized in safe mode')

            except serial.SerialException as e:
                self.get_logger().warn(f'Failed to connect to iRobot: {e}')
                self._publish_status_message('error', f'Connection failed: {str(e)[:50]}')
                time.sleep(2.0)
            except Exception as e:
                self.get_logger().error(f'Unexpected error connecting to iRobot: {e}')
                time.sleep(2.0)

    def _send_command(self, command_bytes: list):
        """Send command to iRobot"""
        if self.serial_conn and self.serial_conn.is_open:
            try:
                self.serial_conn.write(bytes(command_bytes))
            except Exception as e:
                self.get_logger().error(f'Error sending command: {e}')

    def cmd_vel_callback(self, msg: Twist):
        """Handle velocity commands"""
        if not self.connected:
            self.get_logger().warn('iRobot not connected, ignoring command')
            return

        try:
            # Convert Twist to iRobot Create drive command
            # iRobot Create uses velocity (mm/s) and radius (mm)
            linear_x = msg.linear.x  # m/s
            angular_z = msg.angular.z  # rad/s

            # Convert to iRobot units (mm/s and mm)
            velocity_mm_s = int(linear_x * 1000)  # Convert m/s to mm/s
            radius_mm = int((linear_x / angular_z) * 1000) if angular_z != 0 else 32767  # Convert to mm

            # Clamp values to valid range
            velocity_mm_s = max(-500, min(500, velocity_mm_s))  # -500 to 500 mm/s
            radius_mm = max(-2000, min(2000, radius_mm)) if radius_mm != 32767 else 32767  # -2000 to 2000 mm or straight

            # Create 2 drive command: [137, velocity_high, velocity_low, radius_high, radius_low]
            velocity_high = (velocity_mm_s >> 8) & 0xFF
            velocity_low = velocity_mm_s & 0xFF
            radius_high = (radius_mm >> 8) & 0xFF
            radius_low = radius_mm & 0xFF

            self._send_command([137, velocity_high, velocity_low, radius_high, radius_low])
            self.get_logger().debug(f'Command: velocity={velocity_mm_s} mm/s, radius={radius_mm} mm')

        except Exception as e:
            self.get_logger().error(f'Error sending command to iRobot: {e}')

    def _read_sensor_data(self):
        """Read sensor data from iRobot"""
        if not self.connected or not self.serial_conn or not self.serial_conn.is_open:
            return None

        try:
            # Request sensor data (Create 2 sensor packet)
            # Packet ID 1 = Bumps and wheel drops
            # Packet ID 3 = Battery charge
            self._send_command([142, 3])  # Request sensor packet 3 (battery charge)

            # Read response (2 bytes for battery charge, 0-65535)
            time.sleep(0.05)  # Wait for response
            if self.serial_conn.in_waiting >= 2:
                data = self.serial_conn.read(2)
                if len(data) == 2:
                    charge = (data[0] << 8) | data[1]
                    return {'battery_charge': charge}
        except Exception as e:
            self.get_logger().debug(f'Error reading sensor data: {e}')

        return None

    def _publish_status_callback(self):
        """Timer callback to publish iRobot status"""
        if not self.connected:
            return

        try:
            # Read sensor data
            sensor_data = self._read_sensor_data()

            # Publish status
            status_msg = String()
            if self.serial_conn and self.serial_conn.is_open:
                status_text = f'connected: iRobot {self.robot_type} operational'
                if sensor_data:
                    status_text += f', battery={sensor_data.get("battery_charge", "unknown")}'
                status_msg.data = status_text
            else:
                status_msg.data = 'disconnected'
            self.status_pub.publish(status_msg)

            # Publish battery state
            if sensor_data and 'battery_charge' in sensor_data:
                battery_msg = BatteryState()
                battery_msg.header = Header()
                battery_msg.header.stamp = self.get_clock().now().to_msg()
                battery_msg.header.frame_id = 'irobot_base'

                # iRobot Create 2 battery is 0-65535, max charge is typically ~16000
                charge = sensor_data['battery_charge']
                max_charge = 16000  # Typical max charge value
                battery_msg.percentage = min(1.0, charge / max_charge)
                battery_msg.voltage = 14.4 * battery_msg.percentage  # Approximate voltage
                battery_msg.present = True

                self.battery_pub.publish(battery_msg)

        except Exception as e:
            self.get_logger().error(f'Error publishing status: {e}')

    def _publish_status_message(self, status: str, message: str = ''):
        """Publish a status message"""
        status_msg = String()
        status_msg.data = f'{status}: {message}' if message else status
        self.status_pub.publish(status_msg)

    def destroy_node(self):
        """Cleanup on shutdown"""
        if self.connected and self.serial_conn and self.serial_conn.is_open:
            # Stop iRobot
            self._send_command([173])  # Stop command
            time.sleep(0.1)
            self.serial_conn.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = iRobotSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
