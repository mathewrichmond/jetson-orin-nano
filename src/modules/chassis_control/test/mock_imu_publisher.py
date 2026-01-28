#!/usr/bin/env python3
"""
Mock IMU data publisher for testing IMU processor node
Publishes synthetic IMU data at 50Hz
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
import numpy as np
import math


class MockIMUPublisher(Node):
    """Publishes mock IMU data for testing"""

    def __init__(self):
        super().__init__('mock_imu_publisher')

        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('output_topic', '/test/hardware/phat/imu')

        rate = self.get_parameter('publish_rate').value
        topic = self.get_parameter('output_topic').value

        self.publisher = self.create_publisher(Imu, topic, 10)
        self.timer = self.create_timer(1.0 / rate, self.publish_imu)

        self.counter = 0
        self.get_logger().info(f'Mock IMU publisher started: {topic} @ {rate} Hz')

    def publish_imu(self):
        """Publish synthetic IMU data"""
        msg = Imu()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        # Generate synthetic data with some variation
        t = self.counter * 0.02  # 50Hz = 0.02s period

        # Linear acceleration (with gravity on Z and small noise)
        msg.linear_acceleration.x = 0.0 + np.random.normal(0, 0.01)
        msg.linear_acceleration.y = 0.0 + np.random.normal(0, 0.01)
        msg.linear_acceleration.z = 9.81 + np.random.normal(0, 0.01)

        # Angular velocity (small rotation)
        msg.angular_velocity.x = 0.0 + np.random.normal(0, 0.001)
        msg.angular_velocity.y = 0.0 + np.random.normal(0, 0.001)
        msg.angular_velocity.z = 0.1 * math.sin(t) + np.random.normal(0, 0.001)

        # Covariance (dummy values)
        msg.linear_acceleration_covariance = [0.01] * 9
        msg.angular_velocity_covariance = [0.001] * 9
        msg.orientation_covariance = [-1.0] * 9  # Orientation not available

        self.publisher.publish(msg)

        # Log first few messages
        if self.counter < 3:
            self.get_logger().info(
                f'Published IMU: accel=[{msg.linear_acceleration.x:.3f}, '
                f'{msg.linear_acceleration.y:.3f}, {msg.linear_acceleration.z:.3f}], '
                f'gyro=[{msg.angular_velocity.x:.4f}, {msg.angular_velocity.y:.4f}, '
                f'{msg.angular_velocity.z:.4f}]'
            )

        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = MockIMUPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
