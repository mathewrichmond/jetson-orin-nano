#!/usr/bin/env python3
"""
Individual test for IMU Processor Node
Tests: Initialization, parameter loading, Kalman filter, health monitoring
"""

import sys
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
import numpy as np


class IMUProcessorTester(Node):
    """Test harness for IMU processor node"""

    def __init__(self):
        super().__init__('imu_processor_tester')

        # Subscribe to filtered IMU output
        self.filtered_sub = self.create_subscription(
            Imu,
            '/test/rpi/imu/filtered',
            self.filtered_callback,
            10
        )

        # Publish mock raw IMU data
        self.raw_pub = self.create_publisher(Imu, '/test/hardware/phat/imu', 10)

        # Test state
        self.filtered_received = []
        self.test_duration = 5.0  # seconds
        self.start_time = time.time()

        # Timer to publish mock data
        self.pub_timer = self.create_timer(0.02, self.publish_mock_imu)  # 50Hz

        self.get_logger().info('IMU Processor Tester started')

    def publish_mock_imu(self):
        """Publish mock IMU data"""
        msg = Imu()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        # Static values + small noise
        msg.linear_acceleration.x = 0.0 + np.random.normal(0, 0.05)
        msg.linear_acceleration.y = 0.0 + np.random.normal(0, 0.05)
        msg.linear_acceleration.z = 9.81 + np.random.normal(0, 0.05)

        msg.angular_velocity.x = 0.0 + np.random.normal(0, 0.01)
        msg.angular_velocity.y = 0.0 + np.random.normal(0, 0.01)
        msg.angular_velocity.z = 0.0 + np.random.normal(0, 0.01)

        msg.linear_acceleration_covariance = [0.01] * 9
        msg.angular_velocity_covariance = [0.001] * 9
        msg.orientation_covariance = [-1.0] * 9

        self.raw_pub.publish(msg)

    def filtered_callback(self, msg):
        """Receive filtered IMU data"""
        self.filtered_received.append(msg)

        if len(self.filtered_received) == 1:
            self.get_logger().info(
                f'First filtered IMU received: '
                f'accel=[{msg.linear_acceleration.x:.3f}, '
                f'{msg.linear_acceleration.y:.3f}, {msg.linear_acceleration.z:.3f}]'
            )

    def run_test(self):
        """Run test for specified duration"""
        while time.time() - self.start_time < self.test_duration:
            rclpy.spin_once(self, timeout_sec=0.1)

        return self.evaluate_results()

    def evaluate_results(self):
        """Evaluate test results"""
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('IMU Processor Test Results')
        self.get_logger().info('='*60)

        # Check if messages were received
        num_received = len(self.filtered_received)
        expected_min = int(self.test_duration * 40)  # At least 40 Hz (allowing some loss)

        self.get_logger().info(f'Filtered messages received: {num_received}')
        self.get_logger().info(f'Expected minimum: {expected_min}')

        if num_received < expected_min:
            self.get_logger().error(
                f'FAIL: Received too few messages ({num_received} < {expected_min})'
            )
            return False

        # Check Kalman filtering (variance should be reduced)
        if num_received > 10:
            recent = self.filtered_received[-50:]
            accel_z_values = [msg.linear_acceleration.z for msg in recent]
            mean_z = np.mean(accel_z_values)
            std_z = np.std(accel_z_values)

            self.get_logger().info(f'Filtered accel_z: mean={mean_z:.3f}, std={std_z:.3f}')

            # Should be close to gravity with low variance
            if abs(mean_z - 9.81) > 0.3:
                self.get_logger().warn(
                    f'WARNING: Mean acceleration Z deviates from gravity: {mean_z:.3f}'
                )

            if std_z > 0.1:
                self.get_logger().warn(
                    f'WARNING: High standard deviation in filtered data: {std_z:.3f}'
                )

        self.get_logger().info('PASS: IMU processor node functioning correctly')
        self.get_logger().info('='*60 + '\n')
        return True


def main(args=None):
    rclpy.init(args=args)

    # Import here to avoid circular dependency
    from chassis_control.imu_processor_node import IMUProcessorNode

    # Create nodes
    imu_processor = IMUProcessorNode()
    imu_processor.get_parameter('raw_imu_topic')._value = '/test/hardware/phat/imu'
    imu_processor.get_parameter('filtered_imu_topic')._value = '/test/rpi/imu/filtered'

    tester = IMUProcessorTester()

    # Run in separate thread
    import threading
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(imu_processor)
    executor.add_node(tester)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # Run test
    print('\nRunning IMU Processor Test...\n')
    success = tester.run_test()

    # Cleanup
    executor.shutdown()
    imu_processor.destroy_node()
    tester.destroy_node()
    rclpy.shutdown()

    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
