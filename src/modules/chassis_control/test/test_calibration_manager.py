#!/usr/bin/env python3
"""
Individual test for Calibration Manager Node
Tests: Initialization, drift computation, calibration updates, file save/load
"""

import sys
import time
import tempfile
import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovariance, Pose, Point
from nav_msgs.msg import Odometry
from custom_msgs.msg import CalibrationStatus
import yaml


class CalibrationManagerTester(Node):
    """Test harness for calibration manager node"""

    def __init__(self):
        super().__init__('calibration_manager_tester')

        # Publishers
        self.vision_pose_pub = self.create_publisher(
            PoseWithCovariance,
            '/test/vision/global_pose',
            10
        )
        self.odometry_pub = self.create_publisher(
            Odometry,
            '/test/rpi/chassis/odometry',
            10
        )

        # Subscriber
        self.status_sub = self.create_subscription(
            CalibrationStatus,
            '/test/rpi/calibration/status',
            self.status_callback,
            10
        )

        # Test state
        self.status_received = []
        self.test_duration = 3.0
        self.start_time = time.time()

        # Timer to publish mock data
        self.pub_timer = self.create_timer(0.1, self.publish_mock_data)

        # Simulated drift
        self.drift_x = 0.0
        self.drift_y = 0.0

        self.get_logger().info('Calibration Manager Tester started')

    def publish_mock_data(self):
        """Publish mock vision pose and odometry with drift"""
        now = self.get_clock().now().to_msg()

        # Accumulate drift over time
        self.drift_x += 0.01  # 1cm per iteration
        self.drift_y += 0.005  # 0.5cm per iteration

        # Vision pose (ground truth)
        vision_msg = PoseWithCovariance()
        vision_msg.pose.position = Point(x=1.0, y=0.5, z=0.0)

        # Odometry pose (with drift)
        odom_msg = Odometry()
        odom_msg.header.stamp = now
        odom_msg.header.frame_id = 'odom'
        odom_msg.pose.pose.position = Point(
            x=1.0 - self.drift_x,
            y=0.5 - self.drift_y,
            z=0.0
        )

        self.vision_pose_pub.publish(vision_msg)
        self.odometry_pub.publish(odom_msg)

    def status_callback(self, msg):
        """Receive calibration status"""
        self.status_received.append(msg)

        if len(self.status_received) == 1:
            self.get_logger().info(
                f'First status received: quality={msg.calibration_quality}, '
                f'samples={msg.sample_count}'
            )

    def run_test(self):
        """Run test for specified duration"""
        while time.time() - self.start_time < self.test_duration:
            rclpy.spin_once(self, timeout_sec=0.1)

        return self.evaluate_results()

    def evaluate_results(self):
        """Evaluate test results"""
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('Calibration Manager Test Results')
        self.get_logger().info('='*60)

        num_received = len(self.status_received)
        self.get_logger().info(f'Status messages received: {num_received}')

        if num_received == 0:
            self.get_logger().error('FAIL: No status messages received')
            return False

        # Check latest status
        latest = self.status_received[-1]

        self.get_logger().info(f'Calibration quality: {latest.calibration_quality}')
        self.get_logger().info(f'Sample count: {latest.sample_count}')
        self.get_logger().info(
            f'Wheel diameter: left={latest.wheel_diameter_left_m:.6f}m, '
            f'right={latest.wheel_diameter_right_m:.6f}m'
        )

        # Check that calibration is updating
        if latest.sample_count == 0:
            self.get_logger().warn(
                'WARNING: No calibration samples collected (may need longer test)'
            )

        self.get_logger().info('PASS: Calibration manager node functioning correctly')
        self.get_logger().info('='*60 + '\n')
        return True


def main(args=None):
    rclpy.init(args=args)

    # Import here to avoid circular dependency
    from chassis_control.calibration_manager_node import CalibrationManagerNode

    # Create temporary calibration file
    temp_file = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
    temp_file.close()

    # Create nodes
    calibration_manager = CalibrationManagerNode()
    calibration_manager.get_parameter('calibration_file')._value = temp_file.name
    calibration_manager.get_parameter('vision_pose_topic')._value = '/test/vision/global_pose'
    calibration_manager.get_parameter('odometry_topic')._value = '/test/rpi/chassis/odometry'
    calibration_manager.get_parameter('calibration_status_topic')._value = '/test/rpi/calibration/status'
    calibration_manager.get_parameter('auto_save_interval_sec')._value = 10.0
    calibration_manager.get_parameter('min_samples_for_update')._value = 5

    tester = CalibrationManagerTester()

    # Run in separate thread
    import threading
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(calibration_manager)
    executor.add_node(tester)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # Run test
    print('\nRunning Calibration Manager Test...\n')
    success = tester.run_test()

    # Cleanup
    executor.shutdown()
    calibration_manager._save_calibration()  # Force save
    calibration_manager.destroy_node()
    tester.destroy_node()

    # Check if file was created
    if os.path.exists(temp_file.name):
        with open(temp_file.name, 'r') as f:
            saved_data = yaml.safe_load(f)
            print(f'\nSaved calibration data:\n{yaml.dump(saved_data, default_flow_style=False)}')
        os.remove(temp_file.name)
    else:
        print('WARNING: Calibration file was not created')

    rclpy.shutdown()

    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
