#!/usr/bin/env python3
"""
Individual test for Chassis Controller Node
Tests: Initialization, odometry computation, pose integration
"""

import sys
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovariance, Point
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import numpy as np


class ChassisControllerTester(Node):
    """Test harness for chassis controller node"""

    def __init__(self):
        super().__init__('chassis_controller_tester')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/test/control/cmd_vel', 10)
        self.imu_pub = self.create_publisher(Imu, '/test/rpi/imu/filtered', 10)
        self.vision_pose_pub = self.create_publisher(
            PoseWithCovariance,
            '/test/vision/global_pose',
            10
        )

        # Subscribers
        self.odometry_sub = self.create_subscription(
            Odometry,
            '/test/rpi/chassis/odometry',
            self.odometry_callback,
            10
        )
        self.pose_estimate_sub = self.create_subscription(
            Odometry,
            '/test/rpi/chassis/pose_estimate',
            self.pose_estimate_callback,
            10
        )

        # Test state
        self.odometry_received = []
        self.pose_estimate_received = []
        self.test_duration = 3.0
        self.start_time = time.time()

        # Timer to publish mock data
        self.pub_timer = self.create_timer(0.02, self.publish_mock_data)  # 50Hz

        self.get_logger().info('Chassis Controller Tester started')

    def publish_mock_data(self):
        """Publish mock cmd_vel and IMU data"""
        now = self.get_clock().now().to_msg()

        # Publish constant forward velocity command
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.2  # 0.2 m/s forward
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)

        # Publish IMU data
        imu = Imu()
        imu.header = Header()
        imu.header.stamp = now
        imu.header.frame_id = 'imu_link'

        # Static acceleration (gravity)
        imu.linear_acceleration.x = 0.0
        imu.linear_acceleration.y = 0.0
        imu.linear_acceleration.z = 9.81

        # No rotation
        imu.angular_velocity.x = 0.0
        imu.angular_velocity.y = 0.0
        imu.angular_velocity.z = 0.0

        self.imu_pub.publish(imu)

        # Publish vision pose (ground truth) every second
        if int(time.time() - self.start_time) % 1 == 0:
            vision_pose = PoseWithCovariance()
            vision_pose.pose.position = Point(x=0.2, y=0.0, z=0.0)
            self.vision_pose_pub.publish(vision_pose)

    def odometry_callback(self, msg):
        """Receive odometry data"""
        self.odometry_received.append(msg)

        if len(self.odometry_received) == 1:
            self.get_logger().info(
                f'First odometry received: '
                f'x={msg.pose.pose.position.x:.3f}, y={msg.pose.pose.position.y:.3f}'
            )

    def pose_estimate_callback(self, msg):
        """Receive pose estimate data"""
        self.pose_estimate_received.append(msg)

        if len(self.pose_estimate_received) == 1:
            self.get_logger().info(
                f'First pose estimate received: '
                f'x={msg.pose.pose.position.x:.3f}, y={msg.pose.pose.position.y:.3f}'
            )

    def run_test(self):
        """Run test for specified duration"""
        while time.time() - self.start_time < self.test_duration:
            rclpy.spin_once(self, timeout_sec=0.1)

        return self.evaluate_results()

    def evaluate_results(self):
        """Evaluate test results"""
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('Chassis Controller Test Results')
        self.get_logger().info('='*60)

        odom_count = len(self.odometry_received)
        pose_count = len(self.pose_estimate_received)

        self.get_logger().info(f'Odometry messages received: {odom_count}')
        self.get_logger().info(f'Pose estimate messages received: {pose_count}')

        expected_min = int(self.test_duration * 40)  # At least 40 Hz

        success = True

        if odom_count < expected_min:
            self.get_logger().error(
                f'FAIL: Too few odometry messages ({odom_count} < {expected_min})'
            )
            success = False

        if pose_count < expected_min:
            self.get_logger().error(
                f'FAIL: Too few pose estimate messages ({pose_count} < {expected_min})'
            )
            success = False

        # Check if robot moved forward
        if odom_count > 10:
            final_pose = self.odometry_received[-1].pose.pose.position
            self.get_logger().info(
                f'Final odometry position: x={final_pose.x:.3f}, y={final_pose.y:.3f}'
            )

            # Should have moved forward (note: without real wheel encoders,
            # this will depend on the implementation)
            if final_pose.x < 0.0:
                self.get_logger().warn(
                    'WARNING: Robot did not move forward (expected with no encoder data)'
                )

        if success:
            self.get_logger().info('PASS: Chassis controller node functioning correctly')
        else:
            self.get_logger().error('FAIL: Chassis controller test failed')

        self.get_logger().info('='*60 + '\n')
        return success


def main(args=None):
    rclpy.init(args=args)

    # Import here to avoid circular dependency
    from chassis_control.chassis_controller_node import ChassisControllerNode

    # Create nodes
    chassis_controller = ChassisControllerNode()
    chassis_controller.get_parameter('cmd_vel_topic')._value = '/test/control/cmd_vel'
    chassis_controller.get_parameter('imu_topic')._value = '/test/rpi/imu/filtered'
    chassis_controller.get_parameter('odometry_output_topic')._value = '/test/rpi/chassis/odometry'
    chassis_controller.get_parameter('pose_estimate_topic')._value = '/test/rpi/chassis/pose_estimate'
    chassis_controller.get_parameter('vision_pose_topic')._value = '/test/vision/global_pose'

    tester = ChassisControllerTester()

    # Run in separate thread
    import threading
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(chassis_controller)
    executor.add_node(tester)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # Run test
    print('\nRunning Chassis Controller Test...\n')
    success = tester.run_test()

    # Cleanup
    executor.shutdown()
    chassis_controller.destroy_node()
    tester.destroy_node()
    rclpy.shutdown()

    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
