#!/usr/bin/env python3
"""
IMU Processor Node
Processes raw IMU data with Kalman filtering for dead reckoning on Raspberry Pi
Part of the modular chassis_control module
"""

# Standard library
import numpy as np

# Third-party
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header

# Local
from isaac_utils import HealthStatusPublisher, InputWatchdog, KalmanFilter


class IMUProcessorNode(Node):
    """ROS 2 node for IMU processing with Kalman filtering"""

    def __init__(self):
        super().__init__("imu_processor_node")

        # Parameters
        self.declare_parameter("raw_imu_topic", "/hardware/phat/imu")
        self.declare_parameter("filtered_imu_topic", "/rpi/imu/filtered")
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("kalman_filter_enabled", True)
        self.declare_parameter("imu_process_noise", 0.01)
        self.declare_parameter("imu_measurement_noise", 0.1)

        # Calibration parameters (IMU biases)
        self.declare_parameter("imu_bias_x", 0.0)
        self.declare_parameter("imu_bias_y", 0.0)
        self.declare_parameter("imu_bias_z", 0.0)
        self.declare_parameter("gyro_bias_x", 0.0)
        self.declare_parameter("gyro_bias_y", 0.0)
        self.declare_parameter("gyro_bias_z", 0.0)

        # Dead reckoning support
        self.declare_parameter("dead_reckoning_enabled", False)

        # Health monitoring
        self.declare_parameter("health_topic", f"health/{self.get_name()}")
        self.declare_parameter("health_publish_rate", 1.0)
        self.declare_parameter("health_warn_timeout_sec", 0.1)
        self.declare_parameter("health_stale_timeout_sec", 0.2)

        # Get parameters
        self.raw_imu_topic = str(self.get_parameter("raw_imu_topic").value)
        self.filtered_imu_topic = str(self.get_parameter("filtered_imu_topic").value)
        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.kalman_filter_enabled = bool(self.get_parameter("kalman_filter_enabled").value)
        self.imu_process_noise = float(self.get_parameter("imu_process_noise").value)
        self.imu_measurement_noise = float(self.get_parameter("imu_measurement_noise").value)

        # Calibration biases
        self.imu_bias = np.array([
            float(self.get_parameter("imu_bias_x").value),
            float(self.get_parameter("imu_bias_y").value),
            float(self.get_parameter("imu_bias_z").value)
        ])
        self.gyro_bias = np.array([
            float(self.get_parameter("gyro_bias_x").value),
            float(self.get_parameter("gyro_bias_y").value),
            float(self.get_parameter("gyro_bias_z").value)
        ])

        self.dead_reckoning_enabled = bool(self.get_parameter("dead_reckoning_enabled").value)

        health_topic = str(self.get_parameter("health_topic").value)
        health_rate = float(self.get_parameter("health_publish_rate").value)
        health_warn_timeout = float(self.get_parameter("health_warn_timeout_sec").value)
        health_stale_timeout = float(self.get_parameter("health_stale_timeout_sec").value)

        # Initialize Kalman filter
        if self.kalman_filter_enabled:
            self.kalman_filter = KalmanFilter(
                process_noise=self.imu_process_noise,
                measurement_noise=self.imu_measurement_noise
            )
            self.get_logger().info(
                f"Kalman filter enabled: process_noise={self.imu_process_noise}, "
                f"measurement_noise={self.imu_measurement_noise}"
            )
        else:
            self.kalman_filter = None
            self.get_logger().info("Kalman filter disabled")

        # Log calibration status
        if np.any(self.imu_bias != 0) or np.any(self.gyro_bias != 0):
            self.get_logger().info(
                f"IMU calibration loaded: accel_bias={self.imu_bias}, gyro_bias={self.gyro_bias}"
            )

        # Health monitoring
        self.health = HealthStatusPublisher(self, health_topic, health_rate)
        self.health.add_watchdog(
            InputWatchdog(
                "raw_imu",
                expected_rate_hz=self.publish_rate,
                warn_timeout_sec=health_warn_timeout,
                error_timeout_sec=health_stale_timeout,
            )
        )

        # Subscriber to raw IMU
        self.imu_sub = self.create_subscription(
            Imu,
            self.raw_imu_topic,
            self._imu_callback,
            10
        )

        # Publisher for filtered IMU
        self.filtered_imu_pub = self.create_publisher(
            Imu,
            self.filtered_imu_topic,
            10
        )

        # State tracking
        self.last_imu_data = None
        self._imu_callback_count = 0

        self.get_logger().info(
            f"IMU Processor initialized: {self.raw_imu_topic} → {self.filtered_imu_topic} "
            f"@ {self.publish_rate} Hz"
        )

    def _imu_callback(self, msg: Imu):
        """Process and publish filtered IMU data"""
        self.health.record_input("raw_imu")

        # Extract raw measurements
        accel_raw = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ])
        gyro_raw = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        ])

        # Apply calibration (subtract biases)
        accel_calibrated = accel_raw - self.imu_bias
        gyro_calibrated = gyro_raw - self.gyro_bias

        # Debug logging for first few callbacks
        self._imu_callback_count += 1
        if self._imu_callback_count <= 3:
            self.get_logger().info(
                f"[IMU] Received data: accel_raw={accel_raw}, gyro_raw={gyro_raw}"
            )
            if np.any(self.imu_bias != 0) or np.any(self.gyro_bias != 0):
                self.get_logger().info(
                    f"[IMU] After calibration: accel={accel_calibrated}, gyro={gyro_calibrated}"
                )

        # Apply Kalman filter if enabled
        if self.kalman_filter is not None:
            measurement = np.concatenate([accel_calibrated, gyro_calibrated])
            filtered_state = self.kalman_filter.update(measurement)

            # Create filtered IMU message
            filtered_imu = Imu()
            filtered_imu.header = Header()
            filtered_imu.header.stamp = self.get_clock().now().to_msg()
            filtered_imu.header.frame_id = msg.header.frame_id

            filtered_imu.linear_acceleration.x = filtered_state[0]
            filtered_imu.linear_acceleration.y = filtered_state[1]
            filtered_imu.linear_acceleration.z = filtered_state[2]
            filtered_imu.angular_velocity.x = filtered_state[3]
            filtered_imu.angular_velocity.y = filtered_state[4]
            filtered_imu.angular_velocity.z = filtered_state[5]

            # Copy covariance from original
            filtered_imu.linear_acceleration_covariance = msg.linear_acceleration_covariance
            filtered_imu.angular_velocity_covariance = msg.angular_velocity_covariance
        else:
            # No filtering, just apply calibration
            filtered_imu = Imu()
            filtered_imu.header = Header()
            filtered_imu.header.stamp = self.get_clock().now().to_msg()
            filtered_imu.header.frame_id = msg.header.frame_id

            filtered_imu.linear_acceleration.x = accel_calibrated[0]
            filtered_imu.linear_acceleration.y = accel_calibrated[1]
            filtered_imu.linear_acceleration.z = accel_calibrated[2]
            filtered_imu.angular_velocity.x = gyro_calibrated[0]
            filtered_imu.angular_velocity.y = gyro_calibrated[1]
            filtered_imu.angular_velocity.z = gyro_calibrated[2]

            filtered_imu.linear_acceleration_covariance = msg.linear_acceleration_covariance
            filtered_imu.angular_velocity_covariance = msg.angular_velocity_covariance

        # Debug publishing
        if self._imu_callback_count <= 5:
            self.get_logger().info(
                f"[IMU Publish] Publishing filtered IMU: "
                f"accel=[{filtered_imu.linear_acceleration.x:.3f}, "
                f"{filtered_imu.linear_acceleration.y:.3f}, "
                f"{filtered_imu.linear_acceleration.z:.3f}]"
            )

        # Publish filtered IMU
        self.filtered_imu_pub.publish(filtered_imu)

        # Store for dead reckoning (if needed by chassis controller)
        self.last_imu_data = filtered_imu


def main(args=None):
    rclpy.init(args=args)
    node = IMUProcessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
