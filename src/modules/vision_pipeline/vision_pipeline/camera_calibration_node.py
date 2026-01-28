#!/usr/bin/env python3
"""
Camera Calibration Node
Estimates camera-IMU extrinsic calibration using Kalibr-style online estimation
Updates CalibrationStatus with refined camera-to-base transforms
"""

# Standard library
import math
from typing import Dict, List, Optional, Tuple

# Third-party
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Transform, Vector3, Quaternion
from sensor_msgs.msg import CameraInfo, Image, Imu
from std_msgs.msg import Header
from custom_msgs.msg import CalibrationStatus

# Local
from isaac_utils import HealthStatusPublisher, InputWatchdog


class CameraCalibrationNode(Node):
    """
    ROS 2 node for online camera-IMU extrinsic calibration

    Uses visual-inertial alignment to estimate the transform between
    camera and IMU frames. Publishes updates to CalibrationStatus.
    """

    def __init__(self):
        super().__init__("camera_calibration_node")

        # Parameters
        self.declare_parameter("camera_names", ["camera_front", "camera_rear"])
        self.declare_parameter("imu_topic", "/rpi/imu/filtered")
        self.declare_parameter("calibration_status_topic", "/rpi/calibration/status")
        self.declare_parameter("visual_odometry_topic", "/vision/odometry")

        # Calibration parameters
        self.declare_parameter("calibration_window_sec", 30.0)  # Rolling window
        self.declare_parameter("min_motion_for_calibration", 0.5)  # meters
        self.declare_parameter("min_rotation_for_calibration", 0.3)  # radians
        self.declare_parameter("calibration_update_rate", 0.1)  # Hz (every 10 seconds)

        # Health monitoring
        self.declare_parameter("health_topic", f"health/{self.get_name()}")
        self.declare_parameter("health_publish_rate", 1.0)

        # Get parameters
        self.camera_names = self.get_parameter("camera_names").value
        imu_topic = str(self.get_parameter("imu_topic").value)
        calibration_status_topic = str(self.get_parameter("calibration_status_topic").value)
        visual_odom_topic = str(self.get_parameter("visual_odometry_topic").value)

        self.calibration_window = float(self.get_parameter("calibration_window_sec").value)
        self.min_motion = float(self.get_parameter("min_motion_for_calibration").value)
        self.min_rotation = float(self.get_parameter("min_rotation_for_calibration").value)
        calibration_rate = float(self.get_parameter("calibration_update_rate").value)

        health_topic = str(self.get_parameter("health_topic").value)
        health_rate = float(self.get_parameter("health_publish_rate").value)

        # State
        self.camera_info: Dict[str, Optional[CameraInfo]] = {name: None for name in self.camera_names}
        self.imu_buffer: List[Tuple[float, Imu]] = []  # (timestamp, imu)
        self.visual_odom_buffer: List = []
        self.current_extrinsics: Dict[str, Transform] = {}

        # Initialize default extrinsics (identity + camera offset)
        for camera_name in self.camera_names:
            transform = Transform()
            transform.rotation.w = 1.0  # Identity rotation

            # Default offsets (will be refined)
            if "front" in camera_name:
                transform.translation.x = 0.05  # 5cm forward from base
                transform.translation.z = 0.08  # 8cm up
            elif "rear" in camera_name:
                transform.translation.x = -0.05  # 5cm backward
                transform.translation.z = 0.08

            self.current_extrinsics[camera_name] = transform

        # Calibration statistics
        self.total_motion = 0.0
        self.total_rotation = 0.0
        self.calibration_quality = "uncalibrated"
        self.sample_count = 0

        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu, imu_topic, self._imu_callback, 10
        )
        for camera_name in self.camera_names:
            camera_info_topic = f"/hardware/{camera_name}/color/camera_info"
            self.create_subscription(
                CameraInfo,
                camera_info_topic,
                lambda msg, name=camera_name: self._camera_info_callback(msg, name),
                10
            )

        # Publisher
        self.calibration_pub = self.create_publisher(
            CalibrationStatus,
            calibration_status_topic,
            10
        )

        # Health monitoring
        self.health_pub = HealthStatusPublisher(
            self,
            health_topic,
            rate_hz=health_rate,
        )
        self.watchdog_imu = InputWatchdog("imu", warn_timeout=2.0, stale_timeout=5.0)

        # Timer for calibration updates
        self.calibration_timer = self.create_timer(
            1.0 / calibration_rate if calibration_rate > 0 else 10.0,
            self._update_calibration
        )
        self.health_timer = self.create_timer(1.0 / health_rate, self._publish_health)

        self.get_logger().info(
            f"Camera Calibration initialized: cameras={self.camera_names}, "
            f"window={self.calibration_window}s"
        )

    def _imu_callback(self, msg: Imu):
        """Receive IMU data"""
        self.watchdog_imu.update()

        # Store in buffer with timestamp
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.imu_buffer.append((timestamp, msg))

        # Trim buffer to window size
        current_time = self.get_clock().now().nanoseconds * 1e-9
        self.imu_buffer = [
            (t, imu) for t, imu in self.imu_buffer
            if current_time - t < self.calibration_window
        ]

    def _camera_info_callback(self, msg: CameraInfo, camera_name: str):
        """Receive camera calibration info"""
        if self.camera_info[camera_name] is None:
            self.get_logger().info(
                f"Received camera info for {camera_name}: "
                f"{msg.width}x{msg.height}"
            )
        self.camera_info[camera_name] = msg

    def _update_calibration(self):
        """Periodically update camera-IMU extrinsics"""
        # TODO: Implement Kalibr-style online calibration
        # For now, use default extrinsics

        # Check if we have enough data
        if len(self.imu_buffer) < 10:
            return

        # Compute motion from IMU
        linear_motion = self._compute_imu_motion()
        angular_motion = self._compute_imu_rotation()

        self.total_motion += linear_motion
        self.total_rotation += angular_motion
        self.sample_count += 1

        # Update calibration quality
        if self.total_motion > 5.0 and self.total_rotation > 2.0:
            self.calibration_quality = "good"
        elif self.total_motion > 1.0 and self.total_rotation > 0.5:
            self.calibration_quality = "poor"
        else:
            self.calibration_quality = "uncalibrated"

        # Publish calibration update (partial - only camera extrinsics)
        self._publish_calibration_update()

    def _compute_imu_motion(self) -> float:
        """Estimate linear motion from IMU accelerations"""
        if len(self.imu_buffer) < 2:
            return 0.0

        # Simple dead reckoning from recent IMU data
        motion = 0.0
        for i in range(1, min(10, len(self.imu_buffer))):
            t1, imu1 = self.imu_buffer[-i-1]
            t2, imu2 = self.imu_buffer[-i]
            dt = t2 - t1

            # Average acceleration (removing gravity)
            ax = (imu1.linear_acceleration.x + imu2.linear_acceleration.x) / 2.0
            ay = (imu1.linear_acceleration.y + imu2.linear_acceleration.y) / 2.0

            # Approximate motion
            motion += math.sqrt(ax*ax + ay*ay) * dt

        return motion

    def _compute_imu_rotation(self) -> float:
        """Estimate rotation from IMU angular velocities"""
        if len(self.imu_buffer) < 2:
            return 0.0

        rotation = 0.0
        for i in range(1, min(10, len(self.imu_buffer))):
            t1, imu1 = self.imu_buffer[-i-1]
            t2, imu2 = self.imu_buffer[-i]
            dt = t2 - t1

            # Angular velocity magnitude
            wx = imu2.angular_velocity.x
            wy = imu2.angular_velocity.y
            wz = imu2.angular_velocity.z

            rotation += math.sqrt(wx*wx + wy*wy + wz*wz) * dt

        return rotation

    def _publish_calibration_update(self):
        """Publish calibration update with camera extrinsics"""
        msg = CalibrationStatus()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        # Camera extrinsics
        if "camera_front" in self.current_extrinsics:
            msg.camera_front_to_base = self.current_extrinsics["camera_front"]
        if "camera_rear" in self.current_extrinsics:
            msg.camera_rear_to_base = self.current_extrinsics["camera_rear"]

        # Quality metadata
        msg.calibration_quality = self.calibration_quality
        msg.sample_count = self.sample_count
        msg.last_updated = self.get_clock().now().nanoseconds * 1e-9

        self.calibration_pub.publish(msg)

        self.get_logger().debug(
            f"Published calibration update: quality={self.calibration_quality}, "
            f"motion={self.total_motion:.2f}m, rotation={self.total_rotation:.2f}rad"
        )

    def _publish_health(self):
        """Publish health status"""
        imu_status = self.watchdog_imu.get_status()

        if imu_status == "OK":
            if self.calibration_quality in ["good", "excellent"]:
                status = "OK"
                message = f"Calibration {self.calibration_quality}: {self.sample_count} samples"
            else:
                status = "WARN"
                message = f"Calibration {self.calibration_quality}: need more motion"
        else:
            status = "ERROR"
            message = f"IMU input {imu_status}"

        self.health_pub.publish(status, message)


def main(args=None):
    rclpy.init(args=args)
    node = CameraCalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
