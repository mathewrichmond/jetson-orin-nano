#!/usr/bin/env python3
"""
Calibration Manager Node
Handles auto-calibration of IMU biases, wheel odometry, camera-IMU extrinsics, and servo offsets
Uses vision feedback to continuously improve calibration parameters
"""

# Standard library
import math
import os
import time
from typing import Dict, Optional

# Third-party
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovariance, Transform
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import yaml

# Local
from isaac_utils import HealthStatusPublisher
from custom_msgs.msg import CalibrationStatus


class CalibrationManagerNode(Node):
    """ROS 2 node for auto-calibration management"""

    def __init__(self):
        super().__init__("calibration_manager_node")

        # Parameters
        self.declare_parameter("calibration_file", "/home/nano/.config/robot/calibration.yaml")
        self.declare_parameter("auto_save_interval_sec", 300.0)  # Save every 5 minutes
        self.declare_parameter("use_last_known_calibration", True)
        self.declare_parameter("min_samples_for_update", 10)
        self.declare_parameter("calibration_learning_rate", 0.01)

        # Topics
        self.declare_parameter("vision_pose_topic", "/vision/global_pose")
        self.declare_parameter("odometry_topic", "/rpi/chassis/odometry")
        self.declare_parameter("calibration_status_topic", "/rpi/calibration/status")

        # Health monitoring
        self.declare_parameter("health_topic", f"health/{self.get_name()}")
        self.declare_parameter("health_publish_rate", 1.0)

        # Get parameters
        self.calibration_file = str(self.get_parameter("calibration_file").value)
        self.auto_save_interval = float(self.get_parameter("auto_save_interval_sec").value)
        self.use_last_known = bool(self.get_parameter("use_last_known_calibration").value)
        self.min_samples = int(self.get_parameter("min_samples_for_update").value)
        self.learning_rate = float(self.get_parameter("calibration_learning_rate").value)

        vision_pose_topic = str(self.get_parameter("vision_pose_topic").value)
        odometry_topic = str(self.get_parameter("odometry_topic").value)
        calibration_status_topic = str(self.get_parameter("calibration_status_topic").value)

        health_topic = str(self.get_parameter("health_topic").value)
        health_rate = float(self.get_parameter("health_publish_rate").value)

        # Calibration state
        self.calibration = {
            "imu_bias_x": 0.0,
            "imu_bias_y": 0.0,
            "imu_bias_z": 0.0,
            "gyro_bias_x": 0.0,
            "gyro_bias_y": 0.0,
            "gyro_bias_z": 0.0,
            "wheel_diameter_left_m": 0.072,
            "wheel_diameter_right_m": 0.072,
            "wheelbase_m": 0.235,
            "servo_pan_offset_deg": 0.0,
            "servo_tilt_offset_deg": 0.0,
            "calibration_quality": "uncalibrated",  # uncalibrated, poor, good, excellent
            "sample_count": 0,
            "last_updated": time.time(),
        }

        # Drift estimation buffers
        self.drift_samples = []
        self.max_drift_samples = 100

        # Latest data
        self.last_vision_pose: Optional[PoseWithCovariance] = None
        self.last_odometry: Optional[Odometry] = None
        self.last_save_time = time.time()

        # Load existing calibration
        if self.use_last_known:
            self._load_calibration()

        # Health monitoring
        self.health = HealthStatusPublisher(self, health_topic, health_rate)

        # Subscribers
        self.vision_pose_sub = self.create_subscription(
            PoseWithCovariance, vision_pose_topic, self._vision_pose_callback, 10
        )
        self.odometry_sub = self.create_subscription(
            Odometry, odometry_topic, self._odometry_callback, 10
        )

        # Publishers
        self.calibration_status_pub = self.create_publisher(
            CalibrationStatus, calibration_status_topic, 10
        )

        # Timer for periodic calibration updates and saves
        self.update_timer = self.create_timer(1.0, self._update_calibration)
        self.save_timer = self.create_timer(self.auto_save_interval, self._save_calibration)

        self.get_logger().info(
            f"Calibration Manager initialized: file={self.calibration_file}, "
            f"auto_save_interval={self.auto_save_interval}s"
        )

    def _vision_pose_callback(self, msg: PoseWithCovariance):
        """Store latest vision pose"""
        self.last_vision_pose = msg

    def _odometry_callback(self, msg: Odometry):
        """Store latest odometry and compute drift"""
        self.last_odometry = msg

        # If we have both vision and odometry, compute drift
        if self.last_vision_pose is not None and self.last_odometry is not None:
            self._compute_drift()

    def _compute_drift(self):
        """Compute drift between vision and odometry"""
        vision_x = self.last_vision_pose.pose.position.x
        vision_y = self.last_vision_pose.pose.position.y
        odom_x = self.last_odometry.pose.pose.position.x
        odom_y = self.last_odometry.pose.pose.position.y

        # Compute Euclidean drift
        drift_x = vision_x - odom_x
        drift_y = vision_y - odom_y
        drift_magnitude = math.sqrt(drift_x**2 + drift_y**2)

        # Store drift sample
        drift_sample = {
            "drift_x": drift_x,
            "drift_y": drift_y,
            "drift_magnitude": drift_magnitude,
            "timestamp": time.time(),
        }

        self.drift_samples.append(drift_sample)

        # Keep only recent samples
        if len(self.drift_samples) > self.max_drift_samples:
            self.drift_samples.pop(0)

    def _update_calibration(self):
        """Update calibration parameters based on drift estimation"""
        if len(self.drift_samples) < self.min_samples:
            # Not enough samples for reliable calibration
            return

        # Calculate average drift
        avg_drift_x = np.mean([s["drift_x"] for s in self.drift_samples])
        avg_drift_y = np.mean([s["drift_y"] for s in self.drift_samples])
        avg_drift_mag = np.mean([s["drift_magnitude"] for s in self.drift_samples])

        # Update wheel diameter scale based on drift magnitude
        # Simplified model: drift indicates odometry scale error
        if avg_drift_mag > 0.01:  # Only update if drift is significant (>1cm)
            # Scale correction proportional to drift
            scale_correction = avg_drift_mag * self.learning_rate

            # Apply to wheel diameter (affects odometry distance calculation)
            self.calibration["wheel_diameter_left_m"] *= (1.0 + scale_correction)
            self.calibration["wheel_diameter_right_m"] *= (1.0 + scale_correction)

            self.calibration["sample_count"] += len(self.drift_samples)
            self.calibration["last_updated"] = time.time()

            self.get_logger().info(
                f"Calibration updated: avg_drift={avg_drift_mag:.4f}m, "
                f"wheel_diameter={self.calibration['wheel_diameter_left_m']:.6f}m, "
                f"samples={self.calibration['sample_count']}"
            )

        # Update calibration quality based on sample count and drift
        self._update_quality(avg_drift_mag)

        # Publish calibration status
        self._publish_status()

        # Clear drift samples after update
        self.drift_samples.clear()

    def _update_quality(self, avg_drift: float):
        """Update calibration quality estimate"""
        sample_count = self.calibration["sample_count"]

        if sample_count < 100:
            quality = "poor"
        elif sample_count < 500:
            quality = "good" if avg_drift < 0.05 else "poor"
        else:
            if avg_drift < 0.02:
                quality = "excellent"
            elif avg_drift < 0.05:
                quality = "good"
            else:
                quality = "poor"

        self.calibration["calibration_quality"] = quality

    def _publish_status(self):
        """Publish calibration status using CalibrationStatus message"""
        msg = CalibrationStatus()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        # IMU biases
        msg.imu_bias_x = float(self.calibration["imu_bias_x"])
        msg.imu_bias_y = float(self.calibration["imu_bias_y"])
        msg.imu_bias_z = float(self.calibration["imu_bias_z"])
        msg.gyro_bias_x = float(self.calibration["gyro_bias_x"])
        msg.gyro_bias_y = float(self.calibration["gyro_bias_y"])
        msg.gyro_bias_z = float(self.calibration["gyro_bias_z"])

        # Wheel odometry
        msg.wheel_diameter_left_m = float(self.calibration["wheel_diameter_left_m"])
        msg.wheel_diameter_right_m = float(self.calibration["wheel_diameter_right_m"])
        msg.wheelbase_m = float(self.calibration["wheelbase_m"])

        # Camera extrinsics (placeholder transforms for now)
        msg.camera_front_to_base = Transform()
        msg.camera_rear_to_base = Transform()

        # Servo calibration
        msg.servo_pan_offset_deg = float(self.calibration["servo_pan_offset_deg"])
        msg.servo_tilt_offset_deg = float(self.calibration["servo_tilt_offset_deg"])

        # Quality metrics
        msg.calibration_quality = str(self.calibration["calibration_quality"])
        msg.sample_count = int(self.calibration["sample_count"])
        msg.last_updated = float(self.calibration["last_updated"])

        self.calibration_status_pub.publish(msg)

    def _load_calibration(self):
        """Load calibration from YAML file"""
        if not os.path.exists(self.calibration_file):
            self.get_logger().info(f"No existing calibration file found at {self.calibration_file}")
            return

        try:
            with open(self.calibration_file, "r") as f:
                loaded = yaml.safe_load(f)
                if loaded:
                    self.calibration.update(loaded)
                    self.get_logger().info(
                        f"Loaded calibration from {self.calibration_file}: "
                        f"quality={self.calibration['calibration_quality']}, "
                        f"samples={self.calibration['sample_count']}"
                    )
        except Exception as e:
            self.get_logger().error(f"Failed to load calibration: {e}")

    def _save_calibration(self):
        """Save calibration to YAML file"""
        try:
            # Ensure directory exists
            os.makedirs(os.path.dirname(self.calibration_file), exist_ok=True)

            # Write calibration to file
            with open(self.calibration_file, "w") as f:
                yaml.dump(self.calibration, f, default_flow_style=False)

            self.last_save_time = time.time()
            self.get_logger().info(
                f"Saved calibration to {self.calibration_file}: "
                f"quality={self.calibration['calibration_quality']}"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to save calibration: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Save calibration on shutdown
        node._save_calibration()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
