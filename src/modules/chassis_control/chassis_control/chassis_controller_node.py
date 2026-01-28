#!/usr/bin/env python3
"""
Chassis Controller Node
Integrates wheel odometry with IMU for dead reckoning and pose estimation
Handles vision feedback for drift correction and keyframe alignment
"""

# Standard library
import math
import time
from typing import Optional

# Third-party
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseWithCovariance, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Empty, Float32, Header, String

# Local
from isaac_utils import HealthStatusPublisher, InputWatchdog


class ChassisControllerNode(Node):
    """ROS 2 node for chassis control with dead reckoning and pose integration"""

    def __init__(self):
        super().__init__("chassis_controller_node")

        # Parameters - Hardware
        self.declare_parameter("wheel_diameter_m", 0.072)  # iRobot Create2: 72mm wheels
        self.declare_parameter("wheelbase_m", 0.235)  # iRobot Create2: 235mm wheelbase
        self.declare_parameter("encoder_counts_per_rev", 508.8)  # Create2 encoder resolution

        # Parameters - Control
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("cmd_vel_topic", "/control/cmd_vel")
        self.declare_parameter("low_power_mode", False)

        # Parameters - Pose Integration
        self.declare_parameter("enable_vision_feedback", True)
        self.declare_parameter("keyframe_drift_threshold_m", 0.1)  # Trigger alignment if drift > 0.1m
        self.declare_parameter("keyframe_drift_threshold_rad", 0.087)  # ~5 degrees
        self.declare_parameter("pose_correction_weight", 0.5)  # Blend vision with dead reckoning

        # Parameters - Topics
        self.declare_parameter("imu_topic", "/rpi/imu/filtered")
        self.declare_parameter("battery_topic", "/irobot/battery")
        self.declare_parameter("status_topic", "/irobot/status")
        self.declare_parameter("vision_global_pose_topic", "/vision/global_pose")
        self.declare_parameter("odometry_topic", "/rpi/chassis/odometry")
        self.declare_parameter("pose_estimate_topic", "/rpi/chassis/pose_estimate")
        self.declare_parameter("battery_level_topic", "/rpi/power/battery_level")

        # Parameters - Health Monitoring
        self.declare_parameter("health_topic", f"health/{self.get_name()}")
        self.declare_parameter("health_publish_rate", 1.0)
        self.declare_parameter("health_warn_timeout_sec", 1.0)
        self.declare_parameter("health_stale_timeout_sec", 2.0)

        # Get parameters
        self.wheel_diameter = float(self.get_parameter("wheel_diameter_m").value)
        self.wheelbase = float(self.get_parameter("wheelbase_m").value)
        self.encoder_counts_per_rev = float(self.get_parameter("encoder_counts_per_rev").value)
        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.low_power_mode = bool(self.get_parameter("low_power_mode").value)

        self.enable_vision_feedback = bool(self.get_parameter("enable_vision_feedback").value)
        self.keyframe_drift_threshold_m = float(self.get_parameter("keyframe_drift_threshold_m").value)
        self.keyframe_drift_threshold_rad = float(self.get_parameter("keyframe_drift_threshold_rad").value)
        self.pose_correction_weight = float(self.get_parameter("pose_correction_weight").value)

        # Topic names
        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        imu_topic = str(self.get_parameter("imu_topic").value)
        battery_topic = str(self.get_parameter("battery_topic").value)
        status_topic = str(self.get_parameter("status_topic").value)
        vision_pose_topic = str(self.get_parameter("vision_global_pose_topic").value)
        odometry_topic = str(self.get_parameter("odometry_topic").value)
        pose_estimate_topic = str(self.get_parameter("pose_estimate_topic").value)
        battery_level_topic = str(self.get_parameter("battery_level_topic").value)

        health_topic = str(self.get_parameter("health_topic").value)
        health_rate = float(self.get_parameter("health_publish_rate").value)
        health_warn_timeout = float(self.get_parameter("health_warn_timeout_sec").value)
        health_stale_timeout = float(self.get_parameter("health_stale_timeout_sec").value)

        # Computed constants
        self.wheel_circumference = math.pi * self.wheel_diameter
        self.meters_per_count = self.wheel_circumference / self.encoder_counts_per_rev

        # State - Odometry
        self.x = 0.0  # meters
        self.y = 0.0  # meters
        self.theta = 0.0  # radians
        self.vx = 0.0  # m/s
        self.vtheta = 0.0  # rad/s

        # State - Encoders (will be populated when available)
        self.last_encoder_left = None
        self.last_encoder_right = None
        self.last_encoder_time = None

        # State - Vision feedback
        self.last_vision_pose: Optional[PoseWithCovariance] = None
        self.last_vision_time = None
        self.last_keyframe_time = time.time()
        self.drift_estimate_m = 0.0

        # State - IMU
        self.last_imu: Optional[Imu] = None
        self.last_imu_time = None

        # State - Battery
        self.battery_percentage = 100.0

        # Health monitoring
        self.health = HealthStatusPublisher(self, health_topic, health_rate)
        self.health.add_watchdog(
            InputWatchdog(
                "imu",
                expected_rate_hz=50.0,
                warn_timeout_sec=health_warn_timeout,
                error_timeout_sec=health_stale_timeout,
            )
        )
        self.health.add_watchdog(
            InputWatchdog(
                "cmd_vel",
                expected_rate_hz=10.0,
                warn_timeout_sec=health_warn_timeout,
                error_timeout_sec=health_stale_timeout,
            )
        )
        if self.enable_vision_feedback:
            self.health.add_watchdog(
                InputWatchdog(
                    "vision_pose",
                    expected_rate_hz=5.0,
                    warn_timeout_sec=2.0,
                    error_timeout_sec=5.0,
                )
            )

        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu, imu_topic, self._imu_callback, 10
        )
        self.battery_sub = self.create_subscription(
            String, battery_topic, self._battery_callback, 10
        )
        self.status_sub = self.create_subscription(
            String, status_topic, self._status_callback, 10
        )
        self.cmd_vel_sub = self.create_subscription(
            Twist, cmd_vel_topic, self._cmd_vel_callback, 10
        )

        if self.enable_vision_feedback:
            self.vision_pose_sub = self.create_subscription(
                PoseWithCovariance, vision_pose_topic, self._vision_pose_callback, 10
            )

        # Publishers
        self.odometry_pub = self.create_publisher(Odometry, odometry_topic, 10)
        self.pose_estimate_pub = self.create_publisher(PoseWithCovariance, pose_estimate_topic, 10)
        self.battery_level_pub = self.create_publisher(Float32, battery_level_topic, 10)

        # Timer for odometry updates
        self.timer = self.create_timer(1.0 / self.publish_rate, self._update_odometry)

        self.get_logger().info(
            f"Chassis Controller initialized: wheel_diameter={self.wheel_diameter}m, "
            f"wheelbase={self.wheelbase}m, rate={self.publish_rate}Hz, "
            f"vision_feedback={self.enable_vision_feedback}"
        )

    def _imu_callback(self, msg: Imu):
        """Store latest IMU data"""
        self.health.record_input("imu")
        self.last_imu = msg
        self.last_imu_time = time.time()

    def _battery_callback(self, msg: String):
        """Parse battery data and publish battery level"""
        # iRobot serial publishes battery as string, parse it
        # Expected format: "battery: XX%"
        try:
            if "battery" in msg.data.lower():
                parts = msg.data.split(":")
                if len(parts) > 1:
                    pct_str = parts[1].strip().replace("%", "")
                    self.battery_percentage = float(pct_str)

                    # Publish battery level
                    battery_msg = Float32()
                    battery_msg.data = self.battery_percentage
                    self.battery_level_pub.publish(battery_msg)
        except Exception as e:
            self.get_logger().warn(f"Failed to parse battery: {e}")

    def _status_callback(self, msg: String):
        """Store latest status data"""
        # Placeholder for status processing
        pass

    def _cmd_vel_callback(self, msg: Twist):
        """Receive velocity commands from planner"""
        self.health.record_input("cmd_vel")

        # Update velocity estimate from command
        # (In full implementation, this would forward to iRobot serial node)
        self.vx = msg.linear.x
        self.vtheta = msg.angular.z

    def _vision_pose_callback(self, msg: PoseWithCovariance):
        """Receive SLAM global pose for drift correction"""
        self.health.record_input("vision_pose")
        self.last_vision_pose = msg
        self.last_vision_time = time.time()

        # Check if keyframe alignment is needed
        if self._should_align_to_keyframe():
            self._align_to_keyframe(msg)

    def _update_odometry(self):
        """Update dead reckoning odometry and publish pose estimates"""
        current_time = time.time()
        dt = 1.0 / self.publish_rate

        # Integration method: Simple Euler integration (dead reckoning)
        # In production, this would integrate wheel encoder counts + IMU
        # For now, using velocity model as placeholder

        # Update position using current velocity
        delta_x = self.vx * math.cos(self.theta) * dt
        delta_y = self.vx * math.sin(self.theta) * dt
        delta_theta = self.vtheta * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Create dead reckoning odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        # Position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Orientation (quaternion from yaw)
        odom_msg.pose.pose.orientation = self._yaw_to_quaternion(self.theta)

        # Velocity
        odom_msg.twist.twist.linear.x = self.vx
        odom_msg.twist.twist.angular.z = self.vtheta

        # Covariance (placeholder - should increase over time for dead reckoning)
        odom_msg.pose.covariance = [0.1] * 36  # Identity placeholder
        odom_msg.twist.covariance = [0.01] * 36

        # Publish odometry
        self.odometry_pub.publish(odom_msg)

        # Create pose estimate (potentially corrected by vision)
        pose_estimate = PoseWithCovariance()
        pose_estimate.pose = odom_msg.pose.pose
        pose_estimate.covariance = odom_msg.pose.covariance

        # Publish pose estimate
        self.pose_estimate_pub.publish(pose_estimate)

    def _should_align_to_keyframe(self) -> bool:
        """Check if keyframe alignment is needed based on drift"""
        if not self.enable_vision_feedback or self.last_vision_pose is None:
            return False

        # Calculate drift between vision and dead reckoning
        vision_x = self.last_vision_pose.pose.position.x
        vision_y = self.last_vision_pose.pose.position.y

        # Euclidean distance
        drift_m = math.sqrt((vision_x - self.x)**2 + (vision_y - self.y)**2)

        # Angular drift (simplified - compare yaw)
        vision_yaw = self._quaternion_to_yaw(self.last_vision_pose.pose.orientation)
        drift_rad = abs(vision_yaw - self.theta)

        # Normalize to [-pi, pi]
        while drift_rad > math.pi:
            drift_rad -= 2 * math.pi
        drift_rad = abs(drift_rad)

        self.drift_estimate_m = drift_m

        # Trigger alignment if drift exceeds thresholds
        return (drift_m > self.keyframe_drift_threshold_m or
                drift_rad > self.keyframe_drift_threshold_rad)

    def _align_to_keyframe(self, vision_pose: PoseWithCovariance):
        """Align dead reckoning to vision keyframe with weighted correction"""
        vision_x = vision_pose.pose.position.x
        vision_y = vision_pose.pose.position.y
        vision_yaw = self._quaternion_to_yaw(vision_pose.pose.orientation)

        # Calculate drift
        drift_x = vision_x - self.x
        drift_y = vision_y - self.y
        drift_yaw = vision_yaw - self.theta

        # Apply weighted correction
        correction_x = drift_x * self.pose_correction_weight
        correction_y = drift_y * self.pose_correction_weight
        correction_yaw = drift_yaw * self.pose_correction_weight

        self.x += correction_x
        self.y += correction_y
        self.theta += correction_yaw

        self.last_keyframe_time = time.time()

        self.get_logger().info(
            f"Keyframe alignment: drift=({drift_x:.3f}m, {drift_y:.3f}m, {drift_yaw:.3f}rad), "
            f"correction=({correction_x:.3f}m, {correction_y:.3f}m, {correction_yaw:.3f}rad)"
        )

    def _yaw_to_quaternion(self, yaw: float) -> Quaternion:
        """Convert yaw angle to quaternion"""
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    def _quaternion_to_yaw(self, q: Quaternion) -> float:
        """Extract yaw from quaternion"""
        # yaw = atan2(2(w*z + x*y), 1 - 2(y^2 + z^2))
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = ChassisControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
