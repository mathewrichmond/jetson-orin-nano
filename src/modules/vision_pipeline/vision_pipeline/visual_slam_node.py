#!/usr/bin/env python3
"""
Visual SLAM Node
Wraps RTAB-Map for visual odometry and pose estimation
Provides global pose feedback for chassis drift correction
"""

# Standard library
import math
from typing import Optional

# Third-party
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovariance, TransformStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster
import numpy as np

# Local
from isaac_utils import HealthStatusPublisher, InputWatchdog


class VisualSLAMNode(Node):
    """
    ROS 2 node for Visual SLAM using RTAB-Map

    This node manages RTAB-Map for visual odometry and loop closure detection.
    It publishes global pose estimates for chassis drift correction.
    """

    def __init__(self):
        super().__init__("visual_slam_node")

        # Parameters
        self.declare_parameter("camera_name", "camera_front")
        self.declare_parameter("use_depth", True)  # Visual-inertial or RGB-D SLAM
        self.declare_parameter("publish_global_pose", True)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("global_frame_id", "map")
        self.declare_parameter("odom_frame_id", "odom_visual")
        self.declare_parameter("base_frame_id", "base_link")

        # Topic names
        self.declare_parameter("rgb_image_topic", "/hardware/camera_front/color/image_raw")
        self.declare_parameter("depth_image_topic", "/hardware/camera_front/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic", "/hardware/camera_front/color/camera_info")
        self.declare_parameter("imu_topic", "/rpi/imu/filtered")
        self.declare_parameter("global_pose_topic", "/vision/global_pose")
        self.declare_parameter("visual_odometry_topic", "/vision/odometry")
        self.declare_parameter("trajectory_topic", "/vision/trajectory")

        # SLAM parameters
        self.declare_parameter("rtabmap_min_inliers", 20)
        self.declare_parameter("rtabmap_max_depth", 4.0)  # meters
        self.declare_parameter("rtabmap_detector_type", "GFTT")  # GFTT, ORB, SIFT, SURF
        self.declare_parameter("rtabmap_loop_closure_threshold", 0.15)
        self.declare_parameter("rtabmap_memory_management", True)  # Delete old features

        # Health monitoring
        self.declare_parameter("health_topic", f"health/{self.get_name()}")
        self.declare_parameter("health_publish_rate", 1.0)
        self.declare_parameter("health_warn_timeout_sec", 2.0)
        self.declare_parameter("health_stale_timeout_sec", 5.0)

        # Get parameters
        self.camera_name = str(self.get_parameter("camera_name").value)
        self.use_depth = bool(self.get_parameter("use_depth").value)
        self.publish_global_pose = bool(self.get_parameter("publish_global_pose").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.global_frame_id = str(self.get_parameter("global_frame_id").value)
        self.odom_frame_id = str(self.get_parameter("odom_frame_id").value)
        self.base_frame_id = str(self.get_parameter("base_frame_id").value)

        rgb_topic = str(self.get_parameter("rgb_image_topic").value)
        depth_topic = str(self.get_parameter("depth_image_topic").value)
        camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        imu_topic = str(self.get_parameter("imu_topic").value)
        global_pose_topic = str(self.get_parameter("global_pose_topic").value)
        visual_odom_topic = str(self.get_parameter("visual_odometry_topic").value)
        trajectory_topic = str(self.get_parameter("trajectory_topic").value)

        health_topic = str(self.get_parameter("health_topic").value)
        health_rate = float(self.get_parameter("health_publish_rate").value)
        health_warn_timeout = float(self.get_parameter("health_warn_timeout_sec").value)
        health_stale_timeout = float(self.get_parameter("health_stale_timeout_sec").value)

        # State
        self.camera_info = None
        self.current_pose = PoseStamped()
        self.current_pose.header.frame_id = self.global_frame_id
        self.trajectory = Path()
        self.trajectory.header.frame_id = self.global_frame_id
        self.slam_initialized = False
        self.feature_count = 0
        self.loop_closures = 0

        # Subscribers
        self.rgb_sub = self.create_subscription(
            Image, rgb_topic, self._rgb_callback, 10
        )
        if self.use_depth:
            self.depth_sub = self.create_subscription(
                Image, depth_topic, self._depth_callback, 10
            )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self._camera_info_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Image, imu_topic, self._imu_callback, 10
        )

        # Publishers
        if self.publish_global_pose:
            self.global_pose_pub = self.create_publisher(
                PoseWithCovariance, global_pose_topic, 10
            )
        self.visual_odom_pub = self.create_publisher(
            Odometry, visual_odom_topic, 10
        )
        self.trajectory_pub = self.create_publisher(
            Path, trajectory_topic, 10
        )

        # TF broadcaster
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

        # Health monitoring
        self.health_pub = HealthStatusPublisher(
            self,
            health_topic,
            rate_hz=health_rate,
        )
        self.watchdogs = {
            "rgb": InputWatchdog(
                "rgb_image",
                warn_timeout=health_warn_timeout,
                stale_timeout=health_stale_timeout,
            ),
            "camera_info": InputWatchdog(
                "camera_info",
                warn_timeout=health_warn_timeout,
                stale_timeout=health_stale_timeout,
            ),
        }
        if self.use_depth:
            self.watchdogs["depth"] = InputWatchdog(
                "depth_image",
                warn_timeout=health_warn_timeout,
                stale_timeout=health_stale_timeout,
            )

        # Timer for periodic updates
        self.update_timer = self.create_timer(0.1, self._update_callback)  # 10Hz

        self.get_logger().info(
            f"Visual SLAM initialized: camera={self.camera_name}, "
            f"depth={self.use_depth}, frame={self.global_frame_id}"
        )

    def _rgb_callback(self, msg: Image):
        """Receive RGB image"""
        self.watchdogs["rgb"].update()
        # TODO: Process with RTAB-Map
        # For now, this is a placeholder that will integrate with rtabmap_ros

    def _depth_callback(self, msg: Image):
        """Receive depth image"""
        if "depth" in self.watchdogs:
            self.watchdogs["depth"].update()
        # TODO: Process with RTAB-Map

    def _camera_info_callback(self, msg: CameraInfo):
        """Receive camera calibration"""
        if self.camera_info is None:
            self.get_logger().info(
                f"Received camera info: {msg.width}x{msg.height}, "
                f"fx={msg.k[0]:.1f}, fy={msg.k[4]:.1f}"
            )
        self.camera_info = msg
        self.watchdogs["camera_info"].update()

    def _imu_callback(self, msg):
        """Receive IMU data for visual-inertial odometry"""
        # TODO: Integrate IMU with RTAB-Map for VIO
        pass

    def _update_callback(self):
        """Periodic update for publishing pose and health status"""
        self._publish_health_status()

        # Publish trajectory
        if len(self.trajectory.poses) > 0:
            self.trajectory_pub.publish(self.trajectory)

    def _publish_health_status(self):
        """Publish health status"""
        # Aggregate watchdog status
        watchdog_status = [wd.get_status() for wd in self.watchdogs.values()]
        all_ok = all(status == "OK" for status in watchdog_status)

        if all_ok and self.slam_initialized:
            status = "OK"
            message = f"SLAM running: features={self.feature_count}, loops={self.loop_closures}"
        elif all_ok and not self.slam_initialized:
            status = "WARN"
            message = "SLAM not yet initialized"
        else:
            status = "ERROR"
            failed = [
                name for name, wd in self.watchdogs.items()
                if wd.get_status() != "OK"
            ]
            message = f"Missing inputs: {', '.join(failed)}"

        self.health_pub.publish(status, message)

    def _publish_pose(self, pose: PoseStamped):
        """Publish global pose for drift correction"""
        if self.publish_global_pose:
            # Convert to PoseWithCovariance for calibration manager
            msg = PoseWithCovariance()
            msg.pose = pose.pose
            # TODO: Set covariance based on RTAB-Map confidence
            msg.covariance = [0.01] * 36  # Placeholder
            self.global_pose_pub.publish(msg)

        # Publish visual odometry
        odom = Odometry()
        odom.header = pose.header
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose = pose.pose
        # TODO: Set twist from RTAB-Map
        self.visual_odom_pub.publish(odom)

        # Publish TF
        if self.publish_tf:
            t = TransformStamped()
            t.header = pose.header
            t.child_frame_id = self.odom_frame_id
            t.transform.translation.x = pose.pose.position.x
            t.transform.translation.y = pose.pose.position.y
            t.transform.translation.z = pose.pose.position.z
            t.transform.rotation = pose.pose.orientation
            self.tf_broadcaster.sendTransform(t)

        # Update trajectory
        self.trajectory.poses.append(pose)
        # Limit trajectory length
        if len(self.trajectory.poses) > 1000:
            self.trajectory.poses.pop(0)
        self.trajectory.header.stamp = pose.header.stamp


def main(args=None):
    rclpy.init(args=args)
    node = VisualSLAMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
