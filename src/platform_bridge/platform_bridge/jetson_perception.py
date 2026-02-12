#!/usr/bin/env python3
"""
Jetson Perception Adapter

Wraps ROS 2 perception nodes (nvblox, RealSense, Visual SLAM) and publishes
to platform-standard interfaces via Zenoh.
"""

# Standard library
import struct
import threading

# Third-party
from geometry_msgs.msg import PoseWithCovariance
from rclpy.node import Node
from robotics_common import TimeSync, create_timestamp_dict

# Platform SDK
from robotics_sdk import PerceptionBase
from sensor_msgs.msg import Image
from std_msgs.msg import String


class JetsonPerception(PerceptionBase):
    """
    Jetson perception adapter.

    Bridges ROS 2 perception outputs to platform interfaces:
    - nvblox TSDF/ESDF → Platform TSDFUpdate/ESDF messages
    - Visual SLAM poses → Platform RobotPose messages
    - Camera images → Platform sensor topics
    """

    def __init__(self, robot_id: str, ros2_node: Node, zenoh_bridge=None):
        """
        Initialize Jetson perception adapter.

        Args:
            robot_id: Unique robot identifier
            ros2_node: ROS 2 node for subscriptions
            zenoh_bridge: ZenohBridge instance (if None, creates new session)
        """
        super().__init__(robot_id, zenoh_bridge)

        self.ros2_node = ros2_node
        self.time_sync = TimeSync(use_ptp=True)

        # Sequence numbers for messages
        self._tsdf_seq = 0
        self._esdf_seq = 0
        self._pose_seq = 0

        # Thread safety
        self.lock = threading.Lock()

        # ROS 2 subscriptions
        self._setup_subscriptions()

        self.ros2_node.get_logger().info(f"[JetsonPerception] Initialized for robot {robot_id}")

    def _setup_subscriptions(self):
        """Set up ROS 2 subscriptions to perception nodes."""

        # Subscribe to nvblox TSDF updates
        # Note: In real implementation, nvblox publishes custom message types
        # For now, we'll use a placeholder subscription structure
        # You'll need to adapt this to actual nvblox message types

        # Visual SLAM pose estimates
        self.ros2_node.create_subscription(
            PoseWithCovariance, "/vision/global_pose", self._pose_callback, 10
        )

        # Camera images (for forwarding to platform)
        self.ros2_node.create_subscription(
            Image,
            "/hardware/camera_front/color/image_raw",
            lambda msg: self._image_callback(msg, "left"),
            10,
        )

        self.ros2_node.create_subscription(
            Image,
            "/hardware/camera_rear/color/image_raw",
            lambda msg: self._image_callback(msg, "right"),
            10,
        )

        # Depth images
        self.ros2_node.create_subscription(
            Image,
            "/hardware/camera_front/aligned_depth_to_color/image_raw",
            lambda msg: self._depth_callback(msg, "left"),
            10,
        )

        self.ros2_node.create_subscription(
            Image,
            "/hardware/camera_rear/aligned_depth_to_color/image_raw",
            lambda msg: self._depth_callback(msg, "right"),
            10,
        )

        # nvblox status (to detect TSDF updates)
        self.ros2_node.create_subscription(
            String, "/nvblox/status", self._nvblox_status_callback, 10
        )

        self.ros2_node.get_logger().info("[JetsonPerception] Subscriptions created")

    def _pose_callback(self, msg: PoseWithCovariance):
        """Handle pose updates from visual SLAM."""
        try:
            with self.lock:
                self._pose_seq += 1

                # Create platform RobotPose message
                timestamp_dict = create_timestamp_dict(self.time_sync)

                pose_dict = {
                    **timestamp_dict,
                    "robot_id": self.robot_id,
                    "sequence_number": self._pose_seq,
                    # Position
                    "x": msg.pose.position.x,
                    "y": msg.pose.position.y,
                    "z": msg.pose.position.z,
                    # Orientation (quaternion)
                    "qx": msg.pose.orientation.x,
                    "qy": msg.pose.orientation.y,
                    "qz": msg.pose.orientation.z,
                    "qw": msg.pose.orientation.w,
                    # Covariance (6x6 matrix)
                    "covariance": list(msg.covariance),
                    # Metadata
                    "source_type": "visual_slam",
                    "confidence": 0.9,  # TODO: Extract from SLAM quality metrics
                    "tracking_quality": 0.85,
                    # Velocity (not available from this message)
                    "has_velocity": False,
                    # Factor graph metadata
                    "is_keyframe": False,  # TODO: Detect keyframes
                    "loop_closure_id": -1,
                    "observed_landmark_ids": [],
                }

                # Publish to platform
                self.publish_pose(pose_dict)

        except Exception as e:
            self.ros2_node.get_logger().error(f"Error in pose callback: {e}")

    def _image_callback(self, msg: Image, camera_name: str):
        """
        Handle camera image updates.

        Args:
            msg: ROS 2 Image message
            camera_name: Camera identifier ("left" or "right")
        """
        try:
            # For now, just forward the raw image data
            # In production, you might want to compress or convert format

            # Simple serialization: pack image metadata + raw data
            # Format: width(uint32) + height(uint32) + encoding(str) + data
            encoding_bytes = msg.encoding.encode("utf-8")
            header = struct.pack("<II", msg.width, msg.height)
            header += struct.pack("<I", len(encoding_bytes)) + encoding_bytes
            image_data = header + bytes(msg.data)

            self.publish_image(camera_name, image_data)

        except Exception as e:
            self.ros2_node.get_logger().error(f"Error in image callback ({camera_name}): {e}")

    def _depth_callback(self, msg: Image, camera_name: str):
        """
        Handle depth image updates.

        Args:
            msg: ROS 2 depth Image message
            camera_name: Camera identifier
        """
        try:
            # Pack depth image similar to RGB
            encoding_bytes = msg.encoding.encode("utf-8")
            header = struct.pack("<II", msg.width, msg.height)
            header += struct.pack("<I", len(encoding_bytes)) + encoding_bytes
            depth_data = header + bytes(msg.data)

            self.publish_depth(camera_name, depth_data)

        except Exception as e:
            self.ros2_node.get_logger().error(f"Error in depth callback ({camera_name}): {e}")

    def _nvblox_status_callback(self, msg: String):
        """
        Handle nvblox status updates.

        When nvblox updates the TSDF, extract and publish to platform.
        """
        # TODO: Implement proper nvblox TSDF extraction
        # This requires subscribing to actual nvblox TSDF messages
        # and converting to platform TSDFUpdate format

        # Placeholder: Detect "updated" status and trigger TSDF publish
        if "updated" in msg.data.lower():
            self._publish_tsdf_update()

    def _publish_tsdf_update(self):
        """
        Extract TSDF from nvblox and publish to platform.

        This is a placeholder implementation. In production, you would:
        1. Subscribe to nvblox TSDF layer messages
        2. Extract voxel block data
        3. Serialize to platform TSDFUpdate format
        4. Publish via self.publish_tsdf()
        """
        try:
            with self.lock:
                self._tsdf_seq += 1

                # TODO: Real implementation would extract from nvblox
                # For now, create placeholder data structure
                timestamp_dict = create_timestamp_dict(self.time_sync)

                tsdf_dict = {
                    **timestamp_dict,
                    "robot_id": self.robot_id,
                    "camera_id": "left",  # Or "fused" for multi-camera
                    "sequence_number": self._tsdf_seq,
                    "voxel_size": 0.05,  # 5cm voxels
                    "truncation_distance": 0.1,
                    "block_size": 8,
                    "num_blocks": 0,
                    "total_voxels": 0,
                }

                # Serialize to JSON for now (in production, use protobuf)
                # Standard library
                import json

                tsdf_data = json.dumps(tsdf_dict).encode("utf-8")

                self.publish_tsdf(tsdf_data)

                self.ros2_node.get_logger().debug(f"Published TSDF update #{self._tsdf_seq}")

        except Exception as e:
            self.ros2_node.get_logger().error(f"Error publishing TSDF: {e}")

    def start(self):
        """Start perception processing."""
        super().start()
        self.ros2_node.get_logger().info("[JetsonPerception] Started")

    def stop(self):
        """Stop perception processing."""
        super().stop()
        self.ros2_node.get_logger().info("[JetsonPerception] Stopped")
