#!/usr/bin/env python3
"""
Sensor Fusion Node
Synchronizes sensor data to camera frames, applies Kalman filtering to IMU data,
downsamples 3D data (pointclouds, mesh, TSDF) and images for feature builder and visualization.
"""

# Standard library
from collections import deque
import json
import threading
import time
from typing import Dict, Optional

# Third-party
import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, Image, Imu, PointCloud2, PointField
from std_msgs.msg import Header, String
from visualization_msgs.msg import Marker, MarkerArray


class KalmanFilter:
    """Simple Kalman filter for IMU data smoothing"""

    def __init__(self, process_noise: float = 0.01, measurement_noise: float = 0.1):
        """
        Initialize Kalman filter

        Args:
            process_noise: Process noise covariance (Q)
            measurement_noise: Measurement noise covariance (R)
        """
        self.process_noise = process_noise
        self.measurement_noise = measurement_noise

        # State: [accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z]
        self.state = np.zeros(6)
        self.covariance = np.eye(6) * 1.0

    def update(self, measurement: np.ndarray) -> np.ndarray:
        """
        Update filter with new measurement

        Args:
            measurement: 6D array [accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z]

        Returns:
            Filtered state estimate
        """
        # Prediction step
        # Simple constant velocity model (state doesn't change)
        predicted_state = self.state
        predicted_cov = self.covariance + np.eye(6) * self.process_noise

        # Update step
        innovation = measurement - predicted_state
        innovation_cov = predicted_cov + np.eye(6) * self.measurement_noise
        kalman_gain = predicted_cov @ np.linalg.inv(innovation_cov)

        self.state = predicted_state + kalman_gain @ innovation
        self.covariance = (np.eye(6) - kalman_gain) @ predicted_cov

        return self.state.copy()


class SensorSyncNode(Node):
    """Synchronizes sensor data to camera frames with Kalman filtering and downsampling"""

    def __init__(self):
        super().__init__("sensor_sync_node")

        # Parameters
        self.declare_parameter("sync_to_camera", True)
        self.declare_parameter("sync_camera_frames", True)  # Sync multiple cameras together
        self.declare_parameter("camera_frame_sync_tolerance", 0.05)  # Max time diff between cameras
        self.declare_parameter("target_frequency", 15.0)  # Match camera FPS
        self.declare_parameter("imu_filter_enabled", True)
        self.declare_parameter("imu_process_noise", 0.01)
        self.declare_parameter("imu_measurement_noise", 0.1)
        self.declare_parameter("max_buffer_size", 100)
        self.declare_parameter("time_sync_tolerance", 0.1)  # seconds

        # Camera topics
        self.declare_parameter("camera_topics", ["/hardware/camera_front/color/image_raw"])
        self.declare_parameter("imu_topic", "/phat/imu")
        self.declare_parameter("battery_topic", "/irobot/battery")
        self.declare_parameter("status_topic", "/irobot/status")

        # Output topics
        self.declare_parameter("output_namespace", "/sensor_sync")
        self.declare_parameter("publish_vlm_features", True)

        # Feature topic downsampling (moderate)
        self.declare_parameter("feature_image_width", 480)
        self.declare_parameter("feature_image_height", 360)
        self.declare_parameter("feature_pointcloud_factor", 2)
        self.declare_parameter("feature_mesh_decimation", 0.5)  # 50% reduction
        self.declare_parameter("feature_tsdf_voxel_size", 0.1)  # Coarser voxels

        # Viz topic downsampling (aggressive)
        self.declare_parameter("publish_viz_topics", True)
        self.declare_parameter("viz_frequency", 10.0)
        self.declare_parameter("viz_resolution_width", 320)
        self.declare_parameter("viz_resolution_height", 240)
        self.declare_parameter("viz_pointcloud_factor", 16)
        self.declare_parameter("viz_mesh_decimation", 0.75)  # 75% reduction
        self.declare_parameter("viz_tsdf_extract_mesh", True)
        self.declare_parameter("viz_tsdf_fps", 2.0)

        # Get parameters
        self.sync_to_camera = bool(self.get_parameter("sync_to_camera").value)
        self.sync_camera_frames = bool(self.get_parameter("sync_camera_frames").value)
        self.camera_frame_sync_tolerance = float(
            self.get_parameter("camera_frame_sync_tolerance").value
        )
        self.target_frequency = float(self.get_parameter("target_frequency").value)
        self.imu_filter_enabled = bool(self.get_parameter("imu_filter_enabled").value)
        self.max_buffer_size = int(self.get_parameter("max_buffer_size").value)
        self.time_sync_tolerance = float(self.get_parameter("time_sync_tolerance").value)

        # Feature downsampling parameters
        self.feature_image_width = int(self.get_parameter("feature_image_width").value)
        self.feature_image_height = int(self.get_parameter("feature_image_height").value)
        self.feature_pointcloud_factor = int(self.get_parameter("feature_pointcloud_factor").value)
        self.feature_mesh_decimation = float(self.get_parameter("feature_mesh_decimation").value)
        self.feature_tsdf_voxel_size = float(self.get_parameter("feature_tsdf_voxel_size").value)

        # Viz downsampling parameters
        self.publish_viz_topics = bool(self.get_parameter("publish_viz_topics").value)
        self.viz_frequency = float(self.get_parameter("viz_frequency").value)
        self.viz_resolution_width = int(self.get_parameter("viz_resolution_width").value)
        self.viz_resolution_height = int(self.get_parameter("viz_resolution_height").value)
        self.viz_pointcloud_factor = int(self.get_parameter("viz_pointcloud_factor").value)
        self.viz_mesh_decimation = float(self.get_parameter("viz_mesh_decimation").value)
        self.viz_tsdf_extract_mesh = bool(self.get_parameter("viz_tsdf_extract_mesh").value)
        self.viz_tsdf_fps = float(self.get_parameter("viz_tsdf_fps").value)

        # Kalman filter for IMU
        if self.imu_filter_enabled:
            process_noise = float(self.get_parameter("imu_process_noise").value)
            measurement_noise = float(self.get_parameter("imu_measurement_noise").value)
            self.kalman_filter = KalmanFilter(
                process_noise=process_noise, measurement_noise=measurement_noise
            )
        else:
            self.kalman_filter = None

        # CV Bridge for image processing
        self.bridge = CvBridge()

        # Buffers for sensor data (thread-safe with locks)
        self.imu_buffer: deque = deque(maxlen=self.max_buffer_size)
        self.battery_buffer: deque = deque(maxlen=self.max_buffer_size)
        self.status_buffer: deque = deque(maxlen=self.max_buffer_size)
        self.camera_timestamps: deque = deque(maxlen=self.max_buffer_size)

        # Camera frame sync tracking (for multi-camera synchronization)
        self.camera_frame_sync: Dict[str, Optional[Image]] = {}
        self.camera_frame_timestamps: Dict[str, float] = {}

        # Latched camera frames (last received frame per camera, used when new frames don't arrive)
        self.latched_camera_frames: Dict[str, Optional[Image]] = {}
        self.latched_camera_timestamps: Dict[str, float] = {}

        # 3D data buffers (from nvblox)
        self.pointcloud_buffers: Dict[str, deque] = {}
        self.mesh_buffers: Dict[str, deque] = {}
        self.tsdf_buffers: Dict[str, deque] = {}

        self.buffer_lock = threading.Lock()

        # Latest synchronized data
        self.latest_sync_data: Optional[Dict] = None

        # Viz publishing timers
        self.last_viz_publish_time: Dict[str, float] = {}
        self.last_viz_tsdf_publish_time = 0.0

        # Subscribers
        imu_topic = str(self.get_parameter("imu_topic").value)
        self.imu_sub = self.create_subscription(Imu, imu_topic, self._imu_callback, 10)

        # Chassis data subscribers
        battery_topic = str(self.get_parameter("battery_topic").value)
        status_topic = str(self.get_parameter("status_topic").value)
        self.battery_sub = self.create_subscription(
            BatteryState, battery_topic, self._battery_callback, 10
        )
        self.status_sub = self.create_subscription(String, status_topic, self._status_callback, 10)

        # Camera subscribers (for synchronization)
        camera_topics = self.get_parameter("camera_topics").value
        self.camera_subs = []
        if camera_topics:
            for topic in camera_topics:
                sub = self.create_subscription(
                    Image, str(topic), lambda msg, t=topic: self._camera_callback(msg, str(t)), 10
                )
                self.camera_subs.append(sub)
                camera_name = topic.split("/")[-3]  # Extract camera name
                self.pointcloud_buffers[camera_name] = deque(maxlen=self.max_buffer_size)
                self.mesh_buffers[camera_name] = deque(maxlen=self.max_buffer_size)
                self.tsdf_buffers[camera_name] = deque(maxlen=self.max_buffer_size)
                self.last_viz_publish_time[camera_name] = 0.0
                # Initialize latched frame storage
                self.latched_camera_frames[camera_name] = None
                self.latched_camera_timestamps[camera_name] = 0.0

        # nvblox subscribers (full quality 3D data)
        self.nvblox_pointcloud_subs: Dict[str, rclpy.subscription.Subscription] = {}
        self.nvblox_mesh_sub: Optional[rclpy.subscription.Subscription] = None
        self.nvblox_tsdf_sub: Optional[rclpy.subscription.Subscription] = None
        self.nvblox_image_subs: Dict[str, rclpy.subscription.Subscription] = {}

        # Subscribe to nvblox full quality outputs
        for camera_name in ["camera_front", "camera_rear"]:
            self.nvblox_pointcloud_subs[camera_name] = self.create_subscription(
                PointCloud2,
                f"/nvblox/full/{camera_name}/pointcloud",
                lambda msg, name=camera_name: self._nvblox_pointcloud_callback(msg, name),
                10,
            )
            self.nvblox_image_subs[camera_name] = self.create_subscription(
                Image,
                f"/nvblox/full/{camera_name}/image",
                lambda msg, name=camera_name: self._nvblox_image_callback(msg, name),
                10,
            )

        self.nvblox_mesh_sub = self.create_subscription(
            MarkerArray, "/nvblox/full/mesh", self._nvblox_mesh_callback, 10
        )
        self.nvblox_tsdf_sub = self.create_subscription(
            MarkerArray, "/nvblox/full/tsdf", self._nvblox_tsdf_callback, 10
        )

        # Publishers
        output_ns = str(self.get_parameter("output_namespace").value)

        # Filtered/synchronized IMU
        self.filtered_imu_pub = self.create_publisher(Imu, f"{output_ns}/imu/filtered", 10)

        # Synchronized chassis data
        self.synced_battery_pub = self.create_publisher(
            BatteryState, f"{output_ns}/chassis/battery", 10
        )
        self.synced_status_pub = self.create_publisher(String, f"{output_ns}/chassis/status", 10)

        # Feature topics (moderate downsampling)
        self.feature_camera_pubs: Dict[str, rclpy.publisher.Publisher] = {}
        self.feature_pointcloud_pubs: Dict[str, rclpy.publisher.Publisher] = {}
        self.feature_mesh_pub = self.create_publisher(MarkerArray, f"{output_ns}/three_d/mesh", 10)
        self.feature_tsdf_pub = self.create_publisher(MarkerArray, f"{output_ns}/three_d/tsdf", 10)

        for camera_name in ["camera_front", "camera_rear"]:
            self.feature_camera_pubs[camera_name] = self.create_publisher(
                Image, f"{output_ns}/{camera_name}/color/image_raw", 10
            )
            self.feature_pointcloud_pubs[camera_name] = self.create_publisher(
                PointCloud2, f"{output_ns}/three_d/{camera_name}/pointcloud", 10
            )

        # Viz topics (aggressive downsampling)
        if self.publish_viz_topics:
            self.viz_imu_pub = self.create_publisher(Imu, "/viz/remote/imu/filtered", 10)
            self.viz_chassis_pub = self.create_publisher(
                BatteryState, "/viz/remote/chassis/battery", 10
            )
            self.viz_camera_front_pub = self.create_publisher(
                Image, "/viz/remote/camera_front/color/image_raw", 10
            )
            self.viz_pointcloud_pub = self.create_publisher(
                PointCloud2, "/viz/remote/three_d/pointcloud", 10
            )
            self.viz_mesh_pub = self.create_publisher(MarkerArray, "/viz/remote/three_d/mesh", 10)
            self.viz_tsdf_mesh_pub = self.create_publisher(
                MarkerArray, "/viz/remote/three_d/tsdf_mesh", 10
            )

        # Synchronized sensor data (for VLM)
        if self.get_parameter("publish_vlm_features").value:
            self.vlm_features_pub = self.create_publisher(String, f"{output_ns}/vlm_features", 10)

        # Status
        self.status_pub = self.create_publisher(String, f"{output_ns}/status", 10)

        # Timer for synchronized publishing
        if self.sync_to_camera:
            # Will be triggered by camera callbacks, but also use timer as fallback
            # This ensures topics are published even if cameras aren't available
            timer_period = 1.0 / float(self.target_frequency)
            self.sync_timer = self.create_timer(timer_period, self._publish_synchronized_data)
        else:
            # Fixed rate publishing
            timer_period = 1.0 / float(self.target_frequency)
            self.sync_timer = self.create_timer(timer_period, self._publish_synchronized_data)

        self.get_logger().info("Sensor fusion node started")
        self.get_logger().info(f"Sync to camera: {self.sync_to_camera}")
        self.get_logger().info(f"Sync camera frames: {self.sync_camera_frames}")
        self.get_logger().info(f"Target frequency: {self.target_frequency} Hz")
        self.get_logger().info(f"IMU filtering: {self.imu_filter_enabled}")
        self.get_logger().info(
            f"Feature image size: {self.feature_image_width}x{self.feature_image_height}"
        )
        self.get_logger().info(
            f"Viz image size: {self.viz_resolution_width}x{self.viz_resolution_height}"
        )

    def _imu_callback(self, msg: Imu):
        """Store IMU data in buffer"""
        with self.buffer_lock:
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.imu_buffer.append(
                {
                    "timestamp": timestamp,
                    "msg": msg,
                    "accel": np.array(
                        [
                            msg.linear_acceleration.x,
                            msg.linear_acceleration.y,
                            msg.linear_acceleration.z,
                        ]
                    ),
                    "gyro": np.array(
                        [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
                    ),
                }
            )

    def _battery_callback(self, msg: BatteryState):
        """Store battery data in buffer"""
        with self.buffer_lock:
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.battery_buffer.append({"timestamp": timestamp, "msg": msg})

    def _status_callback(self, msg: String):
        """Store status data in buffer"""
        with self.buffer_lock:
            timestamp = time.time()  # Status messages may not have timestamps
            self.status_buffer.append({"timestamp": timestamp, "msg": msg})

    def _camera_callback(self, msg: Image, topic: str):
        """Handle camera frame - latch it and trigger synchronization"""
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        camera_name = topic.split("/")[-3]  # Extract camera name

        with self.buffer_lock:
            self.camera_timestamps.append(
                {"timestamp": timestamp, "topic": topic, "header": msg.header}
            )
            # Store latest frame for each camera (for frame sync)
            self.camera_frame_sync[topic] = msg
            self.camera_frame_timestamps[topic] = timestamp

            # Latch the frame (always keep the last frame for each camera)
            self.latched_camera_frames[camera_name] = msg
            self.latched_camera_timestamps[camera_name] = timestamp

        # Always attempt to sync cameras, but don't block publishing
        if self.sync_to_camera:
            # Check if we should wait for all cameras to sync
            if self.sync_camera_frames:
                # Try to sync cameras, but don't block forever
                if self._check_camera_frame_sync():
                    # All cameras are synced, use average timestamp
                    avg_timestamp = sum(self.camera_frame_timestamps.values()) / len(
                        self.camera_frame_timestamps
                    )
                    self._publish_synchronized_data(avg_timestamp)
                else:
                    # Not all cameras synced yet, but publish anyway if we have at least one camera
                    # This prevents blocking when cameras aren't perfectly synchronized
                    if len(self.camera_frame_timestamps) > 0:
                        # Use the most recent timestamp
                        latest_timestamp = max(self.camera_frame_timestamps.values())
                        self._publish_synchronized_data(latest_timestamp)
            else:
                # Publish immediately on any camera frame
                self._publish_synchronized_data(timestamp)

    def _nvblox_pointcloud_callback(self, msg: PointCloud2, camera_name: str):
        """Store nvblox pointcloud data"""
        with self.buffer_lock:
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if camera_name in self.pointcloud_buffers:
                self.pointcloud_buffers[camera_name].append({"timestamp": timestamp, "msg": msg})

    def _nvblox_mesh_callback(self, msg: MarkerArray):
        """Store nvblox mesh data"""
        with self.buffer_lock:
            timestamp = time.time()  # MarkerArray may not have timestamp
            # Store in a generic buffer (fused mesh)
            if "fused" not in self.mesh_buffers:
                self.mesh_buffers["fused"] = deque(maxlen=self.max_buffer_size)
            self.mesh_buffers["fused"].append({"timestamp": timestamp, "msg": msg})

    def _nvblox_tsdf_callback(self, msg: MarkerArray):
        """Store nvblox TSDF data"""
        with self.buffer_lock:
            timestamp = time.time()  # MarkerArray may not have timestamp
            # Store in a generic buffer (fused TSDF)
            if "fused" not in self.tsdf_buffers:
                self.tsdf_buffers["fused"] = deque(maxlen=self.max_buffer_size)
            self.tsdf_buffers["fused"].append({"timestamp": timestamp, "msg": msg})

    def _nvblox_image_callback(self, msg: Image, camera_name: str):
        """Store nvblox full quality images (for downsampling)"""
        # These can be used if we want to downsample from nvblox images instead of raw camera images
        # For now, we use raw camera images
        pass

    def _check_camera_frame_sync(self) -> bool:
        """
        Check if all cameras have frames within sync tolerance.
        Always attempts to sync, but returns False if cameras aren't perfectly synced.
        This allows the system to function with or without hardware sync.
        """
        if len(self.camera_frame_timestamps) < len(self.camera_subs):
            return False  # Not all cameras have frames yet

        timestamps = list(self.camera_frame_timestamps.values())
        if len(timestamps) < 2:
            return True  # Single camera or no cameras

        # Check if all timestamps are within tolerance
        if len(timestamps) == 0:
            return False
        min_ts = min(timestamps)
        max_ts = max(timestamps)
        tolerance = float(self.camera_frame_sync_tolerance)
        synced = (max_ts - min_ts) <= tolerance

        # Log sync status for debugging
        if not synced and len(timestamps) > 1:
            time_diff = max_ts - min_ts
            self.get_logger().debug(
                f"Camera sync check: {len(timestamps)} cameras, "
                f"time diff: {time_diff:.4f}s (tolerance: {tolerance:.4f}s)"
            )

        return synced

    def _downsample_image(self, img_msg: Image, width: int, height: int) -> Image:
        """Downsample image to target resolution"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="passthrough")
            resized = cv2.resize(cv_image, (width, height), interpolation=cv2.INTER_LINEAR)
            downsampled_msg = self.bridge.cv2_to_imgmsg(resized, encoding=img_msg.encoding)
            downsampled_msg.header = img_msg.header
            return downsampled_msg
        except Exception as e:
            self.get_logger().error(f"Error downsampling image: {e}")
            return img_msg

    def _downsample_pointcloud(self, pointcloud_msg: PointCloud2, factor: int) -> PointCloud2:
        """Downsample pointcloud by keeping every Nth point"""
        try:
            # Extract points from PointCloud2
            points = np.frombuffer(pointcloud_msg.data, dtype=np.float32).reshape(
                -1, pointcloud_msg.point_step // 4
            )

            # Extract x, y, z (assuming they're first 12 bytes)
            xyz = points[:, :3]

            # Downsample by keeping every Nth point
            downsampled_xyz = xyz[::factor]

            # Create new PointCloud2 message
            downsampled_msg = PointCloud2()
            downsampled_msg.header = pointcloud_msg.header
            downsampled_msg.height = 1
            downsampled_msg.width = len(downsampled_xyz)
            downsampled_msg.fields = [
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            ]
            downsampled_msg.is_bigendian = False
            downsampled_msg.point_step = 12
            downsampled_msg.row_step = downsampled_msg.point_step * downsampled_msg.width
            downsampled_msg.is_dense = True
            downsampled_msg.data = downsampled_xyz.astype(np.float32).tobytes()

            return downsampled_msg
        except Exception as e:
            self.get_logger().error(f"Error downsampling pointcloud: {e}")
            return pointcloud_msg

    def _decimate_mesh(self, mesh_msg: MarkerArray, decimation: float) -> MarkerArray:
        """Decimate mesh by reducing triangle count"""
        # Simple decimation: keep every Nth triangle
        # More sophisticated decimation (QEM) would require additional libraries
        try:
            decimated_markers = MarkerArray()
            for marker in mesh_msg.markers:
                if marker.type == Marker.TRIANGLE_LIST:
                    decimated_marker = Marker()
                    decimated_marker.header = marker.header
                    decimated_marker.ns = marker.ns
                    decimated_marker.id = marker.id
                    decimated_marker.type = Marker.TRIANGLE_LIST
                    decimated_marker.action = Marker.ADD
                    decimated_marker.scale = marker.scale
                    decimated_marker.color = marker.color

                    # Keep every Nth triangle (3 points per triangle)
                    keep_factor = int(1.0 / (1.0 - decimation))
                    points = marker.points
                    decimated_points = points[:: keep_factor * 3]
                    # Ensure we have multiples of 3
                    num_triangles = len(decimated_points) // 3
                    decimated_marker.points = decimated_points[: num_triangles * 3]

                    decimated_markers.markers.append(decimated_marker)
                else:
                    decimated_markers.markers.append(marker)

            return decimated_markers
        except Exception as e:
            self.get_logger().error(f"Error decimating mesh: {e}")
            return mesh_msg

    def _process_tsdf(
        self, tsdf_msg: MarkerArray, voxel_size: float, extract_mesh: bool
    ) -> MarkerArray:
        """Process TSDF: coarsen voxel grid or extract mesh"""
        if extract_mesh:
            # Extract mesh from TSDF (simplified - just return as mesh)
            # In a full implementation, this would use marching cubes
            return self._decimate_mesh(tsdf_msg, 0.5)  # Decimate extracted mesh
        else:
            # Coarsen voxel grid by increasing voxel size
            # This is a simplified version - full implementation would resample TSDF
            return tsdf_msg  # For now, return as-is

    def _publish_synchronized_data(self, sync_timestamp: Optional[float] = None):
        """Publish synchronized sensor data with downsampling"""
        if sync_timestamp is None:
            sync_timestamp = time.time()

        current_time = time.time()

        with self.buffer_lock:
            # Find closest IMU data
            imu_data = self._find_closest_sensor_data(self.imu_buffer, sync_timestamp)

            # If no IMU data, still publish empty/placeholder messages for viz topics
            # to ensure topics exist for bridge discovery
            if imu_data is None:
                # Publish placeholder messages for viz topics so bridge can discover them
                if self.publish_viz_topics:
                    # Create empty IMU message
                    empty_imu = Imu()
                    empty_imu.header.stamp = self.get_clock().now().to_msg()
                    empty_imu.header.frame_id = "base_link"
                    if "imu" not in self.last_viz_publish_time or (
                        current_time - self.last_viz_publish_time["imu"]
                    ) >= (1.0 / self.viz_frequency):
                        self.viz_imu_pub.publish(empty_imu)
                        self.last_viz_publish_time["imu"] = current_time

                    # Create empty battery message
                    empty_battery = BatteryState()
                    empty_battery.header.stamp = self.get_clock().now().to_msg()
                    empty_battery.header.frame_id = "base_link"
                    if "battery" not in self.last_viz_publish_time or (
                        current_time - self.last_viz_publish_time["battery"]
                    ) >= (1.0 / self.viz_frequency):
                        self.viz_chassis_pub.publish(empty_battery)
                        self.last_viz_publish_time["battery"] = current_time
                return

            # Apply Kalman filter if enabled
            if self.kalman_filter:
                measurement = np.concatenate([imu_data["accel"], imu_data["gyro"]])
                filtered_state = self.kalman_filter.update(measurement)

                # Create filtered IMU message
                filtered_imu = Imu()
                filtered_imu.header = Header()
                filtered_imu.header.stamp = self.get_clock().now().to_msg()
                filtered_imu.header.frame_id = imu_data["msg"].header.frame_id

                filtered_imu.linear_acceleration.x = filtered_state[0]
                filtered_imu.linear_acceleration.y = filtered_state[1]
                filtered_imu.linear_acceleration.z = filtered_state[2]
                filtered_imu.angular_velocity.x = filtered_state[3]
                filtered_imu.angular_velocity.y = filtered_state[4]
                filtered_imu.angular_velocity.z = filtered_state[5]

                # Copy covariance from original
                filtered_imu.linear_acceleration_covariance = imu_data[
                    "msg"
                ].linear_acceleration_covariance
                filtered_imu.angular_velocity_covariance = imu_data[
                    "msg"
                ].angular_velocity_covariance
            else:
                filtered_imu = imu_data["msg"]
                filtered_imu.header.stamp = self.get_clock().now().to_msg()

            # Find closest chassis data
            battery_data = self._find_closest_sensor_data(self.battery_buffer, sync_timestamp)
            status_data = self._find_closest_sensor_data(self.status_buffer, sync_timestamp)

            # Publish filtered IMU (feature topic)
            self.filtered_imu_pub.publish(filtered_imu)

            # Publish viz IMU (if enabled and time for viz publish)
            if self.publish_viz_topics:
                if "imu" not in self.last_viz_publish_time or (
                    current_time - self.last_viz_publish_time["imu"]
                ) >= (1.0 / self.viz_frequency):
                    self.viz_imu_pub.publish(filtered_imu)
                    self.last_viz_publish_time["imu"] = current_time

            # Publish synchronized chassis data
            if battery_data:
                synced_battery = BatteryState()
                synced_battery.header = Header()
                synced_battery.header.stamp = self.get_clock().now().to_msg()
                synced_battery.header.frame_id = battery_data["msg"].header.frame_id
                synced_battery.voltage = battery_data["msg"].voltage
                synced_battery.percentage = battery_data["msg"].percentage
                synced_battery.current = battery_data["msg"].current
                synced_battery.present = battery_data["msg"].present
                synced_battery.power_supply_status = battery_data["msg"].power_supply_status
                synced_battery.power_supply_health = battery_data["msg"].power_supply_health
                self.synced_battery_pub.publish(synced_battery)

                if self.publish_viz_topics:
                    if "battery" not in self.last_viz_publish_time or (
                        current_time - self.last_viz_publish_time["battery"]
                    ) >= (1.0 / self.viz_frequency):
                        self.viz_chassis_pub.publish(synced_battery)
                        self.last_viz_publish_time["battery"] = current_time

            if status_data:
                synced_status = String()
                synced_status.data = status_data["msg"].data
                self.synced_status_pub.publish(synced_status)

            # Process camera images
            # Always attempt to sync cameras, using latched frames if sync fails
            cameras_to_process = {}

            # First, try to get synced frames from camera_frame_sync
            if self.sync_camera_frames and self._check_camera_frame_sync():
                # All cameras synced, use synced frames
                for topic, camera_msg in self.camera_frame_sync.items():
                    camera_name = topic.split("/")[-3]
                    cameras_to_process[camera_name] = camera_msg
            else:
                # Not synced or sync disabled, use latched frames
                # (last received frame per camera)
                # This ensures we always have frames to publish, even if cameras
                # aren't perfectly synced
                cameras_to_process = self.latched_camera_frames.copy()

            # Process each camera (synced or latched)
            for camera_name, camera_msg in cameras_to_process.items():
                if camera_msg is None:
                    continue

                if camera_name in self.feature_camera_pubs:
                    # Downsample for feature topics
                    feature_img = self._downsample_image(
                        camera_msg, self.feature_image_width, self.feature_image_height
                    )
                    feature_img.header.stamp = self.get_clock().now().to_msg()
                    self.feature_camera_pubs[camera_name].publish(feature_img)

                    # Downsample for viz topics
                    if self.publish_viz_topics and camera_name == "camera_front":
                        if camera_name not in self.last_viz_publish_time or (
                            current_time - self.last_viz_publish_time[camera_name]
                        ) >= (1.0 / self.viz_frequency):
                            viz_img = self._downsample_image(
                                camera_msg, self.viz_resolution_width, self.viz_resolution_height
                            )
                            viz_img.header.stamp = self.get_clock().now().to_msg()
                            self.viz_camera_front_pub.publish(viz_img)
                            self.last_viz_publish_time[camera_name] = current_time

            # If no camera frames available, publish blank frames for viz topics
            if self.publish_viz_topics and len(cameras_to_process) == 0:
                if "camera_front" not in self.last_viz_publish_time or (
                    current_time - self.last_viz_publish_time.get("camera_front", 0)
                ) >= (1.0 / self.viz_frequency):
                    # Create blank image
                    blank_img = Image()
                    blank_img.header.stamp = self.get_clock().now().to_msg()
                    blank_img.header.frame_id = "camera_front_optical_frame"
                    blank_img.height = self.viz_resolution_height
                    blank_img.width = self.viz_resolution_width
                    blank_img.encoding = "rgb8"
                    blank_img.is_bigendian = False
                    blank_img.step = self.viz_resolution_width * 3
                    blank_img.data = bytes(
                        self.viz_resolution_width * self.viz_resolution_height * 3
                    )
                    self.viz_camera_front_pub.publish(blank_img)
                    self.last_viz_publish_time["camera_front"] = current_time

            # Process 3D data (pointclouds, mesh, TSDF)
            for camera_name in self.pointcloud_buffers.keys():
                # Feature pointclouds
                pointcloud_data = self._find_closest_sensor_data(
                    self.pointcloud_buffers[camera_name], sync_timestamp
                )
                if pointcloud_data and camera_name in self.feature_pointcloud_pubs:
                    downsampled_pc = self._downsample_pointcloud(
                        pointcloud_data["msg"], self.feature_pointcloud_factor
                    )
                    downsampled_pc.header.stamp = self.get_clock().now().to_msg()
                    self.feature_pointcloud_pubs[camera_name].publish(downsampled_pc)

                # Viz pointcloud (fused, only front camera for now)
                if self.publish_viz_topics and camera_name == "camera_front" and pointcloud_data:
                    if "pointcloud" not in self.last_viz_publish_time or (
                        current_time - self.last_viz_publish_time["pointcloud"]
                    ) >= (
                        1.0 / 5.0
                    ):  # 5 Hz for viz pointcloud
                        viz_pc = self._downsample_pointcloud(
                            pointcloud_data["msg"], self.viz_pointcloud_factor
                        )
                        viz_pc.header.stamp = self.get_clock().now().to_msg()
                        self.viz_pointcloud_pub.publish(viz_pc)
                        self.last_viz_publish_time["pointcloud"] = current_time

            # Feature mesh
            if "fused" in self.mesh_buffers:
                mesh_data = self._find_closest_sensor_data(
                    self.mesh_buffers["fused"], sync_timestamp
                )
                if mesh_data:
                    decimated_mesh = self._decimate_mesh(
                        mesh_data["msg"], self.feature_mesh_decimation
                    )
                    decimated_mesh.markers[0].header.stamp = self.get_clock().now().to_msg()
                    self.feature_mesh_pub.publish(decimated_mesh)

                    # Viz mesh
                    if self.publish_viz_topics:
                        if "mesh" not in self.last_viz_publish_time or (
                            current_time - self.last_viz_publish_time["mesh"]
                        ) >= (
                            1.0 / 5.0
                        ):  # 5 Hz for viz mesh
                            viz_mesh = self._decimate_mesh(
                                mesh_data["msg"], self.viz_mesh_decimation
                            )
                            viz_mesh.markers[0].header.stamp = self.get_clock().now().to_msg()
                            self.viz_mesh_pub.publish(viz_mesh)
                            self.last_viz_publish_time["mesh"] = current_time

            # Feature TSDF
            if "fused" in self.tsdf_buffers:
                tsdf_data = self._find_closest_sensor_data(
                    self.tsdf_buffers["fused"], sync_timestamp
                )
                if tsdf_data:
                    processed_tsdf = self._process_tsdf(
                        tsdf_data["msg"], self.feature_tsdf_voxel_size, False
                    )
                    processed_tsdf.markers[0].header.stamp = self.get_clock().now().to_msg()
                    self.feature_tsdf_pub.publish(processed_tsdf)

                    # Viz TSDF mesh
                    if self.publish_viz_topics:
                        if (current_time - self.last_viz_tsdf_publish_time) >= (
                            1.0 / self.viz_tsdf_fps
                        ):
                            tsdf_mesh = self._process_tsdf(
                                tsdf_data["msg"], self.feature_tsdf_voxel_size, True
                            )
                            tsdf_mesh.markers[0].header.stamp = self.get_clock().now().to_msg()
                            self.viz_tsdf_mesh_pub.publish(tsdf_mesh)
                            self.last_viz_tsdf_publish_time = current_time

            # Prepare VLM features if enabled
            if self.get_parameter("publish_vlm_features").value:
                self._publish_vlm_features(filtered_imu, sync_timestamp, battery_data, status_data)

    def _find_closest_sensor_data(self, buffer: deque, target_timestamp: float) -> Optional[Dict]:
        """Find sensor data closest to target timestamp"""
        if len(buffer) == 0:
            return None

        closest = None
        min_diff = float("inf")

        for data in buffer:
            diff = abs(data["timestamp"] - target_timestamp)
            if diff < min_diff and diff < self.time_sync_tolerance:
                min_diff = diff
                closest = data

        return closest

    def _publish_vlm_features(
        self,
        imu_msg: Imu,
        timestamp: float,
        battery_data: Optional[Dict] = None,
        status_data: Optional[Dict] = None,
    ):
        """Publish synchronized sensor data as VLM features"""
        # Create feature string (can be extended to JSON or custom message)
        features = {
            "timestamp": timestamp,
            "imu": {
                "accel": [
                    imu_msg.linear_acceleration.x,
                    imu_msg.linear_acceleration.y,
                    imu_msg.linear_acceleration.z,
                ],
                "gyro": [
                    imu_msg.angular_velocity.x,
                    imu_msg.angular_velocity.y,
                    imu_msg.angular_velocity.z,
                ],
            },
        }

        # Add chassis data if available
        if battery_data:
            features["chassis"] = {
                "battery": {
                    "voltage": battery_data["msg"].voltage,
                    "percentage": battery_data["msg"].percentage,
                    "current": battery_data["msg"].current,
                }
            }

        if status_data and "msg" in status_data:
            features["chassis"] = features.get("chassis", {})
            features["chassis"]["status"] = status_data["msg"].data

        # For now, publish as JSON string (can be extended to custom message type)
        feature_msg = String()
        feature_msg.data = json.dumps(features)
        self.vlm_features_pub.publish(feature_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SensorSyncNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
