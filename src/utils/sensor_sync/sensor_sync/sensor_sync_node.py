#!/usr/bin/env python3
"""
Sensor Fusion Node

Takes high-frequency, time-mismatched raw sensor data and applies post-processing
and re-sampling to create fused, time-aligned data. This node is the single source
of truth for all downstream consumers.

Inputs (high-frequency, time-mismatched):
- Raw sensor data (IMU ~50Hz, chassis ~10Hz)
- Camera timestamps (for synchronization)
- nvblox-processed camera/3D data (full quality)
- System status (temperatures, CPU/GPU, etc.)

Processing:
- Time alignment: Synchronizes all data to camera frame timestamps
- Post-processing: Kalman filtering (IMU), downsampling (images, 3D data)
- Re-sampling: Converts high-frequency data to uniform camera frame rate (~15Hz)
- Data fusion: Combines sensor, chassis, and system information

Outputs (fused, time-aligned, ~15Hz):
- Chassis information (battery, status)
- Sensor information (IMU, cameras, 3D data)
- System information (temperatures, CPU/GPU, memory, disk, power, alerts)
- Visualization topics (aggressively downsampled for remote monitoring)

Downstream consumers:
- VLA Feature Builder: Consumes feature topics + control plan, applies history buffering
- Visualization: Consumes viz topics for remote monitoring
- Logging: Consumes feature topics for data recording

Note: Control plan is NOT published by this node (comes from planner).
"""

# Standard library
from collections import deque
import json
import threading
import time
from typing import Dict, Optional

# Third-party
# Local imports - shared utilities
from isaac_utils import KalmanFilter, downsample_image
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, Image, Imu, PointCloud2, PointField, Temperature
from std_msgs.msg import (
    Float32,
    Float32MultiArray,
    Header,
    MultiArrayDimension,
    String,
    UInt8MultiArray,
)
from visualization_msgs.msg import Marker, MarkerArray


class SensorSyncNode(Node):
    """Synchronizes sensor data to camera frames with Kalman filtering and downsampling"""

    def __init__(self, defer_init: bool = False, node_name: Optional[str] = None):
        node_name = node_name if node_name else "sensor_sync_node"
        super().__init__(node_name)
        self._defer_init = defer_init

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

        # System monitor topics (optional - will subscribe if topics exist)
        self.declare_parameter("system_status_topic", "/system/status")
        self.declare_parameter("system_cpu_temp_topic", "/system/temperature/cpu")
        self.declare_parameter("system_gpu_temp_topic", "/system/temperature/gpu")
        self.declare_parameter("system_cpu_usage_topic", "/system/cpu/usage")
        self.declare_parameter("system_gpu_usage_topic", "/system/gpu/usage")
        self.declare_parameter("system_memory_usage_topic", "/system/memory/usage")
        self.declare_parameter("system_disk_usage_topic", "/system/disk/usage")
        self.declare_parameter("system_power_topic", "/system/power")
        self.declare_parameter("system_alerts_topic", "/system/alerts")
        self.declare_parameter(
            "enable_system_status", True
        )  # Allow disabling if system monitor not available

        # Audio synchronization parameters
        self.declare_parameter("enable_audio_sync", False)  # Enable audio synchronization
        self.declare_parameter("audio_topic", "/microphone/audio")
        self.declare_parameter("audio_sample_rate", 16000)
        self.declare_parameter("audio_channels", 2)
        self.declare_parameter("audio_format", "S16_LE")
        self.declare_parameter("audio_sync_tolerance", 0.1)

        # Output topics
        self.declare_parameter(
            "output_namespace", "/sensor_fusion"
        )  # Default changed to match config
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

        # Image processing now uses shared library (isaac_utils)

        # Buffers for sensor data (thread-safe with locks)
        self.imu_buffer: deque = deque(maxlen=self.max_buffer_size)
        self.battery_buffer: deque = deque(maxlen=self.max_buffer_size)
        self.status_buffer: deque = deque(maxlen=self.max_buffer_size)
        self.camera_timestamps: deque = deque(maxlen=self.max_buffer_size)

        # System status buffers
        self.system_status_buffer: deque = deque(maxlen=self.max_buffer_size)
        self.system_cpu_temp_buffer: deque = deque(maxlen=self.max_buffer_size)
        self.system_gpu_temp_buffer: deque = deque(maxlen=self.max_buffer_size)
        self.system_cpu_usage_buffer: deque = deque(maxlen=self.max_buffer_size)
        self.system_gpu_usage_buffer: deque = deque(maxlen=self.max_buffer_size)
        self.system_memory_usage_buffer: deque = deque(maxlen=self.max_buffer_size)
        self.system_disk_usage_buffer: deque = deque(maxlen=self.max_buffer_size)
        self.system_power_buffer: deque = deque(maxlen=self.max_buffer_size)
        self.system_alerts_buffer: deque = deque(maxlen=self.max_buffer_size)

        # Audio buffer
        self.audio_buffer: deque = deque(maxlen=self.max_buffer_size)

        # Camera sync status tracking
        self.camera_sync_status: Dict[str, any] = {}  # Track sync status per camera pair
        self.last_sync_check_time = 0.0

        # Camera timestamp tracking (raw camera topics ONLY for sync timestamps, not image data)
        # Raw camera subscriptions are used only to get timestamps for synchronization
        self.camera_frame_timestamps: Dict[str, float] = {}
        self.last_camera_frame_time: Dict[str, float] = {}  # Track when each camera last updated

        # Latched nvblox images (ONLY source for image processing - no fallbacks)
        # Design: Raw cameras → nvblox → sensor_sync (single data path)
        self.latched_nvblox_images: Dict[str, Optional[Image]] = {}
        self.latched_nvblox_timestamps: Dict[str, float] = {}

        # OPTIMIZATION: Cache downsampled images to avoid redundant processing
        # Key: (camera_name, width, height), Value: (downsampled_image, source_timestamp)
        self.downsampled_image_cache: Dict[tuple, tuple] = {}
        self.cache_lock = threading.Lock()

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

        # System monitor subscribers (only if enabled)
        enable_system_status = bool(self.get_parameter("enable_system_status").value)
        if enable_system_status:
            system_status_topic = str(self.get_parameter("system_status_topic").value)
            system_cpu_temp_topic = str(self.get_parameter("system_cpu_temp_topic").value)
            system_gpu_temp_topic = str(self.get_parameter("system_gpu_temp_topic").value)
            system_cpu_usage_topic = str(self.get_parameter("system_cpu_usage_topic").value)
            system_gpu_usage_topic = str(self.get_parameter("system_gpu_usage_topic").value)
            system_memory_usage_topic = str(self.get_parameter("system_memory_usage_topic").value)
            system_disk_usage_topic = str(self.get_parameter("system_disk_usage_topic").value)
            system_power_topic = str(self.get_parameter("system_power_topic").value)
            system_alerts_topic = str(self.get_parameter("system_alerts_topic").value)

            self.system_status_sub = self.create_subscription(
                String, system_status_topic, self._system_status_callback, 10
            )
            self.system_cpu_temp_sub = self.create_subscription(
                Temperature, system_cpu_temp_topic, self._system_cpu_temp_callback, 10
            )
            self.system_gpu_temp_sub = self.create_subscription(
                Temperature, system_gpu_temp_topic, self._system_gpu_temp_callback, 10
            )
            self.system_cpu_usage_sub = self.create_subscription(
                Float32, system_cpu_usage_topic, self._system_cpu_usage_callback, 10
            )
            self.system_gpu_usage_sub = self.create_subscription(
                Float32, system_gpu_usage_topic, self._system_gpu_usage_callback, 10
            )
            self.system_memory_usage_sub = self.create_subscription(
                Float32, system_memory_usage_topic, self._system_memory_usage_callback, 10
            )
            self.system_disk_usage_sub = self.create_subscription(
                Float32, system_disk_usage_topic, self._system_disk_usage_callback, 10
            )
            self.system_power_sub = self.create_subscription(
                Float32, system_power_topic, self._system_power_callback, 10
            )
            self.system_alerts_sub = self.create_subscription(
                String, system_alerts_topic, self._system_alerts_callback, 10
            )
        else:
            # Create dummy subscribers to avoid errors
            self.system_status_sub = None

        # Audio subscriber (optional - will subscribe if enabled)
        enable_audio_sync = bool(self.get_parameter("enable_audio_sync").value)
        if enable_audio_sync:
            audio_topic = str(self.get_parameter("audio_topic").value)
            self.audio_sub = self.create_subscription(
                UInt8MultiArray, audio_topic, self._audio_callback, 10
            )
            self.audio_sample_rate = int(self.get_parameter("audio_sample_rate").value)
            self.audio_channels = int(self.get_parameter("audio_channels").value)
            self.audio_format = str(self.get_parameter("audio_format").value)
            self.audio_sync_tolerance = float(self.get_parameter("audio_sync_tolerance").value)
        else:
            self.audio_sub = None
            # Set default values for audio parameters (used for initialization even when disabled)
            self.audio_sample_rate = 16000
            self.audio_channels = 2
            self.audio_format = "S16_LE"
            self.audio_sync_tolerance = 0.1

        # Camera subscribers (for synchronization)
        # Use BEST_EFFORT QoS to match camera publishers (they use BEST_EFFORT to prevent blocking)
        # Third-party
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

        camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10
        )
        camera_topics = self.get_parameter("camera_topics").value
        self.camera_subs = []
        if camera_topics:
            self.get_logger().info(f"Subscribing to camera topics: {camera_topics}")
            for topic in camera_topics:
                topic_str = str(topic)
                camera_name = topic_str.split("/")[-3]  # Extract camera name

                # Create callback with proper closure to avoid lambda issues
                def make_callback(topic_path, cam_name):
                    def callback(msg):
                        self._camera_callback(msg, topic_path)

                    return callback

                callback_func = make_callback(topic_str, camera_name)
                sub = self.create_subscription(
                    Image,
                    topic_str,
                    callback_func,
                    camera_qos,  # BEST_EFFORT QoS to match publisher
                )
                self.camera_subs.append(sub)

                self.get_logger().info(
                    f"Created subscription for {topic_str} (camera_name: {camera_name}) with BEST_EFFORT QoS"
                )

                self.pointcloud_buffers[camera_name] = deque(maxlen=self.max_buffer_size)
                self.mesh_buffers[camera_name] = deque(maxlen=self.max_buffer_size)
                self.tsdf_buffers[camera_name] = deque(maxlen=self.max_buffer_size)
                self.last_viz_publish_time[camera_name] = 0.0
                # Initialize timestamp tracking (raw camera topics only for sync, not image data)
                self.last_camera_frame_time[camera_name] = 0.0

        # nvblox subscribers (full quality 3D data)
        self.nvblox_pointcloud_subs: Dict[str, rclpy.subscription.Subscription] = {}
        self.nvblox_mesh_sub: Optional[rclpy.subscription.Subscription] = None
        self.nvblox_tsdf_sub: Optional[rclpy.subscription.Subscription] = None
        self.nvblox_image_subs: Dict[str, rclpy.subscription.Subscription] = {}

        # Subscribe to nvblox full quality outputs
        # OPTIMIZATION: Use nvblox images directly for downsampling (eliminates 4 copies per frame)
        for camera_name in ["camera_front", "camera_rear"]:
            # Use functools.partial to avoid lambda closure bug
            # Third-party
            from functools import partial
            
            self.nvblox_pointcloud_subs[camera_name] = self.create_subscription(
                PointCloud2,
                f"/nvblox/full/{camera_name}/pointcloud",
                partial(self._nvblox_pointcloud_callback, camera_name=camera_name),
                10,
            )
            self.nvblox_image_subs[camera_name] = self.create_subscription(
                Image,
                f"/nvblox/full/{camera_name}/image",
                lambda msg, name=camera_name: self._nvblox_image_callback(msg, name),
                10,
            )
            # Initialize nvblox image latch
            self.latched_nvblox_images[camera_name] = None
            self.latched_nvblox_timestamps[camera_name] = 0.0

        self.nvblox_mesh_sub = self.create_subscription(
            MarkerArray, "/nvblox/full/mesh", self._nvblox_mesh_callback, 10
        )
        self.nvblox_tsdf_sub = self.create_subscription(
            MarkerArray, "/nvblox/full/tsdf", self._nvblox_tsdf_callback, 10
        )

        # Publishers - create after parameters are set (handled in _initialize if deferred)
        if not self._defer_init:
            self._create_publishers()

    def _create_publishers(self):
        """Create all publishers (called from __init__ or _initialize)"""
        output_ns = str(self.get_parameter("output_namespace").value)

        # Filtered/synchronized IMU
        self.filtered_imu_pub = self.create_publisher(Imu, f"{output_ns}/imu/filtered", 10)

        # Synchronized chassis data
        self.synced_battery_pub = self.create_publisher(
            BatteryState, f"{output_ns}/chassis/battery", 10
        )
        self.synced_status_pub = self.create_publisher(String, f"{output_ns}/chassis/status", 10)

        # System status publishers (feature topics - for controller)
        self.system_status_pub = self.create_publisher(String, f"{output_ns}/system/status", 10)
        self.system_cpu_temp_pub = self.create_publisher(
            Temperature, f"{output_ns}/system/temperature/cpu", 10
        )
        self.system_gpu_temp_pub = self.create_publisher(
            Temperature, f"{output_ns}/system/temperature/gpu", 10
        )
        self.system_cpu_usage_pub = self.create_publisher(
            Float32, f"{output_ns}/system/cpu/usage", 10
        )
        self.system_gpu_usage_pub = self.create_publisher(
            Float32, f"{output_ns}/system/gpu/usage", 10
        )
        self.system_memory_usage_pub = self.create_publisher(
            Float32, f"{output_ns}/system/memory/usage", 10
        )
        self.system_disk_usage_pub = self.create_publisher(
            Float32, f"{output_ns}/system/disk/usage", 10
        )
        self.system_power_pub = self.create_publisher(Float32, f"{output_ns}/system/power", 10)
        self.system_alerts_pub = self.create_publisher(String, f"{output_ns}/system/alerts", 10)
        # Camera sync status
        self.system_camera_sync_status_pub = self.create_publisher(
            String, f"{output_ns}/system/hardware/camera_sync_status", 10
        )

        # Audio publishers (feature topic - raw audio synchronized to camera frames)
        enable_audio_sync = bool(self.get_parameter("enable_audio_sync").value)
        if enable_audio_sync:
            self.synced_audio_pub = self.create_publisher(
                UInt8MultiArray, f"{output_ns}/audio/raw", 10
            )
        else:
            self.synced_audio_pub = None

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
            # Use BEST_EFFORT QoS for viz images to prevent blocking and reduce latency
            # Third-party
            from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

            image_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=5,  # Small queue for low-latency
            )
            self.viz_camera_front_pub = self.create_publisher(
                Image, "/viz/remote/camera_front/color/image_raw", image_qos
            )
            self.viz_camera_rear_pub = self.create_publisher(
                Image, "/viz/remote/camera_rear/color/image_raw", image_qos
            )
            self.viz_pointcloud_pub = self.create_publisher(
                PointCloud2, "/viz/remote/three_d/pointcloud", 10
            )
            self.viz_mesh_pub = self.create_publisher(MarkerArray, "/viz/remote/three_d/mesh", 10)
            self.viz_tsdf_mesh_pub = self.create_publisher(
                MarkerArray, "/viz/remote/three_d/tsdf_mesh", 10
            )
            # System status for visualization (lower frequency)
            self.viz_system_status_pub = self.create_publisher(
                String, "/viz/remote/system/status", 10
            )
            self.viz_system_cpu_temp_pub = self.create_publisher(
                Temperature, "/viz/remote/system/temperature/cpu", 10
            )
            self.viz_system_gpu_temp_pub = self.create_publisher(
                Temperature, "/viz/remote/system/temperature/gpu", 10
            )
            self.viz_system_cpu_usage_pub = self.create_publisher(
                Float32, "/viz/remote/system/cpu/usage", 10
            )
            self.viz_system_gpu_usage_pub = self.create_publisher(
                Float32, "/viz/remote/system/gpu/usage", 10
            )
            self.viz_system_memory_usage_pub = self.create_publisher(
                Float32, "/viz/remote/system/memory/usage", 10
            )
            self.viz_system_alerts_pub = self.create_publisher(
                String, "/viz/remote/system/alerts", 10
            )
            # Camera sync status for visualization
            self.viz_system_camera_sync_status_pub = self.create_publisher(
                String, "/viz/remote/system/hardware/camera_sync_status", 10
            )
            # Audio stats for visualization (synchronized to camera frames)
            if enable_audio_sync:
                self.viz_audio_stats_pub = self.create_publisher(
                    Float32MultiArray, "/viz/remote/audio/stats", 10
                )
            else:
                self.viz_audio_stats_pub = None

        # Synchronized sensor data (for VLM) - deprecated
        try:
            publish_vlm_features = bool(self.get_parameter("publish_vlm_features").value)
        except Exception:
            publish_vlm_features = False

        if publish_vlm_features:
            self.vlm_features_pub = self.create_publisher(String, f"{output_ns}/vlm_features", 10)

        # Status
        self.status_pub = self.create_publisher(String, f"{output_ns}/status", 10)

        # If deferred initialization, don't start timer yet
        if self._defer_init:
            self.get_logger().info(
                "Deferred initialization - publishers created, timer will start after _initialize()"
            )
            return

        # Timer for synchronized publishing
        # OPTIMIZATION: Use longer timer period to reduce CPU usage
        # Timer is fallback - camera callbacks trigger immediate publishing when frames arrive
        if self.sync_to_camera:
            # Will be triggered by camera callbacks, but also use timer as fallback
            # Use slower fallback timer (half frequency) since callbacks handle most publishing
            timer_period = 2.0 / float(self.target_frequency)  # Half frequency fallback
            self.sync_timer = self.create_timer(timer_period, self._publish_synchronized_data)
        else:
            # Fixed rate publishing
            timer_period = 1.0 / float(self.target_frequency)
            self.sync_timer = self.create_timer(timer_period, self._publish_synchronized_data)

        # Track last processing time to skip redundant work
        self.last_processing_time = 0.0
        self.last_camera_frame_time = {}  # Track when each camera last updated

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

    def _initialize(self):
        """Complete initialization after parameters are set (for deferred init)"""
        if not self._defer_init:
            return  # Already initialized

        # Create publishers now that parameters are set
        self._create_publishers()

        # Start timer for synchronized publishing
        self._start_timer()

        self.get_logger().info("Sensor fusion node initialization complete (deferred)")
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

    def _start_timer(self):
        """Start the synchronized publishing timer"""
        # OPTIMIZATION: Use longer timer period to reduce CPU usage
        # Timer is fallback - camera callbacks trigger immediate publishing when frames arrive
        if self.sync_to_camera:
            # Will be triggered by camera callbacks, but also use timer as fallback
            # Use slower fallback timer (half frequency) since callbacks handle most publishing
            timer_period = 2.0 / float(self.target_frequency)  # Half frequency fallback
            self.sync_timer = self.create_timer(timer_period, self._publish_synchronized_data)
        else:
            # Fixed rate publishing
            timer_period = 1.0 / float(self.target_frequency)
            self.sync_timer = self.create_timer(timer_period, self._publish_synchronized_data)

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

    def _audio_callback(self, msg: UInt8MultiArray):
        """Store audio chunk data"""
        timestamp = time.time()
        with self.buffer_lock:
            self.audio_buffer.append({"timestamp": timestamp, "msg": msg})

    # System monitor callbacks
    def _system_status_callback(self, msg: String):
        """Store system status in buffer"""
        with self.buffer_lock:
            timestamp = time.time()
            self.system_status_buffer.append({"timestamp": timestamp, "msg": msg})

    def _system_cpu_temp_callback(self, msg: Temperature):
        """Store CPU temperature in buffer"""
        with self.buffer_lock:
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.system_cpu_temp_buffer.append({"timestamp": timestamp, "msg": msg})

    def _system_gpu_temp_callback(self, msg: Temperature):
        """Store GPU temperature in buffer"""
        with self.buffer_lock:
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.system_gpu_temp_buffer.append({"timestamp": timestamp, "msg": msg})

    def _system_cpu_usage_callback(self, msg: Float32):
        """Store CPU usage in buffer"""
        with self.buffer_lock:
            timestamp = time.time()
            self.system_cpu_usage_buffer.append({"timestamp": timestamp, "msg": msg})

    def _system_gpu_usage_callback(self, msg: Float32):
        """Store GPU usage in buffer"""
        with self.buffer_lock:
            timestamp = time.time()
            self.system_gpu_usage_buffer.append({"timestamp": timestamp, "msg": msg})

    def _system_memory_usage_callback(self, msg: Float32):
        """Store memory usage in buffer"""
        with self.buffer_lock:
            timestamp = time.time()
            self.system_memory_usage_buffer.append({"timestamp": timestamp, "msg": msg})

    def _system_disk_usage_callback(self, msg: Float32):
        """Store disk usage in buffer"""
        with self.buffer_lock:
            timestamp = time.time()
            self.system_disk_usage_buffer.append({"timestamp": timestamp, "msg": msg})

    def _system_power_callback(self, msg: Float32):
        """Store power consumption in buffer"""
        with self.buffer_lock:
            timestamp = time.time()
            self.system_power_buffer.append({"timestamp": timestamp, "msg": msg})

    def _system_alerts_callback(self, msg: String):
        """Store system alerts in buffer"""
        with self.buffer_lock:
            timestamp = time.time()
            self.system_alerts_buffer.append({"timestamp": timestamp, "msg": msg})

    def _camera_callback(self, msg: Image, topic: str):
        """Handle camera frame - latch it and trigger synchronization"""
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        camera_name = topic.split("/")[-3]  # Extract camera name

        # Debug: Log all camera callbacks (first few only)
        if (
            camera_name not in self.last_camera_frame_time
            or self.last_camera_frame_time[camera_name] == 0.0
        ):
            self.get_logger().info(
                f"Camera callback received: {camera_name} from {topic}, "
                f"{msg.width}x{msg.height}, encoding={msg.encoding}"
            )

        with self.buffer_lock:
            self.camera_timestamps.append(
                {"timestamp": timestamp, "topic": topic, "header": msg.header}
            )
            # Store timestamp only (no image data - images come from nvblox)
            self.camera_frame_timestamps[topic] = timestamp

            # NOTE: We don't store image data here - only timestamps
            # Image data comes exclusively from nvblox

        # OPTIMIZATION: Don't trigger publishing from camera callbacks when sync_to_camera=True
        # The timer will handle publishing at the correct rate (15 Hz)
        # Camera callbacks only update latched frames for the timer to use
        # This prevents double-triggering (callbacks + timer) which was causing 44 Hz instead of 15 Hz
        # Only trigger from callbacks if sync_to_camera is False (timer-only mode)
        if not self.sync_to_camera:
            # Fixed rate mode - publish immediately on camera frame
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
        # OPTIMIZATION: Use nvblox images directly instead of raw camera images
        # This eliminates 4 memory copies per frame (nvblox → sensor fusion path)
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self.buffer_lock:
            self.latched_nvblox_images[camera_name] = msg
            self.latched_nvblox_timestamps[camera_name] = timestamp

    def _check_camera_frame_sync(self) -> bool:
        """
        Check if all cameras have timestamps within sync tolerance.
        Uses timestamps from raw camera topics (not image data).
        Returns True if cameras are synced, False otherwise.
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

        # Update camera sync status tracking
        sync_status_str = "synced" if synced else "not_synced"
        if len(timestamps) > 1:
            time_diff = max_ts - min_ts if len(timestamps) > 1 else 0.0
            self.camera_sync_status["status"] = sync_status_str
            self.camera_sync_status["time_diff"] = time_diff
            self.camera_sync_status["tolerance"] = tolerance
            self.camera_sync_status["num_cameras"] = len(timestamps)
        else:
            self.camera_sync_status["status"] = "single_camera"
            self.camera_sync_status["time_diff"] = 0.0
            self.camera_sync_status["tolerance"] = tolerance
            self.camera_sync_status["num_cameras"] = len(timestamps)

        return synced

    def _downsample_image(
        self, img_msg: Image, width: int, height: int, camera_name: Optional[str] = None
    ) -> Image:
        """
        Downsample image to target resolution using shared library.
        Uses caching to avoid redundant processing for same image.
        """
        try:
            # Check if already correct size (avoid unnecessary processing)
            if img_msg.width == width and img_msg.height == height:
                # Already correct size, just update header and return
                img_msg.header.stamp = self.get_clock().now().to_msg()
                return img_msg

            # Get source image timestamp for cache key
            source_timestamp = img_msg.header.stamp.sec + img_msg.header.stamp.nanosec * 1e-9

            # Check cache if camera name provided
            cache_key = None
            if camera_name:
                cache_key = (camera_name, width, height)
                with self.cache_lock:
                    if cache_key in self.downsampled_image_cache:
                        cached_img, cached_timestamp = self.downsampled_image_cache[cache_key]
                        # Reuse cached image if source hasn't changed
                        if abs(source_timestamp - cached_timestamp) < 0.001:  # 1ms tolerance
                            # Update timestamp and return cached image
                            cached_img.header.stamp = self.get_clock().now().to_msg()
                            return cached_img

            # Cache miss or no camera name - perform downsampling using shared library
            downsampled_msg = downsample_image(img_msg, width, height)
            downsampled_msg.header.stamp = self.get_clock().now().to_msg()

            # Cache the result if camera name provided
            if cache_key:
                with self.cache_lock:
                    # Limit cache size to prevent memory growth
                    if len(self.downsampled_image_cache) > 20:
                        # Remove oldest entry (simple FIFO - could be improved)
                        oldest_key = next(iter(self.downsampled_image_cache))
                        del self.downsampled_image_cache[oldest_key]
                    self.downsampled_image_cache[cache_key] = (downsampled_msg, source_timestamp)

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
            # BUT: Continue processing cameras even if IMU is missing
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
                # DON'T return here - continue to process cameras even without IMU
                # Skip IMU processing but continue with camera processing
                imu_data = None  # Ensure it stays None for later checks

            # Process IMU data if available
            if imu_data is not None:
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
                    filtered_imu = Imu()
                    filtered_imu.header = Header()
                    filtered_imu.header.stamp = self.get_clock().now().to_msg()
                    filtered_imu.header.frame_id = imu_data["msg"].header.frame_id
                    filtered_imu.linear_acceleration = imu_data["msg"].linear_acceleration
                    filtered_imu.angular_velocity = imu_data["msg"].angular_velocity
                    filtered_imu.linear_acceleration_covariance = imu_data[
                        "msg"
                    ].linear_acceleration_covariance
                    filtered_imu.angular_velocity_covariance = imu_data[
                        "msg"
                    ].angular_velocity_covariance

                # Publish filtered IMU (feature topic)
                self.filtered_imu_pub.publish(filtered_imu)

                # Publish viz IMU (if enabled and time for viz publish)
                if self.publish_viz_topics:
                    if "imu" not in self.last_viz_publish_time or (
                        current_time - self.last_viz_publish_time["imu"]
                    ) >= (1.0 / self.viz_frequency):
                        self.viz_imu_pub.publish(filtered_imu)
                        self.last_viz_publish_time["imu"] = current_time
            # If no IMU data, skip IMU publishing (already published empty placeholder above if viz enabled)

            # Find closest chassis data
            battery_data = self._find_closest_sensor_data(self.battery_buffer, sync_timestamp)
            status_data = self._find_closest_sensor_data(self.status_buffer, sync_timestamp)

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
            # OPTIMIZATION: Use nvblox images directly instead of raw camera images
            # This eliminates 4 memory copies per frame (nvblox → sensor fusion path)
            # Raw camera frames are still subscribed for sync checking only

            # Use nvblox images for downsampling (more efficient - eliminates copies)
            cameras_to_process = {}

            # Get nvblox images (prefer these over raw camera images)
            for camera_name in self.latched_nvblox_images.keys():
                nvblox_img = self.latched_nvblox_images.get(camera_name)
                if nvblox_img is not None:
                    cameras_to_process[camera_name] = nvblox_img

            # No fallback logic - cameras must come from nvblox
            # If a camera is missing, it means nvblox isn't processing it
            # This is intentional: nvblox is the single source of truth for processed camera data
            if not cameras_to_process:
                # Warn if no nvblox images available (but don't crash)
                if current_time - getattr(self, "_last_nvblox_warning_time", 0) > 5.0:
                    self.get_logger().warn(
                        f"No nvblox images available for processing. "
                        f"Latched images: {list(self.latched_nvblox_images.keys())}, "
                        f"Values: {[k for k, v in self.latched_nvblox_images.items() if v is not None]}"
                    )
                    self._last_nvblox_warning_time = current_time

            # Process each camera (nvblox images only)
            for camera_name, camera_msg in cameras_to_process.items():
                if camera_msg is None:
                    continue

                # Publish feature topics if camera is in feature_camera_pubs
                if camera_name in self.feature_camera_pubs:
                    # OPTIMIZATION: Use caching to avoid redundant downsampling
                    # Downsample for feature topics (with caching)
                    feature_img = self._downsample_image(
                        camera_msg, self.feature_image_width, self.feature_image_height, camera_name
                    )
                    feature_img.header.stamp = self.get_clock().now().to_msg()
                    self.feature_camera_pubs[camera_name].publish(feature_img)

                # Publish viz topics for front/rear cameras (regardless of feature_camera_pubs)
                # This ensures viz topics always publish even if feature topics don't
                if self.publish_viz_topics and camera_name in ["camera_front", "camera_rear"]:
                    if camera_name not in self.last_viz_publish_time or (
                        current_time - self.last_viz_publish_time[camera_name]
                    ) >= (1.0 / self.viz_frequency):
                        try:
                            viz_img = self._downsample_image(
                                camera_msg,
                                self.viz_resolution_width,
                                self.viz_resolution_height,
                                camera_name,
                            )
                            viz_img.header.stamp = self.get_clock().now().to_msg()
                            if camera_name == "camera_front":
                                self.viz_camera_front_pub.publish(viz_img)
                            elif camera_name == "camera_rear":
                                self.viz_camera_rear_pub.publish(viz_img)
                            self.last_viz_publish_time[camera_name] = current_time
                        except Exception as e:
                            self.get_logger().warn(
                                f"Failed to publish viz for {camera_name} from cameras_to_process: {e}"
                            )

            # No fallback logic - cameras must come from nvblox
            # If a camera is missing, it means nvblox isn't processing it
            # This is intentional: nvblox is the single source of truth for processed camera data

            # Always publish viz topics for both cameras, even if blank
            # This ensures downstream consumers always receive data on schedule
            if self.publish_viz_topics:
                # Publish front camera (blank if not in cameras_to_process)
                if "camera_front" not in cameras_to_process:
                    if "camera_front" not in self.last_viz_publish_time or (
                        current_time - self.last_viz_publish_time.get("camera_front", 0)
                    ) >= (1.0 / self.viz_frequency):
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
                
                # Publish rear camera (blank if not in cameras_to_process)
                if "camera_rear" not in cameras_to_process:
                    if "camera_rear" not in self.last_viz_publish_time or (
                        current_time - self.last_viz_publish_time.get("camera_rear", 0)
                    ) >= (1.0 / self.viz_frequency):
                        blank_img = Image()
                        blank_img.header.stamp = self.get_clock().now().to_msg()
                        blank_img.header.frame_id = "camera_rear_optical_frame"
                        blank_img.height = self.viz_resolution_height
                        blank_img.width = self.viz_resolution_width
                        blank_img.encoding = "rgb8"
                        blank_img.is_bigendian = False
                        blank_img.step = self.viz_resolution_width * 3
                        blank_img.data = bytes(
                            self.viz_resolution_width * self.viz_resolution_height * 3
                        )
                        self.viz_camera_rear_pub.publish(blank_img)
                        self.last_viz_publish_time["camera_rear"] = current_time

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

            # Publish synchronized system status
            self._publish_system_status(sync_timestamp, current_time)

            # Publish synchronized audio (raw for features, stats for viz)
            if self.synced_audio_pub is not None:
                self._publish_audio(sync_timestamp, current_time)

            # NOTE: VLA feature building is handled by a separate node that consumes
            # the synchronized sensor data published here. The publish_vlm_features
            # parameter is deprecated and will be removed in a future version.
            if self.get_parameter("publish_vlm_features").value:
                # Only publish VLM features if we have IMU data
                if imu_data is not None:
                    self._publish_vlm_features(
                        filtered_imu, sync_timestamp, battery_data, status_data
                    )

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

    def _publish_system_status(self, sync_timestamp: float, current_time: float):
        """Publish synchronized system status (feature topics and viz topics)"""
        # Find closest system status data
        system_status_data = self._find_closest_sensor_data(
            self.system_status_buffer, sync_timestamp
        )
        cpu_temp_data = self._find_closest_sensor_data(self.system_cpu_temp_buffer, sync_timestamp)
        gpu_temp_data = self._find_closest_sensor_data(self.system_gpu_temp_buffer, sync_timestamp)
        cpu_usage_data = self._find_closest_sensor_data(
            self.system_cpu_usage_buffer, sync_timestamp
        )
        gpu_usage_data = self._find_closest_sensor_data(
            self.system_gpu_usage_buffer, sync_timestamp
        )
        memory_usage_data = self._find_closest_sensor_data(
            self.system_memory_usage_buffer, sync_timestamp
        )
        disk_usage_data = self._find_closest_sensor_data(
            self.system_disk_usage_buffer, sync_timestamp
        )
        power_data = self._find_closest_sensor_data(self.system_power_buffer, sync_timestamp)
        alerts_data = self._find_closest_sensor_data(self.system_alerts_buffer, sync_timestamp)

        # Publish feature topics (for controller) - synchronized with sensor data
        if system_status_data:
            synced_status = String()
            synced_status.data = system_status_data["msg"].data
            self.system_status_pub.publish(synced_status)

        if cpu_temp_data:
            synced_cpu_temp = Temperature()
            synced_cpu_temp.header.stamp = self.get_clock().now().to_msg()
            synced_cpu_temp.header.frame_id = cpu_temp_data["msg"].header.frame_id
            synced_cpu_temp.temperature = cpu_temp_data["msg"].temperature
            synced_cpu_temp.variance = cpu_temp_data["msg"].variance
            self.system_cpu_temp_pub.publish(synced_cpu_temp)

        if gpu_temp_data:
            synced_gpu_temp = Temperature()
            synced_gpu_temp.header.stamp = self.get_clock().now().to_msg()
            synced_gpu_temp.header.frame_id = gpu_temp_data["msg"].header.frame_id
            synced_gpu_temp.temperature = gpu_temp_data["msg"].temperature
            synced_gpu_temp.variance = gpu_temp_data["msg"].variance
            self.system_gpu_temp_pub.publish(synced_gpu_temp)

        if cpu_usage_data:
            synced_cpu_usage = Float32()
            synced_cpu_usage.data = cpu_usage_data["msg"].data
            self.system_cpu_usage_pub.publish(synced_cpu_usage)

        if gpu_usage_data:
            synced_gpu_usage = Float32()
            synced_gpu_usage.data = gpu_usage_data["msg"].data
            self.system_gpu_usage_pub.publish(synced_gpu_usage)

        if memory_usage_data:
            synced_memory_usage = Float32()
            synced_memory_usage.data = memory_usage_data["msg"].data
            self.system_memory_usage_pub.publish(synced_memory_usage)

        if disk_usage_data:
            synced_disk_usage = Float32()
            synced_disk_usage.data = disk_usage_data["msg"].data
            self.system_disk_usage_pub.publish(synced_disk_usage)

        if power_data:
            synced_power = Float32()
            synced_power.data = power_data["msg"].data
            self.system_power_pub.publish(synced_power)

        if alerts_data:
            synced_alerts = String()
            synced_alerts.data = alerts_data["msg"].data
            self.system_alerts_pub.publish(synced_alerts)

        # Publish camera sync status
        sync_status_msg = String()
        sync_status_json = {
            "status": self.camera_sync_status.get("status", "unknown"),
            "time_diff": self.camera_sync_status.get("time_diff", 0.0),
            "tolerance": self.camera_sync_status.get("tolerance", 0.05),
            "num_cameras": self.camera_sync_status.get("num_cameras", 0),
        }
        sync_status_msg.data = json.dumps(sync_status_json)
        self.system_camera_sync_status_pub.publish(sync_status_msg)

        # Publish viz topics (lower frequency, same data)
        if self.publish_viz_topics:
            # Publish camera sync status to viz topic
            if "camera_sync_status" not in self.last_viz_publish_time or (
                current_time - self.last_viz_publish_time.get("camera_sync_status", 0)
            ) >= (1.0 / self.viz_frequency):
                self.viz_system_camera_sync_status_pub.publish(sync_status_msg)
                self.last_viz_publish_time["camera_sync_status"] = current_time
            # Check if it's time to publish viz system status
            if "system_status" not in self.last_viz_publish_time or (
                current_time - self.last_viz_publish_time["system_status"]
            ) >= (1.0 / self.viz_frequency):
                if system_status_data:
                    self.viz_system_status_pub.publish(synced_status)

                if cpu_temp_data:
                    self.viz_system_cpu_temp_pub.publish(synced_cpu_temp)

                if gpu_temp_data:
                    self.viz_system_gpu_temp_pub.publish(synced_gpu_temp)

                if cpu_usage_data:
                    self.viz_system_cpu_usage_pub.publish(synced_cpu_usage)

                if gpu_usage_data:
                    synced_gpu_usage = Float32()
                    synced_gpu_usage.data = gpu_usage_data["msg"].data
                    self.viz_system_gpu_usage_pub.publish(synced_gpu_usage)

                if memory_usage_data:
                    self.viz_system_memory_usage_pub.publish(synced_memory_usage)

                if alerts_data:
                    self.viz_system_alerts_pub.publish(synced_alerts)

                self.last_viz_publish_time["system_status"] = current_time

    def _find_overlapping_audio_chunks(
        self, frame_timestamp: float, frame_duration: float
    ) -> Optional[list]:
        """
        Find audio chunks that overlap with the camera frame timestamp.
        Returns list of audio chunks that cover the frame duration.
        """
        if len(self.audio_buffer) == 0:
            return None

        frame_start = frame_timestamp - frame_duration
        frame_end = frame_timestamp

        overlapping_chunks = []
        with self.buffer_lock:
            # Calculate chunk duration based on sample rate and chunk size
            # Each chunk is typically 1024 bytes = 512 samples (S16_LE, 2 bytes per sample)
            # For stereo: 512 samples = 256 samples per channel
            # Duration = bytes / (sample_rate * channels * bytes_per_sample)
            # bytes_per_sample = 2 for S16_LE
            bytes_per_sample = 2
            chunk_duration = 1024.0 / (
                self.audio_sample_rate * self.audio_channels * bytes_per_sample
            )

            for audio_data in self.audio_buffer:
                chunk_timestamp = audio_data["timestamp"]
                # Estimate chunk duration from actual data size
                actual_chunk_size = len(audio_data["msg"].data)
                actual_chunk_duration = actual_chunk_size / (
                    self.audio_sample_rate * self.audio_channels * bytes_per_sample
                )
                chunk_start = chunk_timestamp - actual_chunk_duration
                chunk_end = chunk_timestamp

                # Check if chunk overlaps with frame window (with tolerance)
                if chunk_end >= (frame_start - self.audio_sync_tolerance) and chunk_start <= (
                    frame_end + self.audio_sync_tolerance
                ):
                    overlapping_chunks.append(audio_data)

        if not overlapping_chunks:
            return None

        # Sort by timestamp
        overlapping_chunks.sort(key=lambda x: x["timestamp"])
        return overlapping_chunks

    def _resample_audio_chunks(
        self, chunks: list, frame_timestamp: float, frame_duration: float
    ) -> Optional[UInt8MultiArray]:
        """
        Resample/concatenate audio chunks to match camera frame duration.
        Returns synchronized audio chunk or None if no valid chunks.
        """
        if not chunks:
            return None

        try:
            # Calculate target audio size for frame duration
            # Bytes per second = sample_rate * channels * bytes_per_sample
            bytes_per_second = self.audio_sample_rate * self.audio_channels * 2  # S16_LE = 2 bytes
            target_bytes = int(frame_duration * bytes_per_second)

            # Concatenate chunks
            audio_data_list = []
            for chunk_data in chunks:
                if "msg" in chunk_data and hasattr(chunk_data["msg"], "data"):
                    audio_data_list.extend(chunk_data["msg"].data)

            if not audio_data_list:
                return None

            # If we have more data than needed, truncate
            if len(audio_data_list) > target_bytes:
                audio_data_list = audio_data_list[:target_bytes]
            # If we have less data than needed, pad with zeros
            elif len(audio_data_list) < target_bytes:
                audio_data_list.extend([0] * (target_bytes - len(audio_data_list)))

            # Create synchronized audio message
            synced_audio = UInt8MultiArray()
            synced_audio.data = audio_data_list
            # Set layout dimensions
            # Third-party
            from std_msgs.msg import MultiArrayDimension

            dim = MultiArrayDimension()
            dim.label = "audio"
            dim.size = len(audio_data_list)
            dim.stride = 1
            synced_audio.layout.dim = [dim]
            synced_audio.layout.data_offset = 0

            return synced_audio
        except Exception as e:
            self.get_logger().warn(f"Error resampling audio chunks: {e}")
            return None

    def _calculate_audio_stats(self, audio_msg: UInt8MultiArray) -> tuple:
        """
        Calculate max volume per channel from raw audio chunk.
        Returns (max_volume_left, max_volume_right) normalized to 0-1.
        """
        try:
            # Convert bytes to numpy array
            audio_bytes = np.frombuffer(bytes(audio_msg.data), dtype=np.uint8)

            if len(audio_bytes) < 2:
                return 0.0, 0.0

            # Convert to 16-bit signed integers (S16_LE format)
            samples = np.frombuffer(audio_bytes, dtype=np.int16)

            # De-interleave stereo channels
            if self.audio_channels == 2:
                left_samples = samples[0::2]  # Every other sample starting at 0
                right_samples = samples[1::2]  # Every other sample starting at 1
            else:
                # Mono - use same data for both
                left_samples = samples
                right_samples = samples

            # Normalize to -1.0 to 1.0 range (int16 max is 32767)
            left_normalized = left_samples.astype(np.float32) / 32768.0
            right_normalized = right_samples.astype(np.float32) / 32768.0

            # Calculate max amplitude per channel
            max_left = float(np.max(np.abs(left_normalized)))
            max_right = float(np.max(np.abs(right_normalized)))

            return max_left, max_right
        except Exception as e:
            self.get_logger().debug(f"Error calculating audio stats: {e}")
            return 0.0, 0.0

    def _publish_audio(self, sync_timestamp: float, current_time: float):
        """Publish synchronized audio (raw for features, stats for viz)"""
        if not self.synced_audio_pub:
            return

        # Check if audio buffer has data
        if len(self.audio_buffer) == 0:
            return

        try:
            # Calculate frame duration (inverse of target frequency)
            frame_duration = 1.0 / self.target_frequency

            # Find overlapping audio chunks
            overlapping_chunks = self._find_overlapping_audio_chunks(sync_timestamp, frame_duration)

            if not overlapping_chunks:
                return

            # Resample/concatenate chunks to match frame duration
            synced_audio = self._resample_audio_chunks(
                overlapping_chunks, sync_timestamp, frame_duration
            )

            if not synced_audio:
                return

            # Calculate stats from raw audio
            max_volume_left, max_volume_right = self._calculate_audio_stats(synced_audio)

            # Publish raw audio for features (synchronized to camera frame)
            # Note: UInt8MultiArray doesn't have header, timestamp is implicit in sync
            self.synced_audio_pub.publish(synced_audio)

            # Publish stats for visualization (same chunk, low-res stats)
            if self.viz_audio_stats_pub and self.publish_viz_topics:
                stats_msg = Float32MultiArray()
                stats_msg.data = [max_volume_left, max_volume_right]
                # Set layout dimensions
                dim = MultiArrayDimension()
                dim.label = "audio_stats"
                dim.size = 2
                dim.stride = 1
                stats_msg.layout.dim.append(dim)
                stats_msg.layout.data_offset = 0
                self.viz_audio_stats_pub.publish(stats_msg)
        except Exception as e:
            self.get_logger().warn(f"Error publishing synchronized audio: {e}")


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
