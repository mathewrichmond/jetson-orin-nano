#!/usr/bin/env python3
"""
RealSense Camera Node
Publishes color and depth images from Intel RealSense cameras
Supports multiple cameras with configurable namespaces
"""

# Standard library
from collections import deque
import threading
import time
from typing import Dict, List, Optional

# Third-party
from cv_bridge import CvBridge
import numpy as np
import pyrealsense2 as rs
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import String

# Local
from isaac_utils import CameraFrameBuffer


class RealSenseCameraNode(Node):
    """ROS 2 node for Intel RealSense depth cameras"""

    def __init__(
        self,
        defer_init: bool = False,
        node_name: Optional[str] = None,
        namespace: Optional[str] = None,
    ):
        # Use provided node_name or default
        actual_node_name = node_name if node_name else "realsense_camera_node"
        # ROS 2 Node constructor accepts namespace parameter
        if namespace:
            super().__init__(actual_node_name, namespace=namespace)
        else:
            super().__init__(actual_node_name)

        # Parameters
        self.declare_parameter("camera_serial_numbers", [])
        self.declare_parameter("camera_names", ["camera_front", "camera_rear"])
        self.declare_parameter("enable_color", True)
        self.declare_parameter("enable_depth", True)
        self.declare_parameter("enable_pointcloud", False)
        self.declare_parameter("color_width", 640)
        self.declare_parameter("color_height", 480)
        self.declare_parameter("color_fps", 30)
        self.declare_parameter("depth_width", 640)
        self.declare_parameter("depth_height", 480)
        self.declare_parameter("depth_fps", 30)
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("align_depth_to_color", True)
        self.declare_parameter("status_topic", "realsense/status")
        self.declare_parameter("frame_timeout_ms", 1000)  # Milliseconds to wait for frames
        self.declare_parameter("shutdown_delay", 0.5)  # Seconds to wait for threads on shutdown

        # Inter-camera sync configuration (firmware-based)
        self.declare_parameter(
            "enable_inter_cam_sync", False
        )  # Enable firmware-based inter-camera sync
        self.declare_parameter("inter_cam_sync_mode", 0)  # 0=None, 1=Master, 2=Slave
        # If multiple cameras, first is master, others are slaves
        self.declare_parameter("inter_cam_sync_master_serial", "")
        self.declare_parameter("sync_status_interval_sec", 5.0)
        self.declare_parameter("sync_status_tolerance_ms", 5.0)

        # Initialize basic structures
        self.bridge = CvBridge()
        self.pipelines: Dict[str, rs.pipeline] = {}
        self.configs: Dict[str, rs.config] = {}
        self.aligns: Dict[str, rs.align] = {}
        self.camera_infos: Dict[str, Dict] = {}
        self.running = True
        self.last_sync_status_time = 0.0
        self.camera_threads: List[threading.Thread] = []
        self.frame_queues: Dict[str, deque] = {}
        self.timer = None
        self.status_pub = None
        self._initialized = False
        
        # Shared memory buffer for zero-copy frame passing
        self.frame_buffer = CameraFrameBuffer.get_instance()
        self.get_logger().info("Using shared CameraFrameBuffer for zero-copy frame passing")

        # If defer_init is False (normal case), initialize immediately
        # If True (composable container case), wait for parameters to be set
        if not defer_init:
            self._initialize_cameras()

    def _initialize_cameras(self):
        """Initialize cameras - can be called after parameters are set"""
        if self._initialized:
            self.get_logger().warn("Cameras already initialized, skipping")
            return

        # Get parameters (with safe fallback to defaults)
        try:
            self.camera_serials = self.get_parameter("camera_serial_numbers").value
        except Exception:
            self.camera_serials = []

        try:
            self.camera_names = self.get_parameter("camera_names").value
        except Exception:
            self.camera_names = ["camera_front", "camera_rear"]
        self.enable_color = self.get_parameter("enable_color").value
        self.enable_depth = self.get_parameter("enable_depth").value
        self.enable_pointcloud = self.get_parameter("enable_pointcloud").value
        self.color_width = self.get_parameter("color_width").value
        self.color_height = self.get_parameter("color_height").value
        self.color_fps = self.get_parameter("color_fps").value
        self.depth_width = self.get_parameter("depth_width").value
        self.depth_height = self.get_parameter("depth_height").value
        self.depth_fps = self.get_parameter("depth_fps").value
        self.publish_rate = self.get_parameter("publish_rate").value
        self.align_depth_to_color = self.get_parameter("align_depth_to_color").value
        self.status_topic = self.get_parameter("status_topic").value
        self.frame_timeout_ms = self.get_parameter("frame_timeout_ms").value
        self.shutdown_delay = self.get_parameter("shutdown_delay").value
        self.enable_inter_cam_sync = self.get_parameter("enable_inter_cam_sync").value
        self.inter_cam_sync_mode = self.get_parameter("inter_cam_sync_mode").value
        self.inter_cam_sync_master_serial = str(
            self.get_parameter("inter_cam_sync_master_serial").value
        ).strip()
        self.sync_status_interval_sec = float(self.get_parameter("sync_status_interval_sec").value)
        self.sync_status_tolerance_ms = float(self.get_parameter("sync_status_tolerance_ms").value)

        # Status publisher
        self.status_pub = self.create_publisher(String, self.status_topic, 10)

        # Discover and initialize cameras (with retry logic)
        max_retries = 3
        retry_delay = 1.0

        for attempt in range(max_retries):
            self._discover_cameras()
            if len(self.pipelines) > 0:
                break
            if attempt < max_retries - 1:
                self.get_logger().warn(
                    f"No cameras initialized (attempt {attempt + 1}/{max_retries}), retrying in {retry_delay}s..."
                )
                time.sleep(retry_delay)
            else:
                self.get_logger().error(
                    f"Failed to initialize any cameras after {max_retries} attempts"
                )
                self.publish_status("error", "No cameras detected after retries")

        # Only proceed if we have cameras
        if len(self.pipelines) == 0:
            self.get_logger().error("Cannot start camera node: No cameras initialized")
            self._initialized = True  # Mark as initialized even if failed, to prevent retries
            return

        # Create publishers for each camera
        self._create_publishers()

        # Frame queues for each camera (thread-safe communication)
        # Increased maxlen to prevent frame drops when publishing is slower than capture
        # At 15 Hz publish rate, we need at least 2-3 frames buffer for 30 FPS capture
        self.frame_queues: Dict[str, deque] = {}
        queue_size = max(
            5, int(self.color_fps / self.publish_rate) + 2
        )  # Buffer for 2+ publish cycles
        for camera_name in self.pipelines.keys():
            self.frame_queues[camera_name] = deque(maxlen=queue_size)
            self.get_logger().info(f"Frame queue for {camera_name}: maxlen={queue_size}")

        # Start camera capture threads (only for frame capture, not publishing)
        self.camera_threads: List[threading.Thread] = []

        for camera_name in self.pipelines.keys():
            thread = threading.Thread(
                target=self._camera_capture_loop, args=(camera_name,), daemon=True
            )
            thread.start()
            self.camera_threads.append(thread)

        # Create timer for publishing (runs on main thread)
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self._publish_frames)
        self.get_logger().info(
            f"Created publishing timer with period {timer_period:.3f}s (rate: {self.publish_rate} Hz)"
        )
        self.get_logger().info(
            f"Timer object: {self.timer}, period_ns: {self.timer.timer_period_ns if self.timer else 'None'}"
        )
        self._publish_call_count = 0  # Debug counter
        self._last_publish_time = time.time()  # Track actual publish timing

        self.get_logger().info(
            f"RealSense Camera Node started with {len(self.pipelines)} camera(s)"
        )
        self.publish_status("initialized", f"Started with {len(self.pipelines)} camera(s)")
        self._initialized = True

    def _check_camera_availability(self, serial: str) -> tuple[bool, str]:
        """Check if a camera is available for use

        Returns:
            (is_available, error_message) tuple
        """
        try:
            ctx = rs.context()
            devices = ctx.query_devices()

            # Find the device
            device = None
            for dev in devices:
                if dev.get_info(rs.camera_info.serial_number) == serial:
                    device = dev
                    break

            if device is None:
                return False, f"Camera with serial {serial} not found"

            # Try to create a test pipeline to check if device is available
            test_pipeline = rs.pipeline()
            test_config = rs.config()
            test_config.enable_device(serial)
            test_config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 15)

            try:
                profile = test_pipeline.start(test_config)
                test_pipeline.stop()
                return True, ""
            except RuntimeError as e:
                error_str = str(e)
                if "Device or resource busy" in error_str or "errno=16" in error_str:
                    return False, f"Camera {serial} is currently in use by another process"
                else:
                    return False, f"Camera {serial} initialization test failed: {error_str}"
        except Exception as e:
            return False, f"Error checking camera availability: {e}"

    def _discover_cameras(self):
        """Discover available RealSense cameras with resource availability checking"""
        ctx = rs.context()
        devices = ctx.query_devices()

        if len(devices) == 0:
            self.get_logger().error("No RealSense devices found!")
            self.publish_status("error", "No cameras detected")
            return

        self.get_logger().info(f"Found {len(devices)} RealSense device(s)")

        # If serial numbers specified, use those; otherwise use all devices
        available_serials = [dev.get_info(rs.camera_info.serial_number) for dev in devices]

        if self.camera_serials and len(self.camera_serials) > 0:
            # Use specified serials
            target_serials = [s for s in self.camera_serials if s in available_serials]
            if len(target_serials) != len(self.camera_serials):
                missing = set(self.camera_serials) - set(available_serials)
                self.get_logger().warn(f"Some requested cameras not found: {missing}")
        else:
            # Use all available cameras
            target_serials = available_serials[: len(self.camera_names)]

        if len(target_serials) == 0:
            self.get_logger().error("No matching cameras found")
            self.publish_status("error", "No matching cameras found")
            return

        # Check availability of all cameras before initializing
        unavailable_cameras = []
        for serial in target_serials:
            is_available, error_msg = self._check_camera_availability(serial)
            if not is_available:
                unavailable_cameras.append((serial, error_msg))
                self.get_logger().error(f"Camera {serial} unavailable: {error_msg}")

        if unavailable_cameras:
            error_summary = "; ".join([f"{s}: {msg}" for s, msg in unavailable_cameras])
            self.get_logger().error(
                f"Cannot initialize cameras - {len(unavailable_cameras)} camera(s) unavailable: {error_summary}"
            )
            self.publish_status("error", f"Cameras unavailable: {error_summary}")
            # Don't return - try to initialize available cameras

        # Initialize cameras
        master_serial = None
        if self.enable_inter_cam_sync and self.inter_cam_sync_master_serial:
            if self.inter_cam_sync_master_serial in available_serials:
                master_serial = self.inter_cam_sync_master_serial
            else:
                self.get_logger().warn(
                    "Inter-camera sync master serial not found; using first camera as master"
                )

        # Log camera name mapping for identification
        self.get_logger().info("Camera name mapping:")
        initialized_count = 0
        failed_count = 0

        for i, serial in enumerate(target_serials):
            if serial not in available_serials:
                self.get_logger().warn(f"Camera with serial {serial} not found")
                failed_count += 1
                continue

            if i >= len(self.camera_names):
                camera_name = f"camera_{i}"
            else:
                camera_name = self.camera_names[i]

            # Skip if we already know this camera is unavailable
            if any(s == serial for s, _ in unavailable_cameras):
                failed_count += 1
                self.get_logger().info(
                    f"  {camera_name} → Serial: {serial} (SKIPPED - unavailable)"
                )
                continue

            self.get_logger().info(f"  {camera_name} → Serial: {serial}")

            # Determine sync mode: first camera is master (1), others are slaves (2)
            if self.enable_inter_cam_sync:
                if master_serial:
                    sync_mode = 1 if serial == master_serial else 2
                else:
                    sync_mode = 1 if i == 0 else 2  # First camera = master, others = slaves
            else:
                sync_mode = 0  # No sync

            try:
                self._initialize_camera(
                    camera_name, serial, devices[available_serials.index(serial)], sync_mode
                )
                initialized_count += 1
            except Exception as e:
                failed_count += 1
                error_msg = str(e)
                if "Device or resource busy" in error_msg or "errno=16" in error_msg:
                    self.get_logger().error(
                        f"FAILED to initialize camera {camera_name} ({serial}): Device is in use by another process. "
                        f"Please stop other processes using this camera (check with: lsof | grep video) and retry."
                    )
                else:
                    self.get_logger().error(
                        f"FAILED to initialize camera {camera_name} ({serial}): {error_msg}"
                    )
                self.publish_status("error", f"Failed to initialize {camera_name}: {error_msg}")

        # Summary
        if initialized_count == 0:
            self.get_logger().error(
                f"CRITICAL: Failed to initialize any cameras ({failed_count} failed, {len(target_serials)} total)"
            )
        elif failed_count > 0:
            self.get_logger().warn(
                f"Partially initialized: {initialized_count} camera(s) initialized, {failed_count} failed"
            )

    def _initialize_camera(
        self,
        camera_name: str,
        serial: str,
        device: rs.device,
        sync_mode: int = 0,
        retry_count: int = 3,
    ):
        """Initialize a single camera

        Args:
            camera_name: Name for the camera
            serial: Camera serial number
            device: RealSense device object
            sync_mode: Inter-camera sync mode (0=None, 1=Master, 2=Slave)
            retry_count: Number of retry attempts if initialization fails

        Raises:
            RuntimeError: If camera initialization fails after all retries
        """
        self.get_logger().info(f"Initializing camera: {camera_name} (serial: {serial})")

        # Check availability first
        is_available, error_msg = self._check_camera_availability(serial)
        if not is_available:
            raise RuntimeError(f"Camera {serial} unavailable: {error_msg}")

        # Configure inter-camera sync (firmware-based, requires physical sync cables)
        # Set mode even when 0 to clear any persistent slave/master state.
        try:
            sensor = device.first_depth_sensor()
            if sensor.supports(rs.option.inter_cam_sync_mode):
                sensor.set_option(rs.option.inter_cam_sync_mode, sync_mode)
                if sync_mode == 0:
                    sync_mode_name = "Disabled"
                else:
                    sync_mode_name = "Master" if sync_mode == 1 else "Slave"
                self.get_logger().info(
                    f"Configured {camera_name} for inter-camera sync: {sync_mode_name}"
                )
            else:
                self.get_logger().warn(f"Camera {camera_name} does not support inter-camera sync")
        except Exception as e:
            self.get_logger().warn(f"Failed to configure inter-camera sync for {camera_name}: {e}")
            self.get_logger().info(
                "Note: Inter-camera sync requires physical sync cables between cameras"
            )

        # Create pipeline and config
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(serial)

        # Configure streams
        if self.enable_color:
            config.enable_stream(
                rs.stream.color, self.color_width, self.color_height, rs.format.rgb8, self.color_fps
            )

        if self.enable_depth:
            config.enable_stream(
                rs.stream.depth, self.depth_width, self.depth_height, rs.format.z16, self.depth_fps
            )

        # Start pipeline (with retry for device busy errors)
        profile = None
        last_error = None
        for attempt in range(retry_count):
            try:
                profile = pipeline.start(config)
                self.get_logger().info(f"✓ Successfully started pipeline for {camera_name}")
                break  # Success
            except RuntimeError as e:
                error_str = str(e)
                if "Device or resource busy" in error_str or "errno=16" in error_str:
                    last_error = e
                    if attempt < retry_count - 1:
                        wait_time = 0.5 * (attempt + 1)  # Exponential backoff: 0.5s, 1.0s, 1.5s
                        self.get_logger().warn(
                            f"Camera {camera_name} (serial: {serial}) is busy - attempt {attempt + 1}/{retry_count}. "
                            f"Another process may be using this camera. Waiting {wait_time}s before retry..."
                        )
                        time.sleep(wait_time)
                        # Try to stop any existing pipeline before retry
                        try:
                            pipeline.stop()
                        except:
                            pass
                        # Create a new pipeline for retry
                        pipeline = rs.pipeline()
                        config.enable_device(serial)
                        if self.enable_color:
                            config.enable_stream(
                                rs.stream.color,
                                self.color_width,
                                self.color_height,
                                rs.format.rgb8,
                                self.color_fps,
                            )
                        if self.enable_depth:
                            config.enable_stream(
                                rs.stream.depth,
                                self.depth_width,
                                self.depth_height,
                                rs.format.z16,
                                self.depth_fps,
                            )
                    else:
                        # Last attempt failed - provide clear error message
                        error_msg = (
                            f"Camera {camera_name} (serial: {serial}) failed to initialize after {retry_count} attempts. "
                            f"Device is in use by another process. "
                            f"Please stop other processes using this camera (check with: lsof | grep video) "
                            f"or wait for them to release the device."
                        )
                        self.get_logger().error(error_msg)
                        raise RuntimeError(error_msg)
                else:
                    # Not a device busy error, re-raise immediately with context
                    error_msg = f"Camera {camera_name} (serial: {serial}) initialization failed: {error_str}"
                    self.get_logger().error(error_msg)
                    raise RuntimeError(error_msg)

        if profile is None:
            error_msg = f"Failed to start pipeline for {camera_name} after {retry_count} attempts: {last_error}"
            self.get_logger().error(error_msg)
            raise RuntimeError(error_msg)

        # Get camera intrinsics
        if camera_name not in self.camera_infos:
            self.camera_infos[camera_name] = {}

        if self.enable_color:
            color_profile = rs.video_stream_profile(profile.get_stream(rs.stream.color))
            color_intrinsics = color_profile.get_intrinsics()
            self.camera_infos[camera_name]["color"] = self._intrinsics_to_camera_info(
                color_intrinsics, camera_name, "color"
            )

        if self.enable_depth:
            depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
            depth_intrinsics = depth_profile.get_intrinsics()
            self.camera_infos[camera_name]["depth"] = self._intrinsics_to_camera_info(
                depth_intrinsics, camera_name, "depth"
            )

        # Create align object if needed
        align = None
        if self.align_depth_to_color and self.enable_color and self.enable_depth:
            align = rs.align(rs.stream.color)

        self.pipelines[camera_name] = pipeline
        self.configs[camera_name] = config
        self.aligns[camera_name] = align

        self.get_logger().info(f"Camera {camera_name} initialized successfully")

    def _intrinsics_to_camera_info(
        self, intrinsics, camera_name: str, stream_type: str
    ) -> CameraInfo:
        """Convert RealSense intrinsics to ROS CameraInfo"""
        info = CameraInfo()
        info.header.frame_id = f"{camera_name}_{stream_type}_optical_frame"
        info.width = intrinsics.width
        info.height = intrinsics.height
        info.distortion_model = "plumb_bob"

        # Camera matrix (3x3 row-major)
        info.k = [
            intrinsics.fx,
            0.0,
            intrinsics.ppx,
            0.0,
            intrinsics.fy,
            intrinsics.ppy,
            0.0,
            0.0,
            1.0,
        ]

        # Distortion coefficients
        info.d = list(intrinsics.coeffs[:5])

        # Rectification matrix (identity)
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

        # Projection matrix
        info.p = [
            intrinsics.fx,
            0.0,
            intrinsics.ppx,
            0.0,
            0.0,
            intrinsics.fy,
            intrinsics.ppy,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ]

        return info

    def _create_publishers(self):
        """Create ROS publishers for each camera"""
        self.camera_publishers = {}

        # Use BEST_EFFORT QoS for image topics to prevent blocking during publish
        # This is critical for real-time performance - RELIABLE QoS can block for seconds
        # waiting for subscriber acknowledgment on large messages (images)
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,  # Smaller queue for faster throughput
        )

        # Use RELIABLE QoS for camera info (small messages)
        info_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10
        )

        for camera_name in self.pipelines.keys():
            self.camera_publishers[camera_name] = {}

            if self.enable_color:
                self.camera_publishers[camera_name]["color_image"] = self.create_publisher(
                    Image, f"{camera_name}/color/image_raw", image_qos
                )
                self.camera_publishers[camera_name]["color_info"] = self.create_publisher(
                    CameraInfo, f"{camera_name}/color/camera_info", info_qos
                )

            if self.enable_depth:
                self.camera_publishers[camera_name]["depth_image"] = self.create_publisher(
                    Image, f"{camera_name}/depth/image_rect_raw", image_qos
                )
                self.camera_publishers[camera_name]["depth_info"] = self.create_publisher(
                    CameraInfo, f"{camera_name}/depth/camera_info", info_qos
                )

            if self.enable_pointcloud and self.enable_depth:
                self.camera_publishers[camera_name]["pointcloud"] = self.create_publisher(
                    PointCloud2, f"{camera_name}/points", image_qos
                )

            if self.enable_imu:
                self.camera_publishers[camera_name]["imu"] = self.create_publisher(
                    Imu, f"{camera_name}/imu", info_qos  # Use reliable QoS for IMU (small, important data)
                )

    def _camera_capture_loop(self, camera_name: str):
        """Capture loop for a single camera (runs in background thread)"""
        pipeline = self.pipelines[camera_name]
        align = self.aligns.get(camera_name)
        consecutive_errors = 0
        max_consecutive_errors = 10

        while self.running:
            try:
                # Wait for frames (non-blocking with timeout)
                frames = pipeline.wait_for_frames(timeout_ms=self.frame_timeout_ms)
                consecutive_errors = 0  # Reset error counter on success

                # Align depth to color if requested
                if align:
                    frames = align.process(frames)

                # Get frames
                color_frame = None
                depth_frame = None

                if self.enable_color:
                    color_frame = frames.get_color_frame()
                if self.enable_depth:
                    depth_frame = frames.get_depth_frame()

                # Store frames in queue (thread-safe)
                if color_frame or depth_frame:
                    timestamp_ms = None
                    frame_number = None
                    if color_frame:
                        timestamp_ms = float(color_frame.get_timestamp())
                        frame_number = int(color_frame.get_frame_number())
                    elif depth_frame:
                        timestamp_ms = float(depth_frame.get_timestamp())
                        frame_number = int(depth_frame.get_frame_number())

                    if timestamp_ms is None:
                        timestamp_ms = time.time() * 1000.0

                    self.frame_queues[camera_name].append(
                        {
                            "color": color_frame,
                            "depth": depth_frame,
                            "timestamp": time.time(),
                            "timestamp_ms": timestamp_ms,
                            "frame_number": frame_number,
                        }
                    )

            except Exception as e:
                consecutive_errors += 1
                if self.running:  # Only log if still running
                    if consecutive_errors <= max_consecutive_errors:
                        self.get_logger().error(f"Error capturing frames for {camera_name}: {e}")
                    elif consecutive_errors == max_consecutive_errors + 1:
                        self.get_logger().error(
                            f"Suppressing further capture errors for {camera_name} (too many consecutive errors)"
                        )
                # Use shorter sleep to avoid blocking too long
                time.sleep(0.01)  # Reduced from 0.1s to 0.01s for faster recovery

        # Cleanup
        try:
            pipeline.stop()
        except Exception:
            pass

    def _publish_frames(self):
        """Publish frames from all cameras (runs on main thread)"""
        callback_start_time = time.time()
        self._publish_call_count = getattr(self, "_publish_call_count", 0) + 1
        self._last_publish_time = callback_start_time

        # Performance tracking
        if not hasattr(self, "_publish_times"):
            self._publish_times = deque(maxlen=100)  # Track last 100 callback durations
            self._queue_sizes = {}  # Track queue sizes over time
            self._last_perf_log_time = time.time()

        # Check if we have any cameras initialized
        if len(self.pipelines) == 0:
            current_time = time.time()
            if not hasattr(self, "_last_no_cameras_warn_time"):
                self._last_no_cameras_warn_time = 0.0
            if current_time - self._last_no_cameras_warn_time > 5.0:
                self.get_logger().warn("No cameras initialized - cannot publish frames")
                self._last_no_cameras_warn_time = current_time
            return

        # Check if we have any publishers
        if not hasattr(self, "camera_publishers") or len(self.camera_publishers) == 0:
            current_time = time.time()
            if not hasattr(self, "_last_no_publishers_warn_time"):
                self._last_no_publishers_warn_time = 0.0
            if current_time - self._last_no_publishers_warn_time > 5.0:
                self.get_logger().warn("No camera publishers created - cannot publish frames")
                self._last_no_publishers_warn_time = current_time
            return

        # Track queue sizes
        for camera_name, queue in self.frame_queues.items():
            queue_len = len(queue)
            if camera_name not in self._queue_sizes:
                self._queue_sizes[camera_name] = deque(maxlen=100)
            self._queue_sizes[camera_name].append(queue_len)

        # Performance logging every 5 seconds
        current_time = time.time()
        if current_time - self._last_perf_log_time > 5.0:
            queue_info = {
                name: (len(sizes), sum(sizes) / len(sizes) if sizes else 0)
                for name, sizes in self._queue_sizes.items()
            }
            avg_callback_time = (
                sum(self._publish_times) / len(self._publish_times) if self._publish_times else 0
            )
            max_callback_time = max(self._publish_times) if self._publish_times else 0
            self.get_logger().info(
                f"Performance: callback avg={avg_callback_time*1000:.1f}ms max={max_callback_time*1000:.1f}ms, "
                f"queues={queue_info}, publish_count={self._publish_call_count}"
            )
            self._last_perf_log_time = current_time

        frame_data_by_camera: Dict[str, Dict] = {}
        frames_published = 0

        for camera_name in self.pipelines.keys():
            try:
                # Check if queue exists
                if camera_name not in self.frame_queues:
                    self.get_logger().warn_throttle(5.0, f"Frame queue missing for {camera_name}")
                    continue

                # Get latest frames from queue (non-blocking)
                if len(self.frame_queues[camera_name]) == 0:
                    # Queue empty - this is normal if capture is slower than publish rate
                    continue

                # Get most recent frame and CLEAR old frames (drop late data)
                frame_data = self.frame_queues[camera_name][-1]
                # Clear queue to prevent backlog
                self.frame_queues[camera_name].clear()

                frame_data_by_camera[camera_name] = frame_data
                color_frame = frame_data["color"]
                depth_frame = frame_data["depth"]

                # Write color and depth to shared buffer (zero-copy)
                if color_frame and depth_frame and self.enable_color and self.enable_depth:
                    try:
                        # Get numpy arrays directly from frames (no conversion needed)
                        color_image = np.asanyarray(color_frame.get_data())  # RGB
                        depth_image = np.asanyarray(depth_frame.get_data())  # uint16 mm
                        
                        # Get timestamp
                        timestamp = self.get_clock().now().seconds_nanoseconds()
                        timestamp_sec = timestamp[0] + timestamp[1] / 1e9
                        
                        # Get camera info as dict
                        camera_info_dict = None
                        if "color" in self.camera_infos.get(camera_name, {}):
                            info = self.camera_infos[camera_name]["color"]
                            camera_info_dict = {
                                "width": info.width,
                                "height": info.height,
                                "fx": info.k[0],
                                "fy": info.k[4],
                                "cx": info.k[2],
                                "cy": info.k[5],
                            }
                        
                        # Write to shared buffer (zero-copy - just stores references)
                        self.frame_buffer.write_raw_frame(
                            camera_name,
                            color_image,
                            depth_image,
                            timestamp_sec,
                            camera_info_dict,
                        )
                        frames_published += 1
                        
                    except Exception as e:
                        self.get_logger().error(
                            f"Error writing frame to buffer for {camera_name}: {e}"
                        )

                # Publish camera_info topics whenever frames are available
                if color_frame and self.enable_color and "color" in self.camera_infos.get(camera_name, {}):
                    info = self.camera_infos[camera_name]["color"]
                    info.header.stamp = self.get_clock().now().to_msg()
                    if (
                        camera_name in self.camera_publishers
                        and "color_info" in self.camera_publishers[camera_name]
                    ):
                        self.camera_publishers[camera_name]["color_info"].publish(info)

                if depth_frame and self.enable_depth and "depth" in self.camera_infos.get(camera_name, {}):
                    info = self.camera_infos[camera_name]["depth"]
                    info.header.stamp = self.get_clock().now().to_msg()
                    if (
                        camera_name in self.camera_publishers
                        and "depth_info" in self.camera_publishers[camera_name]
                    ):
                        self.camera_publishers[camera_name]["depth_info"].publish(info)

                # Note: Depth is now written to shared buffer together with color above
                # No separate depth/pointcloud publishing - nvblox will handle that

            except Exception as e:
                self.get_logger().error(f"Error publishing frames for {camera_name}: {e}")
                # Standard library
                import traceback

                self.get_logger().error(traceback.format_exc())
                # Don't call publish_status here as it might cause recursion issues

        # Track callback duration for performance monitoring
        callback_duration = time.time() - callback_start_time
        self._publish_times.append(callback_duration)

        # Warn if callback is taking too long (should be < timer period)
        timer_period = 1.0 / self.publish_rate
        if callback_duration > timer_period * 0.8:  # Warn if >80% of timer period
            self.get_logger().warn(
                f"Publish callback took {callback_duration*1000:.1f}ms "
                f"(timer period: {timer_period*1000:.1f}ms) - may cause frame drops!"
            )

        # Log publishing status periodically (throttled to avoid spam)
        if frames_published > 0:
            current_time = time.time()
            if not hasattr(self, "_last_publish_log_time"):
                self._last_publish_log_time = 0.0
            if current_time - self._last_publish_log_time > 5.0:
                self.get_logger().debug(
                    f"Published {frames_published} frame(s) from {len(frame_data_by_camera)} camera(s)"
                )
                self._last_publish_log_time = current_time

        self._maybe_publish_sync_status(frame_data_by_camera)

    def _maybe_publish_sync_status(self, frame_data_by_camera: Dict[str, Dict]) -> None:
        """Publish inter-camera sync status based on hardware timestamps."""
        if not self.enable_inter_cam_sync:
            return

        if len(frame_data_by_camera) < 2:
            return

        now = time.time()
        if (now - self.last_sync_status_time) < self.sync_status_interval_sec:
            return

        timestamps_ms = []
        camera_names = []
        for camera_name, frame_data in frame_data_by_camera.items():
            timestamp_ms = frame_data.get("timestamp_ms")
            if timestamp_ms is not None:
                timestamps_ms.append(float(timestamp_ms))
                camera_names.append(camera_name)

        if len(timestamps_ms) < 2:
            return

        delta_ms = max(timestamps_ms) - min(timestamps_ms)
        camera_list = ", ".join(camera_names)

        if delta_ms <= self.sync_status_tolerance_ms:
            self.publish_status(
                "sync_ok",
                f"Inter-camera sync delta {delta_ms:.2f} ms (cameras: {camera_list})",
            )
        else:
            self.publish_status(
                "sync_warn",
                f"Inter-camera sync delta {delta_ms:.2f} ms exceeds {self.sync_status_tolerance_ms:.2f} ms",
            )
            self.get_logger().warn(
                f"Inter-camera sync delta {delta_ms:.2f} ms exceeds {self.sync_status_tolerance_ms:.2f} ms"
            )

        self.last_sync_status_time = now

    def _depth_to_pointcloud(
        self, depth_frame: rs.depth_frame, color_frame: Optional[rs.video_frame], camera_name: str
    ) -> Optional[PointCloud2]:
        """Convert depth frame to PointCloud2 message"""
        try:
            # Validate depth frame
            if not depth_frame:
                self.get_logger().debug(f"No depth frame available for {camera_name}")
                return None

            # Create pointcloud
            points = rs.pointcloud()
            if color_frame:
                points.map_to(color_frame)
            points = points.calculate(depth_frame)

            vertices = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, 3)

            # Check if we have valid vertices
            if len(vertices) == 0:
                self.get_logger().debug(f"No vertices generated for {camera_name} pointcloud")
                return None

            colors = None
            if color_frame:
                colors = (
                    np.asanyarray(points.get_texture_coordinates()).view(np.float32).reshape(-1, 2)
                )

            # Create PointCloud2 message
            msg = PointCloud2()
            msg.header.frame_id = f"{camera_name}_depth_optical_frame"
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.height = 1
            msg.width = len(vertices)
            msg.is_dense = False

            # Point fields
            msg.fields = [
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            ]

            if colors is not None:
                msg.fields.append(
                    PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1)
                )

            msg.point_step = 12 if colors is None else 16
            msg.row_step = msg.point_step * msg.width
            msg.data = vertices.tobytes()

            return msg

        except Exception as e:
            self.get_logger().warn(f"Error creating pointcloud for {camera_name}: {e}")
            return None

    def publish_status(self, status: str, message: str):
        """Publish status message"""
        msg = String()
        msg.data = f"[{status}] {message}"
        self.status_pub.publish(msg)

    def destroy_node(self):
        """Cleanup on shutdown - ensures all camera resources are properly released"""
        self.get_logger().info("Shutting down RealSense cameras...")
        self.running = False

        # Give capture threads time to stop
        if self.camera_threads:
            self.get_logger().info(
                f"Waiting for {len(self.camera_threads)} capture thread(s) to stop..."
            )
            for thread in self.camera_threads:
                if thread.is_alive():
                    thread.join(timeout=self.shutdown_delay)
                    if thread.is_alive():
                        self.get_logger().warn(f"Capture thread {thread.name} did not stop in time")

        # Stop all pipelines and ensure resources are released
        if self.pipelines:
            self.get_logger().info(f"Stopping {len(self.pipelines)} camera pipeline(s)...")
            for camera_name, pipeline in self.pipelines.items():
                try:
                    pipeline.stop()
                    self.get_logger().debug(f"✓ Stopped pipeline for {camera_name}")
                except Exception as e:
                    self.get_logger().warn(f"Error stopping pipeline for {camera_name}: {e}")

            # Clear pipelines dict to ensure they're not reused
            self.pipelines.clear()

        # Clear frame queues
        self.frame_queues.clear()

        # Clear camera info
        self.camera_infos.clear()

        self.get_logger().info("✓ Camera resources released")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseCameraNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
