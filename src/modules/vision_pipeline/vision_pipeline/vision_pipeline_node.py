#!/usr/bin/env python3
"""
Vision Pipeline Orchestrator Node
Coordinates RealSense cameras, nvblox processing, visual SLAM, and calibration
Manages vision system lifecycle and mode transitions
"""

# Standard library
import time
from typing import Dict, Optional

# Third-party
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovariance
from sensor_msgs.msg import Image, CameraInfo
from custom_msgs.msg import ModuleHealth, NodeHealth, PowerRequest

# Local
from isaac_utils import HealthStatusPublisher, InputWatchdog


class VisionPipelineNode(Node):
    """
    ROS 2 node for orchestrating vision pipeline components

    Manages lifecycle of:
    - RealSense camera nodes
    - nvblox processor
    - Visual SLAM
    - Camera calibration

    Implements power-aware mode switching based on battery and thermal state.
    """

    def __init__(self):
        super().__init__("vision_pipeline_node")

        # Parameters
        self.declare_parameter("enable_slam", True)
        self.declare_parameter("enable_calibration", True)
        self.declare_parameter("enable_nvblox", True)
        self.declare_parameter("camera_names", ["camera_front", "camera_rear"])

        # Mode parameters
        self.declare_parameter("mode", "full")  # full, tracking_only, sleep
        self.declare_parameter("auto_mode_switching", True)
        self.declare_parameter("low_battery_threshold", 20.0)  # percent
        self.declare_parameter("high_temp_threshold", 75.0)  # celsius

        # Topic names
        self.declare_parameter("global_pose_topic", "/vision/global_pose")
        self.declare_parameter("module_health_topic", "/jetson/health/vision_pipeline")
        self.declare_parameter("power_request_topic", "/jetson/power/request")

        # Health monitoring
        self.declare_parameter("health_topic", f"health/{self.get_name()}")
        self.declare_parameter("health_publish_rate", 1.0)

        # Get parameters
        self.enable_slam = bool(self.get_parameter("enable_slam").value)
        self.enable_calibration = bool(self.get_parameter("enable_calibration").value)
        self.enable_nvblox = bool(self.get_parameter("enable_nvblox").value)
        self.camera_names = self.get_parameter("camera_names").value

        self.mode = str(self.get_parameter("mode").value)
        self.auto_mode_switching = bool(self.get_parameter("auto_mode_switching").value)
        self.low_battery_threshold = float(self.get_parameter("low_battery_threshold").value)
        self.high_temp_threshold = float(self.get_parameter("high_temp_threshold").value)

        global_pose_topic = str(self.get_parameter("global_pose_topic").value)
        module_health_topic = str(self.get_parameter("module_health_topic").value)
        power_request_topic = str(self.get_parameter("power_request_topic").value)

        health_topic = str(self.get_parameter("health_topic").value)
        health_rate = float(self.get_parameter("health_publish_rate").value)

        # State
        self.pipeline_active = False
        self.slam_initialized = False
        self.current_battery_pct = 100.0
        self.current_gpu_temp = 0.0
        self.node_health_status: Dict[str, NodeHealth] = {}
        self.last_global_pose: Optional[PoseWithCovariance] = None

        # Subscribers
        self.global_pose_sub = self.create_subscription(
            PoseWithCovariance,
            global_pose_topic,
            self._global_pose_callback,
            10
        )

        # Monitor camera health from multiple cameras
        for camera_name in self.camera_names:
            camera_info_topic = f"/hardware/{camera_name}/color/camera_info"
            self.create_subscription(
                CameraInfo,
                camera_info_topic,
                lambda msg, name=camera_name: self._camera_info_callback(msg, name),
                10
            )

        # Publishers
        self.module_health_pub = self.create_publisher(
            ModuleHealth,
            module_health_topic,
            10
        )
        self.power_request_pub = self.create_publisher(
            PowerRequest,
            power_request_topic,
            10
        )

        # Health monitoring
        self.health_pub = HealthStatusPublisher(
            self,
            health_topic,
            rate_hz=health_rate,
        )

        # Watchdogs for pipeline components
        self.watchdogs = {
            "slam": InputWatchdog("visual_slam", warn_timeout=2.0, stale_timeout=5.0),
        }
        for camera_name in self.camera_names:
            self.watchdogs[f"camera_{camera_name}"] = InputWatchdog(
                f"camera_{camera_name}",
                warn_timeout=2.0,
                stale_timeout=5.0
            )

        # Timers
        self.health_timer = self.create_timer(1.0 / health_rate, self._publish_health)
        self.module_health_timer = self.create_timer(1.0, self._publish_module_health)
        self.mode_check_timer = self.create_timer(5.0, self._check_mode_switching)

        # Initialize pipeline
        self._start_pipeline()

        self.get_logger().info(
            f"Vision Pipeline initialized: mode={self.mode}, "
            f"slam={self.enable_slam}, nvblox={self.enable_nvblox}, "
            f"calibration={self.enable_calibration}"
        )

    def _global_pose_callback(self, msg: PoseWithCovariance):
        """Receive global pose from SLAM"""
        self.last_global_pose = msg
        self.watchdogs["slam"].update()
        if not self.slam_initialized:
            self.slam_initialized = True
            self.get_logger().info("SLAM initialized and tracking")

    def _camera_info_callback(self, msg: CameraInfo, camera_name: str):
        """Receive camera info to monitor camera health"""
        watchdog_key = f"camera_{camera_name}"
        if watchdog_key in self.watchdogs:
            self.watchdogs[watchdog_key].update()

    def _start_pipeline(self):
        """Start the vision pipeline"""
        self.pipeline_active = True
        self.get_logger().info(f"Vision pipeline started in {self.mode} mode")

        # Request GPU power if needed
        if self.mode == "full" and (self.enable_slam or self.enable_nvblox):
            self._request_gpu_power("on", "vision_full_mode")

    def _stop_pipeline(self):
        """Stop the vision pipeline"""
        self.pipeline_active = False
        self.slam_initialized = False
        self.get_logger().info("Vision pipeline stopped")

        # Release GPU power
        self._request_gpu_power("off", "vision_sleep_mode")

    def _switch_mode(self, new_mode: str, reason: str):
        """Switch pipeline mode"""
        if new_mode == self.mode:
            return

        self.get_logger().info(f"Switching vision mode: {self.mode} → {new_mode} ({reason})")
        old_mode = self.mode
        self.mode = new_mode

        # Handle mode transitions
        if new_mode == "sleep":
            self._stop_pipeline()
        elif old_mode == "sleep":
            self._start_pipeline()
        elif new_mode == "tracking_only":
            # Reduce processing load
            self._request_gpu_power("sleep", "tracking_only_mode")
        elif new_mode == "full" and old_mode == "tracking_only":
            self._request_gpu_power("wake", "full_mode_resumed")

    def _check_mode_switching(self):
        """Check if mode should be switched based on system state"""
        if not self.auto_mode_switching:
            return

        # Low battery → tracking_only or sleep
        if self.current_battery_pct < self.low_battery_threshold:
            if self.mode == "full":
                self._switch_mode("tracking_only", f"low_battery_{self.current_battery_pct:.1f}%")
            elif self.current_battery_pct < self.low_battery_threshold / 2:
                self._switch_mode("sleep", f"critical_battery_{self.current_battery_pct:.1f}%")

        # High temperature → reduce load
        if self.current_gpu_temp > self.high_temp_threshold:
            if self.mode == "full":
                self._switch_mode("tracking_only", f"high_temp_{self.current_gpu_temp:.1f}°C")

        # Recovery: Good conditions → resume full mode
        if (self.current_battery_pct > self.low_battery_threshold * 1.5 and
            self.current_gpu_temp < self.high_temp_threshold * 0.9 and
            self.mode != "full"):
            self._switch_mode("full", "conditions_improved")

    def _request_gpu_power(self, state: str, reason: str):
        """Request GPU power state change"""
        msg = PowerRequest()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.requested_state = state
        msg.requester_node = self.get_name()
        msg.reason = reason
        msg.priority = 7  # Vision has high priority
        msg.timeout_sec = 0.0  # Indefinite

        self.power_request_pub.publish(msg)
        self.get_logger().debug(f"Requested GPU power: {state} ({reason})")

    def _publish_health(self):
        """Publish node health status"""
        # Aggregate watchdog status
        watchdog_status = {
            name: wd.get_status()
            for name, wd in self.watchdogs.items()
        }

        # Determine overall status
        if not self.pipeline_active:
            status = "OK"
            message = f"Vision pipeline in {self.mode} mode (inactive)"
        elif all(s == "OK" for s in watchdog_status.values()) and self.slam_initialized:
            status = "OK"
            message = f"Vision pipeline running: mode={self.mode}"
        elif any(s == "ERROR" for s in watchdog_status.values()):
            status = "ERROR"
            failed = [name for name, s in watchdog_status.items() if s == "ERROR"]
            message = f"Pipeline errors: {', '.join(failed)}"
        else:
            status = "WARN"
            warned = [name for name, s in watchdog_status.items() if s == "WARN"]
            message = f"Pipeline warnings: {', '.join(warned)}"

        self.health_pub.publish(status, message)

    def _publish_module_health(self):
        """Publish aggregated module health"""
        msg = ModuleHealth()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.module_name = "vision_pipeline"
        msg.host = "jetson"

        # Aggregate status from watchdogs
        watchdog_status = [wd.get_status() for wd in self.watchdogs.values()]

        if all(s == "OK" for s in watchdog_status) and self.pipeline_active:
            msg.status = "OK"
            msg.status_message = f"Vision pipeline healthy in {self.mode} mode"
        elif any(s == "ERROR" for s in watchdog_status):
            msg.status = "ERROR"
            msg.status_message = "One or more vision components failed"
        elif not self.pipeline_active:
            msg.status = "WARN"
            msg.status_message = f"Vision pipeline inactive ({self.mode} mode)"
        else:
            msg.status = "WARN"
            msg.status_message = "Vision pipeline degraded"

        # TODO: Add per-node health details
        # msg.node_health = [...]

        # TODO: Add resource usage
        # msg.cpu_usage = ...
        # msg.memory_usage = ...

        msg.uptime_sec = (self.get_clock().now().nanoseconds * 1e-9)
        msg.started_at = self.get_clock().now().to_msg()
        msg.last_update = self.get_clock().now().to_msg()

        self.module_health_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VisionPipelineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
