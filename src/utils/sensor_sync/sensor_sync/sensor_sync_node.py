#!/usr/bin/env python3
"""
Sensor Synchronization and Filtering Node
Synchronizes sensor data to camera frames, applies Kalman filtering to IMU data,
and prepares synchronized sensor data for VLM feature extraction.
"""

# Standard library
from collections import deque
import json
import threading
import time
from typing import Dict, Optional

# Third-party
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, Image, Imu
from std_msgs.msg import Header, String


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
    """Synchronizes sensor data to camera frames with Kalman filtering"""

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

        self.sync_to_camera = bool(self.get_parameter("sync_to_camera").value)
        self.sync_camera_frames = bool(self.get_parameter("sync_camera_frames").value)
        self.camera_frame_sync_tolerance = float(
            self.get_parameter("camera_frame_sync_tolerance").value
        )
        self.target_frequency = float(self.get_parameter("target_frequency").value)
        self.imu_filter_enabled = bool(self.get_parameter("imu_filter_enabled").value)
        self.max_buffer_size = int(self.get_parameter("max_buffer_size").value)
        self.time_sync_tolerance = float(self.get_parameter("time_sync_tolerance").value)

        # Kalman filter for IMU
        if self.imu_filter_enabled:
            process_noise = float(self.get_parameter("imu_process_noise").value)
            measurement_noise = float(self.get_parameter("imu_measurement_noise").value)
            self.kalman_filter = KalmanFilter(
                process_noise=process_noise, measurement_noise=measurement_noise
            )
        else:
            self.kalman_filter = None

        # Buffers for sensor data (thread-safe with locks)
        self.imu_buffer: deque = deque(maxlen=self.max_buffer_size)
        self.battery_buffer: deque = deque(maxlen=self.max_buffer_size)
        self.status_buffer: deque = deque(maxlen=self.max_buffer_size)
        self.camera_timestamps: deque = deque(maxlen=self.max_buffer_size)

        # Camera frame sync tracking (for multi-camera synchronization)
        self.camera_frame_sync: Dict[str, Optional[Image]] = {}
        self.camera_frame_timestamps: Dict[str, float] = {}

        self.buffer_lock = threading.Lock()

        # Latest synchronized data
        self.latest_sync_data: Optional[Dict] = None

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

        # Publishers
        output_ns = self.get_parameter("output_namespace").value

        # Filtered/synchronized IMU
        self.filtered_imu_pub = self.create_publisher(Imu, f"{output_ns}/imu/filtered", 10)

        # Synchronized chassis data
        self.synced_battery_pub = self.create_publisher(
            BatteryState, f"{output_ns}/chassis/battery", 10
        )
        self.synced_status_pub = self.create_publisher(String, f"{output_ns}/chassis/status", 10)

        # Synchronized sensor data (for VLM)
        if self.get_parameter("publish_vlm_features").value:
            self.vlm_features_pub = self.create_publisher(String, f"{output_ns}/vlm_features", 10)

        # Status
        self.status_pub = self.create_publisher(String, f"{output_ns}/status", 10)

        # Timer for synchronized publishing
        if self.sync_to_camera:
            # Will be triggered by camera callbacks
            self.sync_timer = None
        else:
            # Fixed rate publishing
            timer_period = 1.0 / float(self.target_frequency)
            self.sync_timer = self.create_timer(timer_period, self._publish_synchronized_data)

        self.get_logger().info("Sensor sync node started")
        self.get_logger().info(f"Sync to camera: {self.sync_to_camera}")
        self.get_logger().info(f"Sync camera frames: {self.sync_camera_frames}")
        self.get_logger().info(f"Target frequency: {self.target_frequency} Hz")
        self.get_logger().info(f"IMU filtering: {self.imu_filter_enabled}")

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
        """Handle camera frame - trigger synchronization"""
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        with self.buffer_lock:
            self.camera_timestamps.append(
                {"timestamp": timestamp, "topic": topic, "header": msg.header}
            )
            # Store latest frame for each camera (for frame sync)
            self.camera_frame_sync[topic] = msg
            self.camera_frame_timestamps[topic] = timestamp

        if self.sync_to_camera:
            # Check if we should wait for all cameras to sync
            if self.sync_camera_frames:
                if self._check_camera_frame_sync():
                    # All cameras are synced, use average timestamp
                    avg_timestamp = sum(self.camera_frame_timestamps.values()) / len(
                        self.camera_frame_timestamps
                    )
                    self._publish_synchronized_data(avg_timestamp)
            else:
                # Publish immediately on any camera frame
                self._publish_synchronized_data(timestamp)

    def _check_camera_frame_sync(self) -> bool:
        """Check if all cameras have frames within sync tolerance"""
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
        return (max_ts - min_ts) <= tolerance

    def _publish_synchronized_data(self, sync_timestamp: Optional[float] = None):
        """Publish synchronized sensor data"""
        if sync_timestamp is None:
            sync_timestamp = time.time()

        with self.buffer_lock:
            # Find closest IMU data
            imu_data = self._find_closest_sensor_data(self.imu_buffer, sync_timestamp)

            if imu_data is None:
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

            # Publish filtered IMU
            self.filtered_imu_pub.publish(filtered_imu)

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

            if status_data:
                synced_status = String()
                synced_status.data = status_data["msg"].data
                self.synced_status_pub.publish(synced_status)

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
