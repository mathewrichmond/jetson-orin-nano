"""
Shared memory buffer for zero-copy camera frame passing between nodes.

This module provides a shared buffer that eliminates redundant ROS topic
publishing and message conversions for raw camera data. Instead of:
  Camera → ROS msg → nvblox → ROS msg → sensor_fusion

We now have:
  Camera → buffer → nvblox → buffer → sensor_fusion

This eliminates multiple memory copies and cv_bridge conversions.
"""

# Standard library
from collections import deque
from dataclasses import dataclass, field
import threading
from typing import Dict, Optional, Tuple

# Third-party
import numpy as np


@dataclass
class CameraFrame:
    """Container for a single camera frame with color, depth, and metadata"""

    color: np.ndarray  # RGB or BGR numpy array
    depth: np.ndarray  # Depth array in mm or meters
    timestamp: float  # ROS time in seconds
    frame_number: int = 0  # Sequential frame number
    camera_info: Optional[Dict] = None  # Camera intrinsics


@dataclass
class ProcessedFrame:
    """Container for processed camera frame from nvblox"""

    image: np.ndarray  # Processed color image
    pointcloud: Optional[np.ndarray] = None  # Optional pointcloud (N, 3)
    timestamp: float = 0.0  # ROS time in seconds
    frame_number: int = 0  # Sequential frame number


class CameraFrameBuffer:
    """
    Thread-safe shared buffer for camera frames.

    Provides zero-copy frame sharing between camera, nvblox, and sensor_fusion nodes.
    Uses simple dictionary storage with locks for thread safety.

    Usage:
        # In camera node:
        buffer = CameraFrameBuffer.get_instance()
        buffer.write_raw_frame("camera_front", color, depth, timestamp)

        # In nvblox node:
        frame = buffer.read_raw_frame("camera_front")
        processed = process(frame)
        buffer.write_processed_frame("camera_front", processed_image, timestamp)

        # In sensor_fusion node:
        processed = buffer.read_processed_frame("camera_front")
    """

    _instance = None
    _lock = threading.Lock()

    def __init__(self):
        """Initialize buffer storage"""
        # Raw frames from camera
        self.raw_frames: Dict[str, CameraFrame] = {}
        self.raw_lock = threading.Lock()

        # Processed frames from nvblox
        self.processed_frames: Dict[str, ProcessedFrame] = {}
        self.processed_lock = threading.Lock()

        # Frame counters
        self.frame_counters: Dict[str, int] = {}
        self.counter_lock = threading.Lock()

        # Statistics
        self.stats = {
            "raw_writes": 0,
            "raw_reads": 0,
            "processed_writes": 0,
            "processed_reads": 0,
        }
        self.stats_lock = threading.Lock()

    @classmethod
    def get_instance(cls):
        """Get singleton instance of the buffer (thread-safe)"""
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = cls()
        return cls._instance

    def write_raw_frame(
        self,
        camera_name: str,
        color: np.ndarray,
        depth: np.ndarray,
        timestamp: float,
        camera_info: Optional[Dict] = None,
    ) -> None:
        """
        Write raw camera frame to buffer.

        Args:
            camera_name: Camera identifier (e.g., "camera_front")
            color: Color image as numpy array (H, W, 3)
            depth: Depth image as numpy array (H, W)
            timestamp: Frame timestamp in seconds
            camera_info: Optional camera intrinsics
        """
        # Get frame number
        with self.counter_lock:
            if camera_name not in self.frame_counters:
                self.frame_counters[camera_name] = 0
            self.frame_counters[camera_name] += 1
            frame_number = self.frame_counters[camera_name]

        frame = CameraFrame(
            color=color,
            depth=depth,
            timestamp=timestamp,
            frame_number=frame_number,
            camera_info=camera_info,
        )

        with self.raw_lock:
            self.raw_frames[camera_name] = frame

        with self.stats_lock:
            self.stats["raw_writes"] += 1

    def read_raw_frame(self, camera_name: str) -> Optional[CameraFrame]:
        """
        Read latest raw camera frame from buffer.

        Args:
            camera_name: Camera identifier

        Returns:
            CameraFrame if available, None otherwise
        """
        with self.raw_lock:
            frame = self.raw_frames.get(camera_name)

        if frame is not None:
            with self.stats_lock:
                self.stats["raw_reads"] += 1

        return frame

    def write_processed_frame(
        self,
        camera_name: str,
        image: np.ndarray,
        timestamp: float,
        pointcloud: Optional[np.ndarray] = None,
        frame_number: int = 0,
    ) -> None:
        """
        Write processed frame from nvblox to buffer.

        Args:
            camera_name: Camera identifier
            image: Processed color image
            timestamp: Frame timestamp
            pointcloud: Optional pointcloud array (N, 3)
            frame_number: Frame sequence number
        """
        frame = ProcessedFrame(
            image=image,
            pointcloud=pointcloud,
            timestamp=timestamp,
            frame_number=frame_number,
        )

        with self.processed_lock:
            self.processed_frames[camera_name] = frame

        with self.stats_lock:
            self.stats["processed_writes"] += 1

    def read_processed_frame(self, camera_name: str) -> Optional[ProcessedFrame]:
        """
        Read latest processed frame from buffer.

        Args:
            camera_name: Camera identifier

        Returns:
            ProcessedFrame if available, None otherwise
        """
        with self.processed_lock:
            frame = self.processed_frames.get(camera_name)

        if frame is not None:
            with self.stats_lock:
                self.stats["processed_reads"] += 1

        return frame

    def get_available_cameras(self) -> Tuple[list, list]:
        """
        Get lists of cameras with available raw and processed frames.

        Returns:
            Tuple of (raw_camera_names, processed_camera_names)
        """
        with self.raw_lock:
            raw_cameras = list(self.raw_frames.keys())
        with self.processed_lock:
            processed_cameras = list(self.processed_frames.keys())
        return raw_cameras, processed_cameras

    def get_stats(self) -> Dict:
        """Get buffer statistics"""
        with self.stats_lock:
            return self.stats.copy()

    def clear(self, camera_name: Optional[str] = None) -> None:
        """
        Clear buffer for specific camera or all cameras.

        Args:
            camera_name: Camera to clear, or None for all
        """
        if camera_name is None:
            with self.raw_lock:
                self.raw_frames.clear()
            with self.processed_lock:
                self.processed_frames.clear()
        else:
            with self.raw_lock:
                self.raw_frames.pop(camera_name, None)
            with self.processed_lock:
                self.processed_frames.pop(camera_name, None)
