#!/usr/bin/env python3
"""
Fast Image Publisher Utilities
Zero-copy or minimal-copy image publishing for ROS 2
"""

# Third-party
import numpy as np
from sensor_msgs.msg import Image


def numpy_to_image_msg(
    np_array: np.ndarray, encoding: str, frame_id: str = "", timestamp=None
) -> Image:
    """
    Convert numpy array to ROS Image message with minimal copying.

    Args:
        np_array: Numpy array (height, width) or (height, width, channels)
        encoding: ROS image encoding (e.g., "bgr8", "16UC1")
        frame_id: Frame ID for the image
        timestamp: ROS timestamp (if None, will be set by caller)

    Returns:
        Image message
    """
    msg = Image()
    msg.header.frame_id = frame_id
    if timestamp is not None:
        msg.header.stamp = timestamp

    # Set dimensions
    if len(np_array.shape) == 2:
        msg.height, msg.width = np_array.shape
        msg.step = msg.width * np_array.itemsize
    else:
        msg.height, msg.width, channels = np_array.shape
        msg.step = msg.width * channels * np_array.itemsize

    msg.encoding = encoding
    msg.is_bigendian = False

    # Use tobytes() which is faster than tolist() and creates a copy
    # This copy is unavoidable for ROS message serialization
    msg.data = np_array.tobytes()

    return msg
