#!/usr/bin/env python3
"""
Image Processing Utilities
Shared utilities for image downsampling and conversion
"""

# Third-party
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

# Global bridge instance (reused for efficiency)
_bridge = CvBridge()


def downsample_image(
    image_msg: Image,
    target_width: int,
    target_height: int,
    interpolation: int = cv2.INTER_AREA,
) -> Image:
    """
    Downsample a sensor_msgs Image to target dimensions.

    Args:
        image_msg: Input image message
        target_width: Target width in pixels
        target_height: Target height in pixels
        interpolation: OpenCV interpolation method (default: INTER_AREA for downsampling)

    Returns:
        Downsampled Image message with same encoding and header
    """
    # Convert ROS image to OpenCV
    cv_image = _bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")

    # Get original dimensions
    original_height, original_width = cv_image.shape[:2]

    # Skip if already at or below target size
    if original_width <= target_width and original_height <= target_height:
        # Still create new message to ensure proper header/stamp
        result_msg = _bridge.cv2_to_imgmsg(cv_image, encoding=image_msg.encoding)
        result_msg.header = image_msg.header
        return result_msg

    # Downsample using specified interpolation
    resized = cv2.resize(cv_image, (target_width, target_height), interpolation=interpolation)

    # Convert back to ROS message
    result_msg = _bridge.cv2_to_imgmsg(resized, encoding=image_msg.encoding)
    result_msg.header = image_msg.header

    return result_msg
