#!/usr/bin/env python3
"""
Audio Processor Node
Processes raw audio data from USB microphone to extract:
- Volume levels (RMS, peak)
- Waveform samples for visualization
- Per-channel audio levels
"""

# Standard library
import struct
from typing import Optional

# Third-party
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, UInt8MultiArray


class AudioProcessorNode(Node):
    """Processes raw audio data for visualization"""

    def __init__(self):
        super().__init__("audio_processor_node")

        # Parameters
        self.declare_parameter("audio_topic", "/microphone/audio")
        self.declare_parameter("sample_rate", 16000)
        self.declare_parameter("channels", 2)
        self.declare_parameter("format", "S16_LE")
        self.declare_parameter("volume_history_size", 100)  # Number of volume samples to keep
        self.declare_parameter("waveform_samples", 100)  # Number of waveform samples per channel to publish

        self.audio_topic = self.get_parameter("audio_topic").value
        self.sample_rate = int(self.get_parameter("sample_rate").value)
        self.channels = int(self.get_parameter("channels").value)
        self.format = self.get_parameter("format").value
        self.volume_history_size = int(self.get_parameter("volume_history_size").value)
        self.waveform_samples = int(self.get_parameter("waveform_samples").value)

        # Publishers
        self.volume_pub = self.create_publisher(Float32, "/microphone/volume", 10)
        self.volume_rms_pub = self.create_publisher(Float32, "/microphone/volume_rms", 10)
        self.volume_peak_pub = self.create_publisher(Float32, "/microphone/volume_peak", 10)
        self.volume_left_pub = self.create_publisher(Float32, "/microphone/volume_left", 10)
        self.volume_right_pub = self.create_publisher(Float32, "/microphone/volume_right", 10)
        self.waveform_left_pub = self.create_publisher(Float32MultiArray, "/microphone/waveform_left", 10)
        self.waveform_right_pub = self.create_publisher(Float32MultiArray, "/microphone/waveform_right", 10)

        # Subscriber
        self.audio_sub = self.create_subscription(
            UInt8MultiArray,
            self.audio_topic,
            self._audio_callback,
            10
        )

        self.get_logger().info(
            f"Audio processor started: {self.channels} channels, {self.sample_rate}Hz, format={self.format}"
        )

    def _audio_callback(self, msg: UInt8MultiArray):
        """Process incoming audio data"""
        try:
            # Convert bytes to numpy array
            audio_bytes = np.frombuffer(bytes(msg.data), dtype=np.uint8)

            # Convert to 16-bit signed integers (S16_LE format)
            # Each sample is 2 bytes (little-endian)
            if len(audio_bytes) < 2:
                return

            # Convert bytes to int16 samples
            # For stereo: samples are interleaved [L, R, L, R, ...]
            samples = np.frombuffer(audio_bytes, dtype=np.int16)

            # De-interleave stereo channels
            if self.channels == 2:
                left_samples = samples[0::2]  # Every other sample starting at 0
                right_samples = samples[1::2]  # Every other sample starting at 1
            else:
                # Mono - use same data for both
                left_samples = samples
                right_samples = samples

            # Calculate volume metrics
            # Normalize to -1.0 to 1.0 range (int16 max is 32767)
            left_normalized = left_samples.astype(np.float32) / 32768.0
            right_normalized = right_samples.astype(np.float32) / 32768.0

            # RMS (Root Mean Square) - represents average power
            rms_left = np.sqrt(np.mean(left_normalized ** 2))
            rms_right = np.sqrt(np.mean(right_normalized ** 2))
            rms_total = np.sqrt((rms_left ** 2 + rms_right ** 2) / 2.0)

            # Peak amplitude
            peak_left = np.max(np.abs(left_normalized))
            peak_right = np.max(np.abs(right_normalized))
            peak_total = max(peak_left, peak_right)

            # Overall volume (average of RMS)
            volume_total = (rms_left + rms_right) / 2.0

            # Publish volume metrics
            volume_msg = Float32()
            volume_msg.data = float(volume_total)
            self.volume_pub.publish(volume_msg)

            rms_msg = Float32()
            rms_msg.data = float(rms_total)
            self.volume_rms_pub.publish(rms_msg)

            peak_msg = Float32()
            peak_msg.data = float(peak_total)
            self.volume_peak_pub.publish(peak_msg)

            left_volume_msg = Float32()
            left_volume_msg.data = float(rms_left)
            self.volume_left_pub.publish(left_volume_msg)

            right_volume_msg = Float32()
            right_volume_msg.data = float(rms_right)
            self.volume_right_pub.publish(right_volume_msg)

            # Publish waveform samples (downsampled for visualization)
            # Take evenly spaced samples across the chunk
            if len(left_samples) > 0:
                waveform_left = self._extract_waveform_samples(left_normalized, self.waveform_samples)
                waveform_left_msg = Float32MultiArray()
                waveform_left_msg.data = waveform_left.tolist()
                self.waveform_left_pub.publish(waveform_left_msg)

            if len(right_samples) > 0:
                waveform_right = self._extract_waveform_samples(right_normalized, self.waveform_samples)
                waveform_right_msg = Float32MultiArray()
                waveform_right_msg.data = waveform_right.tolist()
                self.waveform_right_pub.publish(waveform_right_msg)

        except Exception as e:
            self.get_logger().error(f"Error processing audio: {e}", throttle_duration_sec=1.0)

    def _extract_waveform_samples(self, samples: np.ndarray, num_samples: int) -> np.ndarray:
        """Extract evenly spaced samples for waveform visualization"""
        if len(samples) == 0:
            return np.array([])

        if len(samples) <= num_samples:
            return samples

        # Take evenly spaced samples
        indices = np.linspace(0, len(samples) - 1, num_samples, dtype=int)
        return samples[indices]


def main(args=None):
    rclpy.init(args=args)
    node = AudioProcessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
