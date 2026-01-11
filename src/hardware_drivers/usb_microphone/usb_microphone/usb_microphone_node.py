#!/usr/bin/env python3
"""
USB Microphone Node
Captures audio from USB microphone and publishes as ROS 2 audio messages
"""

# Standard library
import array
import subprocess
import threading
import time
from typing import Optional

# Third-party
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header, String


class USBMicrophoneNode(Node):
    """ROS 2 node for USB microphone audio capture"""

    def __init__(self):
        super().__init__("usb_microphone_node")

        # Parameters
        self.declare_parameter("device", "default")
        self.declare_parameter("sample_rate", 16000)
        self.declare_parameter("channels", 1)
        self.declare_parameter("format", "S16_LE")
        self.declare_parameter("chunk_size", 1024)
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("status_topic", "/microphone/status")
        self.declare_parameter("retry_delay", 1.0)  # Seconds between retry attempts
        self.declare_parameter("device_check_timeout", 2.0)  # Seconds for device check timeout

        self.device = self.get_parameter("device").value
        self.sample_rate = self.get_parameter("sample_rate").value
        self.channels = self.get_parameter("channels").value
        self.format = self.get_parameter("format").value
        self.chunk_size = self.get_parameter("chunk_size").value
        self.publish_rate = self.get_parameter("publish_rate").value
        self.status_topic = self.get_parameter("status_topic").value
        self.retry_delay = self.get_parameter("retry_delay").value
        self.device_check_timeout = self.get_parameter("device_check_timeout").value

        # Publishers
        self.status_pub = self.create_publisher(String, self.status_topic, 10)

        # Note: Audio message type would need to be defined or use sensor_msgs/PointCloud2
        # For now, we'll publish status and log audio capture
        self.audio_capturing = False
        self.arecord_process: Optional[subprocess.Popen] = None

        # Timer for status updates
        self.status_timer = self.create_timer(
            1.0 / self.publish_rate, self._publish_status_callback
        )

        # Start audio capture thread
        self.capture_thread = threading.Thread(target=self._capture_audio, daemon=True)
        self.capture_thread.start()

        self.get_logger().info("USB Microphone node started")
        self.publish_status("initialized", "Node initialized")

    def _capture_audio(self):
        """Capture audio from USB microphone using arecord"""
        while rclpy.ok():
            try:
                # Check if device is available
                if not self._check_device():
                    self.publish_status("error", "Microphone device not found")
                    time.sleep(self.retry_delay)
                    continue

                # Start arecord process
                cmd = [
                    "arecord",
                    "-D",
                    self.device,
                    "-r",
                    str(self.sample_rate),
                    "-c",
                    str(self.channels),
                    "-f",
                    self.format,
                    "-t",
                    "raw",
                ]

                self.arecord_process = subprocess.Popen(
                    cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=self.chunk_size
                )

                self.audio_capturing = True
                self.publish_status("capturing", "Audio capture started")

                # Read audio data in chunks
                while self.arecord_process.poll() is None:
                    chunk = self.arecord_process.stdout.read(self.chunk_size)
                    if chunk:
                        # Process audio chunk (could publish here)
                        # For now, just verify we're getting data
                        pass
                    else:
                        break

                self.audio_capturing = False
                if self.arecord_process.returncode != 0:
                    if self.arecord_process.stderr:
                        error = self.arecord_process.stderr.read().decode("utf-8", errors="ignore")
                        self.get_logger().warn(f"arecord error: {error}")
                        self.publish_status("error", f"Capture error: {error[:50]}")
                    else:
                        self.get_logger().warn("arecord error: unknown error")
                        self.publish_status("error", "Capture error: unknown")

            except Exception as e:
                self.get_logger().error(f"Audio capture error: {e}")
                self.publish_status("error", str(e))
                self.audio_capturing = False
                time.sleep(self.retry_delay)

    def _check_device(self) -> bool:
        """Check if microphone device is available"""
        try:
            # List audio devices
            result = subprocess.run(["arecord", "-l"], capture_output=True, text=True, timeout=self.device_check_timeout)
            if result.returncode == 0:
                # Check if USB microphone is in the list
                if "USB" in result.stdout or self.device != "default":
                    return True
            return False
        except Exception as e:
            self.get_logger().debug(f"Device check error: {e}")
            return False

    def _publish_status_callback(self):
        """Timer callback to publish periodic status"""
        if self.audio_capturing:
            self.publish_status("capturing", f"Recording at {self.sample_rate}Hz")
        else:
            self.publish_status("idle", "Waiting for audio device")

    def publish_status(self, status: str, message: str = ""):
        """Publish microphone status"""
        msg = String()
        msg.data = f"{status}: {message}" if message else status
        self.status_pub.publish(msg)
        self.get_logger().debug(f"Status: {status} - {message}")

    def destroy_node(self):
        """Cleanup on shutdown"""
        if self.arecord_process:
            self.arecord_process.terminate()
            self.arecord_process.wait(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = USBMicrophoneNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
