#!/usr/bin/env python3
"""
Speech Recognition Node

Converts audio to text using speech recognition engines:
- Placeholder for Whisper, DeepSpeech, or cloud STT services
- Handles continuous audio stream
- Publishes transcriptions with timestamps
"""

# Standard library
from collections import deque
import time
from typing import Optional

# Third-party
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, UInt8MultiArray

# Local
from isaac_utils import HealthStatusPublisher, InputWatchdog


class SpeechRecognitionNode(Node):
    """Speech-to-text processing node"""

    def __init__(self):
        super().__init__("speech_recognition_node")

        # Parameters
        self.declare_parameter("audio_topic", "/sensor_fusion/audio/raw")
        self.declare_parameter("output_namespace", "/audio")
        self.declare_parameter("sample_rate", 16000)
        self.declare_parameter("channels", 2)
        self.declare_parameter("format", "S16_LE")
        
        # Speech recognition parameters
        self.declare_parameter("recognition_engine", "placeholder")  # whisper, deepspeech, google, etc.
        self.declare_parameter("language", "en-US")
        self.declare_parameter("model_size", "base")  # tiny, base, small, medium, large
        self.declare_parameter("buffer_duration_sec", 2.0)  # Audio buffer size for recognition
        self.declare_parameter("enable_continuous", True)  # Continuous recognition
        self.declare_parameter("vad_enabled", True)  # Use VAD to trigger recognition
        
        # Health monitoring
        self.declare_parameter("health_topic", f"health/{self.get_name()}")
        self.declare_parameter("health_publish_rate", 1.0)
        self.declare_parameter("health_warn_timeout_sec", 3.0)
        self.declare_parameter("health_stale_timeout_sec", 6.0)
        self.declare_parameter("health_expected_audio_rate_hz", 10.0)

        # Get parameters
        self.audio_topic = str(self.get_parameter("audio_topic").value)
        self.output_namespace = str(self.get_parameter("output_namespace").value)
        self.sample_rate = int(self.get_parameter("sample_rate").value)
        self.channels = int(self.get_parameter("channels").value)
        self.format = str(self.get_parameter("format").value)
        self.recognition_engine = str(self.get_parameter("recognition_engine").value)
        self.language = str(self.get_parameter("language").value)
        self.model_size = str(self.get_parameter("model_size").value)
        self.buffer_duration = float(self.get_parameter("buffer_duration_sec").value)
        self.enable_continuous = bool(self.get_parameter("enable_continuous").value)
        self.vad_enabled = bool(self.get_parameter("vad_enabled").value)
        
        health_topic = str(self.get_parameter("health_topic").value)
        health_rate = float(self.get_parameter("health_publish_rate").value)
        health_warn_timeout = float(self.get_parameter("health_warn_timeout_sec").value)
        health_stale_timeout = float(self.get_parameter("health_stale_timeout_sec").value)
        health_expected_audio_rate = float(
            self.get_parameter("health_expected_audio_rate_hz").value
        )

        # Audio buffer (circular buffer for recognition window)
        buffer_samples = int(self.buffer_duration * self.sample_rate)
        self.audio_buffer = deque(maxlen=buffer_samples)
        
        # State
        self.last_recognition_time = 0.0
        self.recognition_count = 0

        # Publishers
        self.transcription_pub = self.create_publisher(
            String, f"{self.output_namespace}/transcription", 10
        )
        self.transcription_confidence_pub = self.create_publisher(
            String, f"{self.output_namespace}/transcription_confidence", 10
        )

        # Subscriber
        self.audio_sub = self.create_subscription(
            UInt8MultiArray, self.audio_topic, self._audio_callback, 10
        )

        # Health monitoring
        self.health = HealthStatusPublisher(self, health_topic, health_rate)
        self.health.add_watchdog(
            InputWatchdog(
                "audio_stream",
                expected_rate_hz=health_expected_audio_rate,
                warn_timeout_sec=health_warn_timeout,
                error_timeout_sec=health_stale_timeout,
            )
        )

        # Initialize recognition engine (placeholder)
        self._init_recognition_engine()

        self.get_logger().info(
            f"Speech recognition started: engine={self.recognition_engine}, "
            f"language={self.language}, model={self.model_size}"
        )

    def _init_recognition_engine(self):
        """Initialize speech recognition engine (placeholder)"""
        if self.recognition_engine == "placeholder":
            self.get_logger().info("Using placeholder STT engine (no actual recognition)")
            self.model = None
        elif self.recognition_engine == "whisper":
            self.get_logger().warn(
                "Whisper engine not yet implemented - install: pip install openai-whisper"
            )
            self.model = None
            # Future: import whisper; self.model = whisper.load_model(self.model_size)
        elif self.recognition_engine == "deepspeech":
            self.get_logger().warn(
                "DeepSpeech engine not yet implemented - install: pip install deepspeech"
            )
            self.model = None
            # Future: from deepspeech import Model; self.model = Model(model_path)
        else:
            self.get_logger().error(f"Unknown recognition engine: {self.recognition_engine}")
            self.model = None

    def _audio_callback(self, msg: UInt8MultiArray):
        """Buffer audio and trigger recognition"""
        try:
            self.health.record_input("audio_stream")

            # Convert bytes to numpy array
            audio_bytes = np.frombuffer(bytes(msg.data), dtype=np.uint8)

            if len(audio_bytes) < 2:
                return

            # Convert to int16 samples (S16_LE format)
            samples = np.frombuffer(audio_bytes, dtype=np.int16)

            # Add to buffer
            self.audio_buffer.extend(samples)

            # Trigger recognition if buffer is full and continuous mode enabled
            current_time = time.time()
            if self.enable_continuous and len(self.audio_buffer) >= self.audio_buffer.maxlen:
                # Throttle recognition (don't run too frequently)
                if (current_time - self.last_recognition_time) > 0.5:  # Max 2Hz
                    self._perform_recognition()
                    self.last_recognition_time = current_time

        except Exception as e:
            self.get_logger().warn(f"Error buffering audio: {e}")

    def _perform_recognition(self):
        """Perform speech recognition on buffered audio"""
        try:
            # Convert buffer to numpy array
            audio_samples = np.array(self.audio_buffer, dtype=np.int16)

            # Normalize to float32 [-1, 1]
            audio_float = audio_samples.astype(np.float32) / 32768.0

            # Split stereo to mono if needed
            if self.channels == 2:
                audio_float = audio_float[::2]  # Take left channel

            # Perform recognition based on engine
            if self.recognition_engine == "placeholder":
                transcription, confidence = self._placeholder_recognition(audio_float)
            elif self.recognition_engine == "whisper":
                transcription, confidence = self._whisper_recognition(audio_float)
            elif self.recognition_engine == "deepspeech":
                transcription, confidence = self._deepspeech_recognition(audio_float)
            else:
                return

            # Publish if non-empty
            if transcription and transcription.strip():
                self._publish_transcription(transcription, confidence)
                self.recognition_count += 1

        except Exception as e:
            self.get_logger().warn(f"Error performing recognition: {e}")

    def _placeholder_recognition(self, audio: np.ndarray) -> tuple:
        """Placeholder recognition (returns mock result)"""
        # Compute simple audio energy
        energy = np.mean(audio ** 2)
        
        # Mock transcription based on energy
        if energy > 0.01:
            transcription = f"[MOCK] Audio detected (energy={energy:.4f})"
            confidence = min(1.0, energy * 10)
        else:
            transcription = ""
            confidence = 0.0
        
        return transcription, confidence

    def _whisper_recognition(self, audio: np.ndarray) -> tuple:
        """Whisper-based recognition (not yet implemented)"""
        # Future implementation:
        # result = self.model.transcribe(audio, language=self.language)
        # return result["text"], result.get("confidence", 1.0)
        
        self.get_logger().warn("Whisper recognition not yet implemented")
        return "", 0.0

    def _deepspeech_recognition(self, audio: np.ndarray) -> tuple:
        """DeepSpeech-based recognition (not yet implemented)"""
        # Future implementation:
        # transcription = self.model.stt(audio)
        # confidence = 1.0  # DeepSpeech doesn't provide confidence
        # return transcription, confidence
        
        self.get_logger().warn("DeepSpeech recognition not yet implemented")
        return "", 0.0

    def _publish_transcription(self, text: str, confidence: float):
        """Publish transcription result"""
        try:
            # Publish transcription
            transcription_msg = String()
            transcription_msg.data = text
            self.transcription_pub.publish(transcription_msg)

            # Publish confidence
            confidence_msg = String()
            confidence_msg.data = f"{text} [confidence: {confidence:.2f}]"
            self.transcription_confidence_pub.publish(confidence_msg)

            self.get_logger().info(
                f"Transcription #{self.recognition_count}: '{text}' (confidence={confidence:.2f})"
            )

        except Exception as e:
            self.get_logger().warn(f"Error publishing transcription: {e}")


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    node = SpeechRecognitionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
