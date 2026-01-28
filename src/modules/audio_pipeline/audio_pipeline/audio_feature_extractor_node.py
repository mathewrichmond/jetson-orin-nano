#!/usr/bin/env python3
"""
Audio Feature Extractor Node

Extracts audio features for VLA model consumption:
- MFCC (Mel-Frequency Cepstral Coefficients)
- Spectrograms
- Audio embeddings
- Voice activity detection
"""

# Standard library
from typing import Optional

# Third-party
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, UInt8MultiArray

# Local
from isaac_utils import HealthStatusPublisher, InputWatchdog


class AudioFeatureExtractorNode(Node):
    """Extracts audio features for VLA model"""

    def __init__(self):
        super().__init__("audio_feature_extractor_node")

        # Parameters
        self.declare_parameter("audio_topic", "/sensor_fusion/audio/raw")
        self.declare_parameter("output_namespace", "/audio")
        self.declare_parameter("sample_rate", 16000)
        self.declare_parameter("channels", 2)
        self.declare_parameter("format", "S16_LE")
        
        # Feature extraction parameters
        self.declare_parameter("mfcc_coefficients", 13)  # Number of MFCC coefficients
        self.declare_parameter("fft_size", 2048)  # FFT window size
        self.declare_parameter("hop_length", 512)  # Hop length for STFT
        self.declare_parameter("enable_vad", True)  # Voice activity detection
        self.declare_parameter("vad_threshold", 0.5)  # VAD energy threshold
        
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
        self.mfcc_coefficients = int(self.get_parameter("mfcc_coefficients").value)
        self.fft_size = int(self.get_parameter("fft_size").value)
        self.hop_length = int(self.get_parameter("hop_length").value)
        self.enable_vad = bool(self.get_parameter("enable_vad").value)
        self.vad_threshold = float(self.get_parameter("vad_threshold").value)
        
        health_topic = str(self.get_parameter("health_topic").value)
        health_rate = float(self.get_parameter("health_publish_rate").value)
        health_warn_timeout = float(self.get_parameter("health_warn_timeout_sec").value)
        health_stale_timeout = float(self.get_parameter("health_stale_timeout_sec").value)
        health_expected_audio_rate = float(
            self.get_parameter("health_expected_audio_rate_hz").value
        )

        # Publishers
        self.mfcc_pub = self.create_publisher(
            Float32MultiArray, f"{self.output_namespace}/features/mfcc", 10
        )
        self.spectrogram_pub = self.create_publisher(
            Float32MultiArray, f"{self.output_namespace}/features/spectrogram", 10
        )
        self.vad_pub = self.create_publisher(
            Float32MultiArray, f"{self.output_namespace}/features/vad", 10
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

        self.get_logger().info(
            f"Audio feature extractor started: {self.channels} ch, {self.sample_rate}Hz, "
            f"MFCC={self.mfcc_coefficients}, FFT={self.fft_size}"
        )

    def _audio_callback(self, msg: UInt8MultiArray):
        """Process audio and extract features"""
        try:
            self.health.record_input("audio_stream")

            # Convert bytes to numpy array
            audio_bytes = np.frombuffer(bytes(msg.data), dtype=np.uint8)

            if len(audio_bytes) < 2:
                return

            # Convert to int16 samples (S16_LE format)
            samples = np.frombuffer(audio_bytes, dtype=np.int16).astype(np.float32)

            # Normalize to [-1, 1]
            samples = samples / 32768.0

            # Split stereo to mono if needed
            if self.channels == 2:
                samples = samples[::2]  # Take left channel

            # Extract features
            mfcc_features = self._extract_mfcc(samples)
            spectrogram = self._extract_spectrogram(samples)
            vad_result = self._voice_activity_detection(samples)

            # Publish features
            self._publish_mfcc(mfcc_features)
            self._publish_spectrogram(spectrogram)
            if self.enable_vad:
                self._publish_vad(vad_result)

        except Exception as e:
            self.get_logger().warn(f"Error processing audio: {e}")

    def _extract_mfcc(self, samples: np.ndarray) -> np.ndarray:
        """
        Extract MFCC features (placeholder implementation)
        
        Real implementation would use librosa or similar:
        mfcc = librosa.feature.mfcc(y=samples, sr=self.sample_rate, 
                                     n_mfcc=self.mfcc_coefficients)
        """
        try:
            # Simple placeholder: compute FFT and take first N coefficients
            fft = np.fft.rfft(samples, n=self.fft_size)
            power = np.abs(fft) ** 2
            mel_coeffs = power[: self.mfcc_coefficients]
            
            # Log scale
            mfcc = np.log10(mel_coeffs + 1e-10)
            return mfcc
        except Exception as e:
            self.get_logger().warn(f"Error extracting MFCC: {e}")
            return np.zeros(self.mfcc_coefficients, dtype=np.float32)

    def _extract_spectrogram(self, samples: np.ndarray) -> np.ndarray:
        """
        Extract spectrogram (placeholder implementation)
        
        Real implementation would use librosa or scipy:
        f, t, Sxx = scipy.signal.spectrogram(samples, fs=self.sample_rate)
        """
        try:
            # Simple placeholder: compute short-time FFT
            # Number of windows
            n_windows = max(1, len(samples) // self.hop_length)
            n_freqs = self.fft_size // 2 + 1
            
            spectrogram = np.zeros((n_freqs, n_windows), dtype=np.float32)
            
            for i in range(n_windows):
                start = i * self.hop_length
                end = min(start + self.fft_size, len(samples))
                window = samples[start:end]
                
                # Pad if needed
                if len(window) < self.fft_size:
                    window = np.pad(window, (0, self.fft_size - len(window)))
                
                # Compute FFT
                fft = np.fft.rfft(window)
                spectrogram[:, i] = np.abs(fft)
            
            # Convert to dB scale
            spectrogram = 20 * np.log10(spectrogram + 1e-10)
            return spectrogram
        except Exception as e:
            self.get_logger().warn(f"Error extracting spectrogram: {e}")
            return np.zeros((self.fft_size // 2 + 1, 1), dtype=np.float32)

    def _voice_activity_detection(self, samples: np.ndarray) -> dict:
        """
        Simple energy-based voice activity detection
        
        Real implementation would use WebRTC VAD or similar
        """
        try:
            # Compute energy
            energy = np.mean(samples ** 2)
            
            # Threshold-based detection
            is_voice = energy > self.vad_threshold
            
            return {
                "is_voice": float(is_voice),
                "energy": float(energy),
                "confidence": float(min(1.0, energy / self.vad_threshold))
            }
        except Exception as e:
            self.get_logger().warn(f"Error in VAD: {e}")
            return {"is_voice": 0.0, "energy": 0.0, "confidence": 0.0}

    def _publish_mfcc(self, mfcc: np.ndarray):
        """Publish MFCC features"""
        try:
            msg = Float32MultiArray()
            msg.data = mfcc.flatten().tolist()
            
            # Add dimension info
            dim = MultiArrayDimension()
            dim.label = "mfcc"
            dim.size = len(mfcc)
            dim.stride = 1
            msg.layout.dim = [dim]
            msg.layout.data_offset = 0
            
            self.mfcc_pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f"Error publishing MFCC: {e}")

    def _publish_spectrogram(self, spectrogram: np.ndarray):
        """Publish spectrogram"""
        try:
            msg = Float32MultiArray()
            msg.data = spectrogram.flatten().tolist()
            
            # Add dimension info
            dim_freq = MultiArrayDimension()
            dim_freq.label = "frequency"
            dim_freq.size = spectrogram.shape[0]
            dim_freq.stride = spectrogram.shape[0] * spectrogram.shape[1]
            
            dim_time = MultiArrayDimension()
            dim_time.label = "time"
            dim_time.size = spectrogram.shape[1]
            dim_time.stride = spectrogram.shape[1]
            
            msg.layout.dim = [dim_freq, dim_time]
            msg.layout.data_offset = 0
            
            self.spectrogram_pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f"Error publishing spectrogram: {e}")

    def _publish_vad(self, vad_result: dict):
        """Publish voice activity detection result"""
        try:
            msg = Float32MultiArray()
            msg.data = [
                vad_result["is_voice"],
                vad_result["energy"],
                vad_result["confidence"]
            ]
            
            # Add labels
            labels = ["is_voice", "energy", "confidence"]
            for i, label in enumerate(labels):
                dim = MultiArrayDimension()
                dim.label = label
                dim.size = 1
                dim.stride = 1
                msg.layout.dim.append(dim)
            
            msg.layout.data_offset = 0
            
            self.vad_pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f"Error publishing VAD: {e}")


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    node = AudioFeatureExtractorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
