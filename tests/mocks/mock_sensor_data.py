"""
Mock Sensor Data Generators

Provides mock data generators for testing without hardware.
"""

import numpy as np
from typing import Tuple, Optional
import time


class MockIMUData:
    """Generate mock IMU data"""
    
    def __init__(self, frequency: float = 50.0, noise_level: float = 0.01):
        self.frequency = frequency
        self.noise_level = noise_level
        self.time_start = time.time()
    
    def generate(self) -> dict:
        """Generate mock IMU reading"""
        t = time.time() - self.time_start
        
        # Simulate some motion with noise
        accel_x = 0.1 * np.sin(2 * np.pi * 0.5 * t) + np.random.normal(0, self.noise_level)
        accel_y = 0.1 * np.cos(2 * np.pi * 0.5 * t) + np.random.normal(0, self.noise_level)
        accel_z = 9.81 + np.random.normal(0, self.noise_level)
        
        gyro_x = 0.01 * np.sin(2 * np.pi * 0.2 * t) + np.random.normal(0, self.noise_level * 0.1)
        gyro_y = 0.01 * np.cos(2 * np.pi * 0.2 * t) + np.random.normal(0, self.noise_level * 0.1)
        gyro_z = 0.0 + np.random.normal(0, self.noise_level * 0.1)
        
        return {
            "timestamp": time.time(),
            "linear_acceleration": {
                "x": float(accel_x),
                "y": float(accel_y),
                "z": float(accel_z),
            },
            "angular_velocity": {
                "x": float(gyro_x),
                "y": float(gyro_y),
                "z": float(gyro_z),
            },
        }


class MockCameraData:
    """Generate mock camera data"""
    
    def __init__(
        self,
        width: int = 640,
        height: int = 480,
        fps: float = 30.0,
    ):
        self.width = width
        self.height = height
        self.fps = fps
        self.frame_count = 0
    
    def generate(self) -> np.ndarray:
        """Generate mock camera frame"""
        # Generate a simple pattern
        frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        
        # Add some dynamic content
        offset = (self.frame_count * 10) % self.width
        frame[:, offset:offset+50, :] = 255
        
        self.frame_count += 1
        
        return frame


class MockAudioData:
    """Generate mock audio data"""
    
    def __init__(
        self,
        sample_rate: int = 16000,
        channels: int = 1,
    ):
        self.sample_rate = sample_rate
        self.channels = channels
        self.time_start = time.time()
    
    def generate(self, duration_sec: float = 0.1) -> np.ndarray:
        """Generate mock audio chunk"""
        t = time.time() - self.time_start
        num_samples = int(duration_sec * self.sample_rate)
        
        # Generate sine wave with some noise
        samples = np.arange(num_samples) / self.sample_rate + t
        audio = 0.1 * np.sin(2 * np.pi * 440 * samples)  # 440 Hz tone
        audio += 0.01 * np.random.randn(num_samples)  # Noise
        
        if self.channels == 2:
            audio = np.column_stack([audio, audio])
        
        return audio.astype(np.float32)


class MockBatteryData:
    """Generate mock battery data"""
    
    def __init__(self, initial_voltage: float = 12.6):
        self.voltage = initial_voltage
        self.time_start = time.time()
    
    def generate(self) -> dict:
        """Generate mock battery reading"""
        t = time.time() - self.time_start
        
        # Simulate slow discharge with noise
        discharge_rate = 0.001  # V/s
        self.voltage = max(10.0, self.voltage - discharge_rate * t / 3600)
        voltage_reading = self.voltage + np.random.normal(0, 0.05)
        
        # Current varies with activity
        current = 2.0 + 1.0 * np.sin(2 * np.pi * 0.1 * t) + np.random.normal(0, 0.1)
        
        # Temperature varies slowly
        temp = 25.0 + 5.0 * np.sin(2 * np.pi * 0.01 * t) + np.random.normal(0, 0.5)
        
        return {
            "timestamp": time.time(),
            "voltage": float(voltage_reading),
            "current": float(current),
            "temperature": float(temp),
            "capacity_percent": float((voltage_reading - 10.0) / (12.6 - 10.0) * 100),
        }


class MockOdometryData:
    """Generate mock odometry data"""
    
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = time.time()
    
    def generate(self, linear_vel: float = 0.0, angular_vel: float = 0.0) -> dict:
        """Generate mock odometry reading"""
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # Update pose
        self.x += linear_vel * np.cos(self.theta) * dt
        self.y += linear_vel * np.sin(self.theta) * dt
        self.theta += angular_vel * dt
        
        # Add noise
        x_noisy = self.x + np.random.normal(0, 0.01)
        y_noisy = self.y + np.random.normal(0, 0.01)
        theta_noisy = self.theta + np.random.normal(0, 0.01)
        
        return {
            "timestamp": current_time,
            "pose": {
                "x": float(x_noisy),
                "y": float(y_noisy),
                "theta": float(theta_noisy),
            },
            "twist": {
                "linear": float(linear_vel + np.random.normal(0, 0.01)),
                "angular": float(angular_vel + np.random.normal(0, 0.001)),
            },
        }
