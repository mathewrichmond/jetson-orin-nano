#!/usr/bin/env python3
"""
Kalman Filter for IMU Data Smoothing
Shared Kalman filter implementation for sensor fusion
"""

# Third-party
import numpy as np


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

    def reset(self):
        """Reset filter state"""
        self.state = np.zeros(6)
        self.covariance = np.eye(6) * 1.0
