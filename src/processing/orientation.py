import numpy as np
from math import cos, sin
from typing import Dict, Tuple
import time

class EMAFilter:
    def __init__(self, alpha=0.1):
        self.alpha = alpha
        self.value = None
    
    def update(self, measurement):
        if self.value is None:
            self.value = measurement
        else:
            self.value = self.alpha * measurement + (1 - self.alpha) * self.value
        return self.value

class Orientation:
    def __init__(self):
        # Initialize orientation tracking
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.last_time = None
        
        # Initialize EMA filter with strong smoothing
        self.ema_magnitude = EMAFilter(alpha=0.05)
        
        # Calibration value for gravity
        self.gravity_magnitude = 0.965  # Based on observed values

    def update_orientation(self, gyro: Dict[str, float], current_time: float) -> None:
        """Update orientation based on gyroscope readings."""
        if self.last_time is None:
            self.last_time = current_time
            return
        
        # Calculate time step
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # Convert gyroscope readings from degrees to radians and update angles
        self.roll += np.radians(gyro['x']) * dt
        self.pitch += np.radians(gyro['y']) * dt
        self.yaw += np.radians(gyro['z']) * dt
        
        # Normalize angles to [-π, π]
        self.roll = np.arctan2(np.sin(self.roll), np.cos(self.roll))
        self.pitch = np.arctan2(np.sin(self.pitch), np.cos(self.pitch))
        self.yaw = np.arctan2(np.sin(self.yaw), np.cos(self.yaw))

    def calculate_magnitude(self, accel: Dict[str, float]) -> float:
        """Calculate filtered magnitude with gravity removed."""
        # Calculate raw magnitude
        magnitude = np.sqrt(accel['x']**2 + accel['y']**2 + accel['z']**2)
        
        # Remove gravity and ensure non-negative
        magnitude = max(0, magnitude - self.gravity_magnitude)
        
        # Apply EMA filter
        return self.ema_magnitude.update(magnitude)

    def get_linear_acceleration(self, accel: Dict[str, float]) -> Dict[str, float]:
        """Get acceleration magnitude with gravity removed."""
        magnitude = self.calculate_magnitude(accel)
        return {'magnitude': magnitude}

    def get_orientation(self) -> Dict[str, float]:
        """Get current orientation angles in degrees."""
        return {
            'roll': np.degrees(self.roll),
            'pitch': np.degrees(self.pitch),
            'yaw': np.degrees(self.yaw)
        }