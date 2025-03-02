import numpy as np
from math import cos, sin
from typing import Dict
import time
from sensors.ms5611 import MS5611
from sensors.bmp280 import BMP280
from sensors.mpu9250 import MPU9250

class Orientation:
    def __init__(self):
        # Initialize orientation tracking
        self.roll = 0.0   # Rotation around X axis
        self.pitch = 0.0  # Rotation around Y axis
        self.yaw = 0.0    # Rotation around Z axis
        self.last_time = None

    def get_rotation_matrix(self) -> np.ndarray:
        """Calculate rotation matrix from Euler angles."""
        # Create rotation matrices for each axis
        cos_r, sin_r = cos(self.roll), sin(self.roll)
        cos_p, sin_p = cos(self.pitch), sin(self.pitch)
        cos_y, sin_y = cos(self.yaw), sin(self.yaw)
        
        # Rotation matrix around X axis (roll)
        R_x = np.array([
            [1, 0, 0],
            [0, cos_r, -sin_r],
            [0, sin_r, cos_r]
        ])
        
        # Rotation matrix around Y axis (pitch)
        R_y = np.array([
            [cos_p, 0, sin_p],
            [0, 1, 0],
            [-sin_p, 0, cos_p]
        ])
        
        # Rotation matrix around Z axis (yaw)
        R_z = np.array([
            [cos_y, -sin_y, 0],
            [sin_y, cos_y, 0],
            [0, 0, 1]
        ])
        
        # Combined rotation matrix
        R = R_z @ R_y @ R_x
        return R

    def update_orientation(self, gyro: Dict[str, float], current_time: float) -> None:
        """Update orientation based on gyroscope readings."""
        if self.last_time is None:
            self.last_time = current_time
            return
        
        # Calculate time step
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # Convert gyroscope readings from degrees to radians
        self.roll += np.radians(gyro['x']) * dt
        self.pitch += np.radians(gyro['y']) * dt
        self.yaw += np.radians(gyro['z']) * dt

    def transform_to_world_frame(self, accel: Dict[str, float], gyro: Dict[str, float]) -> Dict[str, float]:
        """Transform accelerometer and gyroscope data to world frame."""
        # Get rotation matrix
        R = self.get_rotation_matrix()
        
        # Convert accelerometer readings to numpy array
        accel_vector = np.array([accel['x'], accel['y'], accel['z']])
        
        # Rotate accelerometer readings to world frame
        accel_world = R @ accel_vector
        
        # Convert gyroscope readings to numpy array
        gyro_vector = np.array([gyro['x'], gyro['y'], gyro['z']])
        
        # Rotate gyroscope readings to world frame
        gyro_world = R @ gyro_vector
        
        return {
            'accel_x': accel_world[0],
            'accel_y': accel_world[1],
            'accel_z': accel_world[2],
            'gyro_x': gyro_world[0],
            'gyro_y': gyro_world[1],
            'gyro_z': gyro_world[2],
            'roll': np.degrees(self.roll),
            'pitch': np.degrees(self.pitch),
            'yaw': np.degrees(self.yaw)
        }

    def print_orientation(self):
        """Print the 3D directions and quaternion matrix."""
        R = self.get_rotation_matrix()
        print("3D Orientation (Rotation Matrix):")
        print(R)
        print("\nEuler Angles (Degrees):")
        print(f"Roll: {np.degrees(self.roll):.2f}, Pitch: {np.degrees(self.pitch):.2f}, Yaw: {np.degrees(self.yaw):.2f}")

    def calculate_magnitude_in_g(self, accel: Dict[str, float]) -> float:
        """Calculate the magnitude of the acceleration vector in g."""
        x, y, z = accel['x'], accel['y'], accel['z']
        magnitude_m_s2 = np.sqrt(x**2 + y**2 + z**2)
        magnitude_g = magnitude_m_s2 / 9.81  # Convert to g
        return magnitude_g

    def print_orientation_and_magnitude(self, accel: Dict[str, float]):
        """Print the 3D directions, quaternion matrix, and acceleration magnitude in g."""
        self.print_orientation()
        magnitude_g = self.calculate_magnitude_in_g(accel)
        print(f"Acceleration Magnitude: {magnitude_g:.2f} g")

def initialize_sensors():
    """Initialize all sensors and return their instances."""
    ms5611 = MS5611(bus=0)
    bmp280 = BMP280(bus=1)
    mpu9250 = MPU9250(bus=1)
    orientation = Orientation()
    print("All sensors initialized")
    return ms5611, bmp280, mpu9250, orientation

def main():
    # Initialize sensors and orientation
    ms5611, bmp280, mpu9250, orientation = initialize_sensors()
    
    print("Waiting for sensors to stabilize...")
    time.sleep(1)
    
    try:
        while True:
            start_time = time.time()
            
            # Read sensor data
            accel, gyro = mpu9250.read_sensor_data()
            
            # Update orientation
            current_time = time.time()
            orientation.update_orientation(gyro, current_time)
            
            # Print orientation and magnitude
            orientation.print_orientation_and_magnitude(accel)
            
            # Calculate elapsed time and sleep to maintain 60 Hz
            elapsed_time = time.time() - start_time
            sleep_time = max(0, (1/60) - elapsed_time)
            time.sleep(sleep_time)
    except KeyboardInterrupt:
        print("\nProgram terminated by user")

if __name__ == "__main__":
    main() 