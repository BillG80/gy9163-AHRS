from smbus2 import SMBus
import time
from .constants import *
import numpy as np
from math import cos, sin
from typing import Dict, Tuple

class MPU9250:
    """MPU9250 9-axis motion sensor."""
    
    def __init__(self, bus: int, address: int = MPU9250_ADDR) -> None:
        """Initialize MPU9250 sensor.
        
        Args:
            bus: I2C bus number
            address: I2C device address (default: 0x68)
        """
        try:
            self.bus = SMBus(bus)
            self.address = address
            
            # Initialize sensor
            self.reset()
            time.sleep(0.1)  # Wait after reset
            self.configure()
            print("MPU9250 initialized")
            
            # Initialize orientation tracking
            self.roll = 0.0   # Rotation around X axis
            self.pitch = 0.0  # Rotation around Y axis
            self.yaw = 0.0    # Rotation around Z axis
            self.last_time = time.time()
            
        except Exception as e:
            print(f"Failed to initialize MPU9250: {e}")
            raise

    def reset(self) -> None:
        """Reset the MPU9250."""
        try:
            self.bus.write_byte_data(self.address, MPU9250_PWR_MGMT_1, 0x80)  # Reset all registers
            time.sleep(0.1)
            # Clear sleep mode and set clock source
            self.bus.write_byte_data(self.address, MPU9250_PWR_MGMT_1, 0x01)
            time.sleep(0.1)
        except Exception as e:
            print(f"Error resetting MPU9250: {e}")
            raise

    def configure(self) -> None:
        """Configure MPU9250 settings."""
        try:
            # Configure gyroscope to ±500°/s
            self.bus.write_byte_data(self.address, MPU9250_GYRO_CONFIG, 0x08)  # Gyro ±500°/s
            # Configure accelerometer and other settings as needed
            # self.bus.write_byte_data(self.address, MPU9250_ACCEL_CONFIG, 0x18)  # Example for Accel ±16g
            # self.bus.write_byte_data(self.address, MPU9250_CONFIG, 0x03)  # DLPF 41Hz
            
            # Enable I2C bypass to access magnetometer
            self.bus.write_byte_data(self.address, MPU9250_INT_PIN_CFG, 0x02)
            time.sleep(0.1)
            
        except Exception as e:
            print(f"Error configuring MPU9250: {e}")
            raise

    def read_sensor_data(self):
        """
        Read accelerometer and gyroscope data.
        Returns: (accelerometer dict, gyroscope dict)
        """
        try:
            # Read accelerometer data
            accel_data = self.bus.read_i2c_block_data(self.address, MPU9250_ACCEL_XOUT_H, 6)
            # Read gyroscope data
            gyro_data = self.bus.read_i2c_block_data(self.address, MPU9250_GYRO_XOUT_H, 6)

            # Convert to signed values and scale
            accel = {
                'x': self.convert_raw_to_g(accel_data[0], accel_data[1], 16384.0),  # ±2g
                'y': self.convert_raw_to_g(accel_data[2], accel_data[3], 16384.0),
                'z': self.convert_raw_to_g(accel_data[4], accel_data[5], 16384.0)
            }

            gyro = {
                'x': self.convert_raw_to_dps(gyro_data[0], gyro_data[1], 131.0),  # ±250°/s
                'y': self.convert_raw_to_dps(gyro_data[2], gyro_data[3], 131.0),
                'z': self.convert_raw_to_dps(gyro_data[4], gyro_data[5], 131.0)
            }

            return accel, gyro
            
        except Exception as e:
            raise RuntimeError(f"Failed to read MPU9250 sensor data: {e}")

    def convert_raw_to_g(self, msb, lsb, scale):
        """Convert raw accelerometer data to g."""
        value = self.convert_raw_to_int(msb, lsb)
        return value / scale

    def convert_raw_to_dps(self, msb, lsb, scale):
        """Convert raw gyroscope data to degrees per second."""
        value = self.convert_raw_to_int(msb, lsb)
        return value / scale

    def convert_raw_to_int(self, msb, lsb):
        """Convert raw sensor data to signed integer."""
        value = (msb << 8) | lsb
        if value >= 0x8000:
            value -= 0x10000
        return value

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

    def update_orientation(self, gyro: Dict[str, float], dt: float) -> None:
        """Update orientation based on gyroscope readings."""
        # Convert gyroscope readings from degrees to radians
        self.roll += np.radians(gyro['x']) * dt
        self.pitch += np.radians(gyro['y']) * dt
        self.yaw += np.radians(gyro['z']) * dt

    def read_all(self) -> Dict[str, float]:
        """Read all sensor data and return in world frame."""
        try:
            # Get raw sensor readings
            accel, gyro = self.read_sensor_data()
            
            # Calculate time step
            current_time = time.time()
            dt = current_time - self.last_time
            self.last_time = current_time
            
            # Update orientation using gyroscope data
            self.update_orientation(gyro, dt)
            
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
                'mag_x': 0,
                'mag_y': 0,
                'mag_z': 0,
                'roll': np.degrees(self.roll),
                'pitch': np.degrees(self.pitch),
                'yaw': np.degrees(self.yaw)
            }
            
        except Exception as e:
            raise RuntimeError(f"Failed to read MPU9250 data: {e}")
