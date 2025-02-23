from smbus2 import SMBus
import time
from .constants import *

class MPU9250:
    """MPU9250 9-axis motion sensor."""
    
    def __init__(self):
        """Initialize the MPU9250 sensor."""
        self.bus = SMBus(I2C_BUS_1)
        self.init_mpu9250()

    def init_mpu9250(self):
        """Initialize the sensor with specific settings."""
        try:
            # Check MPU9250 ID
            who_am_i = self.bus.read_byte_data(MPU9250_ADDR, MPU9250_WHO_AM_I)
            if who_am_i != 0x75:    # Maybe this chip is MPU6050
                raise RuntimeError(f"MPU9250 WHO_AM_I returned 0x{who_am_i:02X} instead of 0x71")

            # Wake up the MPU9250
            self.bus.write_byte_data(MPU9250_ADDR, MPU9250_PWR_MGMT_1, 0x00)
            time.sleep(0.1)
            
            # Configure MPU9250
            # Gyro config: ±250 degrees/second
            self.bus.write_byte_data(MPU9250_ADDR, MPU9250_GYRO_CONFIG, 0x00)
            # Accel config: ±2g
            self.bus.write_byte_data(MPU9250_ADDR, MPU9250_ACCEL_CONFIG, 0x00)
            # Set DLPF bandwidth to 41Hz
            self.bus.write_byte_data(MPU9250_ADDR, MPU9250_CONFIG, 0x03)
            
            print("MPU9250 initialized")
            
        except Exception as e:
            raise RuntimeError(f"Failed to initialize MPU9250: {e}")

    def read_sensor_data(self):
        """
        Read accelerometer and gyroscope data.
        Returns: (accelerometer dict, gyroscope dict)
        """
        try:
            # Read accelerometer data
            accel_data = self.bus.read_i2c_block_data(MPU9250_ADDR, MPU9250_ACCEL_XOUT_H, 6)
            # Read gyroscope data
            gyro_data = self.bus.read_i2c_block_data(MPU9250_ADDR, MPU9250_GYRO_XOUT_H, 6)

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
