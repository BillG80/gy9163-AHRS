from smbus2 import SMBus
import time
from .constants import *

class MS5611:
    """MS5611 high precision barometric pressure sensor."""
    
    def __init__(self):
        """Initialize the MS5611 sensor."""
        self.bus = SMBus(I2C_BUS_0)
        self.calibration = {}
        self.reset()
        self.read_calibration()

    def reset(self):
        """Reset the sensor."""
        try:
            self.bus.write_byte(MS5611_ADDR, MS5611_RESET)
            time.sleep(0.1)
        except Exception as e:
            raise RuntimeError(f"Failed to reset MS5611: {e}")

    def read_calibration(self):
        """Read factory calibration data."""
        try:
            # Read PROM (6 coefficients + factory data + CRC)
            for i in range(8):
                data = self.bus.read_i2c_block_data(MS5611_ADDR, MS5611_PROM_READ + (i * 2), 2)
                self.calibration[i] = (data[0] << 8) | data[1]
            
            # Verify factory value
            if (self.calibration[0] & 0xFF00) != 0x1000:
                raise RuntimeError("MS5611 factory data incorrect")
                
            # Print calibration data for verification (C1-C6 are the actual coefficients)
            print("MS5611 calibration data:")
            for i in range(1, 7):
                print(f"C{i}: {self.calibration[i]}")
                
        except Exception as e:
            raise RuntimeError(f"Failed to read MS5611 calibration: {e}")

    def read_adc(self, command):
        """Read ADC value."""
        try:
            self.bus.write_byte(MS5611_ADDR, command)
            time.sleep(0.01)  # Wait for conversion
            data = self.bus.read_i2c_block_data(MS5611_ADDR, MS5611_ADC_READ, 3)
            return (data[0] << 16) | (data[1] << 8) | data[2]
        except Exception as e:
            raise RuntimeError(f"Failed to read MS5611 ADC: {e}")

    def get_temperature_and_pressure(self):
        """
        Read and calculate temperature and pressure.
        Returns: (temperature in °C, pressure in hPa)
        """
        try:
            D1 = self.read_adc(MS5611_CONVERT_D1)  # Pressure
            D2 = self.read_adc(MS5611_CONVERT_D2)  # Temperature

            # Calculate temperature
            dT = D2 - (self.calibration[5] << 8)
            TEMP = 2000 + ((dT * self.calibration[6]) >> 23)

            # Calculate temperature compensated pressure
            OFF = (self.calibration[2] << 16) + ((self.calibration[4] * dT) >> 7)
            SENS = (self.calibration[1] << 15) + ((self.calibration[3] * dT) >> 8)

            # Second order temperature compensation
            if TEMP < 2000:
                T2 = (dT * dT) >> 31
                OFF2 = (5 * (TEMP - 2000) * (TEMP - 2000)) >> 1
                SENS2 = (5 * (TEMP - 2000) * (TEMP - 2000)) >> 2
                
                if TEMP < -1500:
                    OFF2 = OFF2 + 7 * (TEMP + 1500) * (TEMP + 1500)
                    SENS2 = SENS2 + ((11 * (TEMP + 1500) * (TEMP + 1500)) >> 1)
                
                TEMP = TEMP - T2
                OFF = OFF - OFF2
                SENS = SENS - SENS2

            # Calculate pressure
            P = ((((D1 * SENS) >> 21) - OFF) >> 15)

            # Convert to proper units
            temperature = TEMP / 100.0  # Convert to degrees Celsius
            pressure = P / 100.0    # Convert to hPa (mbar)

            return temperature, pressure
            
        except Exception as e:
            raise RuntimeError(f"Failed to get MS5611 temperature and pressure: {e}")
