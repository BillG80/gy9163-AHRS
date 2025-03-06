from smbus2 import SMBus
import time
from .constants import *
from typing import Tuple, Dict
import statistics

class BMP280:
    """BMP280 temperature and pressure sensor."""
    
    def __init__(self, bus: int, address: int = BMP280_ADDR) -> None:
        """Initialize BMP280 sensor."""
        try:
            self.bus = SMBus(bus)
            self.address = address
            self.t_fine = 0
            
            # Initialize sensor
            self.reset()
            time.sleep(0.1)  # Wait after reset
            
            # Read factory calibration data
            self.read_calibration()
            
            # Configure sensor
            self.configure()
            
            print(f"BMP280 initialized on bus {bus}, address 0x{address:02X}")
            
        except Exception as e:
            print(f"Failed to initialize BMP280: {e}")
            raise

    def reset(self) -> None:
        """Reset the BMP280 sensor."""
        self.bus.write_byte_data(self.address, BMP280_RESET_REG, BMP280_RESET_VALUE)
        time.sleep(0.2)  # Wait for reset to complete

    def configure(self) -> None:
        """Configure BMP280 settings."""
        try:
            # Configure the sensor
            # osrs_t = 010 (x2 oversampling for temperature)
            # osrs_p = 101 (x8 oversampling for pressure)
            # mode = 11 (normal mode)
            ctrl_meas = (2 << 5) | (5 << 2) | 3
            self.bus.write_byte_data(self.address, BMP280_CTRL_MEAS_REG, ctrl_meas)
            
            # t_sb = 000 (0.5ms standby)
            # filter = 010 (filter coefficient 4)
            # spi3w_en = 0 (3-wire SPI disabled)
            config = (0 << 5) | (2 << 2) | 0
            self.bus.write_byte_data(self.address, BMP280_CONFIG_REG, config)
            
        except Exception as e:
            print(f"Error configuring BMP280: {e}")
            raise

    def read_raw_data(self) -> Tuple[int, int]:
        """Read raw temperature and pressure data."""
        try:
            # Read temperature data (24 bits)
            data = self.bus.read_i2c_block_data(self.address, BMP280_TEMP_MSB, 3)
            raw_temp = (data[0] << 16) | (data[1] << 8) | data[2]
            raw_temp = raw_temp >> 4  # Right shift 4 bits
            
            # Read pressure data (24 bits)
            data = self.bus.read_i2c_block_data(self.address, BMP280_PRESS_MSB, 3)
            raw_press = (data[0] << 16) | (data[1] << 8) | data[2]
            raw_press = raw_press >> 4  # Right shift 4 bits
            
            return raw_temp, raw_press
            
        except Exception as e:
            print(f"Error reading raw data: {e}")
            return 0, 0

    def compensate_temperature(self, raw_temp: int) -> float:
        """Compensate raw temperature reading."""
        var1 = ((raw_temp / 16384.0) - (self.dig_T1 / 1024.0)) * self.dig_T2
        var2 = ((raw_temp / 131072.0) - (self.dig_T1 / 8192.0)) * self.dig_T3
        
        self.t_fine = int(var1 + var2)
        temperature = (var1 + var2) / 5120.0
        
        return temperature

    def compensate_pressure(self, raw_press: int) -> float:
        """Compensate raw pressure reading."""
        var1 = (self.t_fine / 2.0) - 64000.0
        var2 = var1 * var1 * self.dig_P6 / 32768.0
        var2 = var2 + var1 * self.dig_P5 * 2.0
        var2 = (var2 / 4.0) + (self.dig_P4 * 65536.0)
        var1 = (self.dig_P3 * var1 * var1 / 524288.0 + self.dig_P2 * var1) / 524288.0
        var1 = (1.0 + var1 / 32768.0) * self.dig_P1
        
        if var1 == 0:
            return 0
            
        pressure = 1048576.0 - raw_press
        pressure = ((pressure - var2 / 4096.0) * 6250.0) / var1
        var1 = self.dig_P9 * pressure * pressure / 2147483648.0
        var2 = pressure * self.dig_P8 / 32768.0
        pressure = pressure + (var1 + var2 + self.dig_P7) / 16.0
        
        # Convert pressure to hPa
        pressure = pressure / 100.0
        
        return pressure

    def get_temperature_and_pressure(self) -> Tuple[float, float]:
        """Read and calculate temperature and pressure."""
        try:
            raw_temp, raw_press = self.read_raw_data()
            temperature = self.compensate_temperature(raw_temp)
            pressure = self.compensate_pressure(raw_press)
            return temperature, pressure
        except Exception as e:
            print(f"Error reading BMP280: {e}")
            return 0.0, 0.0

    def get_altitude(self, pressure: float, sea_level_pressure: float = 1013.25) -> float:
        """Calculate altitude from pressure using barometric formula.
        
        Args:
            pressure: Current pressure in hPa
            sea_level_pressure: Reference sea level pressure in hPa
            
        Returns:
            Altitude in meters
        """
        try:
            if pressure <= 0:
                return 0.0
            
            # Standard barometric formula:
            # h = 44330 * (1 - (P/P0)^(1/5.255))
            altitude = 44330.0 * (1.0 - pow(pressure / sea_level_pressure, 1/5.255))
            return round(altitude, 1)
            
        except Exception as e:
            print(f"Error calculating altitude: {e}")
            return 0.0

    def read_calibration(self) -> None:
        """Read factory calibration data."""
        try:
            # Read temperature calibration data
            self.dig_T1 = self.read_word(BMP280_DIG_T1_REG)  # Unsigned
            self.dig_T2 = self.read_signed_word(BMP280_DIG_T2_REG)
            self.dig_T3 = self.read_signed_word(BMP280_DIG_T3_REG)
            
            # Read pressure calibration data
            self.dig_P1 = self.read_word(BMP280_DIG_P1_REG)  # Unsigned
            self.dig_P2 = self.read_signed_word(BMP280_DIG_P2_REG)
            self.dig_P3 = self.read_signed_word(BMP280_DIG_P3_REG)
            self.dig_P4 = self.read_signed_word(BMP280_DIG_P4_REG)
            self.dig_P5 = self.read_signed_word(BMP280_DIG_P5_REG)
            self.dig_P6 = self.read_signed_word(BMP280_DIG_P6_REG)
            self.dig_P7 = self.read_signed_word(BMP280_DIG_P7_REG)
            self.dig_P8 = self.read_signed_word(BMP280_DIG_P8_REG)
            self.dig_P9 = self.read_signed_word(BMP280_DIG_P9_REG)
            
        except Exception as e:
            print(f"Error reading calibration data: {e}")
            raise

    def read_word(self, reg: int) -> int:
        """Read unsigned 16-bit word from register."""
        data = self.bus.read_i2c_block_data(self.address, reg, 2)
        return data[1] << 8 | data[0]

    def read_signed_word(self, reg: int) -> int:
        """Read signed 16-bit word from register."""
        val = self.read_word(reg)
        if val >= 32768:
            val -= 65536
        return val

    def get_statistics(self) -> Dict:
        """Get current statistics for temperature and pressure readings.
        
        Returns:
            Dict containing mean, min, max, and standard deviation for both measurements
        """
        if len(self.temp_buffer) < 2:  # Need at least 2 samples for statistics
            return None
            
        return {
            'temperature': {
                'mean': statistics.mean(self.temp_buffer),
                'min': min(self.temp_buffer),
                'max': max(self.temp_buffer),
                'std_dev': statistics.stdev(self.temp_buffer)
            },
            'pressure': {
                'mean': statistics.mean(self.pressure_buffer),
                'min': min(self.pressure_buffer),
                'max': max(self.pressure_buffer),
                'std_dev': statistics.stdev(self.pressure_buffer)
            }
        }
