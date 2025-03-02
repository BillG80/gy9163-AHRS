from smbus2 import SMBus
import time
from .constants import *
from typing import Tuple

class BMP280:
    """BMP280 temperature and pressure sensor."""
    
    def __init__(self, bus: int, address: int = BMP280_ADDR) -> None:
        """Initialize BMP280 sensor.
        
        Args:
            bus: I2C bus number
            address: I2C device address (default: 0x76)
        """
        try:
            self.bus = SMBus(bus)
            self.address = address
            
            # Initialize sensor
            self.reset()
            time.sleep(0.1)  # Wait after reset
            self.read_calibration()
            self.configure()
            print("BMP280 initialized")
            
        except Exception as e:
            print(f"Failed to initialize BMP280: {e}")
            raise

    def reset(self) -> None:
        """Reset the BMP280 sensor."""
        try:
            self.bus.write_byte_data(self.address, BMP280_RESET_REG, BMP280_RESET_VALUE)
            time.sleep(0.2)  # Wait for reset to complete
        except Exception as e:
            print(f"Error resetting BMP280: {e}")
            raise

    def init_bmp280(self):
        """Initialize the sensor with specific settings."""
        try:
            # Check BMP280 ID
            chip_id = self.bus.read_byte_data(BMP280_ADDR, BMP280_CHIP_ID)
            if chip_id != 0x58:
                raise RuntimeError(f"BMP280 CHIP_ID returned 0x{chip_id:02X} instead of 0x58")
            
            # Reset BMP280
            self.bus.write_byte_data(BMP280_ADDR, BMP280_RESET, 0xB6)
            time.sleep(0.2)
            
            # Configure BMP280
            # osrs_t[7:5] = 001 (x1 Temperature oversampling)
            # osrs_p[4:2] = 011 (x4 Pressure oversampling)
            # mode[1:0]   = 11  (Normal mode)
            self.bus.write_byte_data(BMP280_ADDR, BMP280_CTRL_MEAS, 0x37)
            time.sleep(0.2)
            
            # Configure filter and timing
            # t_sb[7:5]   = 110 (100ms standby time)
            # filter[4:2] = 010 (Filter coefficient = 4)
            # none[1:0]   = 00  (Reserved bits)
            self.bus.write_byte_data(BMP280_ADDR, BMP280_CONFIG, 0xC8)
            time.sleep(0.2)
            
            print("BMP280 initialized")
            
        except Exception as e:
            raise RuntimeError(f"Failed to initialize BMP280: {e}")

    def get_calibration_data(self):
        """Read factory calibration data."""
        try:
            # Read temperature calibration data
            self.cal['dig_T1'] = self.read_word(BMP280_DIG_T1)  # unsigned short
            self.cal['dig_T2'] = self.read_signed_word(BMP280_DIG_T1 + 2)  # signed short
            self.cal['dig_T3'] = self.read_signed_word(BMP280_DIG_T1 + 4)  # signed short

            # Read pressure calibration data
            self.cal['dig_P1'] = self.read_word(BMP280_DIG_P1)  # unsigned short
            self.cal['dig_P2'] = self.read_signed_word(BMP280_DIG_P1 + 2)
            self.cal['dig_P3'] = self.read_signed_word(BMP280_DIG_P1 + 4)
            self.cal['dig_P4'] = self.read_signed_word(BMP280_DIG_P1 + 6)
            self.cal['dig_P5'] = self.read_signed_word(BMP280_DIG_P1 + 8)
            self.cal['dig_P6'] = self.read_signed_word(BMP280_DIG_P1 + 10)
            self.cal['dig_P7'] = self.read_signed_word(BMP280_DIG_P1 + 12)
            self.cal['dig_P8'] = self.read_signed_word(BMP280_DIG_P1 + 14)
            self.cal['dig_P9'] = self.read_signed_word(BMP280_DIG_P1 + 16)
            
        except Exception as e:
            raise RuntimeError(f"Failed to read BMP280 calibration data: {e}")

    def read_word(self, reg):
        """Read unsigned 16-bit value."""
        high = self.bus.read_byte_data(BMP280_ADDR, reg)
        low = self.bus.read_byte_data(BMP280_ADDR, reg + 1)
        return (high << 8) + low

    def read_signed_word(self, reg):
        """Read signed 16-bit value."""
        val = self.read_word(reg)
        if val >= 32768:  # 2^15
            val -= 65536  # 2^16
        return val

    def read_raw_data(self):
        """Read raw temperature and pressure data."""
        try:
            # Read temperature and pressure data in one transaction
            data = self.bus.read_i2c_block_data(self.address, BMP280_PRESS_MSB, 6)
            
            # Combine pressure bytes (MSB, LSB, XLSB)
            raw_press = ((data[0] << 16) | (data[1] << 8) | data[2]) >> 4
            
            # Combine temperature bytes (MSB, LSB, XLSB)
            raw_temp = ((data[3] << 16) | (data[4] << 8) | data[5]) >> 4
            
            return raw_temp, raw_press
            
        except Exception as e:
            raise RuntimeError(f"Failed to read BMP280 raw data: {e}")

    def compensate_temperature(self, adc_T):
        """
        Calculate temperature from raw value.
        Returns temperature in DegC, resolution is 0.01 DegC
        """
        var1 = ((adc_T >> 3) - (self.cal['dig_T1'] << 1)) * self.cal['dig_T2'] >> 11
        var2 = (((((adc_T >> 4) - self.cal['dig_T1']) * 
                ((adc_T >> 4) - self.cal['dig_T1'])) >> 12) * 
                self.cal['dig_T3']) >> 14

        self.t_fine = var1 + var2
        temp = (self.t_fine * 5 + 128) >> 8
        return temp / 100.0

    def compensate_pressure(self, adc_P):
        """
        Calculate pressure from raw value.
        Returns pressure in Pa
        """
        var1 = self.t_fine - 128000
        var2 = var1 * var1 * self.cal['dig_P6']
        var2 = var2 + ((var1 * self.cal['dig_P5']) << 17)
        var2 = var2 + (self.cal['dig_P4'] << 35)
        var1 = ((var1 * var1 * self.cal['dig_P3']) >> 8) + ((var1 * self.cal['dig_P2']) << 12)
        var1 = (((1 << 47) + var1)) * self.cal['dig_P1'] >> 33

        if var1 == 0:
            return 0

        p = 1048576 - adc_P
        p = (((p << 31) - var2) * 3125) // var1
        var1 = (self.cal['dig_P9'] * (p >> 13) * (p >> 13)) >> 25
        var2 = (self.cal['dig_P8'] * p) >> 19

        p = ((p + var1 + var2) >> 8) + (self.cal['dig_P7'] << 4)
        return p / 256.0 / 100.0  # Convert to hPa

    def get_temperature_and_pressure(self) -> Tuple[float, float]:
        """Calculate compensated temperature and pressure."""
        # Read raw values
        raw_temp, raw_press = self.read_raw_data()
        
        # Temperature compensation
        var1 = ((raw_temp >> 3) - (self.calibration['dig_T1'] << 1)) * self.calibration['dig_T2'] >> 11
        var2 = (((((raw_temp >> 4) - self.calibration['dig_T1']) * 
                  ((raw_temp >> 4) - self.calibration['dig_T1'])) >> 12) * 
                self.calibration['dig_T3']) >> 14
                
        t_fine = var1 + var2
        temperature = (t_fine * 5 + 128) >> 8
        temperature = temperature / 100.0

        # Pressure compensation
        var1 = t_fine - 128000
        var2 = var1 * var1 * self.calibration['dig_P6']
        var2 = var2 + ((var1 * self.calibration['dig_P5']) << 17)
        var2 = var2 + (self.calibration['dig_P4'] << 35)
        var1 = ((var1 * var1 * self.calibration['dig_P3']) >> 8) + ((var1 * self.calibration['dig_P2']) << 12)
        var1 = ((1 << 47) + var1) * self.calibration['dig_P1'] >> 33

        if var1 == 0:
            return temperature, 0

        p = 1048576 - raw_press
        p = (((p << 31) - var2) * 3125) // var1
        var1 = (self.calibration['dig_P9'] * (p >> 13) * (p >> 13)) >> 25
        var2 = (self.calibration['dig_P8'] * p) >> 19

        p = ((p + var1 + var2) >> 8) + (self.calibration['dig_P7'] << 4)
        pressure = float(p) / 25600.0  # Changed from 256.0 to 25600.0 to get hPa
        
        return temperature, pressure

    def configure(self) -> None:
        """Configure BMP280 settings."""
        try:
            # Set configuration registers
            # osrs_t[7:5] = 001 (x1 Temperature oversampling)
            # osrs_p[4:2] = 011 (x4 Pressure oversampling)
            # mode[1:0]   = 11  (Normal mode)
            ctrl = 0x37  # 0b00110111
            
            # t_sb[7:5]   = 000 (0.5ms standby time)
            # filter[4:2] = 010 (Filter coefficient = 4)
            # none[1:0]   = 00  (Reserved bits)
            config = 0x08  # 0b00001000
            
            self.bus.write_byte_data(self.address, BMP280_CTRL_MEAS_REG, ctrl)
            self.bus.write_byte_data(self.address, BMP280_CONFIG_REG, config)
            time.sleep(0.1)
        except Exception as e:
            print(f"Error configuring BMP280: {e}")
            raise

    def read_calibration(self) -> None:
        """Read factory calibration data."""
        try:
            # Read calibration data
            self.calibration = {}
            
            # Temperature calibration
            self.calibration['dig_T1'] = self.read_word(BMP280_DIG_T1_REG)
            self.calibration['dig_T2'] = self.read_signed_word(BMP280_DIG_T2_REG)
            self.calibration['dig_T3'] = self.read_signed_word(BMP280_DIG_T3_REG)
            
            # Pressure calibration
            self.calibration['dig_P1'] = self.read_word(BMP280_DIG_P1_REG)
            self.calibration['dig_P2'] = self.read_signed_word(BMP280_DIG_P2_REG)
            self.calibration['dig_P3'] = self.read_signed_word(BMP280_DIG_P3_REG)
            self.calibration['dig_P4'] = self.read_signed_word(BMP280_DIG_P4_REG)
            self.calibration['dig_P5'] = self.read_signed_word(BMP280_DIG_P5_REG)
            self.calibration['dig_P6'] = self.read_signed_word(BMP280_DIG_P6_REG)
            self.calibration['dig_P7'] = self.read_signed_word(BMP280_DIG_P7_REG)
            self.calibration['dig_P8'] = self.read_signed_word(BMP280_DIG_P8_REG)
            self.calibration['dig_P9'] = self.read_signed_word(BMP280_DIG_P9_REG)
            
            print("BMP280 calibration data:")
            for key, value in self.calibration.items():
                print(f"{key}: {value}")
                
        except Exception as e:
            print(f"Error reading BMP280 calibration: {e}")
            raise

    def read_word(self, reg: int) -> int:
        """Read unsigned 16-bit word from register."""
        data = self.bus.read_i2c_block_data(self.address, reg, 2)
        return data[1] << 8 | data[0]

    def read_signed_word(self, reg: int) -> int:
        """Read signed 16-bit word from register."""
        val = self.read_word(reg)
        if val >= 32768:  # Convert to signed value
            val -= 65536
        return val
