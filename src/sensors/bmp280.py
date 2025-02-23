from smbus2 import SMBus
import time
from .constants import *

class BMP280:
    """BMP280 temperature and pressure sensor."""
    
    def __init__(self):
        """Initialize the BMP280 sensor."""
        self.bus = SMBus(I2C_BUS_1)
        self.cal = {}
        self.t_fine = 0
        self.init_bmp280()
        self.get_calibration_data()
        # Print calibration data for verification
        print("\nBMP280 calibration data:")
        for key, value in self.cal.items():
            print(f"{key}: {value}")

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
            
            # Configure filter and timing
            # t_sb[7:5]   = 110 (100ms standby time)
            # filter[4:2] = 010 (Filter coefficient = 4)
            # none[1:0]   = 00  (Reserved bits)
            self.bus.write_byte_data(BMP280_ADDR, BMP280_CONFIG, 0xC8)
            
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
            # Read temperature (msb, lsb, xlsb)
            data = self.bus.read_i2c_block_data(BMP280_ADDR, BMP280_TEMP_MSB, 3)
            raw_temp = ((data[0] << 16) | (data[1] << 8) | data[2]) >> 4

            # Read pressure (msb, lsb, xlsb)
            data = self.bus.read_i2c_block_data(BMP280_ADDR, BMP280_PRESS_MSB, 3)
            raw_press = ((data[0] << 16) | (data[1] << 8) | data[2]) >> 4

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

    def get_temperature_and_pressure(self):
        """
        Read and calculate temperature and pressure.
        Returns: (temperature in °C, pressure in hPa)
        """
        try:
            raw_temp, raw_press = self.read_raw_data()
            temperature = self.compensate_temperature(raw_temp)
            pressure = self.compensate_pressure(raw_press)
            return temperature, pressure
        except Exception as e:
            raise RuntimeError(f"Failed to get BMP280 temperature and pressure: {e}")
