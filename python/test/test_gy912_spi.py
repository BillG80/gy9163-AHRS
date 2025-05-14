import spidev
import time

# SPI device settings
SPI_BUS = 0
ICM20948_CS = 0  # CE0
BMP388_CS = 1    # CE1

# ICM20948 registers
ICM20948_WHO_AM_I = 0x00
ICM20948_WHO_AM_I_EXPECTED = 0xEA

# BMP388 registers
BMP388_CHIP_ID = 0x00
BMP388_CHIP_ID_EXPECTED = 0x50

def spi_read(spi, reg_addr):
    """Read a register (MSB=1) and return the data byte."""
    read_addr = reg_addr | 0x80
    resp = spi.xfer2([read_addr, 0x00])
    return resp[1]

def test_icm20948(spi):
    whoami = spi_read(spi, ICM20948_WHO_AM_I)
    print(f"ICM20948 WHO_AM_I: 0x{whoami:02X} (expected 0x{ICM20948_WHO_AM_I_EXPECTED:02X})")

def test_bmp388(spi):
    chip_id = spi_read(spi, BMP388_CHIP_ID)
    print(f"BMP388 CHIP_ID: 0x{chip_id:02X} (expected 0x{BMP388_CHIP_ID_EXPECTED:02X})")

def main():
    print("Starting SPI sensor test...")

    # Test ICM20948
    icm_spi = spidev.SpiDev()
    icm_spi.open(SPI_BUS, ICM20948_CS)
    icm_spi.max_speed_hz = 1000000
    icm_spi.mode = 0b00
    print("Testing ICM20948:")
    test_icm20948(icm_spi)
    icm_spi.close()

    # Delay between chip selects
    time.sleep(0.001)

    # Test BMP388
    bmp_spi = spidev.SpiDev()
    bmp_spi.open(SPI_BUS, BMP388_CS)
    bmp_spi.max_speed_hz = 1000000
    bmp_spi.mode = 0b00
    print("Testing BMP388:")
    test_bmp388(bmp_spi)
    bmp_spi.close()

    print("SPI sensor test complete.")

if __name__ == "__main__":
    main()