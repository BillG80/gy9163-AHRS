import spidev
import RPi.GPIO as GPIO
import time

# Pin assignments
SPI_BUS = 0
MPU9250_CS = 0  # CE0
MS5611_CS = 1   # CE1
MS5611_PS = 25  # GPIO25

# Register addresses
MPU9250_WHO_AM_I = 0x75
MPU9250_WHO_AM_I_EXPECTED = 0x71  # Sometimes 0x68 for MPU6500/9250

MS5611_PROM_READ = 0xA0  # Base address for PROM read (0xA0, 0xA2, ..., 0xAE)

def read_mpu9250_whoami(spi):
    # Read WHO_AM_I register (set MSB for read)
    resp = spi.xfer2([MPU9250_WHO_AM_I | 0x80, 0x00])
    return resp[1]

def read_ms5611_prom(spi, prom_addr):
    # PROM read command (no MSB set for SPI)
    resp = spi.xfer2([prom_addr, 0x00, 0x00])
    return (resp[1] << 8) | resp[2]

def main():
    # Set GPIO25 HIGH for MS5611 SPI mode
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(MS5611_PS, GPIO.OUT)
    GPIO.output(MS5611_PS, GPIO.HIGH)
    time.sleep(0.01)

    # Test MPU9250 (CE0)
    mpu_spi = spidev.SpiDev()
    mpu_spi.open(SPI_BUS, MPU9250_CS)
    mpu_spi.max_speed_hz = 1000000
    mpu_spi.mode = 0b11  # Most MPU9250 modules use SPI mode 3

    whoami = read_mpu9250_whoami(mpu_spi)
    print(f"MPU9250 WHO_AM_I: 0x{whoami:02X} (expected 0x{MPU9250_WHO_AM_I_EXPECTED:02X})")
    mpu_spi.close()

    # Test MS5611 (CE1)
    ms_spi = spidev.SpiDev()
    ms_spi.open(SPI_BUS, MS5611_CS)
    ms_spi.max_speed_hz = 1000000
    ms_spi.mode = 0b00  # MS5611 uses SPI mode 0

    print("MS5611 PROM values:")
    for i in range(0, 8):
        prom_addr = MS5611_PROM_READ + (i * 2)
        value = read_ms5611_prom(ms_spi, prom_addr)
        print(f"  PROM[{i}] (0x{prom_addr:02X}): 0x{value:04X}")
    ms_spi.close()

    GPIO.cleanup()

if __name__ == "__main__":
    print("Starting WCMCU-117 SPI test...")
    main()
    print("Test complete.")