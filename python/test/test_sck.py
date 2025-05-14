import spidev
import time

spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 50000
spi.mode = 0
while True:
    spi.xfer2([0xAA])
    time.sleep(0.5)