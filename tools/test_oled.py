#!/usr/bin/env python3
import time
import smbus2
print("TEST_OLED: starting")  
BUS  = 0        # /dev/i2c-0
ADDR = 0x3C

bus = smbus2.SMBus(BUS)

# Toggle display OFF/ON
bus.write_byte_data(ADDR, 0x00, 0xAE)
time.sleep(0.1)
bus.write_byte_data(ADDR, 0x00, 0xAF)
time.sleep(0.1)

# Quick white‐flash test (one page at a time)
for page in range(8):
    # set page address
    bus.write_byte_data(ADDR, 0x00, 0xB0 | page)   # SSD1306_SETPAGESTARTADDR|page
    bus.write_byte_data(ADDR, 0x00, 0x00)            # col start low
    bus.write_byte_data(ADDR, 0x00, 0x7F)            # col end high (127)

    # send 128 bytes of 0xFF
    for _ in range(128):
        bus.write_byte_data(ADDR, 0x40, 0xFF)

time.sleep(1)

# then clear
for page in range(8):
    bus.write_byte_data(ADDR, 0x00, 0xB0 | page)
    bus.write_byte_data(ADDR, 0x00, 0x00)
    bus.write_byte_data(ADDR, 0x00, 0x7F)
    for _ in range(128):
        bus.write_byte_data(ADDR, 0x40, 0x00)