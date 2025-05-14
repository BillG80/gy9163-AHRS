#!/usr/bin/env python3
import time
import smbus

BUS  = 0      # your SSD1306 is on /dev/i2c-0
ADDR = 0x3C   # or 0x3D for some modules

bus = smbus.SMBus(BUS)

def ssd1306_init():
    # Standard Adafruit init sequence
    cmds = [
        0xAE,             # DISPLAYOFF
        0x20, 0x00,       # MEMORYMODE → horizontal
        0xB0,             # PAGESTART 0
        0xC8,             # COMSCANDEC
        0x00,             # SEGMENTADDRLOW
        0x10,             # SEGMENTADDRHIGH
        0x40,             # STARTLINE 0
        0x81, 0xFF,       # CONTRAST
        0xA1,             # SEGREMAP
        0xA6,             # NORMALDISPLAY
        0xA8, 0x3F,       # MUXRATIO 64
        0xA4,             # DISPLAYALLON_RESUME
        0xD3, 0x00,       # DISPLAYOFFSET
        0xD5, 0xF0,       # DISPLAYCLOCKDIV
        0xD9, 0x22,       # PRECHARGE
        0xDA, 0x12,       # COMPINHW
        0xDB, 0x20,       # VCOMDETECT
        0x8D, 0x14,       # CHARGEPUMP
        0xAF              # DISPLAYON
    ]
    for cmd in cmds:
        bus.write_byte_data(ADDR, 0x00, cmd)

def fill_display(color=0xFF):
    for page in range(8):
        bus.write_byte_data(ADDR, 0x00, 0xB0 | page)  # set page
        bus.write_byte_data(ADDR, 0x00, 0x00)          # col low
        bus.write_byte_data(ADDR, 0x00, 0x10)          # col high
        # send 128 bytes in blocks of 32 to avoid SMBus limit
        for offset in range(0, 128, 32):
            bus.write_i2c_block_data(ADDR, 0x40, [color]*32)

def main():
    print("TEST_SSD1306: init…")
    ssd1306_init()
    time.sleep(0.1)

    print("TEST_SSD1306: full-white flash")
    fill_display(0xFF)
    time.sleep(1)

    print("TEST_SSD1306: clear")
    fill_display(0x00)
    time.sleep(0.1)

if __name__ == "__main__":
    main()