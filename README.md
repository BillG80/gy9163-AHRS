# ICM20948 & MS5611 Sensor System

A C/C++-based sensor system for reading data from ICM20948 and MS5611 sensors on Raspberry Pi Zero 2W.

## Features
- Temperature and pressure readings from MS5611
- 9-axis motion sensing (accelerometer, gyroscope and magnetmeter) from ICM20948
- SPI interface from 7Mhz to 20Mhz(too high, 1M should be enough for pressure and temp)
- Automatic startup service
- Error handling and logging
- Real-time vertical acceleration tracking
- Heading calculation from AHRS
- Trip distance accumulation
- Zero velocity updates for drift correction
- Low-pass filtering for noise reduction

## Hardware Requirements
- Raspberry Pi Zero 2W
- GY912 module (connected to spi0)
- GY63 module (connected to spi1)
- SSD1306 128x64 (connected to iic1)

## Installation
1. Clone the repository:
