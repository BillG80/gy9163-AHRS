# GY91 & GY63 Sensor System

A Python-based sensor system for reading data from GY91 (MPU9250 + BMP280) and GY63 (MS5611) sensors on Raspberry Pi Zero W.

## Features
- Temperature and pressure readings from both BMP280 and MS5611
- 6-axis motion sensing (accelerometer and gyroscope) from MPU9250
- 10Hz update rate
- Automatic startup service
- Error handling and logging

## Hardware Requirements
- Raspberry Pi Zero W
- GY91 module (connected to I2C-1)
- GY63 module (connected to I2C-0)

## Installation
1. Clone the repository:
