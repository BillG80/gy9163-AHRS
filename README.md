# GY9163 Sensor System

A high-performance Raspberry Pi C-based sensor and display system for elevator motion tracking, featuring real-time sensor fusion, advanced calibration, and dynamic OLED visualization. Supports GY912 (ICM20948 + BMP3) & GY63 (MS5611) modules and SSD1306 OLED display.

---

## Overview
This project provides a robust platform for real-time elevator motion analysis and visualization. It integrates:
- Multi-sensor fusion (9-axis IMU + barometer)
- Advanced calibration routines
- Dynamic SSD1306 OLED display with custom font rendering
- Modular architecture for easy extension and testing

Typical use cases include elevator ride analysis, motion visualization, and sensor evaluation.

---

## Architecture
- **Drivers:** Low-level C drivers for ICM20948 (IMU), MS5611 (barometer), and SSD1306 (OLED)
- **Sensor Fusion:** Real-time AHRS algorithms (EKF, Madgwick, Mahony)
- **Calibration:** Standalone module for bias, scale, and alignment (see [README_sensor_fusion_calibration.md](README_sensor_fusion_calibration.md))
- **Display:** Animated arrow icons, real-time data, and unit display
- **Stubs/Simulation:** Simulated data for development without hardware
- **Logging & Navigation:** Elevator state tracking, event logging

---

## Features
- Real-time temperature, pressure, altitude, velocity, and acceleration display
- 9-axis motion sensing (ICM20948: accel, gyro, mag)
- Robust SPI/I2C bus handling
- 100-200Hz update rate for smooth animation
- Automatic startup via systemd service
- Error handling, logging, and LED status indication
- Capacitive touch button for user input
- Advanced calibration routines (gyro, accel, mag, cross-alignment)
- Simulation mode for development/testing

---

## Hardware Requirements
- **Raspberry Pi Zero 2W** (or compatible)
- **GY912 module** (SPI0)
- **GY63 module** (I2C-1)
- **SSD1306 OLED display** (I2C-0)
- **Capacitive touch button** (GPIO5)
- **Blue LED** (GPIO12)
- **Red LED** (GPIO16)

---

## Installation
1. **Clone the repository:**
   ```bash
   git clone https://github.com/your-username/gy9163-AHRS.git
   cd gy9163-AHRS
   ```
2. **Install dependencies:**
   - System libraries:
     ```bash
     sudo apt-get update
     sudo apt-get install build-essential python3-pip python3-gpiozero python3-smbus python3-scipy i2c-tools
     ```
   - Python dependencies:
     ```bash
     pip3 install -r requirements.txt
     ```
   - bcm2835 library (for SPI/I2C):
     ```bash
     # Download and install bcm2835 v1.75 from official site
     ```
   - (Optional) Adafruit BMP library for reference:
     ```bash
     git clone https://github.com/adafruit/Adafruit_Python_BMP.git
     cd Adafruit_Python_BMP
     sudo python setup.py install
     ```
3. **Compile the program:**
   ```bash
   make
   ```
4. **Install and start service:**
   ```bash
   sudo cp elevator_app.service /etc/systemd/system/
   sudo systemctl daemon-reload
   sudo systemctl enable elevator_app
   sudo systemctl start elevator_app
   ```
5. **Run manually (for testing):**
   ```bash
   sudo ./elevator_app
   ```

---

## Usage
- **Default:** System starts automatically and displays live sensor data with animated icons.
- **Calibration:**
  - Run calibration routines as described in [README_sensor_fusion_calibration.md](README_sensor_fusion_calibration.md)
  - Calibration results are loaded at startup for accurate sensor fusion
- **Testing/Simulation:**
  - Use provided stubs to simulate sensor data and test display/logic without hardware
- **Output:**
  - OLED shows pressure (hPa), temperature (°C), altitude (m), speed (m/s), distance (m), and animated elevator motion

---

## Example Output
```
+-----------------------------+
| P: 1013.25 hPa  T: 24.5°C   |
| [↑] Alt: 12.3 m             |
|     Speed: 0.45 m/s         |
|     Dist: 7.9 m             |
+-----------------------------+
```
(Arrow and numbers update in real time)

---

## Configuration
- **Calibration file:** `/calibration.json` (see calibration README)
- **Service config:** `elevator_app.service`
- **Font:** Uses SansCondensed for optimized OLED rendering

---

## Troubleshooting
- **No sensor data:** Check wiring, I2C/SPI bus permissions, and sensor power
- **Display issues:** Confirm OLED address and connections; see font troubleshooting in docs
- **Build errors:** Ensure all dependencies and correct bcm2835 version are installed
- **Service fails:** Check `journalctl -u elevator_app` for logs

---

## Development & Contribution
- Modular codebase: add new drivers, fusion algorithms, or display features easily
- See [README_sensor_fusion_calibration.md](README_sensor_fusion_calibration.md) for calibration/dev details
- Contributions, bug reports, and feature requests are welcome!
- See code comments for documentation, or contact maintainers (see AUTHORS)

---

## References
- [README_sensor_fusion_calibration.md](README_sensor_fusion_calibration.md)
- `src/core/fusion/calibration.c`, `include/core/fusion/calibration.h`
- `src/core/fusion/sensor_fusion.c`
- `src/drivers/`, `src/display/`, `src/core/navigation/`
- [bcm2835 library](http://www.airspayce.com/mikem/bcm2835/)

---
