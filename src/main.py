import time
from sensors.bmp280 import BMP280
from sensors.mpu9250 import MPU9250
from displays.ssd1306 import SSD1306
from processing.orientation import Orientation
import signal
import sys

# Global flag for program control
running = True

def signal_handler(signum, frame):
    """Handle Ctrl+C and other signals"""
    global running
    running = False

def initialize_sensors():
    """Initialize all sensors and return their instances."""
    bmp280 = BMP280(1)
    mpu9250 = MPU9250(1)
    display = SSD1306(0)
    print("All sensors initialized")
    return bmp280, mpu9250, display

def calibrate_gyro(mpu9250, display, samples=500):
    """Calibrate gyroscope by calculating average bias"""
    print("Calibrating gyroscope...")
    display.clear()
    display.display_text("Calibrating...", 2, 8)
    
    gyro_data = []
    for i in range(samples):
        _, gyro = mpu9250.read_sensor_data()
        gyro_data.append([gyro['x'], gyro['y'], gyro['z']])
        time.sleep(0.002)  # 500Hz sampling
        
        if i % 50 == 0:  # Update progress every 50 samples
            progress = int((i / samples) * 100)
            display.clear()
            display.display_text(f"Calibrating {progress}%", 2, 8)
    
    # Calculate average bias
    gyro_bias = np.mean(gyro_data, axis=0)
    print(f"Gyro bias: {gyro_bias}")
    return gyro_bias

def main():
    # Set up signal handler
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    display = None
    try:
        # Initialize sensors and display
        display = SSD1306(0)
        bmp280 = BMP280(1)
        mpu9250 = MPU9250(1)
        orientation = Orientation()
        
        # Update intervals
        pressure_interval = 1/20.0  # 20 Hz
        accel_interval = 1/60.0     # 60 Hz
        
        # Timing variables
        pressure_last_update = time.time()
        accel_last_update = time.time()
        
        # Measurement variables
        pressure = 0
        temperature = 0
        altitude = 0
        acceleration = 0
        
        print("Sensors initialized. Press Ctrl+C to exit")
        
        # Main loop
        while running:
            try:
                current_time = time.time()
                
                # Update pressure and temperature at 20 Hz
                if current_time - pressure_last_update >= pressure_interval:
                    temperature, pressure = bmp280.get_temperature_and_pressure()
                    altitude = bmp280.get_altitude(pressure)
                    pressure_last_update = current_time
                
                # Update acceleration at 60 Hz
                if current_time - accel_last_update >= accel_interval:
                    accel, gyro = mpu9250.read_sensor_data()
                    orientation.update_orientation(gyro, current_time)
                    linear_accel = orientation.get_linear_acceleration(accel)
                    acceleration = linear_accel['magnitude']
                    accel_last_update = current_time
                
                # Update display
                display.display_measurements(pressure, acceleration, temperature, altitude)
                
                # Small sleep to prevent CPU overload
                time.sleep(0.001)
                
            except Exception as e:
                print(f"Error in main loop: {e}")
                time.sleep(0.1)
                
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if display:
            display.shutdown()
        print("\nProgram terminated")

if __name__ == "__main__":
    main()
