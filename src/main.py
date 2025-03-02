import time
from sensors.bmp280 import BMP280
from sensors.mpu9250 import MPU9250
from processing.orientation import Orientation
from displays.ssd1306 import SSD1306

def initialize_sensors():
    """Initialize all sensors and return their instances."""
    bmp280 = BMP280(bus=1)
    mpu9250 = MPU9250(bus=1)
    orientation = Orientation()
    display = SSD1306(bus=0)  # Using bus 0 for display
    print("All sensors initialized")
    return bmp280, mpu9250, orientation, display

def main():
    # Initialize sensors, orientation, and display
    bmp280, mpu9250, orientation, display = initialize_sensors()
    
    print("Waiting for sensors to stabilize...")
    time.sleep(1)
    
    # Clear the display at start
    display.clear_display()
    
    try:
        while True:
            start_time = time.time()
            
            # Read sensor data
            accel, gyro = mpu9250.read_sensor_data()
            temperature, pressure = bmp280.get_temperature_and_pressure()
            altitude = 44330 * (1.0 - pow(pressure / 1013.25, 0.1903))  # Calculate altitude using barometric formula
            
            # Update orientation
            current_time = time.time()
            orientation.update_orientation(gyro, current_time)
            
            # Calculate acceleration magnitude using dictionary keys
            accel_magnitude = (accel['x']**2 + accel['y']**2 + accel['z']**2)**0.5
            
            # Update display with measurements
            display.display_measurements(
                pressure=pressure,
                altitude=altitude,
                temperature=temperature,
                acceleration=accel_magnitude
            )
            
            # Print orientation and magnitude
            orientation.print_orientation_and_magnitude(accel)
            
            # Calculate elapsed time and sleep to maintain 60 Hz
            elapsed_time = time.time() - start_time
            sleep_time = max(0, (1/60) - elapsed_time)
            time.sleep(sleep_time)
    except KeyboardInterrupt:
        print("\nProgram terminated by user")
    finally:
        # Clear display before exiting
        display.clear_display()

if __name__ == "__main__":
    main()
