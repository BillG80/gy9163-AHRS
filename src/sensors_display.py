import time
from sensors.ms5611 import MS5611
from sensors.bmp280 import BMP280
from sensors.mpu9250 import MPU9250

def initialize_sensors():
    """Initialize all sensors and return their instances."""
    ms5611 = MS5611(bus=0)
    bmp280 = BMP280(bus=1)
    mpu9250 = MPU9250(bus=1)
    print("All sensors initialized")
    return ms5611, bmp280, mpu9250

def read_sensors(ms5611, bmp280, mpu9250):
    """Read data from all sensors and return it."""
    ms5611_temp, ms5611_press = ms5611.get_temperature_and_pressure()
    bmp280_temp, bmp280_press = bmp280.get_temperature_and_pressure()
    accel, gyro = mpu9250.read_sensor_data()
    
    return ms5611_temp, ms5611_press, bmp280_temp, bmp280_press, accel, gyro

def display_data(ms5611_temp, ms5611_press, bmp280_temp, bmp280_press, accel, gyro):
    """Display sensor data."""
    print("\nSensor Readings:")
    print(f"MS5611  - Temperature: {ms5611_temp:.1f}°C, Pressure: {ms5611_press:.1f}hPa")
    print(f"BMP280  - Temperature: {bmp280_temp:.1f}°C, Pressure: {bmp280_press:.1f}hPa")
    print(f"MPU9250 - Accel: X:{accel['x']:.2f} Y:{accel['y']:.2f} Z:{accel['z']:.2f}")
    print(f"         Gyro:  X:{gyro['x']:.2f} Y:{gyro['y']:.2f} Z:{gyro['z']:.2f}")

def run_sensor_display():
    """Main function to run the sensor display."""
    print("Initializing sensor system...")
    ms5611, bmp280, mpu9250 = initialize_sensors()
    
    print("Waiting for sensors to stabilize...")
    time.sleep(1)
    
    try:
        while True:
            try:
                # Read and display sensor data
                ms5611_temp, ms5611_press, bmp280_temp, bmp280_press, accel, gyro = read_sensors(ms5611, bmp280, mpu9250)
                display_data(ms5611_temp, ms5611_press, bmp280_temp, bmp280_press, accel, gyro)
                time.sleep(1)
            except Exception as e:
                print(f"Error reading sensors: {e}")
                time.sleep(1)
    except KeyboardInterrupt:
        print("\nProgram terminated by user")

if __name__ == "__main__":
    run_sensor_display() 