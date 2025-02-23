import time
from sensors.ms5611 import MS5611
from sensors.bmp280 import BMP280
from sensors.mpu9250 import MPU9250

class SensorSystem:
    """Main sensor system class."""
    
    def __init__(self):
        """Initialize all sensors."""
        print("Initializing sensor system...")
        self.ms5611 = MS5611()
        self.mpu9250 = MPU9250()
        self.bmp280 = BMP280()
        print("All sensors initialized")

    def read_all_sensors(self):
        """Read data from all sensors."""
        try:
            # Read MS5611
            ms5611_temp, ms5611_press = self.ms5611.get_temperature_and_pressure()
            
            # Read BMP280
            bmp280_temp, bmp280_press = self.bmp280.get_temperature_and_pressure()
            
            # Read MPU9250
            accel, gyro = self.mpu9250.read_sensor_data()
            
            return {
                'ms5611': {'temp': ms5611_temp, 'press': ms5611_press},
                'bmp280': {'temp': bmp280_temp, 'press': bmp280_press},
                'mpu9250': {'accel': accel, 'gyro': gyro}
            }
        except Exception as e:
            raise RuntimeError(f"Error reading sensors: {e}")

def main():
    """Main program loop."""
    try:
        sensors = SensorSystem()
        print("Waiting for sensors to stabilize...")
        time.sleep(1)

        while True:
            data = sensors.read_all_sensors()
            
            # Clear screen
            print("\033[2J\033[H")
            
            # Print MS5611 data
            print("MS5611 (GY63):")
            print(f"Temperature: {data['ms5611']['temp']:.2f}°C")
            print(f"Pressure: {data['ms5611']['press']:.2f} hPa")
            
            # Print BMP280 data
            print("\nBMP280 (GY91):")
            print(f"Temperature: {data['bmp280']['temp']:.2f}°C")
            print(f"Pressure: {data['bmp280']['press']:.2f} hPa")
            
            # Print MPU9250 data
            print("\nMPU9250 (GY91):")
            print("Accelerometer (g):")
            print(f"  X: {data['mpu9250']['accel']['x']:+7.3f}")
            print(f"  Y: {data['mpu9250']['accel']['y']:+7.3f}")
            print(f"  Z: {data['mpu9250']['accel']['z']:+7.3f}")
            
            print("\nGyroscope (degrees/s):")
            print(f"  X: {data['mpu9250']['gyro']['x']:+8.2f}")
            print(f"  Y: {data['mpu9250']['gyro']['y']:+8.2f}")
            print(f"  Z: {data['mpu9250']['gyro']['z']:+8.2f}")
            
            print("\nPress Ctrl+C to exit")
            print("-" * 40)
            
            time.sleep(0.1)  # 10Hz update rate
            
    except KeyboardInterrupt:
        print("\nProgram terminated by user")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()
