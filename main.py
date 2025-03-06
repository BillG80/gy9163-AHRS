from sensors.bmp280 import BMP280
from sensors.orientation import Orientation
import time

def main():
    try:
        # Initialize sensors
        bmp280 = BMP280(bus=1)
        orientation = Orientation()
        
        while True:
            # Get temperature and pressure readings
            temperature, pressure = bmp280.get_temperature_and_pressure()
            
            # Calculate altitude (using default sea level pressure)
            altitude = bmp280.get_altitude(pressure)
            
            # Get acceleration magnitude
            accel = orientation.get_linear_acceleration({'x': 0, 'y': 0, 'z': 0})  # Replace with actual accelerometer data
            accel_magnitude = accel['magnitude']
            
            # Print all measurements
            print(f"Temperature: {temperature:.2f}°C")
            print(f"Pressure: {pressure:.2f} hPa")
            print(f"Altitude: {altitude:.1f} m")
            print(f"Acceleration: {accel_magnitude:.2f} m/s²")
            print("-" * 40)
            
            # Wait before next reading
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\nProgram terminated by user")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main() 