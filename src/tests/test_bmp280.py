import sys
import os

# Add parent directory to Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from sensors.bmp280 import BMP280
import time
import statistics

def test_bmp280():
    """Test program for BMP280 sensor performance."""
    try:
        # Initialize BMP280
        bmp280 = BMP280(bus=1)
        print("\nBMP280 Test Program")
        print("===================")
        
        # Print configuration
        print("\nSensor Configuration:")
        print("Temperature oversampling: x1")
        print("Pressure oversampling: x4")
        print("Filter coefficient: 4")
        print("Standby time: 125ms")
        print("Mode: Normal")
        
        # Collect samples
        samples = 10
        temps = []
        pressures = []
        
        print(f"\nCollecting {samples} samples with 1 second interval...")
        for i in range(samples):
            temp, pressure = bmp280.get_temperature_and_pressure()
            temps.append(temp)
            pressures.append(pressure)
            
            print(f"\nSample {i+1}:")
            print(f"Temperature: {temp:.2f}°C ({(temp * 9/5) + 32:.2f}°F)")
            print(f"Pressure: {pressure:.2f} hPa")
            time.sleep(1)
        
        # Calculate statistics
        print("\nStatistics:")
        print("Temperature:")
        print(f"  Mean: {statistics.mean(temps):.2f}°C ({(statistics.mean(temps) * 9/5) + 32:.2f}°F)")
        print(f"  Min: {min(temps):.2f}°C ({(min(temps) * 9/5) + 32:.2f}°F)")
        print(f"  Max: {max(temps):.2f}°C ({(max(temps) * 9/5) + 32:.2f}°F)")
        print(f"  Std Dev: {statistics.stdev(temps):.4f}°C")
        
        print("\nPressure:")
        print(f"  Mean: {statistics.mean(pressures):.2f} hPa")
        print(f"  Min: {min(pressures):.2f} hPa")
        print(f"  Max: {max(pressures):.2f} hPa")
        print(f"  Std Dev: {statistics.stdev(pressures):.4f} hPa")
        
        # Print calibration coefficients
        print("\nCalibration Coefficients:")
        print("Temperature:")
        print(f"dig_T1: {bmp280.calibration['dig_T1']}")
        print(f"dig_T2: {bmp280.calibration['dig_T2']}")
        print(f"dig_T3: {bmp280.calibration['dig_T3']}")
        
        print("\nPressure:")
        for i in range(1, 10):
            print(f"dig_P{i}: {bmp280.calibration[f'dig_P{i}']}")
            
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    except Exception as e:
        print(f"Error during test: {e}")

if __name__ == "__main__":
    test_bmp280() 