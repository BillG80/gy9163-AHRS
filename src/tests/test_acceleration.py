import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from sensors.mpu9250 import MPU9250
from processing.orientation import Orientation
import time
import statistics

def test_acceleration():
    """Compare different acceleration calculations."""
    try:
        mpu9250 = MPU9250(bus=1)
        orientation = Orientation()
        
        print("\nAcceleration Comparison Test")
        print("===========================")
        
        samples = 20
        print(f"\nCollecting {samples} samples with 0.5 second interval...")
        print("Place the sensor still on a flat surface")
        time.sleep(2)
        
        for i in range(samples):
            accel, gyro = mpu9250.read_sensor_data()
            
            # 1. Raw magnitude (from main.py)
            raw_magnitude = (accel['x']**2 + accel['y']**2 + accel['z']**2)**0.5
            
            # 2. Z-axis without gravity (from orientation.py)
            z_accel = orientation.calculate_z_acceleration_without_gravity(accel)
            
            # Update orientation
            orientation.update_orientation(gyro, time.time())
            
            print(f"\nSample {i+1}:")
            print(f"Raw X: {accel['x']:.3f}g")
            print(f"Raw Y: {accel['y']:.3f}g")
            print(f"Raw Z: {accel['z']:.3f}g")
            print(f"1. Raw Magnitude: {raw_magnitude:.3f}g")
            print(f"2. Z-axis (no gravity): {z_accel:.3f}g")
            print(f"Roll: {orientation.roll:.1f}°")
            print(f"Pitch: {orientation.pitch:.1f}°")
            
            time.sleep(0.5)
            
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    except Exception as e:
        print(f"Error during test: {e}")

if __name__ == "__main__":
    test_acceleration() 