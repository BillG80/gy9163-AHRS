import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from sensors.mpu9250 import MPU9250
from processing.orientation import Orientation
import time
import numpy as np

def test_simple_magnitude():
    """Test simplified acceleration magnitude calculation."""
    try:
        mpu9250 = MPU9250(bus=1)
        orientation = Orientation()
        
        print("\nSimple Acceleration Test")
        print("======================")
        print("Place sensor still on flat surface")
        time.sleep(2)
        
        samples = 20
        for i in range(samples):
            accel, _ = mpu9250.read_sensor_data()
            
            # Calculate raw magnitude
            raw_magnitude = np.sqrt(accel['x']**2 + accel['y']**2 + accel['z']**2)
            
            # Get filtered magnitude with gravity removed
            linear_accel = orientation.get_linear_acceleration(accel)
            
            print(f"\nSample {i+1}:")
            print(f"Raw magnitude (with gravity): {raw_magnitude:.3f}g")
            print(f"Linear acceleration (no gravity): {linear_accel['magnitude']:.3f}g")
            
            time.sleep(0.5)
            
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    except Exception as e:
        print(f"Error during test: {e}")

if __name__ == "__main__":
    test_simple_magnitude() 