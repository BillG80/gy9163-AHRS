import numpy as np
from typing import Dict

class Gravity:
    def __init__(self):
        # Initialize any necessary variables
        self.gravity_vector = np.array([0.0, 0.0, 9.81])  # Default gravity vector pointing downwards

    def calculate_gravity(self, accel: Dict[str, float]) -> np.ndarray:
        """Calculate the gravity vector based on accelerometer data."""
        # Assuming the accelerometer data is in m/s²
        accel_vector = np.array([accel['x'], accel['y'], accel['z']])
        
        # Normalize the accelerometer vector to get the gravity direction
        norm = np.linalg.norm(accel_vector)
        if norm == 0:
            return self.gravity_vector  # Avoid division by zero
        
        self.gravity_vector = accel_vector / norm * 9.81  # Scale to standard gravity
        return self.gravity_vector

    def print_gravity(self):
        """Print the current gravity vector."""
        print("Gravity Vector (m/s²):")
        print(self.gravity_vector)

# Example usage
if __name__ == "__main__":
    gravity = Gravity()
    sample_accel_data = {'x': 0.0, 'y': 0.0, 'z': 9.81}
    gravity_vector = gravity.calculate_gravity(sample_accel_data)
    gravity.print_gravity() 