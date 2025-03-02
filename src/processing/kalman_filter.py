import numpy as np
from typing import Dict

class KalmanFilter:
    def __init__(self):
        # State vector [x, y, z, vx, vy, vz, qw, qx, qy, qz]
        self.state = np.zeros(10)
        
        # State covariance matrix
        self.P = np.eye(10)
        
        # Process noise covariance
        self.Q = np.eye(10) * 0.1
        
        # Measurement noise covariance
        self.R = np.eye(6) * 0.1
        
        # Identity matrix
        self.I = np.eye(10)

    def predict(self, dt: float, gyro: Dict[str, float]):
        """Predict the next state based on the gyroscope data."""
        # Convert gyro rates to radians
        wx, wy, wz = np.radians(gyro['x']), np.radians(gyro['y']), np.radians(gyro['z'])
        
        # Update quaternion based on gyro rates
        qw, qx, qy, qz = self.state[6:10]
        q_dot = 0.5 * np.array([
            -qx * wx - qy * wy - qz * wz,
             qw * wx + qy * wz - qz * wy,
             qw * wy - qx * wz + qz * wx,
             qw * wz + qx * wy - qy * wx
        ])
        
        # Update state
        self.state[6:10] += q_dot * dt
        self.state[6:10] /= np.linalg.norm(self.state[6:10])  # Normalize quaternion

        # Update velocity
        self.state[3:6] += self.state[0:3] * dt

        # Update state covariance
        self.P += self.Q * dt

    def update(self, accel: Dict[str, float]):
        """Update the state based on accelerometer data."""
        # Measurement vector
        z = np.array([accel['x'], accel['y'], accel['z'], 0, 0, 0])
        
        # Measurement matrix
        H = np.zeros((6, 10))
        H[0:3, 0:3] = np.eye(3)  # Acceleration
        H[3:6, 3:6] = np.eye(3)  # Velocity
        
        # Innovation
        y = z - H @ self.state
        
        # Innovation covariance
        S = H @ self.P @ H.T + self.R
        
        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # Update state
        self.state += K @ y
        
        # Update state covariance
        self.P = (self.I - K @ H) @ self.P

    def get_state(self):
        """Return the current state."""
        return {
            'position': self.state[0:3],
            'velocity': self.state[3:6],
            'quaternion': self.state[6:10]
        } 