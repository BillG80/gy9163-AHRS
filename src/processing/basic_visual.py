import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from typing import List, Tuple

class Visualization:
    def __init__(self):
        # Initialize plots
        self.fig = plt.figure(figsize=(15, 5))
        
        # 3D Orientation plot
        self.ax_orientation = self.fig.add_subplot(131, projection='3d')
        self.ax_orientation.set_title('3D Orientation')
        
        # Z-axis Acceleration and Speed plot
        self.ax_z_axis = self.fig.add_subplot(132)
        self.ax_z_axis.set_title('Z-axis Acceleration and Speed')
        self.z_accel_data = []
        self.z_speed_data = []
        
        # Pressure Record plot
        self.ax_pressure = self.fig.add_subplot(133)
        self.ax_pressure.set_title('Pressure Record (24hrs)')
        self.gy91_pressure_data = []
        self.gy63_pressure_data = []
        
        plt.ion()  # Interactive mode on

    def update_orientation(self, quaternion: Tuple[float, float, float, float]):
        """Update the 3D orientation plot."""
        qw, qx, qy, qz = quaternion
        # Convert quaternion to rotation matrix
        R = np.array([
            [1 - 2*qy**2 - 2*qz**2, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
            [2*qx*qy + 2*qz*qw, 1 - 2*qx**2 - 2*qz**2, 2*qy*qz - 2*qx*qw],
            [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2]
        ])
        
        # Plot orientation
        self.ax_orientation.clear()
        self.ax_orientation.quiver(0, 0, 0, R[0, 0], R[1, 0], R[2, 0], color='r')
        self.ax_orientation.quiver(0, 0, 0, R[0, 1], R[1, 1], R[2, 1], color='g')
        self.ax_orientation.quiver(0, 0, 0, R[0, 2], R[1, 2], R[2, 2], color='b')
        self.ax_orientation.set_xlim([-1, 1])
        self.ax_orientation.set_ylim([-1, 1])
        self.ax_orientation.set_zlim([-1, 1])
        self.ax_orientation.set_xlabel('X')
        self.ax_orientation.set_ylabel('Y')
        self.ax_orientation.set_zlabel('Z')

    def update_z_axis(self, z_accel: float, z_speed: float):
        """Update the Z-axis acceleration and speed plot."""
        self.z_accel_data.append(z_accel)
        self.z_speed_data.append(z_speed)
        
        self.ax_z_axis.clear()
        self.ax_z_axis.plot(self.z_accel_data, label='Z Acceleration')
        self.ax_z_axis.plot(self.z_speed_data, label='Z Speed')
        self.ax_z_axis.legend()

    def update_pressure(self, gy91_pressure: float, gy63_pressure: float):
        """Update the pressure record plot."""
        self.gy91_pressure_data.append(gy91_pressure)
        self.gy63_pressure_data.append(gy63_pressure)
        
        self.ax_pressure.clear()
        self.ax_pressure.plot(self.gy91_pressure_data, label='GY91 Pressure')
        self.ax_pressure.plot(self.gy63_pressure_data, label='GY63 Pressure')
        self.ax_pressure.legend()

    def refresh(self):
        """Refresh the plots."""
        plt.draw()
        plt.pause(0.01) 