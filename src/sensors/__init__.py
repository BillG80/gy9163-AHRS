"""
Sensor package initialization
Makes sensors available for import directly from the package
"""

from .ms5611 import MS5611
from .bmp280 import BMP280
from .mpu9250 import MPU9250

__all__ = ['MS5611', 'BMP280', 'MPU9250']
