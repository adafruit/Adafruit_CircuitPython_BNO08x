
from micropython import const
_FRS_RECORD_SHAKE = 0x7D7D
_SENSOR_METADATA_SHAKE = 0xE318
"""
0 Minimum time
1 Maximum time
2 Threshold
3 Shake count
4 Enable flags
""""

_FRS_RECORD_ACCEL = const(0xE302)
_FRS_RECORD_GYRO = const(0xE306)
_FRS_RECORD_MAGN = const(0xE309)
_FRS_RECORD_ROTATION_VECTOR = const(0xE30B)


