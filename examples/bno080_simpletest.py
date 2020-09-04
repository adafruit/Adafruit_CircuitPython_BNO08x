# SPDX-FileCopyrightText: 2020 Bryan Siepert, written for Adafruit Industries
#
# SPDX-License-Identifier: Unlicense
import time
import board
import busio
import adafruit_bno080
from adafruit_bno080.i2c import BNO080_I2C
from digitalio import DigitalInOut

i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
reset_pin = DigitalInOut(board.G0)
bno = BNO080_I2C(i2c, reset=reset_pin)

bno.enable_feature(adafruit_bno080.BNO_REPORT_ACCELEROMETER)
#bno.enable_feature(adafruit_bno080.BNO_REPORT_GYROSCOPE)
#bno.enable_feature(adafruit_bno080.BNO_REPORT_MAGNETIC_FIELD)

while True:

    print("Acceleration:")
    accel_x, accel_y, accel_z = bno.acceleration  # pylint:disable=no-member
    print("X: %0.6f  Y: %0.6f Z: %0.6f  m/s^2" % (accel_x, accel_y, accel_z))
    print("")

    """
    print("Gyro:")
    gyro_x, gyro_y, gyro_z = bno.gyro  # pylint:disable=no-member
    print("X: %0.6f  Y: %0.6f Z: %0.6f rads/s" % (gyro_x, gyro_y, gyro_z))
    print("")

    print("Magnetometer:")
    mag_x, mag_y, mag_z = bno.magnetic  # pylint:disable=no-member
    print("X: %0.6f  Y: %0.6f Z: %0.6f uT" % (mag_x, mag_y, mag_z))
    print("")

    print("Linear Acceleration:")
    (
        linear_accel_x,
        linear_accel_y,
        linear_accel_z,
    ) = bno.linear_acceleration  # pylint:disable=no-member
    print(
        "X: %0.6f  Y: %0.6f Z: %0.6f m/s^2"
        % (linear_accel_x, linear_accel_y, linear_accel_z)
    )
    print("")
    print("Rotation Vector Quaternion:")
    quat_i, quat_j, quat_k, quat_real = bno.quaternion  # pylint:disable=no-member

    print(
        "I: %0.6f  J: %0.6f K: %0.6f  Real: %0.6f" % (quat_i, quat_j, quat_k, quat_real)
    )
    print("")
    print("Geomagnetic Rotation Vector Quaternion:")
    (
        geo_quat_i,
        geo_quat_j,
        geo_quat_k,
        geo_quat_real,
    ) = bno.geomagnetic_quaternion  # pylint:disable=no-member

    print(
        "I: %0.6f  J: %0.6f K: %0.6f  Real: %0.6f" % (quat_i, quat_j, quat_k, quat_real)
    )
    print("")
    print("Steps detected:", bno.steps)
    print("")
    if bno.shake:
        print("SHAKE DETECTED!")
        print("")

    sleep(0.5)
    """
