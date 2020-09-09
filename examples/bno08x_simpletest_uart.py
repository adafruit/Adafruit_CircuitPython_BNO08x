# SPDX-FileCopyrightText: 2020 Bryan Siepert, written for Adafruit Industries
#
# SPDX-License-Identifier: Unlicense
import time
import board
import busio
import adafruit_bno08x
from adafruit_bno08x.uart import BNO08X_UART

# import serial
# uart = serial.Serial("COM49", baudrate=3000000, timeout=1)
# print(uart.name)
uart = busio.UART(board.TX, board.RX, baudrate=3000000, receiver_buffer_size=2048)
bno = BNO08X_UART(uart, reset=None, debug=False)

bno.enable_feature(adafruit_bno08x.BNO_REPORT_ACCELEROMETER)
bno.enable_feature(adafruit_bno08x.BNO_REPORT_GYROSCOPE)
bno.enable_feature(adafruit_bno08x.BNO_REPORT_MAGNETIC_FIELD)
bno.enable_feature(adafruit_bno08x.BNO_REPORT_LINEAR_ACCELERATION)
bno.enable_feature(adafruit_bno08x.BNO_REPORT_ROTATION_VECTOR)
bno.enable_feature(adafruit_bno08x.BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR)
bno.enable_feature(adafruit_bno08x.BNO_REPORT_STEP_COUNTER)
bno.enable_feature(adafruit_bno08x.BNO_REPORT_SHAKE_DETECTOR)

while True:
    time.sleep(0.1)

    print("Acceleration:")
    accel_x, accel_y, accel_z = bno.acceleration  # pylint:disable=no-member
    print("X: %0.6f  Y: %0.6f Z: %0.6f  m/s^2" % (accel_x, accel_y, accel_z))
    print("")

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
