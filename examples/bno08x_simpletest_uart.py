# SPDX-FileCopyrightText: 2020 Bryan Siepert, written for Adafruit Industries
#
# SPDX-License-Identifier: Unlicense
import time

import board
import busio

import adafruit_bno08x
from adafruit_bno08x.uart import BNO08X_UART

uart = busio.UART(board.TX, board.RX, baudrate=3000000, receiver_buffer_size=2048)

# uncomment and comment out the above for use with Raspberry Pi
# import serial
# uart = serial.Serial("/dev/serial0", 3000000)

# for USB Serial (via FTDI cable):
# import serial
# uart = serial.Serial("/dev/ttyUSB0", baudrate=3000000)

bno = BNO08X_UART(uart)

bno.enable_feature(adafruit_bno08x.BNO_REPORT_ACCELEROMETER)
bno.enable_feature(adafruit_bno08x.BNO_REPORT_GYROSCOPE)
bno.enable_feature(adafruit_bno08x.BNO_REPORT_MAGNETOMETER)
bno.enable_feature(adafruit_bno08x.BNO_REPORT_ROTATION_VECTOR)

# More reports! Uncomment the enable line below along with the print
# in the loop below
# bno.enable_feature(adafruit_bno08x.BNO_REPORT_LINEAR_ACCELERATION)
# bno.enable_feature(adafruit_bno08x.BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR)
# bno.enable_feature(adafruit_bno08x.BNO_REPORT_GAME_ROTATION_VECTOR)
# bno.enable_feature(adafruit_bno08x.BNO_REPORT_STEP_COUNTER)
# bno.enable_feature(adafruit_bno08x.BNO_REPORT_STABILITY_CLASSIFIER)
# bno.enable_feature(adafruit_bno08x.BNO_REPORT_ACTIVITY_CLASSIFIER)
# bno.enable_feature(adafruit_bno08x.BNO_REPORT_SHAKE_DETECTOR)
# bno.enable_feature(adafruit_bno08x.BNO_REPORT_RAW_ACCELEROMETER)
# bno.enable_feature(adafruit_bno08x.BNO_REPORT_RAW_GYROSCOPE)
# bno.enable_feature(adafruit_bno08x.BNO_REPORT_RAW_MAGNETOMETER)

while True:
    time.sleep(0.5)

    print("Acceleration:")
    accel_x, accel_y, accel_z = bno.acceleration
    print("X: %0.6f  Y: %0.6f Z: %0.6f  m/s^2" % (accel_x, accel_y, accel_z))
    print("")

    print("Gyro:")
    gyro_x, gyro_y, gyro_z = bno.gyro
    print("X: %0.6f  Y: %0.6f Z: %0.6f rads/s" % (gyro_x, gyro_y, gyro_z))
    print("")

    print("Magnetometer:")
    mag_x, mag_y, mag_z = bno.magnetic
    print("X: %0.6f  Y: %0.6f Z: %0.6f uT" % (mag_x, mag_y, mag_z))
    print("")

    print("Rotation Vector Quaternion:")
    quat_i, quat_j, quat_k, quat_real = bno.quaternion
    print("I: %0.6f  J: %0.6f K: %0.6f  Real: %0.6f" % (quat_i, quat_j, quat_k, quat_real))
    print("")

    # print("Linear Acceleration:")
    # (
    #     linear_accel_x,
    #     linear_accel_y,
    #     linear_accel_z,
    # ) = bno.linear_acceleration
    # print(
    #     "X: %0.6f  Y: %0.6f Z: %0.6f m/s^2"
    #     % (linear_accel_x, linear_accel_y, linear_accel_z)
    # )
    # print("")

    # print("Geomagnetic Rotation Vector Quaternion:")
    # (
    #     geo_quat_i,
    #     geo_quat_j,
    #     geo_quat_k,
    #     geo_quat_real,
    # ) = bno.geomagnetic_quaternion
    # print(
    #     "I: %0.6f  J: %0.6f K: %0.6f  Real: %0.6f"
    #     % (geo_quat_i, geo_quat_j, geo_quat_k, geo_quat_real)
    # )
    # # print("")

    # print("Game Rotation Vector Quaternion:")
    # (
    #     game_quat_i,
    #     game_quat_j,
    #     game_quat_k,
    #     game_quat_real,
    # ) = bno.game_quaternion
    # print(
    #     "I: %0.6f  J: %0.6f K: %0.6f  Real: %0.6f"
    #     % (game_quat_i, game_quat_j, game_quat_k, game_quat_real)
    # )
    # print("")

    # print("Steps detected:", bno.steps)
    # print("")

    # print("Stability classification:", bno.stability_classification)
    # print("")

    # activity_classification = bno.activity_classification
    # most_likely = activity_classification["most_likely"]
    # print(
    #     "Activity classification:",
    #     most_likely,
    #     "confidence: %d/100" % activity_classification[most_likely],
    # )

    # print("Raw Acceleration:")
    # (raw_accel_x, raw_accel_y, raw_accel_z,) = bno.raw_acceleration
    # print(
    #     "X: 0x{0:04X}  Y: 0x{1:04X} Z: 0x{2:04X} LSB".format(
    #         raw_accel_x, raw_accel_y, raw_accel_z
    #     )
    # )
    # print("")

    # print("Raw Gyro:")
    # (raw_accel_x, raw_accel_y, raw_accel_z,) = bno.raw_gyro
    # print(
    #     "X: 0x{0:04X}  Y: 0x{1:04X} Z: 0x{2:04X} LSB".format(
    #         raw_accel_x, raw_accel_y, raw_accel_z
    #     )
    # )
    # print("")

    # print("Raw Magnetometer:")
    # (raw_mag_x, raw_mag_y, raw_mag_z,) = bno.raw_magnetic
    # print(
    #     "X: 0x{0:04X}  Y: 0x{1:04X} Z: 0x{2:04X} LSB".format(
    #         raw_mag_x, raw_mag_y, raw_mag_z
    #     )
    # )
    # print("")
    # time.sleep(0.4)
    # if bno.shake:
    #     print("SHAKE DETECTED!")
    #     print("")
