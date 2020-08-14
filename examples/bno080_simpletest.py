# SPDX-FileCopyrightText: 2017 Scott Shawcroft, written for Adafruit Industries
#
# SPDX-License-Identifier: Unlicense
from time import sleep
import board
import busio
import adafruit_bno080

i2c = busio.I2C(board.SCL, board.SDA)
bno = adafruit_bno080.BNO080(i2c, debug=True)

while True:
    quat = bno.rotation_vector  # pylint:disable=no-member
    print("Rotation Vector Quaternion:")
    print(
        "I: %0.3f J: %0.3f K: %0.3f Real: %0.3f Accuracy: %0.3f"
        % (quat.i, quat.j, quat.k, quat.real, quat.accuracy)
    )
    sleep(0.5)
