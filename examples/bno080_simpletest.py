# SPDX-FileCopyrightText: 2020 Bryan Siepert, written for Adafruit Industries
#
# SPDX-License-Identifier: Unlicense
from time import sleep
import board
import busio
from adafruit_bno080.i2c import BNO080_I2C
from adafruit_bno080 import REPORT_STATUS

i2c = busio.I2C(board.SCL, board.SDA)
bno = BNO080_I2C(i2c)

while True:
    quat = bno.quaternion  # pylint:disable=no-member
    print("Rotation Vector Quaternion:")
    print(
        "I: %0.6f  J: %0.6f K: %0.6f  Real: %0.6f  Accuracy Estimate: %0.6f Status: %s"
        % (quat.i, quat.j, quat.k, quat.real, quat.accuracy, REPORT_STATUS[quat.status])
    )
    print("")
    sleep(0.5)
