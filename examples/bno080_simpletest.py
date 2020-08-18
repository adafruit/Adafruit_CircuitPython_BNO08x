# SPDX-FileCopyrightText: 2017 Scott Shawcroft, written for Adafruit Industries
#
# SPDX-License-Identifier: Unlicense
from time import sleep
import board
import busio
from adafruit_bno080 import BNO080, REPORT_STATUS

i2c = busio.I2C(board.SCL, board.SDA)
bno = BNO080(i2c)
print("rp statu:", REPORT_STATUS)
while True:
    quat = bno.rotation_vector  # pylint:disable=no-member
    print("Rotation Vector Quaternion:")
    print(
        "I: %0.6f  J: %0.6f K: %0.6f  Real: %0.6f  Accuracy Estimate: %0.6f"
        % (quat.i, quat.j, quat.k, quat.real, quat.accuracy)
    )
    #     quat.i,
    print("")
    sleep(0.5)
