# SPDX-FileCopyrightText: 2020 Bryan Siepert, written for Adafruit Industries
#
# SPDX-License-Identifier: Unlicense
from time import sleep
import board
import busio
from digitalio import DigitalInOut, Direction
from adafruit_bno080.spi import BNO080_SPI
from adafruit_bno080 import REPORT_STATUS

spi = busio.SPI(board.SCK, MISO=board.MISO, MOSI=board.MOSI)
cs = DigitalInOut(board.D5)
cs.direction = Direction.OUTPUT
bno = BNO080_SPI(spi, cs, debug=True)
print("We made it out!")
while True:
    quat = bno.quaternion  # pylint:disable=no-member
    print("Rotation Vector Quaternion:")
    print(
        "I: %0.6f  J: %0.6f K: %0.6f  Real: %0.6f  Accuracy Estimate: %0.6f Status: %s"
        % (quat.i, quat.j, quat.k, quat.real, quat.accuracy, REPORT_STATUS[quat.status])
    )
    print("")
    sleep(0.5)
