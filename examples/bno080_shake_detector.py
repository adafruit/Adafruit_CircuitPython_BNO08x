# SPDX-FileCopyrightText: 2020 Bryan Siepert, written for Adafruit Industries
#
# SPDX-License-Identifier: Unlicense
from time import sleep
import board
import busio
from adafruit_bno080.i2c import BNO080_I2C


i2c = busio.I2C(board.SCL, board.SDA)
bno = BNO080_I2C(i2c, debug=True)

while True:
    print("")
    print("Steps detected:", bno.steps)
    print("")
    if bno.shake:
        print("SHAKE DETECTED!")
        print("")

    sleep(0.5)

# TODO:
# Config report read/write
# unify packet handling
# * make send packet take a packet
# * reduce allocations
# More reports

# TODO LATER:
# * find decent shake tune? -
