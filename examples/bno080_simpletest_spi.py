# SPDX-FileCopyrightText: 2020 Bryan Siepert, written for Adafruit Industries
#
# SPDX-License-Identifier: Unlicense
from time import sleep
import board
import busio
from digitalio import DigitalInOut, Direction
from adafruit_bno080.spi import BNO080_SPI

# need to limit clock to 3Mhz
spi = busio.SPI(board.SCK, MISO=board.MISO, MOSI=board.MOSI)

cs = DigitalInOut(board.D5)
cs.direction = Direction.OUTPUT


int_pin = DigitalInOut(board.D6)
int_pin.direction = Direction.INPUT

bno = BNO080_SPI(spi, cs, int_pin, debug=True)
print("\n****** INIT COMPLETE *******")
while True:
    print("getting quat")
    quat = bno.quaternion  # pylint:disable=no-member
    print("Rotation Vector Quaternion:")
    print(
        "I: %0.6f  J: %0.6f K: %0.6f  Real: %0.6f" % (quat.i, quat.j, quat.k, quat.real)
    )
    print("")
    sleep(0.5)
