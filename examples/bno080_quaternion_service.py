# SPDX-FileCopyrightText: 2020 Bryan Siepert, written for Adafruit Industries
#
# SPDX-License-Identifier: MIT
import time
import board
from adafruit_ble import BLERadio
from adafruit_bno080 import BNO080
from adafruit_ble_adafruit.adafruit_service import AdafruitServerAdvertisement
from adafruit_ble_adafruit.quaternion_service import QuaternionService

i2c = board.I2C()
bno = BNO080(i2c)

quat_svc = QuaternionService()
quat_svc.measurement_period = 100
quat_last_read = 0

ble = BLERadio()

# The Web Bluetooth dashboard identifies known boards by their
# advertised name, not by advertising manufacturer data.
ble.name = "Adafruit Hillcrest Laboratories BNO080 Breakout"

adv = AdafruitServerAdvertisement()
adv.pid = 0x8088

while True:
    # Advertise when not connected.
    ble.start_advertising(adv)
    while not ble.connected:
        pass
    ble.stop_advertising()

    while ble.connected:
        now_msecs = time.monotonic_ns() // 1000000  # pylint: disable=no-member

        if now_msecs - quat_last_read >= quat_svc.measurement_period:
            quat_obj = bno.quaternion
            bno_080_quat_out = (quat_obj.i, quat_obj.j, quat_obj.i, quat_obj.real)

            quat_svc.quaternion = bno_080_quat_out
            quat_last_read = now_msecs
