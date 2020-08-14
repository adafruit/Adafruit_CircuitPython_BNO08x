# SPDX-FileCopyrightText: Copyright (c) 2020 Bryan Siepert for Adafruit Industries
#
# SPDX-License-Identifier: MIT
"""
`adafruit_bno080`
================================================================================

Helper library for the Hillcrest Laboratories BNO080 IMU


* Author(s): Bryan Siepert

Implementation Notes
--------------------

**Hardware:**

* `Adafruit BNO080 Breakout <https:www.adafruit.com/products/47XX>`_

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://github.com/adafruit/circuitpython/releases

* `Adafruit's Bus Device library <https://github.com/adafruit/Adafruit_CircuitPython_BusDevice>`_
"""
__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_BNO080.git"

from struct import unpack_from, pack_into
from time import sleep
from micropython import const
import adafruit_bus_device.i2c_device as i2c_device

# Channel 0: the SHTP command channel
_BNO_CHANNEL_EXE = const(1)
_BNO_CHANNEL_CONTROL = const(2)
_BNO_CHANNEL_INPUT_SENSOR_REPORTS = const(3)
_BNO_CHANNEL_WAKE_INPUT_SENSOR_REPORTS = const(4)
_BNO_CHANNEL_GYRO_ROTATION_VECTOR = const(5)

_BNO080_CMD_RESET = const(0x01)

_BNO080_DEFAULT_ADDRESS = const(0x4A)


_SHTP_REPORT_PRODUCT_ID_RESPONSE = const(0xF8)
_SHTP_REPORT_PRODUCT_ID_REQUEST = const(0xF9)

_DATA_BUFFER_SIZE = const(512)  # data buffer size. obviously eats ram


class BNO080:
    """Library for the BNO080 IMU from Hillcrest Laboratories


        :param ~busio.I2C i2c_bus: The I2C bus the BNO080 is connected to.

    """

    def __init__(self, i2c_bus, address=_BNO080_DEFAULT_ADDRESS, debug=False):
        self._debug = debug
        self.i2c_device = i2c_device.I2CDevice(i2c_bus, address)
        self._data_buffer = bytearray(_DATA_BUFFER_SIZE)
        self._sequence_number = [0, 0, 0, 0, 0, 0]
        self.reset()
        self._check_id()

    def reset(self):
        """Reset the sensor to an initial unconfigured state"""
        data = bytearray(1)
        data[0] = 1
        print("Sending reset packet")
        self._send_packet(_BNO_CHANNEL_EXE, data)
        self._dbg("PACKET SENT")
        sleep(0.050)

        sleep(1)
        data_read = True
        while data_read:
            print("Still reading packet")
            data_read = self._read_packet()
            self._dbg("data read:", data_read)

        sleep(0.050)
        data_read = True
        while data_read:
            print("Again reading packet")
            data_read = self._read_packet()
            self._dbg("data read:", data_read)

    def _check_id(self):
        print("Checking ID:")
        data = bytearray(2)
        data[0] = _SHTP_REPORT_PRODUCT_ID_REQUEST
        data[1] = 0  # padding
        self._send_packet(_BNO_CHANNEL_CONTROL, data)
        if self._read_packet():
            print("packet read!")
            sensor_id = self._get_sensor_id()
            if sensor_id:
                print("Sensor id:", sensor_id)
                return True

        return False

    def _get_sensor_id(self):
        if not self._data_buffer[4] == _SHTP_REPORT_PRODUCT_ID_RESPONSE:
            return None
        # 0 Report ID = 0xF8
        # 14 Reserved
        sw_major = self._get_data(2, "<B")
        sw_minor = self._get_data(3, "<B")
        sw_patch = self._get_data(12, "<H")
        sw_part_number = self._get_data(4, "<I")
        sw_build_number = self._get_data(8, "<I")
        print("*** Part Number: %d" % sw_part_number)
        print("*** Software Version: %d.%d.%d" % (sw_major, sw_minor, sw_patch), end="")
        print(" Build: %d" % (sw_build_number))
        return sw_part_number

    def _send_packet(self, channel, data):
        self._dbg("")
        self._dbg("SENDing packet")
        data_length = len(data)
        write_length = data_length + 4
        self._dbg("\tChannel:", channel)
        self._dbg("\tData length:", data_length)
        # struct.pack_into(fmt, buffer, offset, *values)
        pack_into("<H", self._data_buffer, 0, write_length)
        self._data_buffer[2] = channel
        self._data_buffer[3] = self._sequence_number[channel]

        # this is dumb but it's what we have for now
        for idx, send_byte in enumerate(data):
            self._data_buffer[4 + idx] = send_byte

        self._print_header()
        with self.i2c_device as i2c:
            self._dbg("\twriting header and data at once")
            i2c.write(self._data_buffer, end=write_length)

        self._sequence_number[channel] += 1

    # returns true if available data was read
    # the sensor will always tell us how much there is, so no need to track it ourselves
    def _read_packet(self):
        # TODO: FIZXME

        sleep(0.001)
        with self.i2c_device as i2c:
            i2c.readinto(self._data_buffer, end=4)  # this is expecting a header?
        # struct.unpack_from(fmt, data, offset=0)
        self._dbg("")
        self._dbg("READing packet")
        self._print_header()
        packet_byte_count, channel_number, sequence_number = self._get_header()

        self._sequence_number[channel_number] = sequence_number

        if packet_byte_count == 0:
            return False
        # remove header size from read length
        packet_byte_count -= 4
        self._dbg(
            "channel",
            channel_number,
            "has",
            packet_byte_count,
            "bytes available to read",
        )
        # TODO: exception handling
        data_remaining = self._read(packet_byte_count)
        self._print_header()
        _data_len, channel, seq = self._get_header()
        self._sequence_number[channel] = seq

        if data_remaining:
            self._dbg("Unread data still for channel", channel_number)

        return True

    # returns true if all requested data was read
    def _read(self, requested_read_length):
        self._dbg("trying to read", requested_read_length, "bytes")
        unread_bytes = 0
        # +4 for the header
        total_read_length = requested_read_length + 4
        if total_read_length > _DATA_BUFFER_SIZE:
            unread_bytes = total_read_length - _DATA_BUFFER_SIZE
            total_read_length = _DATA_BUFFER_SIZE
        self._dbg(
            "reading",
            total_read_length,
            "bytes(%d+4)" % requested_read_length,
            "leaving",
            unread_bytes,
            "unread bytes",
        )
        with self.i2c_device as i2c:
            i2c.readinto(self._data_buffer, end=total_read_length)

        return unread_bytes > 0

    def _get_header(self):

        packet_byte_count = unpack_from("<H", self._data_buffer)[0]
        packet_byte_count &= ~0x8000
        channel_number = unpack_from("<B", self._data_buffer, offset=2)[0]
        sequence_number = unpack_from("<B", self._data_buffer, offset=3)[0]
        return (packet_byte_count, channel_number, sequence_number)

    def _dbg(self, *args, **kwargs):
        if self._debug:
            print("\tDBG::", *args, **kwargs)

    def _print_header(self):
        packet_byte_count, channel_number, sequence_number = self._get_header()

        self._dbg("HEADER:")
        raw_len_bytes = self._data_buffer[1] << 8 | self._data_buffer[0]
        is_continue = self._data_buffer[1] & 0x80 > 0
        if is_continue:
            self._dbg("\tCONTINUE")
        self._dbg("\tLen: %d (%s) " % (packet_byte_count, hex(raw_len_bytes)))
        self._dbg("\tChannel:", channel_number)
        self._dbg("\tSequence number:", sequence_number)

    def _get_data(self, index, fmt_string):
        # index arg is not including header, so add 4 into data buffer
        data_index = index + 4
        return unpack_from(fmt_string, self._data_buffer, offset=data_index)[0]

    # Useful features to possibly include:
    # re-map orientation "FRS record (0x2D3E)"
    # DFU Support
    # Angular velocity
    # • Angular position (quaternion)
    # • Data returned at configurable sample rates
    # • Timestamps attached to sensor reports
    # • Low latency, 1kHz gyro rotation vector for AR/VR
    # Built-in stability detector, tap detector, and step counter
