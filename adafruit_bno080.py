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
  https:# github.com/adafruit/circuitpython/releases

* `Adafruit's Bus Device library <https:# github.com/adafruit/Adafruit_CircuitPython_BusDevice>`_
"""
__version__ = "0.0.0-auto.0"
__repo__ = "https:# github.com/adafruit/Adafruit_CircuitPython_BNO080.git"

from struct import unpack_from, pack_into
from collections import namedtuple
from time import sleep, monotonic
from micropython import const
import adafruit_bus_device.i2c_device as i2c_device

# Channel 0: the SHTP command channel
_BNO_CHANNEL_EXE = const(1)
_BNO_CHANNEL_CONTROL = const(2)
_BNO_CHANNEL_INPUT_SENSOR_REPORTS = const(3)
_BNO_CHANNEL_WAKE_INPUT_SENSOR_REPORTS = const(4)
_BNO_CHANNEL_GYRO_ROTATION_VECTOR = const(5)

_BNO_CMD_GET_FEATURE_REQUEST = const(0xFE)
_BNO_CMD_SET_FEATURE_COMMAND = const(0xFD)
_BNO_CMD_GET_FEATURE_RESPONSE = const(0xFC)

_BNO_CMD_BASE_TIMESTAMP = const(0xFB)
_BNO_CMD_TIMESTAMP_REBASE = const(0xFA)
########## TODO ##########
_BNO_CMD_PRODUCT_ID_REQUEST = const(0xF9)
_BNO_CMD_PRODUCT_ID_RESPONSE = const(0xF8)

_SHTP_REPORT_PRODUCT_ID_RESPONSE = const(0xF8)
_SHTP_REPORT_PRODUCT_ID_REQUEST = const(0xF9)
##################

_BNO_CMD_FRS_WRITE_REQUEST = const(0xF7)
_BNO_CMD_FRS_WRITE_DATA = const(0xF6)
_BNO_CMD_FRS_WRITE_RESPONSE = const(0xF5)

_BNO_CMD_FRS_READ_REQUEST = const(0xF4)
_BNO_CMD_FRS_READ_RESPONSE = const(0xF3)

_BNO_CMD_COMMAND_REQUEST = const(0xF2)
_BNO_CMD_COMMAND_RESPONSE = const(0xF1)

_BNO_REPORT_ACCELEROMETER = const(0x01)
_BNO_REPORT_GYROSCOPE = const(0x02)
_BNO_REPORT_MAGNETIC_FIELD = const(0x03)
_BNO_REPORT_LINEAR_ACCELERATION = const(0x04)
_BNO_REPORT_ROTATION_VECTOR = const(0x05)
_BNO_REPORT_GRAVITY = const(0x06)
_BNO_REPORT_UNCALIBRATED_GYROSCOPE = const(0x07)
_BNO_REPORT_GAME_ROTATION_VECTOR = const(0x08)
_BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR = const(0x09)
_BNO_REPORT_PRESSURE = const(0x0A)
_BNO_REPORT_AMBIENT_LIGHT = const(0x0B)
_BNO_REPORT_HUMIDITY = const(0x0C)
_BNO_REPORT_PROXIMITY = const(0x0D)
_BNO_REPORT_TEMPERATURE = const(0x0E)
_BNO_REPORT_UNCALIBRATED_MAGNETIC_FIELD = const(0x0F)
_BNO_REPORT_TAP_DETECTOR = const(0x10)
_BNO_REPORT_STEP_COUNTER = const(0x11)
_BNO_REPORT_SIGNIFICANT_MOTION = const(0x12)
_BNO_REPORT_STABILITY_CLASSIFIER = const(0x13)
_BNO_REPORT_RAW_ACCELEROMETER = const(0x14)
_BNO_REPORT_RAW_GYROSCOPE = const(0x15)
_BNO_REPORT_RAW_MAGNETOMETER = const(0x16)
_BNO_REPORT_SAR = const(0x17)
_BNO_REPORT_STEP_DETECTOR = const(0x18)
_BNO_REPORT_SHAKE_DETECTOR = const(0x19)
_BNO_REPORT_FLIP_DETECTOR = const(0x1A)
_BNO_REPORT_PICKUP_DETECTOR = const(0x1B)
_BNO_REPORT_STABILITY_DETECTOR = const(0x1C)
_BNO_REPORT_PERSONAL_ACTIVITY_CLASSIFIER = const(0x1E)
_BNO_REPORT_SLEEP_DETECTOR = const(0x1F)
_BNO_REPORT_TILT_DETECTOR = const(0x20)
_BNO_REPORT_POCKET_DETECTOR = const(0x21)
_BNO_REPORT_CIRCLE_DETECTOR = const(0x22)
_BNO_REPORT_HEART_RATE_MONITOR = const(0x23)
_BNO_REPORT_ARVR_STABILIZED_ROTATION_VECTOR = const(0x28)
_BNO_REPORT_ARVR_STABILIZED_GAME_ROTATION_VECTOR = const(0x29)

_QUAT_REPORT_INTERVAL = const(50000)  # in microseconds = 50ms
_QUAT_READ_TIMEOUT = 1.000  # timeout in seconds
_BNO080_CMD_RESET = const(0x01)
_QUAT_Q_POINT = const(14)
_BNO080_DEFAULT_ADDRESS = const(0x4A)
_BNO_HEADER_LEN = const(4)

_Q_POINT_14_SCALAR = 2 ** (14 * -1)
_Q_POINT_12_SCALAR = 2 ** (12 * -1)


_DATA_BUFFER_SIZE = const(512)  # data buffer size. obviously eats ram
Quaternion = namedtuple("Quaternion", ["i", "j", "k", "real", "accuracy", "status"],)
PacketHeader = namedtuple(
    "PacketHeader", ["data_length", "channel_number", "sequence_number",],
)

REPORT_STATUS = ["Unreliable", "Accuracy low", "Accuracy medium", "Accuracy high"]


def _elapsed(start_time):
    return monotonic() - start_time


def _get_header(packet_bytes):
    packet_byte_count = unpack_from("<H", packet_bytes)[0]
    packet_byte_count &= ~0x8000
    channel_number = unpack_from("<B", packet_bytes, offset=2)[0]
    sequence_number = unpack_from("<B", packet_bytes, offset=3)[0]
    header = PacketHeader(packet_byte_count - 4, channel_number, sequence_number)
    return header


def _parse_quat(packet):

    status_int = unpack_from("<B", packet.data, offset=7)[0]
    status_int &= 0x03
    quat_i = unpack_from("<H", packet.data, offset=9)[0] * _Q_POINT_14_SCALAR
    quat_j = unpack_from("<H", packet.data, offset=11)[0] * _Q_POINT_14_SCALAR
    quat_k = unpack_from("<H", packet.data, offset=13)[0] * _Q_POINT_14_SCALAR
    quat_real = unpack_from("<H", packet.data, offset=15)[0] * _Q_POINT_14_SCALAR
    acc_est = unpack_from("<H", packet.data, offset=17)[0] * _Q_POINT_12_SCALAR

    return (quat_i, quat_j, quat_k, quat_real, acc_est, status_int)


# This function pulls the data from the command response report

# Unit responds with packet that contains the following:
# shtpHeader[0:3]: First, a 4 byte header
# shtpData[0]: The Report ID
# shtpData[1]: Sequence number (See 6.5.18.2)
# shtpData[2]: Command
# shtpData[3]: Command Sequence Number
# shtpData[4]: Response Sequence Number
# shtpData[5 + 0]: R0
# shtpData[5 + 1]: R1
# shtpData[5 + 2]: R2
# shtpData[5 + 3]: R3
# shtpData[5 + 4]: R4
# shtpData[5 + 5]: R5
# shtpData[5 + 6]: R6
# shtpData[5 + 7]: R7
# shtpData[5 + 8]: R8
class Packet:
    """A class representing a Hillcrest Laboratory **Sensor Hub Transport
Protocol** packet"""

    def __init__(self, packet_bytes):
        self.header = _get_header(packet_bytes)
        data_end_index = self.header.data_length + _BNO_HEADER_LEN
        self.data = packet_bytes[_BNO_HEADER_LEN:data_end_index]

    def __str__(self):
        outstr = ""
        outstr += "HEADER:"

        outstr += "\tLen: %d\n" % (self.header.data_length)
        outstr += "\tChannel: %s\n" % self.header.channel_number
        outstr += "\tSequence number: %s\n" % self.header.sequence_number
        outstr += "\nData:"
        for _idx, _byte in enumerate(self.data):
            outstr += "\t[%0.2d] %x\n" % (_idx, _byte)
        return outstr


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
        self._enable_rotation_vector()

    def reset(self):
        """Reset the sensor to an initial unconfigured state"""
        data = bytearray(1)
        data[0] = 1
        self._send_packet(_BNO_CHANNEL_EXE, data)
        self._dbg("PACKET SENT")
        sleep(0.050)

        sleep(1)
        data_read = True
        while data_read:
            data_read = self._read_packet()
            self._dbg("data read:", data_read)

        sleep(0.050)
        data_read = True
        while data_read:
            data_read = self._read_packet()
            self._dbg("data read:", data_read)

    # Constructs a report  to set a feature
    # later: class-ify
    # def _set_feature_report(self, feature_id):
    @property
    def rotation_vector(self):
        """A quaternion representing the current rotation vector"""
        # create and send a packet to enable the quaternion data

        quat_i = 0.0
        quat_j = 0.0
        quat_k = 0.0
        quat_real = 0.00
        acc_est = 0.0
        status_int = None
        quat = None
        # receive packets, and dump until you get a quat packet
        start_time = monotonic()
        while _elapsed(start_time) < _QUAT_READ_TIMEOUT:
            data_was_read = self._read_packet()
            if not data_was_read:
                break
            new_packet = Packet(self._data_buffer)
            if new_packet.header.channel_number == _BNO_CHANNEL_INPUT_SENSOR_REPORTS:
                if new_packet.data[5] == _BNO_REPORT_ROTATION_VECTOR:
                    (
                        quat_i,
                        quat_j,
                        quat_k,
                        quat_real,
                        acc_est,
                        status_int,
                    ) = _parse_quat(new_packet)
                    return Quaternion(
                        quat_i, quat_j, quat_k, quat_real, acc_est, status_int
                    )

        if quat:
            return quat

        return Quaternion(quat_i, quat_j, quat_k, quat_real, acc_est, status_int)

    def _enable_rotation_vector(self):

        set_feature_report = bytearray(17)
        set_feature_report[0] = _BNO_CMD_SET_FEATURE_COMMAND
        set_feature_report[1] = _BNO_REPORT_ROTATION_VECTOR
        pack_into("<I", set_feature_report, 5, _QUAT_REPORT_INTERVAL)

        self._send_packet(_BNO_CHANNEL_CONTROL, set_feature_report)

    def _check_id(self):
        data = bytearray(2)
        data[0] = _SHTP_REPORT_PRODUCT_ID_REQUEST
        data[1] = 0  # padding
        self._send_packet(_BNO_CHANNEL_CONTROL, data)
        if self._read_packet():
            sensor_id = self._get_sensor_id()
            if sensor_id:
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

        self._dbg("")
        self._dbg("*** Part Number: %d" % sw_part_number)
        self._dbg(
            "*** Software Version: %d.%d.%d" % (sw_major, sw_minor, sw_patch), end=""
        )
        self._dbg(" Build: %d" % (sw_build_number))
        self._dbg("")
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

        self._dbg_print_header()

        with self.i2c_device as i2c:
            self._dbg("\twriting header and data at once")
            i2c.write(self._data_buffer, end=write_length)

        self._sequence_number[channel] = (self._sequence_number[channel] + 1) % 256

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
        self._dbg_print_header()

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
        self._dbg_print_header()

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

    def _dbg_print_header(self):
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
    # data timestamp
    # DFU Support
    # Angular velocity
    # • Angular position (quaternion)
    # • Data returned at configurable sample rates
    # • Timestamps attached to sensor reports
    # • Low latency, 1kHz gyro rotation vector for AR/VR
    # Built-in stability detector, tap detector, and step counter
