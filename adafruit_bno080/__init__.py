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

# Channel 0: the SHTP command channel
_BNO_CHANNEL_SHTP_COMMAND = const(0)
BNO_CHANNEL_EXE = const(1)
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


# Reset reasons from ID Report reponse:
# 0 – Not Applicable
# 1 – Power On Reset
# 2 – Internal System Reset
# 3 – Watchdog Timeout
# 4 – External Reset
# 5 – Other

_QUAT_REPORT_INTERVAL = const(50000)  # in microseconds = 50ms
_QUAT_READ_TIMEOUT = 0.500  # timeout in seconds
_PACKET_READ_TIMEOUT = 15.000  # timeout in seconds
_BNO080_CMD_RESET = const(0x01)
_QUAT_Q_POINT = const(14)
_BNO_HEADER_LEN = const(4)

_Q_POINT_14_SCALAR = 2 ** (14 * -1)
_Q_POINT_12_SCALAR = 2 ** (12 * -1)


DATA_BUFFER_SIZE = const(512)  # data buffer size. obviously eats ram
Quaternion = namedtuple("Quaternion", ["i", "j", "k", "real",],)
PacketHeader = namedtuple(
    "PacketHeader",
    ["channel_number", "sequence_number", "data_length", "packet_byte_count",],
)

REPORT_STATUS = ["Unreliable", "Accuracy low", "Accuracy medium", "Accuracy high"]


def _elapsed(start_time):
    return monotonic() - start_time


def _parse_quat(packet):

    i_raw = unpack_from("<h", packet.data, offset=9)[0]
    quat_i = i_raw * _Q_POINT_14_SCALAR

    j_raw = unpack_from("<h", packet.data, offset=11)[0]
    quat_j = j_raw * _Q_POINT_14_SCALAR

    k_raw = unpack_from("<h", packet.data, offset=13)[0]
    quat_k = k_raw * _Q_POINT_14_SCALAR

    real_raw = unpack_from("<h", packet.data, offset=15)[0]
    quat_real = real_raw * _Q_POINT_14_SCALAR

    _acc_est = unpack_from("<h", packet.data, offset=17)[0] * _Q_POINT_12_SCALAR

    _status_int = unpack_from("<B", packet.data, offset=7)[0]
    _status_int &= 0x03

    return Quaternion(quat_i, quat_j, quat_k, quat_real)


class Packet:
    """A class representing a Hillcrest LaboratorySensor Hub Transport packet"""

    def __init__(self, packet_bytes):
        self.header = self.header_from_buffer(packet_bytes)
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

    @property
    def report_id(self):
        """The Packet's Report ID"""
        return self.data[0]

    @property
    def channel_number(self):
        """The packet channel"""
        return self.header.channel_number

    @classmethod
    def header_from_buffer(cls, packet_bytes):
        """Creates a `PacketHeader` object from a given buffer"""
        packet_byte_count = unpack_from("<H", packet_bytes)[0]
        packet_byte_count &= ~0x8000
        channel_number = unpack_from("<B", packet_bytes, offset=2)[0]
        sequence_number = unpack_from("<B", packet_bytes, offset=3)[0]
        data_length = max(0, packet_byte_count - 4)

        header = PacketHeader(
            channel_number, sequence_number, data_length, packet_byte_count
        )
        return header

    @classmethod
    def is_error(cls, header):
        """Returns True if the header is an error condition"""

        if header.channel_number > 5:
            return True
        if header.packet_byte_count == 0xFFFF and header.sequence_number == 0xFF:
            return True
        return False


class BNO080:
    """Library for the BNO080 IMU from Hillcrest Laboratories

        :param ~busio.I2C i2c_bus: The I2C bus the BNO080 is connected to.

    """

    def __init__(self, debug=False):
        self._debug = debug
        self._dbg("********** __init__ *************")
        self._data_buffer = bytearray(DATA_BUFFER_SIZE)
        # TODO: this is wrong; there should be one per channel per direction

        self._sequence_number = [0, 0, 0, 0, 0, 0]
        # self._sequence_number = {"in": [0, 0, 0, 0, 0, 0], "out": [0, 0, 0, 0, 0, 0]}
        # se;f
        self._wait_for_initialize = True
        self._init_complete = False
        self._id_read = False
        self.initialize()

    def initialize(self):
        """Initialize the sensor"""
        self.reset()
        if not self._check_id():
            raise RuntimeError("Could not read ID")
        self._enable_quaternion()

    @property
    def quaternion(self):
        """A quaternion representing the current rotation vector"""
        # receive packets, and dump until you get a quat packet
        while True:  # add timeout
            new_packet = self._wait_for_packet_type(_BNO_CHANNEL_INPUT_SENSOR_REPORTS)
            print("Got packet for channel", new_packet.header.channel_number)
            # if new_packet.channel_number != _BNO_CHANNEL_INPUT_SENSOR_REPORTS:
            #     continue

            # print("New packet Report ID", hex(new_packet.report_id))
            # if new_packet.report_id != _BNO_REPORT_ROTATION_VECTOR:
            #     sleep(0.001)
            #     continue
            print("New packet thing", new_packet.data[5])
            if new_packet.data[5] != _BNO_REPORT_ROTATION_VECTOR:
                sleep(0.001)
                continue

            return _parse_quat(new_packet)

    def _wait_for_packet_type(self, channel_number, report_id=None, timeout=5.0):
        print(
            "** WAITing for packet on channel",
            channel_number,
            "with report id",
            report_id,
        )
        start_time = monotonic()
        while _elapsed(start_time) < timeout:
            new_packet = self._wait_for_packet()
            print("Got packet for channel", new_packet.header.channel_number)
            print("New packet Report ID", hex(new_packet.report_id))
            if new_packet.channel_number == channel_number:
                print("\tchannel matches")
                if report_id:
                    if new_packet.report_id == report_id:
                        print("\treport ID matches")
                        return new_packet
                    print("\treport ID *DOES NOT* match")
                else:
                    return new_packet
            self._handle_packet(new_packet)

        raise RuntimeError("Timed out waiting for a packet on channel", channel_number)

    def _handle_packet(self, packet):
        # advertisement; match on channel+seq, len
        # DBG::[  0] 0x14 0x81 0x00 0x01
        # DBG::[  4] 0x00 0x01 0x04 0x00

        if packet.channel_number == _BNO_CHANNEL_SHTP_COMMAND:
            if packet.header.data_length == 272:
                print("Got 272 len packet on channel 0")
                self._wait_for_initialize = True
                self._init_complete = False
                self._id_read = False

        # ch 1 command 1 = reset
        # DBG::[  0] 0x05 0x80 0x01 0x01
        # DBG::[  4] 0x01
        if packet.channel_number == BNO_CHANNEL_EXE:
            if packet.data[0] == 1:
                print("********** Found reset packet! ************")
                self._init_complete = False
                print("...sleeping")
                sleep(1)
                print("reinitializing")
                self.initialize()

        # 0xF1 == command response; Command is 0x84? - unsolicited initialize
        # DBG::[  0] 0x14 0x80 0x02 0x01
        # DBG::[  4] 0xF1 0x00 0x84 0x00
        # DBG::[  8] 0x00 0x00 0x01 0x00
        # DBG::[ 12] 0x00 0x00 0x00 0x00
        # DBG::[ 16] 0x00 0x00 0x00 0x00
        if packet.channel_number == _BNO_CHANNEL_CONTROL:
            if packet.report_id == _BNO_CMD_COMMAND_RESPONSE:
                print("Got command response")
                if packet.data[2] == 0x84:
                    print("Got unsolicited init response")
                    if self._wait_for_initialize:
                        self._wait_for_initialize = False
                        self._init_complete = True

                    else:
                        raise RuntimeError(
                            "Unsolicted init received before Advertisement"
                        )

        # Ch 2 ID Response
        # DBG::[  0] 0x14 0x80 0x02 0x03
        # DBG::[  4] 0xF8 0x03 0x03 0x02
        # DBG::[  8] 0x98 0xA4 0x98 0x00
        # DBG::[ 12] 0x72 0x01 0x00 0x00
        # DBG::[ 16] 0x07 0x00 0x00 0x00

        # Ch 3, ReportID: 0xFB timestamp
        # DBG::[  0] 0x17 0x80 0x03 0x01
        # DBG::[  4] 0xFB 0x17 0x00 0x00
        # DBG::[  8] 0x00 0x05 0x00 0x00
        # DBG::[ 12] 0x00 0x3D 0xFD 0xF4
        # DBG::[ 16] 0xFD 0xAB 0x07 0x72
        # DBG::[ 20] 0x3F 0x44 0x32

    def _wait_for_packet(self, timeout=_PACKET_READ_TIMEOUT):
        start_time = monotonic()
        while _elapsed(start_time) < timeout:
            if not self._data_ready():
                continue

            self._dbg("\npacket ready; reading")
            self._read_packet()
            new_packet = Packet(self._data_buffer)
            return new_packet
        raise RuntimeError("Timed out waiting for a packet")

    # Constructs a report  to set a feature
    # later: class-ify
    # def _set_feature_report(self, feature_id):
    def _enable_quaternion(self):
        print("\n********** ENABLE QUATERNIONS **********")
        set_feature_report = bytearray(17)
        set_feature_report[0] = _BNO_CMD_SET_FEATURE_COMMAND
        set_feature_report[1] = _BNO_REPORT_ROTATION_VECTOR
        pack_into("<I", set_feature_report, 5, _QUAT_REPORT_INTERVAL)

        self._send_packet(_BNO_CHANNEL_CONTROL, set_feature_report)
        # There is a substantial delay until data is available
        sleep(0.2)
        while True:
            packet = self._wait_for_packet_type(
                _BNO_CHANNEL_CONTROL, _BNO_CMD_GET_FEATURE_RESPONSE
            )
            if packet.channel_number == _BNO_CHANNEL_CONTROL:
                print("got channel 2")
                report_id = packet.report_id
                print("Report ID:", report_id)
                if report_id == _BNO_CMD_GET_FEATURE_RESPONSE:
                    print("Done!")
                    return True

    def _check_id(self):

        print("\n********** READ ID **********")
        if self._id_read:
            return True
        data = bytearray(2)
        data[0] = _SHTP_REPORT_PRODUCT_ID_REQUEST
        data[1] = 0  # padding
        self._dbg("\n** Sending ID Request Report **")
        self._send_packet(_BNO_CHANNEL_CONTROL, data)
        self._dbg("\n** Waiting for packet **")
        # _a_ packet arrived, but which one?
        while True:
            self._wait_for_packet_type(
                _BNO_CHANNEL_CONTROL, _SHTP_REPORT_PRODUCT_ID_RESPONSE
            )
            sensor_id = self._parse_sensor_id()
            if sensor_id:
                self._id_read = True
                return True
            print("Packet didn't have sensor ID report, trying again")

        return False

    def _parse_sensor_id(self):
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

    def _dbg(self, *args, **kwargs):
        if self._debug:
            print("\t\tDBG::\t\t", *args, **kwargs)

    def _get_data(self, index, fmt_string):
        # index arg is not including header, so add 4 into data buffer
        data_index = index + 4
        return unpack_from(fmt_string, self._data_buffer, offset=data_index)[0]

    def _read_header(self):
        """Reads the first 4 bytes available as a header"""
        with self.bus_device_obj as bus_dev:  # pylint:disable=no-member
            bus_dev.readinto(self._data_buffer, end=4)
        packet_header = Packet.header_from_buffer(self._data_buffer)
        self._dbg(packet_header)
        return packet_header

    def _print_buffer(self, write_full=False):
        header = Packet.header_from_buffer(self._data_buffer)
        length = header.packet_byte_count
        if write_full:
            print(" writing complete buffer")
            length = len(self._data_buffer)

        for idx, packet_byte in enumerate(self._data_buffer[:length]):
            if (idx % 4) == 0:
                print("\n\t\tDBG::[%3d] " % idx, end="")
            print("0x{:02X} ".format(packet_byte), end="")
        print("")

    # pylint:disable=no-self-use
    def _data_ready(self):
        raise RuntimeError("Not implemented")

    def reset(self):
        """Reset the sensor to an initial unconfigured state"""
        raise RuntimeError("Not implemented")

    def _send_packet(self, channel, data):
        raise RuntimeError("Not implemented")

    def _read_packet(self):
        raise RuntimeError("Not implemented")

    def _send_packet(self, channel, data):
        raise RuntimeError("Not implemented")
