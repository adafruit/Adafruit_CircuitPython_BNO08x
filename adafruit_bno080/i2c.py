# SPDX-FileCopyrightText: Copyright (c) 2020 Bryan Siepert for Adafruit Industries
#
# SPDX-License-Identifier: MIT
"""

    Subclass of `adafruit_bno080.BNO080` to use I2C

"""
from time import sleep
from struct import pack_into
import adafruit_bus_device.i2c_device as i2c_device
from . import BNO080, DATA_BUFFER_SIZE, const, Packet

# should be removeable; I _think_ something else should be able to prep the buffers?

_BNO080_DEFAULT_ADDRESS = const(0x4A)


class BNO080_I2C(BNO080):
    """Library for the BNO080 IMU from Hillcrest Laboratories

        :param ~busio.I2C i2c_bus: The I2C bus the BNO080 is connected to.

    """

    def __init__(self, i2c_bus, address=_BNO080_DEFAULT_ADDRESS, debug=False):
        self.bus_device_obj = i2c_device.I2CDevice(i2c_bus, address)
        super().__init__(debug)

    def _send_packet(self, channel, data):
        data_length = len(data)
        write_length = data_length + 4

        # struct.pack_into(fmt, buffer, offset, *values)
        pack_into("<H", self._data_buffer, 0, write_length)
        self._data_buffer[2] = channel

        self._data_buffer[3] = self._sequence_number[channel]

        # this is dumb but it's what we have for now
        for idx, send_byte in enumerate(data):
            self._data_buffer[4 + idx] = send_byte

        # self._print_header()

        with self.bus_device_obj as i2c:
            self._dbg("\twriting header and data at once")
            i2c.write(self._data_buffer, end=write_length)

        self._sequence_number[channel] = (self._sequence_number[channel] + 1) % 256

    # returns true if available data was read
    # the sensor will always tell us how much there is, so no need to track it ourselves

    def _read_packet(self):
        # TODO: FIZXME

        sleep(0.001)
        with self.bus_device_obj as i2c:
            i2c.readinto(self._data_buffer, end=4)  # this is expecting a header?
        # struct.unpack_from(fmt, data, offset=0)
        self._dbg("")
        self._dbg("READing packet")
        # self._print_header()

        # packet_byte_count, channel_number, sequence_number = self._get_header()
        header = Packet.header_from_buffer(self._data_buffer)
        packet_byte_count = header.packet_byte_count
        channel_number = header.channel_number
        sequence_number = header.sequence_number
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
        # self._print_header()

        header = Packet.header_from_buffer(self._data_buffer)
        _data_len = header.packet_byte_count
        channel = header.channel_number
        seq = header.sequence_number

        # _data_len, channel, seq = self._get_header()
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
        if total_read_length > DATA_BUFFER_SIZE:
            unread_bytes = total_read_length - DATA_BUFFER_SIZE
            total_read_length = DATA_BUFFER_SIZE
        self._dbg(
            "reading",
            total_read_length,
            "bytes(%d+4)" % requested_read_length,
            "leaving",
            unread_bytes,
            "unread bytes",
        )
        with self.bus_device_obj as i2c:
            i2c.readinto(self._data_buffer, end=total_read_length)

        return unread_bytes > 0
