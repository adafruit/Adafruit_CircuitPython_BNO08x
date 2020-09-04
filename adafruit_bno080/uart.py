# SPDX-FileCopyrightText: Copyright (c) 2020 Bryan Siepert for Adafruit Industries
#
# SPDX-License-Identifier: MIT
"""

    Subclass of `adafruit_bno080.BNO080` to use UART

"""
import time
from struct import pack_into
from . import BNO080, BNO_CHANNEL_EXE, DATA_BUFFER_SIZE, const, Packet, PacketError


class BNO080_UART(BNO080):
    """Library for the BNO080 IMU from Hillcrest Laboratories

        :param uart: The UART devce the BNO080 is connected to.

    """

    def __init__(self, uart, reset=None, debug=False):
        self._uart = uart
        super().__init__(reset, debug)

    def _send_packet(self, channel, data):
        return 0

    def _read_into(self, buf, num):
        print("Avail:", self._uart.in_waiting, "need", num)
        for idx in range(num):
            data = self._uart.read(1)
            b = data[0]
            if b == 0x7d: # control escape
                data = self._uart.read(1)
                b = data[0]
                b ^=0x20
            buf[idx] = b
        #print("UART Read buffer: ", [hex(i) for i in buf[0:num]])
            
    def _read_packet(self):
        # try to read initial packet start byte
        while True:
            data = self._uart.read(1)
            if not data:
                continue
            b = data[0]
            if b == 0x7e:
                break

        # read protocol id
        data = self._uart.read(1)
        if data and data[0] == 0x7e:   # second 0x7e
            data = self._uart.read(1)
        if not data or data[0] != 0x01:
            raise RuntimeError("Unhandled UART control SHTP protocol")

        # read header
        self._read_into(self._data_buffer, 4)

        print("SHTP Header:", [hex(x) for x in self._data_buffer[0:4]])

        header = Packet.header_from_buffer(self._data_buffer)
        packet_byte_count = header.packet_byte_count
        channel_number = header.channel_number
        sequence_number = header.sequence_number

        self._sequence_number[channel_number] = sequence_number
        if packet_byte_count == 0:
            self._dbg("SKIPPING NO PACKETS AVAILABLE IN i2c._read_packet")
            raise PacketError("No packet available")
        packet_byte_count -= 4
        self._dbg(
            "channel",
            channel_number,
            "has",
            packet_byte_count,
            "bytes available to read",
        )

        self._read(packet_byte_count)

        # TODO: Allocation
        new_packet = Packet(self._data_buffer)
        if self._debug:
            print(new_packet)

        self._update_sequence_number(new_packet)

        return new_packet
        
        #if data is not None:
        print([hex(x) for x in data])

    # returns true if all requested data was read
    def _read(self, requested_read_length):
        self._dbg("trying to read", requested_read_length, "bytes")
        # +4 for the header
        total_read_length = requested_read_length + 4
        if total_read_length > DATA_BUFFER_SIZE:
            self._data_buffer = bytearray(total_read_length)
            self._dbg(
                "!!!!!!!!!!!! ALLOCATION: increased _data_buffer to bytearray(%d) !!!!!!!!!!!!! "
                % total_read_length
            )
        self._read_into(self._data_buffer, total_read_length)
