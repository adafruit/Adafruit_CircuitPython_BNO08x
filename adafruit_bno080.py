# SPDX-FileCopyrightText: 2017 Scott Shawcroft, written for Adafruit Industries
# SPDX-FileCopyrightText: Copyright (c) 2020 Bryan Siepert for Adafruit Industries
#
# SPDX-License-Identifier: MIT
"""
`adafruit_bno080`
================================================================================

CircuitPython driver for the BNO080 PTH sensor


* Author(s): Bryan Siepert

Implementation Notes
--------------------

**Hardware:**

* `Adafruit AS7341 Breakout <https:#www.adafruit.com/products/4716>`_

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https:#github.com/adafruit/circuitpython/releases

 * Adafruit's Bus Device library: https:#github.com/adafruit/Adafruit_CircuitPython_BusDevice
 * Adafruit's Register library: https:#github.com/adafruit/Adafruit_CircuitPython_Register

"""

__version__ = "0.0.0-auto.0"
__repo__ = "https:#github.com/adafruit/Adafruit_CircuitPython_BNO080.git"


from struct import unpack_from, pack_into
from time import sleep
from micropython import const
import adafruit_bus_device.i2c_device as i2c_device


_BNO080_DEFAULT_ADDRESS = const(0x4A)
_BNO080_RESET_CMD = const(0x01)
_BNO080_CHANNEL_EXEC = const(0x01)
MAX_READS=10
_DATA_BUFFER_SIZE = const(512) # data buffer size. obviously eats ram
_I2C_BUFFER_SIZE = const(32) # imaginary i2c buffer size. I don't believe this is nescessary but we'll use it for now to stay as close as possible to the reference code
_HEADER_SIZE = const(4)
_MAX_DATA_READ_LENGTH = _I2C_BUFFER_SIZE-_HEADER_SIZE
class BNO080:
    """Library for the BNO080 IMU from Hillcrest Laboratories


        :param ~busio.I2C i2c_bus: The I2C bus the BNO080 is connected to.

    """

    def __init__(self, i2c_bus, address=_BNO080_DEFAULT_ADDRESS, debug=True):
        self._debug=debug
        self.i2c_device = i2c_device.I2CDevice(i2c_bus, address)
        self._data_buffer = bytearray(_DATA_BUFFER_SIZE)
        self._sequence_number = [0,0,0,0,0,0]
        self.reset()

    def reset(self):
        data = bytearray(1)
        data[0] = 1
        print("Sending reset packet")
        self._send_packet(_BNO080_CHANNEL_EXEC, data)
        self._dbg("PACKET SENT")
        sleep(0.050)

        reads = 0
        read_complete = False
        sleep(1)
        while not read_complete:
            print("Reading packet")
            data_read = self._read_packet()
            self._dbg("data read:", data_read)
            read_complete = (data_read == False)
            reads+=1
            if reads >MAX_READS:
                read_complete = True

        sleep(0.050)
        read_complete = False
        reads = 0
        while not read_complete:
            print("Still reading packet")
            data_read = self._read_packet()
            self._dbg("data read:", data_read)
            read_complete = (data_read == False)
            reads+=1
            if reads >2:
                read_complete = True

    def _print_header(self):
        self._dbg("Header:")
        self._dbg("\tlen LSB:", hex(self._data_buffer[0]))
        self._dbg("\tlen MSB:", hex(self._data_buffer[1]))
        self._dbg("\tchannel:", self._data_buffer[2])
        self._dbg("\tseq num:", self._data_buffer[3])

    def _send_packet(self, channel, data):
        self._dbg("")
        self._dbg("SENDing packet")
        data_length = len(data)
        write_length = data_length+4
        self._dbg("\tChannel:", channel)
        self._dbg("\tData length:", data_length)
        # struct.pack_into(fmt, buffer, offset, *values)
        pack_into("<H", self._data_buffer,  0, write_length)
        self._data_buffer[2] = channel
        self._data_buffer[3] = self._sequence_number[channel]

        # this is dumb but it's what we have for now
        for idx, send_byte in enumerate(data):
            self._data_buffer[4+idx] = send_byte

        self._dbg("\tSend header:")
        self._print_header()
        with self.i2c_device as i2c:
            self._dbg("\twriting header and data at once")
            i2c.write(self._data_buffer, end=write_length)

        self._sequence_number[channel] += 1

        return

    # returns true if available data was read
    # the sensor will always tell us how much there is, so no need to track it ourselves
    def _read_packet(self):
        # TODO: FIZXME

        sleep(0.001)
        with self.i2c_device as i2c:
            i2c.readinto(self._data_buffer, end=4) # this is expecting a header?
        # struct.unpack_from(fmt, data, offset=0)
        self._dbg("")
        self._dbg("READing packet")
        self._print_header()
        packet_byte_count = unpack_from("<H", self._data_buffer)[0]
        packet_byte_count &= ~0x8000
        channel_number = unpack_from("<B", self._data_buffer, offset=2)[0]
        sequence_number = unpack_from("<B", self._data_buffer, offset=3)[0]

        # self._packet_byte_count = packet_byte_count
        # self._packet_channel = channel_number
        self._sequence_number[channel_number] = sequence_number

        if packet_byte_count == 0:
            return False
        # remove header size from read length
        packet_byte_count -= 4
        self._dbg("channel", channel_number, "has", packet_byte_count, "bytes available to read")
        # TODO: exception handling
        data_remaining = self._read(packet_byte_count)
        if data_remaining:
            self._dbg("Unread data still for channel", channel_number)
      
        return True

    # returns true if all requested data was read
    def _read(self, requested_read_length):
        self._dbg("trying to read", requested_read_length, "bytes")
        unread_bytes = 0
        # +4 for the header
        total_read_length = requested_read_length+4
        if total_read_length > _DATA_BUFFER_SIZE:
            unread_bytes = total_read_length-_DATA_BUFFER_SIZE
            total_read_length = _DATA_BUFFER_SIZE
        self._dbg("reading", total_read_length, "bytes(%d+4)"%requested_read_length, "leaving", unread_bytes, "unread bytes")
        with self.i2c_device as i2c:
            i2c.readinto(self._data_buffer, end=total_read_length)

        return ( unread_bytes > 0)

    def _dbg(self, *args):
        if self._debug:
            print("\tDBG::", *args)
