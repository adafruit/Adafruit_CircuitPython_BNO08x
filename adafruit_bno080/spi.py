# SPDX-FileCopyrightText: Copyright (c) 2020 Bryan Siepert for Adafruit Industries
#
# SPDX-License-Identifier: MIT
"""

    Subclass of `adafruit_bno080.BNO080` to use SPI

"""
import time
from struct import pack_into

import board
from digitalio import Direction, Pull
import adafruit_bus_device.spi_device as spi_device

from . import BNO080, BNO_CHANNEL_EXE, DATA_BUFFER_SIZE, _elapsed, const, Packet

# should be removeable; I _think_ something else should be able to prep the buffers?

_BNO080_DEFAULT_ADDRESS = const(0x4A)


class BNO080_SPI(BNO080):
    """Instantiate a `adafruit_bno080.BNO080_SPI` instance to communicate with
    the sensor using SPI

    Args:
        spi_bus ([busio.SPI]): The SPI bus to use to communicate with the BNO080
        cs_pin ([digitalio.DigitalInOut]): The pin object to use for the SPI Chip Select
        debug (bool, optional): Enables print statements used for debugging. Defaults to False.
    """

    # """Library for the BNO080 IMU from Hillcrest Laboratories

    #     :param ~busio.SPI spi_bus: The SPI bus the BNO080 is connected to.

    # """

    def __init__(
        self, spi_bus, cspin, intpin, wakepin, resetpin, baudrate=100000, debug=False
    ):  # pylint:disable=too-many-arguments
        self._spi = spi_device.SPIDevice(
            spi_bus, cspin, baudrate=baudrate, polarity=1, phase=1
        )
        self._int = intpin
        self._wake = wakepin
        super().__init__(resetpin, debug)


    def hard_reset(self):
        """Hardware reset the sensor to an initial unconfigured state"""
        self._reset.direction = Direction.OUTPUT
        self._wake.direction = Direction.OUTPUT
        self._int.direction = Direction.INPUT
        self._int.pull = Pull.UP

        print("Hard resetting...")
        self._wake.value = True # set PS0 high (PS1 also must be tied high)
        self._reset.value = True # perform hardware reset
        time.sleep(0.01)
        self._reset.value = False
        time.sleep(0.01)
        self._reset.value = True
        self._wait_for_int()
        print("Done!")
        self._read_packet()

    def _wait_for_int(self):
        print("Waiting for INT...", end="")
        start_time = time.monotonic()
        while _elapsed(start_time) < 1.0:
            if not self._int.value:
                break
        else:
            raise RuntimeError("Could not wake up")
        print("OK")

    def soft_reset(self):
        """Reset the sensor to an initial unconfigured state"""
        print("Soft resetting...", end="")
        data = bytearray(1)
        data[0] = 1
        seq = self._send_packet(BNO_CHANNEL_EXE, data)
        time.sleep(0.5)
        seq = self._send_packet(BNO_CHANNEL_EXE, data)
        time.sleep(0.5)

        for i in range(3):
            try:
                packet = self._read_packet()
            except PacketError:
                time.sleep(0.5)
        print("OK!");
        # all is good!

    def _read_into(self, buf, start=0, end=None):
        self._wait_for_int()
        
        with self._spi as spi:
            spi.readinto(buf, start=start, end=end)
        print("SPI Read buffer: ", [hex(i) for i in buf[start:end]])

    def _read_header(self):
        """Reads the first 4 bytes available as a header"""
        self._wait_for_int()

        # read header
        with self._spi as spi:
            spi.readinto(self._data_buffer, end=4)
        self._dbg("")
        print("SHTP READ packet header: ", [hex(x) for x in self._data_buffer[0:4]])

    def _read_packet(self):
        self._read_header()
        
        #print([hex(x) for x in self._data_buffer[0:4]])
        header = Packet.header_from_buffer(self._data_buffer)
        packet_byte_count = header.packet_byte_count
        channel_number = header.channel_number
        sequence_number = header.sequence_number

        self._sequence_number[channel_number] = sequence_number
        if packet_byte_count == 0:
            raise PacketError("No packet available")

        self._dbg("channel %d has %d bytes available" % (channel_number, packet_byte_count-4))

        if packet_byte_count > DATA_BUFFER_SIZE:
            self._data_buffer = bytearray(packet_byte_count)

        # re-read header bytes since this is going to be a new transaction
        self._read_into(self._data_buffer, start=0, end=packet_byte_count)
        #print("Packet: ", [hex(i) for i in self._data_buffer[0:packet_byte_count]])
        
        new_packet = Packet(self._data_buffer)
        if self._debug:
            print(new_packet)
        self._update_sequence_number(new_packet)
        return new_packet

    ###### Actually send bytes ##########
    # Note: I _think_ these can be used for either SPI or I2C in the base class by having the
    # subclass  set the 'bus_device_object' to a `bus_device`.SPI` or `bus_device.I2C`

    # returns true if all requested data was read
    def _read(self, requested_read_length):
        self._dbg("trying to read", requested_read_length, "bytes")
        unread_bytes = 0
        # +4 for the header
        total_read_length = requested_read_length + 4
        if total_read_length > DATA_BUFFER_SIZE:
            unread_bytes = total_read_length - DATA_BUFFER_SIZE
            total_read_length = DATA_BUFFER_SIZE

        with self.bus_device_obj as spi:
            spi.readinto(self._data_buffer, end=total_read_length)
        return unread_bytes > 0

    def _send_packet(self, channel, data):
        data_length = len(data)
        write_length = data_length + 4

        # struct.pack_into(fmt, buffer, offset, *values)
        pack_into("<H", self._data_buffer, 0, write_length)
        self._data_buffer[2] = channel
        self._data_buffer[3] = self._sequence_number[channel]
        for idx, send_byte in enumerate(data):
            self._data_buffer[4 + idx] = send_byte

        self._wait_for_int()
        with self._spi as spi:
            spi.write(self._data_buffer, end=write_length)
        self._sequence_number[channel] = (self._sequence_number[channel] + 1) % 256
        return self._sequence_number[channel]

    @property
    def _data_ready(self):
        time.sleep(0.1)
        try:
            self._wait_for_int()
            return True
        except RuntimeError:
            return False
