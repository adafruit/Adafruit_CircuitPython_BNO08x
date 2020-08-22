# SPDX-FileCopyrightText: Copyright (c) 2020 Bryan Siepert for Adafruit Industries
#
# SPDX-License-Identifier: MIT
"""

    Subclass of `adafruit_bno080.BNO080` to use SPI

"""
from time import sleep
from struct import pack_into

import board
from digitalio import DigitalInOut, Direction
import adafruit_bus_device.spi_device as spi_device

from . import BNO080, DATA_BUFFER_SIZE, const, Packet

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
    #     :param ~busio.SPI spi_bus: The SPI bus the BNO080 is connected to.

    # """

    def __init__(
        self, spi_bus, cs_pin_obj, int_pin_obj, baudrate=100000, debug=False
    ):  # pylint:disable=too-many-arguments
        self.bus_device_obj = spi_device.SPIDevice(
            spi_bus, cs_pin_obj, baudrate=baudrate, polarity=1, phase=1
        )
        self._int_pin = int_pin_obj

        self._int_check_pin = DigitalInOut(board.D9)
        self._int_check_pin.direction = Direction.OUTPUT
        self._int_check_pin.value = False

        self._read_pin = DigitalInOut(board.D10)
        self._read_pin.direction = Direction.OUTPUT
        self._read_pin.value = False

        self._write_pin = DigitalInOut(board.D11)
        self._write_pin.direction = Direction.OUTPUT
        self._write_pin.value = False

        super().__init__(debug)

    def reset(self):
        self._dbg("Sleeping")
        sleep(2.050)

        self._dbg("**** waiting for advertising packet **")
        # read and disregard first initial advertising packet
        # The BNO08X uses advertisements to publish the channel maps and the names
        # of the built-in applications.
        _advertising_packet = self._wait_for_packet()
        # read and disregard init completed packet
        self._dbg("**** received packet; waiting for init complete packet **")
        _init_complete_packet = self._wait_for_packet()

    # I think this and `_send_packet` should take a `Packet`
    def _read_packet(self):
        self._read_pin.value = True

        header = Packet.header_from_buffer(self._data_buffer)

        if header.data_length == 0:
            self._read_pin.value = False

            raise RuntimeError("Zero bytes ready")

        read_length = header.data_length
        self._read(read_length)

        # the read has filled the data buffer with as much as possible, so we can now
        # render a header from that data to see what was received
        response_header = Packet.header_from_buffer(self._data_buffer)

        self._dbg(
            "Done reading! Data requested:",
            read_length,
            "Data read:",
            response_header.data_length,
        )

        # update the cached sequence number so we know what to increment from
        # TODO: this is wrong; there should be one per channel per direction
        self._sequence_number[header.channel_number] = header.sequence_number
        self._read_pin.value = False

        return True

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
        self._print_buffer()
        return unread_bytes > 0

    def _send_packet(self, channel, data):
        self._write_pin.value = True
        self._dbg("Sending packet to channel", channel)
        data_length = len(data)
        write_length = data_length + 4

        # struct.pack_into(fmt, buffer, offset, *values)
        pack_into("<H", self._data_buffer, 0, write_length)
        self._data_buffer[2] = channel

        # TODO: this is probably wrong; there should be one per channel per direction
        self._data_buffer[3] = self._sequence_number[channel]

        # this is dumb but it's what we have for now
        for idx, send_byte in enumerate(data):
            self._data_buffer[4 + idx] = send_byte

        # header = Packet.header_from_buffer(self._data_buffer)

        with self.bus_device_obj as spi:
            spi.write(self._data_buffer, end=write_length)
        self._dbg("Sent:")
        self._print_buffer()
        # TODO: this is wrong; there should be one per channel per direction
        self._sequence_number[channel] = (self._sequence_number[channel] + 1) % 256

        self._write_pin.value = False

    def _data_ready(self):
        self._int_check_pin.value = True
        int_value = self._int_pin.value

        self._int_check_pin.value = False
        self._int_check_pin.value = True

        if int_value:
            self._int_check_pin.value = False
            return False
        header = self._read_header()
        if Packet.is_error(header):
            print("ERROR packet")
            self._int_check_pin.value = False
            return False
        if header.data_length == 0:
            print("EMPTY packet")
            self._int_check_pin.value = False
            return False

        self._int_check_pin.value = False
        return True
