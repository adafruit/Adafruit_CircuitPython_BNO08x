# SPDX-FileCopyrightText: Copyright (c) 2020 Bryan Siepert for Adafruit Industries
#
# SPDX-License-Identifier: MIT
"""

    Subclass of `adafruit_bno080.BNO080` to use SPI

"""
from time import sleep
from struct import pack_into

import adafruit_bus_device.spi_device as spi_device

# for when we start poking pins
# import board
# from busio import SPI
# from digitalio import DigitalInOut, Pull, Direction
from . import BNO080, DATA_BUFFER_SIZE, const

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

    def __init__(self, spi_bus, cs_pin_obj, baudrate=100000, debug=False):
        self.spi_device = spi_device.SPIDevice(spi_bus, cs_pin_obj, baudrate=baudrate)
        super().__init__(debug)

    def reset(self):
        print("REEEESET")
        message_received = True
        while message_received:
            message_received = self._read_packet()
        while message_received:
            message_received = self._read_packet()

    def _read_packet(self):
        # TODO: FIZXME

        sleep(0.001)
        with self.spi_device as spi:
            spi.readinto(self._data_buffer, end=4)  # this is expecting a header?
        # struct.unpack_from(fmt, data, offset=0)
        self._dbg("")
        self._dbg("READing packet")
        self._dbg_print_header()

        packet_byte_count, channel_number, sequence_number = self._get_header()
        _data_len, channel, seq = self._get_header()
        if channel == 0xFF and seq == 0xFF:
            print("error, maybe?")
            return False
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
        self._dbg("Done reading! Data read:")
        for idx, packet_byte in enumerate(self._data_buffer[:packet_byte_count]):
            if (idx % 4) == 0:
                print("\n[%3d] " % idx, end="")
            print("0x{:02X} ".format(packet_byte), end="")
        print("")

        self._dbg_print_header()

        _data_len, channel, seq = self._get_header()
        if channel == 0xFF and seq == 0xFF:
            print("error, maybe?")
            return False
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
        with self.spi_device as spi:
            spi.readinto(self._data_buffer, end=total_read_length)

        return unread_bytes > 0

    def _send_packet(self, channel, data):
        self._dbg("Sending packet to channel", channel)
        data_length = len(data)
        write_length = data_length + 4

        # struct.pack_into(fmt, buffer, offset, *values)
        pack_into("<H", self._data_buffer, 0, write_length)
        self._data_buffer[2] = channel

        self._data_buffer[3] = self._sequence_number[channel]

        # this is dumb but it's what we have for now
        for idx, send_byte in enumerate(data):
            self._data_buffer[4 + idx] = send_byte

        self._dbg_print_header()

        with self.spi_device as spi:
            self._dbg("\twriting header and data at once")
            spi.write(self._data_buffer, end=write_length)

        self._sequence_number[channel] = (self._sequence_number[channel] + 1) % 256

    # def __init__(self, wake_pin, reset_pin,
    # int_pin,
    # sck_pin=board.SCK,
    # miso_pin=board.MISO,
    # mosi_pin=board.MOSI,
    # cs_pin=board.D5,
    # debug=False):
    #     """Instantiate a `adafruit_bno080.BNO080_SPI`
    # instance to communicate with the sensor using SPI

    #     Args:
    #         wake_pin ([type]): [description]
    #         reset_pin ([type]): [description]
    #         int_pin ([type]): [description]
    #         sck_pin ([type], optional): [description]. Defaults to board.SCK.
    #         miso_pin ([type], optional): [description]. Defaults to board.MISO.
    #         mosi_pin ([type], optional): [description]. Defaults to board.MOSI.
    #         cs_pin ([type], optional): [description]. Defaults to board.D5.
    #         debug (bool, optional): [description]. Defaults to False.
    #     """

    #     wake_pin_obj = DigitialInOut(wake_pin)
    #     wake_pin_obj.direction = direction.OUTPUT
    #     reset_pin_obj = DigitialInOut(reset_pin)
    #     reset_pin_obj.direction = direction.OUTPUT
    #     int_pin_obj = DigitialInOut(int_pin)
    #     int_pin_obj.direction = direction.INPUT
    #     int_pin_obj.pull = Pull.UP

    #     cs_pin_obj = DigitialInOut(cs_pin)
    #     cs_pin_obj.direction = direction.OUTPUT

    #     # Deselect BNO080
    #     cs_pin_obj.value = True
    #     # Before boot up the PS0/WAK pin must be high to enter SPI mode
    #     wake_pin_obj.value = True

    #     # reset BNO080
    #     reset_pin_obj.value = False
    #     # trying 10ms as a starting point
    #     sleep(0.010)
    #     reset_pin_obj.value = True
    #     # we'll want a timeout here;
    #     # alternatively block for enough time for the device to boot,
    #     # rather than polling the INT pin
    #     while not int_pin_obj.value:
    #         print("Checking INT pin for boot completion")
    #         sleep(0.010)

    #     try:
    #         spi_bus = busio.SPI(sck_pin, MOSI=mosi_pin, MISO=miso_pin)
    #     except RuntimeError as runtime_err:
    #         print("Error creating SPI bus for BNO080 after configuring the sensor for SPI")
    #         raise runtime_err

    #     self.spi_device = spi_device.SPIDevice(spi_bus, address)
    #     print("'Succsessfully' created a SPI bus device:", self.spi_device)

    #     super().__init__(debug)
