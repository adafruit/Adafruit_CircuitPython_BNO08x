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

_DATA_BUFFER_SIZE = const(128) # data buffer size. obviously eats ram
_I2C_BUFFER_SIZE = const(32) # imaginary i2c buffer size. I don't believe this is nescessary but we'll use it for now to stay as close as possible to the reference code

class BNO080:
    """Library for the BNO080 IMU from Hillcrest Laboratories


        :param ~busio.I2C i2c_bus: The I2C bus the BNO080 is connected to.

    """

    def __init__(self, i2c_bus, address=_BNO080_DEFAULT_ADDRESS):

        self.i2c_device = i2c_device.I2CDevice(i2c_bus, address)
        self._header_buffer = bytearray(4)
        self._data_buffer = bytearray(_DATA_BUFFER_SIZE)
        self._sequence_number = [0,0,0,0,0,0]
        self.reset()

    def reset(self):
        self._data_buffer[0] = 1
        self._send_packet(_BNO080_CHANNEL_EXEC, 1)
        print("PACKET SENT")
        sleep(0.050)

        reads = 0
        read_complete = False
        while not read_complete:
            print("reading packet")
            data_read = self._read_packet()
            print("data read:", data_read)
            read_complete = (data_read == False)
            reads+=1
            if reads >3:
                read_complete = True

        sleep(0.050)
        read_complete = False
        reads = 0
        while not read_complete:
            print("reading packet")
            data_read = self._read_packet()
            print("data read:", data_read)
            read_complete = (data_read == False)
            reads+=1
            if reads >3:
                read_complete = True

    def _print_header(self):
        print("Header:")
        print("\tlen LSB:", self._header_buffer[0])
        print("\tlen MSB:", self._header_buffer[1])
        print("\tchannel:", self._header_buffer[2])
        print("\tseq num:", self._header_buffer[3])

    def _send_packet(self, channel, data_length):
        # struct.pack_into(fmt, buffer, offset, *values)
        pack_into("<H",self._header_buffer,  0, data_length+4)
        self._header_buffer[2] = channel
        self._header_buffer[3] = self._sequence_number[channel]

        print("writing header:")
        self._print_header()
        with self.i2c_device as i2c:
            print("writing header")
            i2c.write(self._header_buffer)
            print("Header written")
            
            i2c.write(self._data_buffer,  end=data_length)

        self._sequence_number[channel] += 1

        return

    # returns true if available data was read
    # the sensor will always tell us how much there is, so no need to track it ourselves
    def _read_packet(self):
        # TODO: FIZXME
        self._header_buffer = bytearray(4)
        with self.i2c_device as i2c:
            i2c.readinto(self._header_buffer)
        # struct.unpack_from(fmt, data, offset=0)
        print("header read:")
        self._print_header()
        packet_byte_count = unpack_from("<H", self._header_buffer)[0]

        packet_byte_count &= ~0x8000
        channel_number = unpack_from("<B", self._header_buffer, offset=2)[0]
        sequence_number = unpack_from("<B", self._header_buffer, offset=3)[0]

        # self._packet_byte_count = packet_byte_count
        # self._packet_channel = channel_number
        self._sequence_number[channel_number] = sequence_number

        if packet_byte_count == 0:
            return False
        # remove header size from read length
        packet_byte_count -= 4
        # TODO: exception handling
        data_remaining = self._read(packet_byte_count)
        print("\tbytes remaining:", data_remaining)
        return True

    # returns true if all requested data was read
    def _read(self, requested_read_length):
        print("trying to read", requested_read_length, "bytes")
        data_buffer_offset = 0 # to keep track of position in buffer over multiple reads
        current_read_len = 0
        unread_bytes = requested_read_length
        final_read = False

        while unread_bytes >0 or data_buffer_offset > _DATA_BUFFER_SIZE:
            # try to read all the bytes left unread
            # trim to the size of the i2c buffer - 4 for header
            current_read_len = unread_bytes
            if current_read_len > (_I2C_BUFFER_SIZE -4):
                current_read_len = _I2C_BUFFER_SIZE -4

            # if read would go past the end of the data buffer, trim the read to the available space
            if data_buffer_offset + current_read_len > _DATA_BUFFER_SIZE:
                current_read_len = _DATA_BUFFER_SIZE - data_buffer_offset


            print("reading", current_read_len, "bytes")
            with self.i2c_device as i2c:
                i2c.readinto(self._data_buffer, start=data_buffer_offset, end=data_buffer_offset + current_read_len)
            unread_bytes -=current_read_len
        # we've either read everything or we ran out of space
        return ( unread_bytes > 0)



























    #     """Reset the sensor to an initial unconfigured state"""
    #     self._buffer[0] = _BNO080_HUM_CMD_RESET
    #     with self.i2c_device as i2c:
    #         i2c.write(self._buffer, end=1)

    #     sleep(0.015)
    #     self._buffer[0] = _BNO080_PT_CMD_RESET
    #     with self.pressure_i2c_device as i2c:
    #         i2c.write(self._buffer, end=1)

    #     self._buffer[0] = _BNO080_PT_CALIB_ROM_ADDR + offset
    #     with self.pressure_i2c_device as i2c:
    #         i2c.write_then_readinto(
    #             self._buffer,
    #             self._buffer,
    #             out_start=0,
    #             out_end=1,
    #             in_start=0,
    #             in_end=2,
    #         )

    #     constants.extend(unpack_from(">H", self._buffer[0:2]))


    #     # temp is only 24 bits but unpack wants 4 bytes so add a forth byte
    #     self._buffer[0] = 0
    #     raw_temperature = unpack_from(">I", self._buffer)[0]

    #     # next read pressure
    #     cmd = self._psensor_resolution_osr * 2
    #     cmd |= _BNO080_PT_CMD_PRESS_START
    #     self._buffer[0] = cmd
    #     with self.pressure_i2c_device as i2c:
    #         i2c.write(self._buffer, end=1)

    #     sleep(integration_time)

    #     self._buffer[0] = _BNO080_PT_CMD_READ_ADC
    #     with self.pressure_i2c_device as i2c:
    #         i2c.write_then_readinto(
    #             self._buffer, self._buffer, out_start=0, out_end=1, in_start=1, in_end=3
    #         )
    #     # pressure is only 24 bits but unpack wants 4 bytes so add a forth byte
    #     self._buffer[0] = 0

    #     raw_pressure = unpack_from(">I", self._buffer)[0]
    #     return raw_temperature, raw_pressure

    #     with self.i2c_device as i2c:
    #         i2c.readinto(self._buffer, end=1)

    #     return self._buffer[0]

    # def _set_hum_user_register(self, register_value):
    #     self._buffer[0] = _BNO080_HUM_CMD_WRITE_USR
    #     self._buffer[1] = register_value
    #     with self.i2c_device as i2c:
    #         # shouldn't this end at two?
    #         i2c.write(self._buffer, end=2)
