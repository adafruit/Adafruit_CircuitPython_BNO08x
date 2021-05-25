# SPDX-FileCopyrightText: Copyright (c) 2020 Bryan Siepert for Adafruit Industries
#
# SPDX-License-Identifier: MIT
"""
Subclass of `adafruit_bno08x.BNO08X` to use I2C
===============================================
"""
from struct import pack_into
import adafruit_bus_device.i2c_device as i2c_device
from . import BNO08X, DATA_BUFFER_SIZE, const, Packet, PacketError

_BNO08X_DEFAULT_ADDRESS = const(0x4A)


class BNO08X_I2C(BNO08X):
    """Library for the BNO08x IMUs from Hillcrest Laboratories

    :param ~busio.I2C i2c_bus: The I2C bus the BNO08x is connected to.
    :param ~digitalio.DigitalInOut reset: Optional for I2C use. Connected to the RST pin;
     used to hard-reset the device.
    :param int address: The I2C device address. Defaults to :const:`0x4A`
    :param bool debug: Enables print statements used for debugging. Defaults to `False`


    **Quickstart: Importing and using the device**

        Here is an example of using the :class:`BNO08X_I2C` class.
        First you will need to import the libraries to use the sensor

        .. code-block:: python

            import board
            from adafruit_bno08x.i2c import BNO08X_I2C

        Once this is done you can define your `board.I2C` object and define your sensor object

        .. code-block:: python

            # The sensor can communicate over I2C at 400kHz if you need the higher speed.
            i2c = board.I2C()  # uses board.SCL and board.SDA
            bno = BNO08X_I2C(i2c)

        For this particular you need to define some things to get some data.

        .. code-block:: python

            bno.enable_feature(adafruit_bno08x.BNO_REPORT_ACCELEROMETER)
            bno.enable_feature(adafruit_bno08x.BNO_REPORT_GYROSCOPE)
            bno.enable_feature(adafruit_bno08x.BNO_REPORT_MAGNETOMETER)
            bno.enable_feature(adafruit_bno08x.BNO_REPORT_ROTATION_VECTOR)

        Now you have access to the :attr:`acceleration`, :attr:`gyro`
        :attr:`magnetic` and :attr:`quaternion` attributes

        .. code-block:: python

            accel_x, accel_y, accel_z = bno.acceleration
            gyro_x, gyro_y, gyro_z = bno.gyro
            mag_x, mag_y, mag_z = bno.magnetic
            quat_i, quat_j, quat_k, quat_real = bno.quaternion

    """

    def __init__(
        self, i2c_bus, reset=None, address=_BNO08X_DEFAULT_ADDRESS, debug=False
    ):
        self.bus_device_obj = i2c_device.I2CDevice(i2c_bus, address)
        super().__init__(reset, debug)

    def _send_packet(self, channel, data):
        data_length = len(data)
        write_length = data_length + 4

        pack_into("<H", self._data_buffer, 0, write_length)
        self._data_buffer[2] = channel
        self._data_buffer[3] = self._sequence_number[channel]
        for idx, send_byte in enumerate(data):
            self._data_buffer[4 + idx] = send_byte
        packet = Packet(self._data_buffer)
        self._dbg("Sending packet:")
        self._dbg(packet)
        with self.bus_device_obj as i2c:
            i2c.write(self._data_buffer, end=write_length)

        self._sequence_number[channel] = (self._sequence_number[channel] + 1) % 256
        return self._sequence_number[channel]

    # returns true if available data was read
    # the sensor will always tell us how much there is, so no need to track it ourselves

    def _read_header(self):
        """Reads the first 4 bytes available as a header"""
        with self.bus_device_obj as i2c:
            i2c.readinto(self._data_buffer, end=4)  # this is expecting a header
        packet_header = Packet.header_from_buffer(self._data_buffer)
        self._dbg(packet_header)
        return packet_header

    def _read_packet(self):
        with self.bus_device_obj as i2c:
            i2c.readinto(self._data_buffer, end=4)  # this is expecting a header?
        self._dbg("")

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

        new_packet = Packet(self._data_buffer)
        if self._debug:
            print(new_packet)

        self._update_sequence_number(new_packet)

        return new_packet

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
        with self.bus_device_obj as i2c:
            i2c.readinto(self._data_buffer, end=total_read_length)

    @property
    def _data_ready(self):
        header = self._read_header()

        if header.channel_number > 5:
            self._dbg("channel number out of range:", header.channel_number)
        if header.packet_byte_count == 0x7FFF:
            print("Byte count is 0x7FFF/0xFFFF; Error?")
            if header.sequence_number == 0xFF:
                print("Sequence number is 0xFF; Error?")
            ready = False
        else:
            ready = header.data_length > 0

        return ready
