# Copyright (c) 2018-2019 Mika Tuupola
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of  this software and associated documentation files (the "Software"), to
# deal in  the Software without restriction, including without limitation the
# rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
# sell copied of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

# https://github.com/tuupola/micropython-mpu9250

"""
MicroPython I2C driver for MPU6050 6-axis motion tracking device
"""

__version__ = "0.3.0-dev"

# pylint: disable=import-error
import ustruct
import utime
from machine import I2C, Pin
from micropython import const

# pylint: enable=import-error

_WHO_AM_I_REGISTER = const(0x75)
# _WHO_AM_I = const(0x68)

# Config
_CONFIG = const(0x6B)
_ENABLE_EVERYTHING = const(0b00000000)

_GYRO_CONFIG = const(0x1B)
# Gyro full scale settings
GYRO_FULL_SCALE_250DPS = const(0b00000000)
GYRO_FULL_SCALE_500DPS = const(0b00001000)
GYRO_FULL_SCALE_1000DPS = const(0b00010000)
GYRO_FULL_SCALE_2000DPS = const(0b00011000)

_GYRO_SO_250DPS = 131
_GYRO_SO_500DPS = 62.5
_GYRO_SO_1000DPS = 32.8
_GYRO_SO_2000DPS = 16.4

_ACCEL_CONFIG = const(0x1C)
# Accelerometer full scale settings
ACCEL_FULL_SCALE_2G = const(0b00000000)
ACCEL_FULL_SCALE_4G = const(0b00001000)
ACCEL_FULL_SCALE_8G = const(0b00010000)
ACCEL_FULL_SCALE_16G = const(0b00011000)

_ACCEL_SO_2G = 16384  # 1 / 16384 ie. 0.061 mg / digit
_ACCEL_SO_4G = 8192  # 1 / 8192 ie. 0.122 mg / digit
_ACCEL_SO_8G = 4096  # 1 / 4096 ie. 0.244 mg / digit
_ACCEL_SO_16G = 2048  # 1 / 2048 ie. 0.488 mg / digit

# Temperature scalers
_TEMP_SO = 340
_TEMP_OFFSET = 36.53

# Extra config registers
_SAMPLE_RATE_DIV = const(0x19)
_INT_PIN_CFG = const(0x37)

# Read-Only Registers
_ACCEL_XOUT_H = const(0x3B)
_ACCEL_XOUT_L = const(0x3C)
_ACCEL_YOUT_H = const(0x3D)
_ACCEL_YOUT_L = const(0x3E)
_ACCEL_ZOUT_H = const(0x3F)
_ACCEL_ZOUT_L = const(0x40)
_TEMP_OUT_H = const(0x41)
_TEMP_OUT_L = const(0x42)
_GYRO_XOUT_H = const(0x43)
_GYRO_XOUT_L = const(0x44)
_GYRO_YOUT_H = const(0x45)
_GYRO_YOUT_L = const(0x46)
_GYRO_ZOUT_H = const(0x47)
_GYRO_ZOUT_L = const(0x48)

# Used for enabling and disabling the i2c bypass access
_I2C_BYPASS_MASK = const(0b00000010)
_I2C_BYPASS_EN = const(0b00000010)
_I2C_BYPASS_DIS = const(0b00000000)

UNIT_G = 1
UNIT_METERS_PER_SEC2 = 9.80665  # 1 g = 9.80665 m/s2 ie. standard gravity
UNIT_DEG_PER_SEC = 1
UNIT_RAD_PER_SEC = 0.017453292519943  # 1 deg/s is 0.017453292519943 rad/s
UNIT_ROTATIONS_PER_SEC = 0.002777778  # 1/360

MPU_I2C_ADDRESS0 = 0x68
MPU_I2C_ADDRESS1 = 0x69

MPU6050_WHO_AM_I = 0x68


class MPU6050:
    """Class which provides interface to MPU6050 6-axis motion tracking device."""

    _WHO_AM_I = MPU6050_WHO_AM_I
    _PRODUCT = "MPU6050"

    def __init__(
        self,
        i2c,
        address=MPU_I2C_ADDRESS0,
        accel_full_scale=ACCEL_FULL_SCALE_16G,
        gyro_full_scale=GYRO_FULL_SCALE_2000DPS,
        accel_unit=UNIT_METERS_PER_SEC2,
        gyro_unit=UNIT_RAD_PER_SEC,
        gyro_offset=(0, 0, 0),
    ):
        """
        Initialize sensor chip
        :param i2c: An I2C object
        :param address: Override the I2C sensor address
        :param accel_full_scale: Set the max scale of the accelerometer
        :param gyro_full_scale: Set the max scale of the gyro
        :param accel_unit: Set the unit of the accelerometer
        :param gyro_unit: Set the unit of the gyro
        :param gyro_offset: Set the offsets of the gyro
        """
        self.i2c = i2c
        self.address = address
        who_is = self.whoami
        if self._WHO_AM_I != who_is:
            raise RuntimeError("{} not found in I2C bus. Found: {}".format(self._PRODUCT, hex(self.whoami)))

        self.accel_unit = accel_unit
        self.set_accel_range(accel_full_scale)

        self.gyro_unit = gyro_unit
        self.set_gyro_range(gyro_full_scale)
        self._gyro_offset = gyro_offset

        # Enable all sensors, disable sleep mode
        self._register_char(_CONFIG, _ENABLE_EVERYTHING)

        # Enable I2C bypass to access for MPU9250 magnetometer access.
        char = self._register_char(_INT_PIN_CFG)
        char &= ~_I2C_BYPASS_MASK  # clear I2C bits
        char |= _I2C_BYPASS_EN
        self._register_char(_INT_PIN_CFG, char)

    @property
    def sensors(self):
        raw_values = self._register_shorts(_ACCEL_XOUT_H, 7)
        accel_xyz = tuple([value * self._accel_scale_factor for value in raw_values[0:3]])
        temp = (raw_values[3] / _TEMP_SO) + _TEMP_OFFSET
        gyro_xyz = tuple([(raw_values[4 + idx] * self._gyro_scale_factor) - self._gyro_offset[idx] for idx in range(3)])
        return accel_xyz, temp, gyro_xyz

    @property
    def acceleration(self):
        """
        Acceleration measured by the sensor. By default will return a
        3-tuple of X, Y, Z axis acceleration values in m/s^2 as floats. Will
        return values in g if constructor was provided `accel_sf=UNIT_G`
        parameter.
        """
        xyz = self._register_three_shorts(_ACCEL_XOUT_H)
        return tuple([value * self._accel_scale_factor for value in xyz])

    @property
    def raw_acceleration(self):
        """
        Acceleration measured by the sensor. By default will return a
        3-tuple of X, Y, Z axis acceleration values in m/s^2 as floats. Will
        return values in g if constructor was provided `accel_sf=UNIT_G`
        parameter.
        """
        return self._register_three_shorts(_ACCEL_XOUT_H)

    @property
    def gyro(self):
        """
        X, Y, Z radians per second as floats.
        """
        so = self._gyro_scale_factor
        sf = self.gyro_unit
        ox, oy, oz = self._gyro_offset

        xyz = self._register_three_shorts(_GYRO_XOUT_H)
        xyz = [(value / so) * sf for value in xyz]

        xyz[0] -= ox
        xyz[1] -= oy
        xyz[2] -= oz

        return tuple(xyz)

    @property
    def temperature(self):
        """
        Die temperature in celsius as a float.
        """
        temp = self._register_short(_TEMP_OUT_H)
        return (temp / _TEMP_SO) + _TEMP_OFFSET

    @property
    def whoami(self):
        """ Value of the whoami register. """
        return self._register_char(_WHO_AM_I_REGISTER)

    def calibrate(self, count=256, delay=0):
        ox, oy, oz = (0.0, 0.0, 0.0)
        self._gyro_offset = (0.0, 0.0, 0.0)
        n = float(count)

        while count:
            utime.sleep_ms(delay)
            gx, gy, gz = self.gyro
            ox += gx
            oy += gy
            oz += gz
            count -= 1

        self._gyro_offset = (ox / n, oy / n, oz / n)
        return self._gyro_offset

    def _register_short(self, register, value=None, buf=bytearray(2)):
        if value is None:
            self.i2c.readfrom_mem_into(self.address, register, buf)
            return ustruct.unpack(">h", buf)[0]

        ustruct.pack_into(">h", buf, 0, value)
        return self.i2c.writeto_mem(self.address, register, buf)

    def _register_three_shorts(self, register, buf=bytearray(6)):
        self.i2c.readfrom_mem_into(self.address, register, buf)
        return ustruct.unpack(">hhh", buf)

    def _register_shorts(self, register, n):
        buf = bytearray(n * 2)
        self.i2c.readfrom_mem_into(self.address, register, buf)
        format_string = ">" + ("h" * n)
        return ustruct.unpack(format_string, buf)

    def _register_char(self, register, value=None, buf=bytearray(1)):
        if value is None:
            print("self.i2c.readfrom_mem_into(self.address={}, register={}, buf={})".format(self.address, register, buf))
            self.i2c.readfrom_mem_into(self.address, register, buf)
            print("buf[0]={}".format(buf[0]))
            return buf[0]

        ustruct.pack_into("<b", buf, 0, value)
        return self.i2c.writeto_mem(self.address, register, buf)

    def set_accel_range(self, full_scale_setting):
        self._register_char(_ACCEL_CONFIG, full_scale_setting)

        # Return the sensitivity divider
        if ACCEL_FULL_SCALE_2G == full_scale_setting:
            self._accel_scale_factor = self.accel_unit / _ACCEL_SO_2G
        elif ACCEL_FULL_SCALE_4G == full_scale_setting:
            self._accel_scale_factor = self.accel_unit / _ACCEL_SO_4G
        elif ACCEL_FULL_SCALE_8G == full_scale_setting:
            self._accel_scale_factor = self.accel_unit / _ACCEL_SO_8G
        elif ACCEL_FULL_SCALE_16G == full_scale_setting:
            self._accel_scale_factor = self.accel_unit / _ACCEL_SO_16G

    def set_gyro_range(self, full_scale_setting):
        self._register_char(_GYRO_CONFIG, full_scale_setting)

        # Return the sensitivity divider
        if GYRO_FULL_SCALE_250DPS == full_scale_setting:
            self._gyro_scale_factor = self.gyro_unit / _GYRO_SO_250DPS
        elif GYRO_FULL_SCALE_500DPS == full_scale_setting:
            self._gyro_scale_factor = self.gyro_unit / _GYRO_SO_500DPS
        elif GYRO_FULL_SCALE_1000DPS == full_scale_setting:
            self._gyro_scale_factor = self.gyro_unit / _GYRO_SO_1000DPS
        elif GYRO_FULL_SCALE_2000DPS == full_scale_setting:
            self._gyro_scale_factor = self.gyro_unit / _GYRO_SO_2000DPS

    def __enter__(self):
        return self

    def __exit__(self, exception_type, exception_value, traceback):
        pass
