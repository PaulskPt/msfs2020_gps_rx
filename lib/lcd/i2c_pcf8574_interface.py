# Copyright (C) 2017 Dan Halbert
# Adapted from https://github.com/dbrgn/RPLCD, Copyright (C) 2013-2016 Danilo Bargen

# Permission is hereby granted, free of charge, to any person obtaining a copy of
# this software and associated documentation files (the "Software"), to deal in
# the Software without restriction, including without limitation the rights to
# use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
# the Software, and to permit persons to whom the Software is furnished to do so,
# subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
# FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
# COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
# IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
# CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

"""Low-level interface to PCF8574."""

import busio
import board
from microcontroller import pin, delay_us
#import microcontroller # Added to prevent NameError: name 'microcontroller' is not defined
from adafruit_bus_device.i2c_device import I2CDevice

from .lcd import LCD_4BITMODE, LCD_BACKLIGHT, LCD_NOBACKLIGHT, PIN_ENABLE

# Addition by @paulsk (CircuitPython@discord) because of error ḿodule object has no attribute 'SCL' 
l_scl = pin.GPIO7
l_sda = pin.GPIO6

class I2CPCF8574Interface:
    
    # Bit values to turn backlight on/off. Indexed by a boolean.
    _BACKLIGHT_VALUES = (LCD_NOBACKLIGHT, LCD_BACKLIGHT)

    def __init__(self, address, scl = l_scl, sda = l_sda):
        """
        CharLCD via PCF8574 I2C port expander.

        Pin mapping::

            7  | 6  | 5  | 4  | 3  | 2  | 1  | 0
            D7 | D6 | D5 | D4 | BL | EN | RW | RS

        :param address: The I2C address of your LCD.
        """
        self.address = address
        # following two lines added by @paulsk
        self.scl = l_scl
        self.sda = l_sda
        self._backlight_pin_state = LCD_BACKLIGHT
        # Next line modified by @paulsk
        # After modifying the next line, arrived a 'RuntimeError: SDA or SCL needs a pull up'
        # Solution: adding 2 physical pull-up resistors, each 5.2 kOhm between +5V and the
        # ESP32-S2-Saloa-1R pins 5 and 5 (i.e.: board.IO5 and board.IO4, or SCL and SDA resp).
        self.i2c = busio.I2C(self.scl, self.sda)  # was: (board.SCL, board.SDA)
        self.i2c_device = I2CDevice(self.i2c, self.address)
        self.data_buffer = bytearray(1)

    def deinit(self):
        self.i2c.deinit()

    @property
    def data_bus_mode(self):
        return LCD_4BITMODE

    @property
    def backlight(self):
        return self._backlight_pin_state == LCD_BACKLIGHT

    @backlight.setter
    def backlight(self, value):
        self._backlight_pin_state = _BACKLIGHT_VALUES[value]
        self._i2c_write(self._backlight_pin_state)

    # Low level commands

    def send(self, value, rs_mode):
        """Send the specified value to the display in 4-bit nibbles.
        The rs_mode is either ``_RS_DATA`` or ``_RS_INSTRUCTION``."""
        self._write4bits(rs_mode | (value & 0xF0) | self._backlight_pin_state)
        self._write4bits(rs_mode | ((value << 4) & 0xF0) | self._backlight_pin_state)

    def _write4bits(self, value):
        """Pulse the `enable` flag to process value."""
        with self.i2c_device:
            self._i2c_write(value & ~PIN_ENABLE)
            # This 1us delay is probably unnecessary, given the time needed
            # to execute the statements.
            delay_us(1)
            #microcontroller.delay_us(1)
            self._i2c_write(value | PIN_ENABLE)
            delay_us(1)
            #microcontroller.delay_us(1)
            self._i2c_write(value & ~PIN_ENABLE)
        # Wait for command to complete.
        delay_us(100)
        #microcontroller.delay_us(100)

    def _i2c_write(self, value):
        self.data_buffer[0] = value
        self.i2c_device.write(self.data_buffer)
