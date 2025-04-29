#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-08-13
# modified: 2025-04-26
#
# A script that sends the command line argument as a packet to an I2C slave.
#
# see smbus2
# https://smbus2.readthedocs.io/en/latest/#smbus2.SMBus.write_block_data
#

import sys, traceback
from smbus import SMBus
from enum import Enum

#from response import Response

HELP = '''
motor2040 commands:

    enable                     enable all motors
    disable                    disable all motors
    stop                       stop all motors
    coast                      coast all motors to stop
    brake                      brake all motors to stop
    slow-decay                 mode change
    fast-decay                 mode change
    all_SPEED[_DURATION]       set speed of all motors
    crab_SPEED[_DURATION]      set crab speed
    rotate_SPEED[_DURATION]    set rotation speed
    pfwd_SPEED[_DURATION]      set speed of port-forward motor
    sfwd_SPEED[_DURATION]      set speed of stbd-forward motor
    paft_SPEED[_DURATION]      set speed of port-aft motor
    saft_SPEED[_DURATION]      set speed of stbd-aft motor

where SPEED (0.0 - 1.0) and DURATION (seconds) are real numbers, e.g., "all_0.6_4.2"
'''

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Response(Enum):
    # this variable must include all entries, whitespace-delimited
    __order__ = " INIT OKAY BAD_ADDRESS BAD_REQUEST OUT_OF_SYNC INVALID_CHAR SOURCE_TOO_LARGE UNVALIDATED  EMPTY_PAYLOAD PAYLOAD_TOO_LARGE RUNTIME_ERROR UNKNOWN_ERROR VALUE_OFF VALUE_ON "
    '''
    Provides an enumeration of response codes from the I2C Slave.
    These match the hard-coded values in the MicroPython file.
    '''
    INIT              = (  0, 'init',              0x10 )
    OKAY              = (  1, 'okay',              0x4F )
    BAD_ADDRESS       = (  2, 'bad address',       0x71 )
    BAD_REQUEST       = (  3, 'bad request',       0x72 )
    OUT_OF_SYNC       = (  4, 'out of sync',       0x73 )
    INVALID_CHAR      = (  5, 'invalid character', 0x74 )
    SOURCE_TOO_LARGE  = (  6, 'source too large',  0x75 )
    UNVALIDATED       = (  7, 'unvalidated',       0x76 )
    EMPTY_PAYLOAD     = (  8, 'empty payload',     0x77 )
    PAYLOAD_TOO_LARGE = (  9, 'payload too large', 0x78 )
    RUNTIME_ERROR     = ( 10, 'unknown error',     0x79 )
    UNKNOWN_ERROR     = ( 11, 'unknown error',     0x80 )

    # example extension
    VALUE_OFF         = ( 12, 'off',               0x30 )
    VALUE_ON          = ( 13, 'on',                0x31 )

    # ignore the first param since it's already set by __new__
    def __init__(self, num, name, value):
        self._num   = num
        self._name  = name
        self._value = value

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def num(self):
        '''
        Returns the original enum numerical value.
        '''
        return self._num

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def name(self):
        return self._name

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def value(self):
        return self._value

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def from_value(value):
        for r in Response:
            if value == r.value:
                return r
        return Response.UNKNOWN_ERROR
#       raise NotImplementedError

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def __str__(self):
        return 'Response.{}; value={:4.2f}'.format(self.name, self._value )

# end of class ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

if len(sys.argv) != 2:
    print("\nERROR: expected 1 command line argument: the data to transfer.")
    print(HELP)
    sys.exit(1)

_value = sys.argv[1]
#print('-- value \"{}\"'.format(_value))

# constants ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

I2C_SLAVE_ADDRESS = 0x44
CONFIG_REGISTER   = 1

# main ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

try:

    # convert source string to a list of bytes as a payload
    _payload = list(bytes(_value, 'utf-8'))
    if len(_payload) > 32:
        raise Exception('source text ({:d} chars) too long: 32 maximum.'.format(len(_payload)))

    print('🔺 creating connection to I2C bus on address 0x{:02X}…'.format(I2C_SLAVE_ADDRESS))
    _i2cbus = SMBus(1)  # create a new I2C bus

    print("🔺 writing I2C payload of {:d} chars: '{}'…".format(len(_value), _value))
    _i2cbus.write_block_data(I2C_SLAVE_ADDRESS, CONFIG_REGISTER, _payload)

    print('🔺 writing completion code…')
    _i2cbus.write_byte_data(I2C_SLAVE_ADDRESS, CONFIG_REGISTER, 0xff)

    print('🔺 write complete.')

    _read_data = _i2cbus.read_byte_data(I2C_SLAVE_ADDRESS, CONFIG_REGISTER)
    print("read data: '{}'".format(_read_data))
    _response = Response.from_value(_read_data)
    if _response.value <= Response.OKAY.value:
        print("response: {}".format(_response.name))
    else:
        print("ERROR response: {}".format(_response.name))

except TimeoutError as te:
    print('ERROR: transfer timeout: {}'.format(te))
except Exception as e:
    print('ERROR: {} encountered: {}\n{}'.format(type(e), e, traceback.format_exc()))
finally:
    print('complete.')

#EOF
