#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-11-23
# modified: 2024-11-23
#

import sys
import traceback
from enum import Enum # for Response at bottom of file
from smbus import SMBus
from colorama import init, Fore, Style
init()

from core.component import Component
from core.orientation import Orientation
from core.logger import Logger, Level

class TinyFxController(Component):
    '''
    Connects with a Tiny FX over I2C.
    '''
    def __init__(self, config=None, level=Level.INFO):
        self._log = Logger('tinyfx-ctrl', level)
        Component.__init__(self, self._log, suppressed=False, enabled=True)
        _cfg = config['krzos'].get('hardware').get('tinyfx-controller')
        _bus_number = _cfg.get('bus_number')
        self.bus    = SMBus(_bus_number)
        self._i2c_address        = _cfg.get('i2c_address')
        self._config_register    = 1
        self._max_payload_length = 32
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def play(self, key):
        '''
        A convenience method to play the sound corresponding to the key.
        '''
        if key.startswith('play '):
            self._log.info("> '{}'".format(key))
            self.send_data(key)
        else:
            self._log.debug("play: '{}'".format(key))
            self.send_data('play {}'.format(key))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def channel_on(self, channel_id):
        '''
        Turn on the channel designated by:
            Orientation.PORT : port light
            Orientation.STBD : starboard light
            Orientation.MAST : mast flashing light
        '''
        match channel_id:
            case Orientation.ALL:
                self.send_data('on')
            case Orientation.PORT:
                self.send_data('port')
            case Orientation.STBD:
                self.send_data('stbd')
            case Orientation.MAST:
                self.send_data('mast')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def send_data(self, data):
        '''
        Send a string of data over I2C.
        '''
        try:
            payload = self._convert_to_payload(data)
            self._write_payload(payload)
            self._write_completion_code()
            return self._read_response()
        except TimeoutError as te:
            self._log.error("transfer timeout: {}".format(te))
        except Exception as e:
            self._log.error('{} thrown in thunderborg test: {}\n{}'.format(type(e), e, traceback.format_exc()))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _convert_to_payload(self, data):
        '''
        Convert the string to a payload suitable for I2C transfer.
        '''
        payload = list(bytes(data, "utf-8"))
        if len(payload) > self._max_payload_length:
            raise ValueError(f"Source text ({len(payload)} chars) too long: {self._max_payload_length} maximum.")
        return payload

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _write_payload(self, payload):
        '''
        Write the payload to the I2C bus.
        '''
        self.bus.write_block_data(self._i2c_address, self._config_register, payload)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _write_completion_code(self):
        '''
        Write a completion code to the I2C bus.
        '''
        self._log.debug("writing completion code...")
        self.bus.write_byte_data(self._i2c_address, self._config_register, 0xFF)
        self._log.debug("write complete.")

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _read_response(self):
        '''
        Read the response from the I2C device.
        '''
        read_data = self.bus.read_byte_data(self._i2c_address, self._config_register)
        self._log.debug("read data: '{}'".format(read_data))
        response = Response.from_value(read_data)
        if response.value <= Response.OKAY.value:
            self._log.debug("tinyfx response: {}".format(response.name))
        elif read_data == 32:
            self._log.debug("tinyfx response: okay")
        else:
            self._log.error("tinyfx response: {}; data: 0x{:02X}".format(response.name, read_data))
        return response

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def close(self):
        self.send_data('off')
        self._log.info('closed.')


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Response(Enum):
    # this variable must include all entries, whitespace-delimited
    __order__ = " INIT OKAY BAD_ADDRESS OUT_OF_SYNC INVALID_CHAR SOURCE_TOO_LARGE UNVALIDATED EMPTY_PAYLOAD PAYLOAD_TOO_LARGE UNKNOWN_ERROR VALUE_OFF VALUE_ON "
    '''
    Provides an enumeration of response codes from the I2C Slave.
    These match the hard-coded values in the MicroPython file.

    # response codes: (note: extension values <= 0x4F are considered 'okay')
    INIT              = 0x10
    OKAY              = 0x4F
    BAD_ADDRESS       = 0x71
    OUT_OF_SYNC       = 0x72
    INVALID_CHAR      = 0x73
    SOURCE_TOO_LARGE  = 0x74
    UNVALIDATED       = 0x75
    EMPTY_PAYLOAD     = 0x76
    PAYLOAD_TOO_LARGE = 0x77
    UNKNOWN_ERROR     = 0x78
    '''
    INIT              = (  0, 'init',              0x10 )
    OKAY              = (  1, 'okay',              0x4F )
    BAD_ADDRESS       = (  2, 'bad address',       0x71 )
    OUT_OF_SYNC       = (  3, 'out of sync',       0x72 )
    INVALID_CHAR      = (  4, 'invalid character', 0x73 )
    SOURCE_TOO_LARGE  = (  5, 'source too large',  0x74 )
    UNVALIDATED       = (  6, 'unvalidated',       0x75 )
    EMPTY_PAYLOAD     = (  7, 'empty payload',     0x76 )
    PAYLOAD_TOO_LARGE = (  8, 'payload too large', 0x77 )
    UNKNOWN_ERROR     = (  9, 'unknown error',     0x78 )

    # example extension
    VALUE_OFF         = ( 10, 'off',               0x30 )
    VALUE_ON          = ( 11, 'on',                0x31 )

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
#       print("from value: '{}'".format(value))
        return Response.UNKNOWN_ERROR
#       raise NotImplementedError

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def __str__(self):
        return 'Response.{}; value={:4.2f}'.format(self.name, self._value )

#EOF
