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
from smbus2 import SMBus
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
        self._bus   = SMBus(_bus_number)
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
    def channel_on(self, orientation):
        '''
        Accepts an Orientation argument. Modify the channel designated by:

            Orientation.NONE :  turn off all lights
            Orientation.ALL :   turn on port, starboard and mast lights
            Orientation.PORT :  turn on port light
            Orientation.STBD :  turn on starboard light
            Orientation.MAST :  turn on mast flashing light
        '''
        match orientation:
            case Orientation.NONE:
                self.send_data('off')
            case Orientation.ALL:
                self.send_data('on')
            case Orientation.PORT:
                self.send_data('port')
            case Orientation.STBD:
                self.send_data('stbd')
            case Orientation.MAST:
                self.send_data('mast')
            case Orientation.PIR:
                self.send_data('pir get')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def on(self):
        '''
        A shortcut that turns on all channels.
        '''
        self.channel_on(Orientation.ALL)
        self._log.info('lights on.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def off(self):
        '''
        A shortcut that turns off all channels.
        '''
        self.channel_on(Orientation.NONE)
        self._log.info('lights off.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def pir(self, enabled):
        '''
        Enables or disables the PIR sensor.
        '''
        if enabled:
            self.send_data('pir on')
        else:
            self.send_data('pir off')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def ram(self):
        '''
        Displays free RAM on the console.
        '''
        self.send_data('ram')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def flash(self):
        '''
        Displays flash memory info on the console.
        '''
        self.send_data('flash')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def send_data(self, data):
        '''
        Send a string of data over I2C.
        '''
        try:
            payload = self._convert_to_payload(data)
            self._log.debug('send payload {} as data: {}'.format(payload, data))
            self._write_payload(payload)
            self._write_completion_code()
            return self._read_response()
        except TimeoutError as te:
            self._log.error("transfer timeout: {}".format(te))
        except Exception as e:
            self._log.error('{} thrown sending data to tiny fx: {}\n{}'.format(type(e), e, traceback.format_exc()))

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
        self._bus.write_block_data(self._i2c_address, self._config_register, payload)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _write_completion_code(self):
        '''
        Write a completion code to the I2C bus.
        '''
        self._log.debug("writing completion code...")
        self._bus.write_byte_data(self._i2c_address, self._config_register, 0xFF)
        self._log.debug("write complete.")

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _read_response(self):
        '''
        Read the response from the I2C device.
        '''
        read_data = self._bus.read_byte_data(self._i2c_address, self._config_register)
        response = Response.from_value(read_data)
        self._log.debug("read data: '{}' as response: {}".format(read_data, response))
        if response.value <= Response.OKAY.value:
            self._log.info("tinyfx response: {}".format(response.name))
        elif read_data == 32:
            self._log.info("tinyfx response: okay")
        else:
            self._log.error("tinyfx response: {}; data: 0x{:02X}".format(response.name, read_data))
        return response

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def close(self):
        self.send_data('off')
        self._bus.close()
        self._log.info('closed.')

    # direct commands ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈



# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Response(Enum):
    # this variable must include all entries, whitespace-delimited
    __order__ = " INIT BAD_ADDRESS BAD_REQUEST OUT_OF_SYNC INVALID_CHAR SOURCE_TOO_LARGE UNVALIDATED EMPTY_PAYLOAD PAYLOAD_TOO_LARGE UNKNOWN_ERROR PIR_ACTIVE PIR_IDLE OKAY "
    '''
    Provides an enumeration of response codes from the I2C Slave.
    These match the hard-coded values in the MicroPython file.

    # response codes: (note: extension values <= 0x4F are considered 'okay')
    '''
    INIT              = (  0, 'init',              0x10 )
    BAD_ADDRESS       = (  1, 'bad address',       0x41 )
    BAD_REQUEST       = (  2, 'bad request',       0x42 )
    OUT_OF_SYNC       = (  3, 'out of sync',       0x43 )
    INVALID_CHAR      = (  4, 'invalid character', 0x44 )
    SOURCE_TOO_LARGE  = (  5, 'source too large',  0x45 )
    UNVALIDATED       = (  6, 'unvalidated',       0x46 )
    EMPTY_PAYLOAD     = (  7, 'empty payload',     0x47 )
    PAYLOAD_TOO_LARGE = (  8, 'payload too large', 0x48 )
    UNKNOWN_ERROR     = (  9, 'unknown error',     0x49 )

    # example extension
    PIR_ACTIVE        = ( 11, 'pir active',        0x30 )
    PIR_IDLE          = ( 12, 'pir idle',          0x31 )

    OKAY              = ( 10, 'okay',              0x4F ) # all acceptable values less than 0x4F

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
