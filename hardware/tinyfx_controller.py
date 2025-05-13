#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-11-23
# modified: 2024-11-23
#

import time
import sys
import traceback
import datetime as dt
from smbus2 import SMBus
from colorama import init, Fore, Style
init()

from core.component import Component
from core.orientation import Orientation
from core.logger import Logger, Level
#from enum import Enum # for Response at bottom of file
from hardware.response import Response

class TinyFxController(Component):
    '''
    Connects with a Tiny FX over I2C.
    '''
    def __init__(self, config=None, level=Level.INFO):
        self._log = Logger('tinyfx-ctrl', level)
        Component.__init__(self, self._log, suppressed=False, enabled=False)
        _cfg = config['krzos'].get('hardware').get('tinyfx-controller')
        self._i2c_address        = _cfg.get('i2c_address')
        self._bus_number         = _cfg.get('bus_number')
        self._i2cbus             = None
        self._config_register    = 1
        self._max_payload_length = 32
        self._last_send_time = None  # timestamp of last send
        self._min_send_interval = dt.timedelta(milliseconds=100)  # 100ms minimum send interval
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def enable(self):
        '''
        Enables the TinyFxController if upon pinging the TinyFX there is
        an OKAY response.
        '''
        Component.enable(self)
        try:
            self._i2cbus = SMBus(self._bus_number)
            _response = self.send_data('off')
            time.sleep(1) # otherwise the RP2040 will end up using both cores
            if _response is Response.OKAY:
                self._log.info('enabled; response: ' + Fore.GREEN + '{}'.format(_response.name))
            else:
                raise Exception('tinyfx not enabled; response was not okay: ' + Fore.RED + '{}'.format(_response.name))
        except Exception as e:
            self._log.error('{} raised could not connect to tinyfx: {}'.format(type(e), e))
            # disable if error occurs or TinyFX is unavailable
            Component.disable(self)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def help(self):
        '''
        Print help.
        '''
        self.send_data('help')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def play(self, key):
        '''
        A convenience method to play the sound corresponding to the key,
        returning the Response.
        '''
        if key.startswith('play '):
            self._log.info("> '{}'".format(key))
            return self.send_data(key)
        else:
            self._log.debug("play: '{}'".format(key))
            return self.send_data('play {}'.format(key))

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
                return self.send_data('off')
            case Orientation.ALL:
                return self.send_data('on')
            case Orientation.PORT:
                return self.send_data('port')
            case Orientation.STBD:
                return self.send_data('stbd')
            case Orientation.MAST:
                return self.send_data('mast')
            case Orientation.PIR:
                return self.send_data('pir get')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def on(self):
        '''
        A shortcut that turns on all channels, returning the Response.
        '''
        self._log.info('lights on…')
        return self.channel_on(Orientation.ALL)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def off(self):
        '''
        A shortcut that turns off all channels, returning the Response.
        '''
        self._log.info('lights off…')
        return self.channel_on(Orientation.NONE)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def pir(self, enabled):
        '''
        Enables or disables the PIR sensor, returning the Response.
        '''
        if enabled:
            return self.send_data('pir on')
        else:
            return self.send_data('pir off')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def ram(self):
        '''
        Displays free RAM on the console, returning the Response.
        '''
        return self.send_data('ram')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def sounds(self):
        '''
        Displays the list of sounds, returning the Response.
        '''
        return self.send_data('sounds')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def flash(self):
        '''
        Displays flash memory info on the console, returning the Response.
        '''
        return self.send_data('flash')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def exit(self):
        '''
        Exits the main loop on the TinyFX, returning the Response.
        This will disable the I2C bus so don't do it.
        '''
        return self.send_data('exit')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def send_data(self, data):
        '''
        Send a string of data over I2C, returning the Response.
        '''
        if not self.enabled:
            raise Exception('not enabled.')
        now = dt.datetime.now()
        if self._last_send_time:
            elapsed = now - self._last_send_time
            if elapsed < self._min_send_interval:
                self._log.warning(
                    "send_data skipped: only {:.1f}ms since last send (minimum is {:.1f}ms)".format(
                        elapsed.total_seconds() * 1000,
                        self._min_send_interval.total_seconds() * 1000
                    )
                )
                return Response.SKIPPED
        try:
            payload = self._convert_to_payload(data)
            self._log.info("send payload '{}' as data: '{}'".format(payload, data))
            self._write_payload(payload)
            self._write_completion_code()
            _response = self._read_response()
            self._last_send_time = now # update only on success
            return _response
        except TimeoutError as te:
            self._log.error("transfer timeout: {}".format(te))
            return Response.CONNECTION_ERROR
        except Exception as e:
            self._log.error('{} thrown sending data to tiny fx: {}\n{}'.format(type(e), e, traceback.format_exc()))
        return Response.UNKNOWN_ERROR

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
        self._i2cbus.write_block_data(self._i2c_address, self._config_register, payload)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _write_completion_code(self):
        '''
        Write a completion code to the I2C bus.
        '''
        self._log.info("writing completion code…")
        self._i2cbus.write_byte_data(self._i2c_address, self._config_register, 0xFF)
        self._log.debug("write complete.")

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _read_response(self):
        '''
        Read the response from the I2C device.
        '''
        read_data = self._i2cbus.read_byte_data(self._i2c_address, self._config_register)
        response = Response.from_value(read_data)
        self._log.info("read data: '{}' as response: {}".format(read_data, response))
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
        self._i2cbus.close()
        self._log.info('closed.')

#EOF
