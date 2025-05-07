#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-08-13
# modified: 2025-05-03
#

from smbus import SMBus
from colorama import init, Fore, Style
init()

import core.globals as globals
globals.init()

from core.component import Component
from core.logger import Logger, Level

from motor2040.payload import Payload 
from hardware.response import Response
from hardware.player import Player, Sound
from hardware.tinyfx_controller import TinyFxController

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class MotorController(Component):
    '''
    A motor controller connected over I2C to a Motor 2040. This class must
    be explicitly enabled prior to use.
    '''
    def __init__(self, config, level=Level.INFO):
        self._log = Logger('motor_ctrl', level)
        Component.__init__(self, self._log, suppressed=False, enabled=False)
        _cfg = config['krzos'].get('hardware').get('motor_controller') 
        self._i2c_slave_address = _cfg.get('i2c_address') # 0x44
        self._config_register   = _cfg.get('config_register') # 1
        self._bus_number        = _cfg.get('bus_number') # 1
        self._i2cbus            = None
        self._ping_test         = False
        # set up for sound
        _component_registry = globals.get('component-registry')
        self._tinyfx = _component_registry.get('krzos')
        if self._tinyfx is None:
            self._tinyfx = TinyFxController(config, level=Level.INFO)
            Player.instance(tinyfx=self._tinyfx)
            if not self._tinyfx.enabled:
                self._tinyfx.enable()
        else:
            self._log.warning('tinyFX already existed.')
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def enable(self):
        '''
        Enables the MotorController if upon pinging the Motor 2040 there is
        an OKAY response.
        '''
        Component.enable(self)
        try:
            self._i2cbus = SMBus(self._bus_number)
            if self._ping_test:
                _response = self.write_payload(self.get_payload('stop', 0.0, 0.0, 0.0), verbose=False)
                if _response is Response.OKAY:
                    self._log.info('enabled; response: ' + Fore.GREEN + '{}'.format(_response.name))
                else:
                    Player.instance().play(Sound.BUZZ)
                    raise Exception('not enabled; response was not okay: ' + Fore.RED + '{}'.format(_response.name))
        except Exception as e:
            self._log.error('{} raised, could not connect to Motor 2040: {}'.format(type(e), e))
            # disable if error occurs or Motor 2040 is unavailable
            Component.disable(self)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def lights(self, enable):
        if enable:
            self._tinyfx.on()
        else:
            self._tinyfx.off()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def play(self, name):
        '''
        Since we're holding the instance of the sound Player this is a
        convenience method to play a sound, provided its name.
        '''
        _sound = Sound.from_name(name)
        Player.instance().play(_sound)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def stop(self):
        '''
        A convenience method that stops all motors, returning the Response.
        This may also be used as a ping.
        '''
        return self.send_payload('stop', 0.0, 0.0, 0.0)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def send_payload(self, command, port_speed, stbd_speed, duration):
        '''
        Generates a payload and sends it to the Motor 2040. This is a shortcut
        that simply obtains the payload from get_payload() and sends it to the
        Motor 2040 via write_payload().
        '''
        return self.write_payload(self.get_payload(command, port_speed, stbd_speed, duration))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def get_payload(command, port_speed, stbd_speed, duration):
        '''
        Generates a payload provides the requisite arguments.
        '''
        return Payload(command, port_speed, stbd_speed, duration)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def write_payload(self, payload, verbose=True):
        '''
        Writes the payload argument to the Motor 2040.
        '''
        if not self.enabled:
            raise Exception('not enabled.')
        if verbose:
            self._log.debug("writing payload: " + Fore.WHITE + "'{}'".format(payload.to_string()))
        # send over I2C
        self._i2cbus.write_block_data(self._i2c_slave_address, self._config_register, list(payload.to_bytes()))
        if verbose:
            self._log.info("payload written: " + Fore.WHITE + "'{}'".format(payload.to_string()))
        # read 1-byte response
        _read_data = self._i2cbus.read_byte_data(self._i2c_slave_address, self._config_register)
        # convert response byte to status enum or meaning
        _response = Response.from_value(_read_data)
        if _response.value <= Response.OKAY.value:
            self._log.debug("response: {}".format(_response.name))
        else:
            self._log.error("error response: {}".format(_response.name))
        return _response

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def disable(self):
        Component.disable(self)
        if self._i2cbus:
            self._i2cbus.close()
        self._log.info('disabled.')

#EOF
