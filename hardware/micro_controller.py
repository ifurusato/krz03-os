#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-05-27
# modified: 2025-05-27
#

import itertools
from datetime import datetime as dt
from colorama import init, Fore, Style
init()

from core.component import Component
from core.logger import Logger, Level
from hardware.controller import Controller
from hardware.response import*

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class MicroController(Component):
    NAME  = 'µctrl'
    '''
    Handles communication with multiple microcontrollers.
    '''
    def __init__(self, config=None, level=Level.INFO):
        self._log = Logger(MicroController.NAME, level)
        self._log.info('instantiating MicroController…')
        Component.__init__(self, self._log, suppressed=False, enabled=False)
        _cfg = config['krzos'].get('hardware').get('micro_controller')
        self._name_0         = _cfg.get('name_0', 'ctrl0')
        self._name_1         = _cfg.get('name_1', 'ctrl1')
        self._name_2         = _cfg.get('name_2', 'ctrl2')
        self._i2c_address_0  = _cfg.get('i2c_address_0', 0x43)
        self._i2c_address_1  = _cfg.get('i2c_address_1', 0x44) # motor 2040
        self._i2c_address_2  = _cfg.get('i2c_address_2', 0x45) # tinyfx
        self._clear_on_close = _cfg.get('clear_on_close', False)
        self._bus_number     = _cfg.get('bus_number', 1)
        self._controller_0   = Controller(self._name_0, i2c_bus=self._bus_number, i2c_address=self._i2c_address_0, level=Level.WARN)
        self._controller_1   = Controller(self._name_1, i2c_bus=self._bus_number, i2c_address=self._i2c_address_1, level=Level.WARN)
        self._controller_2   = Controller(self._name_2, i2c_bus=self._bus_number, i2c_address=self._i2c_address_2, level=Level.WARN)
        self._controller_map = {
            self._name_0: self._controller_0,
            self._name_1: self._controller_1,
            self._name_2: self._controller_2,
        }
        self._counter = itertools.count()
        self._count   = 0
        self._log.info('ready.')

    def get_name_0(self):
        return self._name_0

    def get_name_1(self):
        return self._name_1

    def get_name_2(self):
        return self._name_2

    def get_names(self):
        return self._controller_map.keys()

    def get_controllers(self):
        return self._controller_map.values()

    def get_controller(self, name):
        return self._controller_map.get(name)

    def send_payload(self, name, command):
        '''
        Send the command (up to 31 ASCII characters) to the named Controller,
        returning the Response.
        '''
        self._count = next(self._counter)
        start_time = dt.now()
        _controller = self.get_controller(name)
        _response = _controller.send_payload(command)
        elapsed_ms = (dt.now() - start_time).total_seconds() * 1000.0
        if _response is None:
            raise ValueError('null response.')
        elif isinstance(_response, Response):
            if _response == RESPONSE_OKAY:
                self._log.info("[{:04}] response: ".format(self._count) + Fore.GREEN + "'{}'".format(_response.description) + Fore.CYAN + "; {:5.2f}ms elapsed.".format(elapsed_ms))
            else:
                self._log.warning("[{:04}] response: ".format(self._count) + Fore.RED + "'{}'".format(_response.description) + Fore.WHITE + "; {:5.2f}ms elapsed.".format(elapsed_ms))
        elif not isinstance(_response, Response):
            raise ValueError('expected Response, not {}.'.format(type(_response)))
        else:
            self._log.error("error response: {}; {:5.2f}ms elapsed.".format(_response.description, elapsed_ms))
        return _response

    def off(self):
        '''
        Turn off the lights when you leave the room.
        '''
        self._log.info('off.')
        if self._clear_on_close:
            for ctrl in self._controller_map.values():
                if ctrl:
                    ctrl.send_payload('black')

    def close(self):
        '''
        Close the controller.
        '''
        self.off()
        for ctrl in self._controller_map.values():
            if ctrl:
                ctrl.close()
        self._log.info('closed.')

#EOF
