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

from core.component import Component
from core.logger import Logger, Level

from motor2040.payload import Payload 
from hardware.response import Response

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
        self._config_register = _cfg.get('config_register') # 1
        self._i2cbus = None
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def enable(self):
        Component.enable(self)
        self._i2cbus = SMBus(1)

        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_payload(self, command, port_speed, stbd_speed, duration):
        return Payload(command, port_speed, stbd_speed, duration)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def send_payload(self, payload):
        if not self.enabled:
            raise Exception('not enabled.')
        self._log.info("writing payload: " + Fore.WHITE + "'{}'…".format(payload.to_string()))
        # send over I2C
        self._i2cbus.write_block_data(self._i2c_slave_address, self._config_register, list(payload.to_bytes()))
        self._log.info('payload written.')
        # Read 1-byte response
        _read_data = self._i2cbus.read_byte_data(self._i2c_slave_address, self._config_register)
        # convert response byte to status enum or meaning
        _response = Response.from_value(_read_data)
        if _response.value <= Response.OKAY.value:
            self._log.info("response: {}".format(_response.name))
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
