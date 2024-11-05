#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-11-05
# modified: 2024-11-05
#

import traceback
import signal
from colorama import init, Fore, Style
init()

import VL53L1X
#from VL53L1X import VL53L1xDistanceMode

from core.component import Component
from core.logger import Logger, Level

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
class TimeOfFlight(Component):
    '''
    A wrapper class around a VL53L1CX Time of Flight distance sensor.

    :param config:          the application configuration
    :param level            the log level
    '''
    def __init__(self, config, level=Level.INFO):
        self._log = Logger('tof', level)
        Component.__init__(self, self._log, suppressed=False, enabled=False)
        self._log.info('initialising vl53l1cx…')
        if not isinstance(config, dict):
            raise ValueError('wrong type for config argument: {}'.format(type(name)))
        # add matrix display ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._rgbmatrix = None
        # configuration ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        _cfg = config['krzos'].get('hardware').get('vl53l1cx')
        _i2c_address = _cfg.get('i2c_address')
        self._range  = _cfg.get('range') # distance mode 1 = Short | 2 = Medium | 3 = Long
        # open and start the VL53L1X sensor.
        self._tof = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=_i2c_address)
        self._tof.open()
        self._tof.set_distance_mode(self._range) 
        self._log.info('configured vl53l1cx at 0x{:02X}.'.format(_i2c_address))
        # attach a signal handler to catch SIGINT (Ctrl+C) and exit gracefully
        signal.signal(signal.SIGINT, self._exit_handler)
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def enable(self):
        Component.enable(self)
        # we reuse range: 0 = Unchanged | 1 = Short Range | 2 = Medium Range | 3 = Long Range
        self._tof.start_ranging(self._range)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def poll(self):
        '''
        Returns the distance in millimeters.
        '''
        if not self.enabled:
            raise Exception('not enabled.')
        try:
            _distance_in_mm = self._tof.get_distance()
            return _distance_in_mm
        except Exception as e:
            self._log.error('{} raised while polling: {}\n{}'.format(type(e), e, traceback.format_exc()))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _exit_handler(self, signal, frame):
        print('\n')
        self._log.info(Fore.RED + 'exit handler called.')
        self.disable()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def disable(self):
        Component.disable(self)
        self._log.info(Fore.WHITE + 'stop ranging…')
        self._tof.stop_ranging()
        self._log.info('disabled.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def close(self):
        Component.close(self)
        self._tof.close()
        self._log.info('closed.')

#EOF
