#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-11-16
# modified: 2024-11-18
#

import time
import asyncio
from collections import deque
import pigpio

from core.logger import Logger, Level
from core.component import Component
from core.orientation import Orientation

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
class DistanceSensor(Component):
    CLASS_NAME = 'dist'
    '''
    Provides distance information in millimeters from a Pololu PWM-based
    infrared proximity sensor.
    '''
    def __init__(self, config, orientation, level=Level.INFO):
        '''
        Initializes the DistanceSensor.

        :param config:        the application configuration
        :param orientation:   the Orientation of the sensor
        :param level:         the logging Level
        '''
        self._log = Logger('dist:{}'.format(orientation.label), level)
        Component.__init__(self, self._log, suppressed=False, enabled=True)
        if config is None:
            raise ValueError('no configuration provided.')
        _cfg = config['krzos'].get('hardware').get('distance_sensor')
        match orientation:
            case Orientation.PORT:
                self._pin = _cfg.get('pin_port') # pin connected to the port sensor
            case Orientation.CNTR:
                self._pin = _cfg.get('pin_cntr') # pin connected to the center sensor
            case Orientation.STBD:
                self._pin = _cfg.get('pin_stbd') # pin connected to the starboard sensor
            case _:
                raise Exception('unexpected orientation: {}'.format(orientation.name))
        self._orientation = orientation
        self._timeout = _cfg.get('timeout')     # time in seconds to consider sensor as timed out 
        self._smoothing = _cfg.get('smoothing') # enable smoothing of distance readings 
        _smoothing_window    = _cfg.get('smoothing_window')
        self._window = deque(maxlen=_smoothing_window) if self._smoothing else None
        self._loop_interval  = _cfg.get('loop_interval') # interval between distance polling, in seconds
        self._pulse_start    = None
        self._pulse_width_us = None
        self._distance       = None
        self._task           = None
        self._running        = False
        # initialize pigpio
        self._pi = pigpio.pi()
        if not self._pi.connected:
            raise Exception("Failed to connect to pigpio daemon")
        self._pi.set_mode(self._pin, pigpio.INPUT)
        self._callback = self._pi.callback(self._pin, pigpio.EITHER_EDGE, self._pulse_callback)
        self._last_read_time = time.time()
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def orientation(self):
        return self._orientation

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _pulse_callback(self, gpio, level, tick):
        '''
        Callback function to measure PWM pulse width.
        '''
        if level == 1:   # rising edge
            self._pulse_start = tick
        elif level == 0: # falling edge
            if self._pulse_start is not None:
                self._pulse_width_us = pigpio.tickDiff(self._pulse_start, tick)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _compute_distance(self):
        '''
        Compute and update the distance based on the current pulse width.
        '''
        if self._pulse_width_us is not None:
            if 1000 <= self._pulse_width_us <= 1850:
                distance_mm = (self._pulse_width_us - 1000) * 3 / 4
                self._last_read_time = time.time()
                self._pulse_width_us = None # reset after processing
                if self._smoothing:
                    self._window.append(distance_mm)
                    self._distance = sum(self._window) / len(self._window)
                else:
                    self._distance = distance_mm
            else:
                self._distance = None

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_distance(self):
        '''
        Get the latest computed distance.
        '''
        return self._distance

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def check_timeout(self):
        '''
        Check if the sensor has timed out (no pulse received recently).
        '''
        return time.time() - self._last_read_time > self._timeout

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    async def _sensor_loop(self):
        '''
        Asynchronous loop to continuously compute distances.
        '''
        while self._running:
            self._compute_distance()
            await asyncio.sleep(self._loop_interval)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def start(self):
        '''
        Start the sensor's asynchronous loop.
        '''
        self._running = True
        self._task = asyncio.create_task(self._sensor_loop())

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def stop(self):
        '''
        Stop the sensor's asynchronous loop and clean up resources.
        '''
        self._running = False
        if self._task:
            self._task.cancel()
        self._callback.cancel()
        self._pi.stop()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def enable(self):
        Component.enable(self)
 
    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def disable(self):
        self.stop()
        Component.disable(self)
 
    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def close(self):
        '''
        Stop the loop if running, then close the sensor.
        '''
        Component.close(self) # calls disable

#EOF
