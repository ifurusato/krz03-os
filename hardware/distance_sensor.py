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

class DistanceSensor:
    def __init__(self, pin, timeout=1, smoothing=False, smoothing_window=5, loop_interval=0.1):
        '''
        Initializes the DistanceSensor.

        Args:
            pin (int): GPIO pin connected to the sensor.
            timeout (float): Time in seconds to consider sensor as timed out.
            smoothing (bool): Enable smoothing of distance readings.
            smoothing_window (int): Number of samples to use for smoothing.
            loop_interval (float): Interval between distance polling, in seconds.
        '''
        self._pin = pin
        self._timeout = timeout
        self._pulse_start = None
        self._pulse_width_us = None
        self._last_read_time = time.time()
        self._smoothing = smoothing
        self._window = deque(maxlen=smoothing_window) if smoothing else None
        self._loop_interval = loop_interval
        self._running = False
        self._distance = None
        self._task = None

        # Initialize pigpio
        self._pi = pigpio.pi()
        if not self._pi.connected:
            raise Exception("Failed to connect to pigpio daemon")
        self._pi.set_mode(self._pin, pigpio.INPUT)
        self.cb = self._pi.callback(self._pin, pigpio.EITHER_EDGE, self.pulse_callback)

    def pulse_callback(self, gpio, level, tick):
        '''
        Callback function to measure PWM pulse width.
        '''
        if level == 1:  # Rising edge
            self._pulse_start = tick
        elif level == 0:  # Falling edge
            if self._pulse_start is not None:
                self._pulse_width_us = pigpio.tickDiff(self._pulse_start, tick)

    def _compute_distance(self):
        '''
        Compute and update the distance based on the current pulse width.
        '''
        if self._pulse_width_us is not None:
            if 1000 <= self._pulse_width_us <= 1850:
                distance_mm = (self._pulse_width_us - 1000) * 3 / 4
                self._last_read_time = time.time()
                self._pulse_width_us = None  # Reset after processing

                if self._smoothing:
                    self._window.append(distance_mm)
                    self._distance = sum(self._window) / len(self._window)
                else:
                    self._distance = distance_mm
            else:
                self._distance = None

    def get_distance(self):
        '''
        Get the latest computed distance.
        '''
        return self._distance

    def check_timeout(self):
        '''
        Check if the sensor has timed out (no pulse received recently).
        '''
        return time.time() - self._last_read_time > self._timeout

    async def _sensor_loop(self):
        '''
        Asynchronous loop to continuously compute distances.
        '''
        while self._running:
            self._compute_distance()
            await asyncio.sleep(self._loop_interval)

    def start(self):
        '''
        Start the sensor's asynchronous loop.
        '''
        self._running = True
        self._task = asyncio.create_task(self._sensor_loop())

    def stop(self):
        '''
        Stop the sensor's asynchronous loop and clean up resources.
        '''
        self._running = False
        if self._task:
            self._task.cancel()
        self.cb.cancel()
        self._pi.stop()

