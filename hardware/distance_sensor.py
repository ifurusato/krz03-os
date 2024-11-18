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

from collections import deque
import time
import pigpio

class DistanceSensor:
    def __init__(self, pin, timeout=1, smoothing=False, smoothing_window=5):
        '''
        Initializes the DistanceSensor.

        Args:
            pin (int): GPIO pin connected to the sensor.
            timeout (float): Time in seconds to consider sensor as timed out.
            smoothing (bool): Enable smoothing of distance readings.
            smoothing_window (int): Number of samples to use for smoothing.
        '''
        self._pin             = pin
        self._timeout         = timeout
        self._pulse_start     = None
        self._pulse_width_us  = None
        self._last_read_time  = time.time()
        self._smoothing       = smoothing
        self._window = deque(maxlen=smoothing_window) if smoothing else None
        # initialize pigpio
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
                # Correct usage of tickDiff
                self._pulse_width_us = pigpio.tickDiff(self._pulse_start, tick)

    def get_distance(self):
        '''
        Returns the distance calculated from the pulse width.
        Applies smoothing if enabled.
        Returns None if the pulse width is out of range.
        '''
        if self._pulse_width_us is not None:
            if 1000 <= self._pulse_width_us <= 1850:
                # Convert pulse width to distance in mm
                distance_mm = (self._pulse_width_us - 1000) * 3 / 4
                # Update last valid read time only after a successful read
                self._last_read_time = time.time()
                self._pulse_width_us = None  # Reset after processing

                # Apply smoothing if enabled
                if self._smoothing:
                    self._window.append(distance_mm)
                    return sum(self._window) / len(self._window)
                return distance_mm
        return None

    def check_timeout(self):
        '''
        Checks for timeout (no pulse received for a while).
        '''
        return time.time() - self._last_read_time > self._timeout

    def stop(self):
        '''
        Clean up resources before exiting.
        '''
        self.cb.cancel()
        self._pi.stop()

#EOF
