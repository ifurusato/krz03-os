#!/usr/bin/env python3 # -*- coding: utf-8 -*-
#
# Copyright 2020 by Murray Altheim. All rights reserved. This file is part of
# the Robot OS project and is released under the "Apache Licence, Version 2.0".
# Please see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2020-01-18
# modified: 2021-06-29
#

import sys, itertools, time
from math import isclose
from colorama import init, Fore, Style
init()

from core.logger import Level, Logger
from core.component import Component
from core.orientation import Orientation
from hardware.decoder import Decoder

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class SlewLimiter(Component):
    '''
    The slew limiter for this Motor is much simplified from previous
    versions, as it simply uses the step counter to resolve the
    difference between the current speed and the target speed.
    '''
    def __init__(self, motor, scale=1.0, level=Level.INFO):
        self._log = Logger('slew:{}'.format(motor.orientation.label), level)
        Component.__init__(self, self._log, suppressed=False, enabled=True)
        self._motor = motor
        self._current_speed = 0.0
        self._target_speed  = 0.0
        self._scale = scale
        self._log.info('ready.')

    @property
    def speed(self):
        return int(self._current_speed)

    @speed.setter
    def speed(self, speed):
        self._current_speed = float(speed) 

    @property
    def target_speed(self):
        return int(self._target_speed)

    @target_speed.setter
    def target_speed(self, target_speed):
        self._target_speed = float(target_speed)

    def resolve(self):
        _difference = self._target_speed - self._current_speed
        # adjust current_speed by a fraction of the difference
        if abs(_difference) > 0.01:  # Threshold to avoid minor adjustments
            self._current_speed += self._scale * _difference
            self._log.info(Fore.RED    + '{} motor; speed: {:4.2f}'.format(self._motor.orientation, self._current_speed))
        else:
            self._current_speed = self._target_speed # snap to target if close enough
            self._log.info(Fore.YELLOW + '{} motor; speed: {:4.2f}'.format(self._motor.orientation, self._current_speed))
        return self._current_speed

    def __call__(self):
        _speed = self.resolve()
        if _speed == 0:
            return 0
        else:
            return int(_speed)

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Motor(Component):
    '''
    Controls a motor that uses a Hall Effect encoder to determine the robot's
    velocity and distance traveled.

    The motors provide 69.3 steps per axle rotation. The Mecanum wheels have
    a 48mm nominal diameter:

       https://www.robotshop.com/products/48mm-steel-mecanum-wheel-set-2x-left-2x-right

    With the given wheel size, there are 6.63 rotations per meter, or
    461 steps per meter.

    :param picon_zero:  reference to the PiconZero motor controller
    :param orientation: motor orientation
    :param level:       log level
    '''
    def __init__(self, picon_zero, orientation, level=Level.INFO):
        if picon_zero is None:
            raise ValueError('null picon zero argument.')
        self._picon_zero = picon_zero
        self._orientation = orientation
        if self._orientation.side is Orientation.PORT:
            self._channel = 1
        elif self._orientation.side is Orientation.STBD:
            self._channel = 0
        else:
            raise ValueError('expected PORT or STBD side.')
        self._log = Logger('motor:{}'.format(orientation.label), level)
        Component.__init__(self, self._log, suppressed=False, enabled=True)
        self._log.info('initialising {} motor with {} at address 0x{:02X} as motor controller…'.format(
                orientation.name, type(self._picon_zero).__name__, self._picon_zero.I2cAddress))
        # encoder/decoder ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._steps_per_rotation = 69.3
        self.__steps = 0 # step counter
        if orientation is Orientation.PFWD:
            _encoder_a   = 16 # C1
            _encoder_b   = 26 # C2
        elif orientation is Orientation.SFWD:
            _encoder_a   = 21 # C1
            _encoder_b   = 20 # C2
        elif orientation is Orientation.PAFT:
            _encoder_a   =  5 # C1
            _encoder_b   = 12 # C2
        elif orientation is Orientation.SAFT:
            _encoder_a   = 13 # C1
            _encoder_b   =  6 # C2
        else:
            raise Exception('unexpected motor orientation.')
        self._decoder = Decoder(orientation, _encoder_a, _encoder_b, self._callback_step_count, self._log.level)
        # slew limiter ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        _scale = 0.03
        self._slew_limiter = SlewLimiter(self, scale=_scale, level=self._log.level)
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def orientation(self):
        '''
        Returns the orientation of this motor.
        '''
        return self._orientation

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property   
    def decoder(self):
        return self._decoder
        
    @decoder.setter
    def decoder(self, decoder):
        self._decoder = decoder 

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def steps(self):
        return self.__steps

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def reset_steps(self):
        self.__steps = 0

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def steps_per_rotation(self):
        return self._steps_per_rotation

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _callback_step_count(self, pulse):
        '''
        This callback is used to capture encoder steps.

        Note that this is the opposite (i.e.,adding/subtracting steps by
        orientation) than on the KR01 or MR01.
        ''' 
        if self._orientation.side is Orientation.PORT:
            self.__steps = self.__steps - pulse
        elif self._orientation.side is Orientation.STBD:
            self.__steps = self.__steps + pulse
        if self._orientation.side is Orientation.PORT:
#           self._log.info(Fore.RED   + Style.DIM + '{} motor: {:d} steps.'.format(self._orientation, self.__steps))
            pass
        else:
#           self._log.info(Fore.GREEN + Style.DIM + '{} motor: {:d} steps.'.format(self._orientation, self.__steps))
            pass
        _speed = self._slew_limiter()
        if _speed:
            self._picon_zero.set_motor(self._channel, _speed)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def set_speed(self, speed):
        '''
        Set the speed of this motor to the provided value.
        '''
        if self._slew_limiter.speed != speed:
            self._slew_limiter.target_speed = speed
        _current_speed = self._slew_limiter()
        self._picon_zero.set_motor(self._channel, _current_speed)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def stop(self):
        '''
        Stops the motor immediately.
        '''
        self._picon_zero.set_motor(self._channel, 0)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def is_enabled(self):
        return self.enabled

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def enable(self):
        if not self.enabled:
            Component.enable(self)
        self._log.info('enabled.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def disable(self):
        if self.enabled:
            Component.disable(self)
            self._log.info('disabled.')
        else:
            self._log.warning('already disabled.')
        self.off() # in any case

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def close(self):
        # just do it anyway
        self.stop()
        if self.enabled:
            self.disable()
        self._log.info('closed.')

#EOF
