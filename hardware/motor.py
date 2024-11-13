#!/usr/bin/env python3 
# -*- coding: utf-8 -*-
#
# Copyright 2020 by Murray Altheim. All rights reserved. This file is part of
# the Robot OS project and is released under the "Apache Licence, Version 2.0".
# Please see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2020-01-18
# modified: 2024-11-11
#

import itertools
from math import isclose
from datetime import datetime as dt
from colorama import init, Fore, Style
init()

from core.logger import Level, Logger
from core.component import Component
from core.orientation import Orientation

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Motor(Component):

    # Constants ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    SPEED_SCALE = 5.4 # The scaling to apply to the motor's speed to match its real-world speed

    '''
    Controls a motor that uses a Hall Effect encoder to determine the robot's
    velocity and distance traveled. This implementation is mostly a wrapper
    around the motor support from the Pimoroni Inventor HAT Mini.

    The Mecanum wheels have a 48mm nominal diameter:

       https://www.robotshop.com/products/48mm-steel-mecanum-wheel-set-2x-left-2x-right

    The motors provide 69.3 steps per axle rotation. 

    With the given wheel size, there are 6.63 rotations per meter, or
    461 steps per meter.

    :param config:      the YAML based application configuration
    :param orientation: motor orientation
    :param motor:       a reference to the IOE Motor
    :param encoder:     a reference to the IOE Encoder
    :param level:       log level
    '''
    def __init__(self, config, orientation, motor=None, encoder=None, enable_pid=False, level=Level.INFO):
        if config is None:
            raise ValueError('no configuration provided.')
        _cfg = config['krzos'].get('motor')
        self._orientation = orientation
        self._log = Logger('motor:{}'.format(orientation.label), level)
        if motor is None:
            raise ValueError('null motor argument.')
        self._mtr = motor # underlying Motor
        if enable_pid and encoder is None:
            raise ValueError('null encoder argument.')
        self._enc = encoder 
        Component.__init__(self, self._log, suppressed=False, enabled=True)
        self._verbose = False
        self._current_speed = 0.0
        self._enable_pid = enable_pid
        # encoder/decoder ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._counter = itertools.count()
        self._steps_per_rotation = 693 # TODO encoder steps per wheel rotation
        # step limiter ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._step_limit    = None
        self._step_limit_threshold = 50
        self._last_time = dt.now() # for calculating elapsed time
        self._log.info('ready.')

        _api = '''
    /usr/local/lib/python3.11/dist-packages/ioexpander/motor.py

    def __init__(self, config, picon_zero, orientation, level=Level.INFO):
    def pid(self):
    def orientation(self):
    def rpm(self):
    def max_rpm(self):
    def decoder(self):
    def decoder(self, decoder):
    def reset(self):
    def steps(self):
    def steps_per_rotation(self):
    def _apply_rpm_deadband(self, target_rpm):
    def _apply_slew_limit(self, target_speed):
    def step_limit(self):
    def step_limit(self, step_limit):
    def _update_step_limit(self, pulse):
    def _gradually_set_speed_to_zero(self, reduction_factor=0.05):
    def _callback_step_count(self, pulse):
    def _calculate_rpm(self):
    def speed(self):
    def speed(self, speed):
    def target_speed(self):
    def update_speed(self):
    def _set_target_rpm(self, target_rpm=None):
    def _set_motor_power(self, power=None):
    def brake(self):
    def stop(self):
    def is_enabled(self):
    def enable(self):
    def disable(self):
    def close(self):

IOE Motor:

    def __init__(self, direction=NORMAL_DIR, speed_scale=DEFAULT_SPEED_SCALE, zeropoint=DEFAULT_ZEROPOINT, deadzone=DEFAULT_DEADZONE):
    def enable_with_return(self):
    def disable_with_return(self):
    def is_enabled(self):
    def get_duty(self):
    def get_deadzoned_duty(self):
    def set_duty_with_return(self, duty):
    def get_speed(self):
    def set_speed_with_return(self, speed):
    def stop_with_return(self):
    def full_negative_with_return(self):
    def full_positive_with_return(self):
    def to_percent_with_return(self, input, in_min=ZERO_PERCENT, in_max=ONEHUNDRED_PERCENT, speed_min=None, speed_max=None):
    def get_direction(self):
    def set_direction(self, direction):
    def get_speed_scale(self):
    def set_speed_scale(self, speed_scale):
    def get_zeropoint(self):
    def set_zeropoint(self, zeropoint):
    def get_deadzone(self):
    def set_deadzone_with_return(self, deadzone):
    def duty_to_level(duty, resolution):
    def __duty_to_speed(duty, zeropoint, scale):
    def __speed_to_duty(speed, zeropoint, scale):
    def __apply_duty(self, duty, mode, load, wait_for_load):
    def __init__(self, ioe, pins, direction=NORMAL_DIR, speed_scale=MotorState.DEFAULT_SPEED_SCALE, zeropoint=MotorState.DEFAULT_ZEROPOINT,
    def enable(self, load=True, wait_for_load=False):
    def disable(self, load=True, wait_for_load=False):
    def is_enabled(self):
    def duty(self, duty=None, load=True, wait_for_load=False):
    def frequency(self, freq=None, load=True, wait_for_load=False):
    def stop(self, load=True, wait_for_load=False):
    def coast(self, load=True, wait_for_load=False):
    def brake(self, load=True, wait_for_load=False):
    def full_negative(self, load=True, wait_for_load=False):
    def full_positive(self, load=True, wait_for_load=False):
    def to_percent(self, input, in_min=MotorState.ZERO_PERCENT, in_max=MotorState.ONEHUNDRED_PERCENT, speed_min=None, speed_max=None, load=True, wait_for_load=False):
    def direction(self, direction=None):
    def speed_scale(self, speed_scale=None):
    def zeropoint(self, zeropoint=None):
    def deadzone(self, deadzone=None, load=True, wait_for_load=False):
    def decay_mode(self, mode=None, load=True, wait_for_load=False):
    def load(self, wait_for_load=False):
'''

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def orientation(self):
        '''
        Returns the orientation of this motor.
        '''
        return self._orientation

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property   
    def rpm(self):
        return 0 # TODO

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def reset(self):
        '''
        Resets the step counters and other variables related to historical
        movement, as well as resetting the slew limiter to its default rate.
        '''
        pass
        # TODO as well as other detritus

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def steps(self):
        return 0 # TODO

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def set_deadband(self, speed):
        pass

    # underlying motor implementation ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_motor(self):
        print('🤖 get_motor() type: {}'.format(type(self._mtr)))
        return self._mtr

    # step limit ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    @property
    def step_limit(self):
        return self._step_limit

    @step_limit.setter
    def step_limit(self, step_limit):
        self._log.info('set limit to {:d} steps for {} motor.'.format(step_limit, self._orientation.name))
        self._step_limit = step_limit

    def _update_step_limit(self, pulse):
        '''
        This method adjusts the motor’s step limit based on its current value.
        Positive limits are incremented by 1, negative limits are decremented by
        1, and zero values remain unchanged.
        '''
        pass

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def speed(self):
        '''
        Returns the motor speed.

        def speed(self, speed=None, load=True, wait_for_load=False):
        '''
        return self._mtr.speed()

    @speed.setter
    def speed(self, value):
        '''
        Sets the motor speed.
        '''
        self._current_speed = value
        if self._enable_pid:
#           print(Fore.MAGENTA + 'PID-SET speed: {:4.2f}; enabled? {}'.format(value, self.enabled) + Style.RESET_ALL)
            pass
        else:
#           print(Fore.MAGENTA + 'SET speed: {:4.2f}; enabled? {}'.format(value, self.enabled) + Style.RESET_ALL)
            self._mtr.speed(value)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def current_speed(self):
        return self._current_speed

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def update_target_speed(self):
        self._log.info(Fore.MAGENTA + '💜 update target speed: {:4.2f}'.format(self._current_speed))
        pass

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def brake(self):
        '''
        Stops the motor quickly.

        This changes the slew rate, so following brake you should reset the
        slew rate to its default.
        '''
        self._mtr.brake()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def coast(self):
        '''
        Coasts the motor.
        '''
        self._mtr.coast()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def stop(self):
        '''
        Stops the motor immediately.
        '''
        pass

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def is_enabled(self):
        return self.enabled

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def enable(self):
        if not self.enabled:
            Component.enable(self)
#           self._mtr.enable()
            self._log.info(Fore.WHITE + 'enabled.')
        else:
            self._log.warning('already enabled.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def disable(self):
        self.stop() # in any case
        if self.enabled:
            Component.disable(self)
#           self._mtr.disable()
            self._log.info('disabled.')
        else:
            self._log.warning('already disabled.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def close(self):
        # just do it anyway
        self.stop()
        if self.enabled:
            self.disable()
        self._log.info('closed.')

#EOF
