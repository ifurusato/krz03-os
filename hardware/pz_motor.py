#!/usr/bin/env python3 
# -*- coding: utf-8 -*-
#
# Copyright 2020 by Murray Altheim. All rights reserved. This file is part of
# the Robot OS project and is released under the "Apache Licence, Version 2.0".
# Please see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2020-01-18
# modified: 2024-10-31
#

import itertools
from math import isclose
from datetime import datetime as dt
from colorama import init, Fore, Style
init()

from core.logger import Level, Logger
from core.component import Component
from core.orientation import Orientation
from hardware.decoder import Decoder
from hardware.slew_limiter import SlewLimiter
from hardware.pid import PID

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

    :param config:      the YAML based application configuration
    :param picon_zero:  reference to the PiconZero motor controller
    :param orientation: motor orientation
    :param level:       log level
    '''
    def __init__(self, config, picon_zero, orientation, level=Level.INFO):
        if picon_zero is None:
            raise ValueError('null picon zero argument.')
        if config is None:
            raise ValueError('no configuration provided.')
        _cfg = config['krzos'].get('motor')
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
        self._verbose = False
        # encoder/decoder ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._counter = itertools.count()
        self._steps_per_rotation = 693  # encoder steps per wheel rotation
        self._steps      = 0 # step counter
        self._last_steps = 0 # last step count for direction detection
        self._step_timestamps = [] # used for calculating RPM
        self._speed_tolerance = 3 # around zero
        self._speed      = 0   # current speed
        self._target_speed = 0 # target speed
        self._rpm        = 0.0
        self._max_rpm    = 0.0
        self._rotations  = 0.0
        self._timestamp_limit = 10 # the maximum number of timestamps to keep
        self._decoder = Decoder(config, orientation, self._callback_step_count, self._log.level)
        # step limiter ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._step_limit = None
        self._step_limit_threshold = 50
        # PID controller ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._enable_pid_controller = _cfg.get('enable_pid_controller')
        if self._enable_pid_controller:
            _cfg = config['krzos'].get('motor').get('pid_controller')
            _kp         = _cfg.get('kp') # proportional gain
            _ki         = _cfg.get('ki') # integral gain
            _kd         = _cfg.get('kd') # derivative gain
            _min_output = _cfg.get('minimum_output')
            _max_output = _cfg.get('maximum_output')
            _freq_hz    = _cfg.get('sample_freq_hz')
            _period_sec = 1.0 / _freq_hz
            self._log.info('sample frequency: {:d}Hz; period: {:5.2f} sec.'.format(_freq_hz, _period_sec))
            self._pid = PID(orientation.label, _kp, _ki, _kd, _min_output, _max_output, setpoint=0.0, period=0.01, level=Level.INFO)
            self._log.info(Fore.GREEN + 'using PID control for motors.')
        else:
            self._log.info(Fore.GREEN + 'using direct power drive for motors.')
            self._pid = None
        # slew limiter ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._enable_slew_limiter = _cfg.get('enable_slew_limiter')
        if self._enable_slew_limiter:
            self._slew_limiter = SlewLimiter(config, self, level=self._log.level)
        else:
            self._slew_limiter = None
        self._slew_rate = 5
        self._last_time = dt.now() # for calculating elapsed time
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def pid(self):
        '''
        Returns the PID controller of this motor. If disabled this will
        return a None.
        '''
        return self._pid

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
        return self._rpm

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property   
    def max_rpm(self):
        return self._max_rpm

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property   
    def decoder(self):
        return self._decoder
        
    @decoder.setter
    def decoder(self, decoder):
        self._decoder = decoder 

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def reset(self):
        '''
        Resets the step counters and other variables related to historical
        movement, as well as resetting the slew limiter to its default rate.
        '''
        if self._slew_limiterd:
            self._slew_limiter.reset()
        # as well as other detritus
        self._speed        = 0
        self._target_speed = 0
        self._steps        = 0 # step counter
        self._last_steps   = 0 # last step count for direction detection
        self._step_timestamps.clear()
        self._rpm          = 0.0
        self._max_rpm      = 0.0
        self._rotations    = 0.0

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def steps(self):
        return self._steps

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def steps_per_rotation(self):
        return self._steps_per_rotation

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _apply_rpm_deadband(self, target_rpm):
        _deadband = 4
        if target_rpm - _deadband < self._rpm < target_rpm + _deadband:
            return self._rpm
        else:   
            return target_rpm

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        '''
        Limit the rate of change of speed (acceleration).
        '''
    def _apply_slew_limit(self, target_speed):
        if target_speed > self._rpm + self._slew_rate:
            return self._rpm + self._slew_rate
        elif target_speed < self._rpm - self._slew_rate:
            return self._rpm - self._slew_rate
        return target_speed

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
        if self._step_limit is not None:
            self._step_limit -= abs(pulse)
            if self._step_limit < 0:
                self._step_limit = None # prevent negative step limits
                self._log.info(Fore.RED + 'motor: {}; END step limit for {:d} steps.'.format(self._orientation.name, self._steps))
            elif self._step_limit < self._step_limit_threshold:
                self._log.info(Fore.GREEN + 'motor: {}; DECELERATING from limit: {:d} for {:d} steps.'.format(self._orientation.name, self._step_limit, self._steps))
                self._gradually_set_speed_to_zero()
            else:
                self._log.info(Fore.MAGENTA + 'motor: {}; UPDATE limit: {:d} for {:d} steps.'.format(self._orientation.name, self._step_limit, self._steps))
        else:
#           self._log.info(Fore.MAGENTA + Style.DIM + 'motor: {}; NO STEP LIMIT for {} steps.'.format(self._orientation.name, self._steps))
            pass

    def _gradually_set_speed_to_zero(self, reduction_factor=0.05):
        '''
        Gradually reduces the motor speed to zero as the step limit approaches a threshold.
        The reduction is controlled by the reduction_factor multiplied by the step limit.
        If the step limit is None, no changes are made to the motor speed.
        '''
        if self._step_limit is None:
            print('🍇 gradually_set_speed_to_zero    None')
            return
        elif self._step_limit == 0:
            print('🍇 gradually_set_speed_to_zero    Zero')
            self.speed = 0  # set speed to zero if step limit is zero
        elif self._step_limit <= self._step_limit_threshold:
            # calculate the reduction amount based on the step limit and reduction factor
            _speed = self.speed
            _reduction_amount = reduction_factor * (self._step_limit_threshold - self._step_limit) / self._step_limit_threshold * abs(_speed)
            # determine new speed by reducing current speed gradually towards zero
            if _speed > 0:
                new_speed = max(0, int(_speed - _reduction_amount)) # reduce positive speed
            elif _speed < 0:
                new_speed = min(0, int(_speed + _reduction_amount)) # reduce negative speed towards zero
            else:
                new_speed = 0  # If the speed is already zero
            print('🍇 gradually_set_speed_to_zero reduction_amoung: {}; new speed: {}'.format(_reduction_amount, new_speed))
            # update the motor speed
            self.speed = new_speed

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _callback_step_count(self, pulse):
        '''
        This callback is used to capture encoder steps, as well as
        calculate motor RPM.

        Note that this is the opposite (i.e.,adding/subtracting steps by
        orientation) than on the KR01 or MR01.
        ''' 
        if self._orientation.side is Orientation.PORT:
            self._steps = self._steps - pulse
        elif self._orientation.side is Orientation.STBD:
            self._steps = self._steps + pulse
        self._update_step_limit(pulse)
#       if self._step_limit is not None:
#           self._gradually_set_speed_to_zero()
#           pass
        self._calculate_rpm()
        if self._enable_slew_limiter:
            _speed = self._slew_limiter()
            if _speed:
                self._picon_zero.set_motor(self._channel, _speed)
        if self._orientation is Orientation.PFWD:
#           print('_callback_step_count for {} motor: {} steps.'.format(self._orientation.name, self._steps))
            pass

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _calculate_rpm(self):
        '''
        Update step count and timestamps. The step count is incremented or
        decremented based on encoder input.
        '''
        # append current timestamp for each step
        self._step_timestamps.append(dt.now())
        # check if steps are counting up or down based on the last steps count
        _fore = Fore.GREEN
        _multiplier = 1
        if self._steps > self._last_steps:
#           self._steps += 1 # increment step count for forward
            pass
        else:
            _fore = Fore.MAGENTA
            _multiplier = -1
#           self._steps -= 1 # decrement step count for reverse
            pass
        self._last_steps = self._steps # Update the last step count

        if len(self._step_timestamps) > 1:
            # calculate the time intervals between consecutive steps
            intervals = [
                (t2 - t1).total_seconds() for t1, t2 in itertools.pairwise(self._step_timestamps)
            ]
            # calculate average frequency if there are multiple intervals
            avg_interval = sum(intervals) / len(intervals)
            freq_hz = 1 / avg_interval
            self._rpm = _multiplier * (freq_hz / self._steps_per_rotation) * 60
            self._max_rpm = max(self._max_rpm, self._rpm)
            self._rotations = self._steps / self._steps_per_rotation
#           if self._verbose:
#               if next(self._counter) % 20 == 0:
#                   self._log.info(Fore.YELLOW + '{: >5d} steps; {: >4.2f} rotations; freq: {: >4d}Hz; '.format(self._steps, self._rotations, int(freq_hz))
#                           + _fore + 'speed: {: >3d} rpm; '.format(int(self._rpm))
#                           + Fore.WHITE + 'max: {: >3d} rpm; target speed: {:d}'.format(int(self._max_rpm), int(self._slew_limiter.target_speed)))
            # keep only the latest timestamps based on the limit
            if len(self._step_timestamps) > self._timestamp_limit:
                self._step_timestamps = self._step_timestamps[-self._timestamp_limit:]

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def speed(self):
        '''
        Returns the currently-set motor speed, a value from -100 to 100.
        '''
        return self._speed

    @speed.setter
    def speed(self, speed):
        '''
        Depending on whether the PID controller is enabled, this forwards on
        the request to either _set_target_rpm() or _set_motor_power(). The
        argument should be between -100 and 100.
        '''
        self._target_speed = speed
        if self._enable_pid_controller:
            self._set_target_rpm(speed)
            self._log.info(Fore.GREEN + 'set target RPM: {}; _target_speed: {}'.format(speed, self._target_speed))
        else:
            self._log.info(Fore.GREEN + 'set motor power: {}; _target_speed: {}'.format(speed, self._target_speed))
            self._set_motor_power(speed)

    @property
    def target_speed(self):
        '''
        Returns the current target speed, a value from -100 to 100.
        '''
        return self._target_speed

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def update_speed(self):
        '''
        This sets the speed to the current target speed, to be used in loops.
        '''
        self.speed = self._target_speed

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _set_target_rpm(self, target_rpm=None):
        '''
        Set the target RPM of this motor to the provided value. This is
        an alternative to setting the speed directly, and uses a PID
        controller to match the motor RPM with the target.

        Note that if the argument is None we use the current target speed.
        '''
        if self._orientation is Orientation.PFWD and target_rpm > 0:
#           print('_set_target_rpm: {}'.format(target_rpm))
            pass
        _elapsed_ms = round(( dt.now() - self._last_time ).total_seconds() * 1000.0)
        if target_rpm is None:
            target_rpm = self._target_speed
        if isclose(target_rpm, 0.0, abs_tol=self._speed_tolerance):
            self._pid.setpoint = 0.0
            self._pid.target   = 0.0
            self._set_motor_power(0)
        else:
            target_rpm = self._apply_rpm_deadband(target_rpm)
            target_rpm = self._apply_slew_limit(target_rpm)
            self._pid.setpoint = target_rpm
            self._pid.target = self._rpm
            _pid_output = self._pid()
            self._speed += _pid_output
            self._set_motor_power(self._speed)
            _error = self._pid.setpoint - self._pid.target
            if self._verbose and self._orientation is Orientation.PFWD:
                self._log.info(Fore.YELLOW + 'current rpm: {:5.2f}; target rpm: {:5.2f}; speed: {:4.2f};'.format(self._rpm, target_rpm, self._speed)
#                       + Fore.GREEN + Style.DIM + ' kp={: 5.2f}; ki={: 5.2f}; kd={: 5.2f};'.format(self._pid.kp, self._pid.ki, self._pid.kd)
#                       + Fore.GREEN + Style.NORMAL + ' pid.setpoint: {:5.2f}; pid output: {:5.2f};'.format(self._pid.setpoint, _pid_output)
                        + Fore.BLUE + Style.BRIGHT + ' {:5.2f} steps;'.format(self._steps)
                        + Fore.MAGENTA + ' error: {:<5.2f};'.format(_error)
                        + Fore.CYAN + Style.NORMAL + ' elapsed: {:d}ms'.format(_elapsed_ms))
            self._last_time = dt.now()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _set_motor_power(self, power=None):
        '''
        Set the power of this motor to the provided value between -100 and 100.

        Note that if the argument is None we use the current target speed.
        '''
        if power is None:
            power = self._target_speed
        if isinstance(power, float):
            power = int(power)
        if self._enable_slew_limiter:
            if self._slew_limiter.current_speed != power:
                self._slew_limiter.target_speed = power
            for _ in range(20):
                self._slew_limiter.update()
                self._speed = self._slew_limiter()
                self._picon_zero.set_motor(self._channel, self._speed)
        else:
            if self._orientation is Orientation.PFWD and abs(power) > 0:
#               print('set motor power for {} set to power: {}'.format(self._orientation.name, power))
                pass
            self._speed = power
            self._picon_zero.set_motor(self._channel, self._speed)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def brake(self):
        '''
        Stops the motor quickly.

        This changes the slew rate, so following brake you should reset the
        slew rate to its default.
        '''
        if self._slew_limiter:
            self._slew_limiter.braking()
        self._set_motor_power(0)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def stop(self):
        '''
        Stops the motor immediately.
        '''
        if self._slew_limiter:
            self._slew_limiter.stop()
#       self._picon_zero.set_motor(self._channel, 0)
        self._set_motor_power(0)

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
        self.stop() # in any case
        if self.enabled:
            Component.disable(self)
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
