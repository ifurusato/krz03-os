#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-04-25
# modified: 2025-04-25
#
import utime
from motor import FAST_DECAY, SLOW_DECAY, Motor, motor2040

from core.logger import Level, Logger

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class MotorController(object):
    PFWD = 0
    SFWD = 1
    PAFT = 2
    SAFT = 3
    DEFAULT_SPEED = 0.5 # if speed is not specified on motor commands
    '''
    A motor controller for four motors.

    :param level:   the log level
    '''
    def __init__(self, level=Level.INFO):
        super().__init__()
        self._log = Logger('motor_ctrl', level)
        # create motors
        self._motor_pfwd = Motor(motor2040.MOTOR_A)
        self._motor_sfwd = Motor(motor2040.MOTOR_B)
        self._motor_paft = Motor(motor2040.MOTOR_C)
        self._motor_saft = Motor(motor2040.MOTOR_D)
        self._motor_pfwd_speed = 0.0
        self._motor_sfwd_speed = 0.0
        self._motor_paft_speed = 0.0
        self._motor_saft_speed = 0.0
        self._acceleration_delay = 0.1   # for acceleration or any loops
        self._deceleration_delay = 0.2   # for acceleration or any loops
        self._delta              = 0.025 # iterative delta
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def enable(self):
        self._motor_pfwd.enable()
        self._motor_sfwd.enable()
        self._motor_paft.enable()
        self._motor_saft.enable()
        self._log.info('enabled.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def disable(self):
        self._motor_pfwd.disable()
        self._motor_sfwd.disable()
        self._motor_paft.disable()
        self._motor_saft.disable()
        self._log.info('disabled.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def stop(self):
        self._log.info('stop.')
        self._motor_pfwd.stop()
        self._motor_sfwd.stop()
        self._motor_paft.stop()
        self._motor_saft.stop()
        self._motor_pfwd_speed = 0.0
        self._motor_sfwd_speed = 0.0
        self._motor_paft_speed = 0.0
        self._motor_saft_speed = 0.0

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def coast(self):
        self._log.info('coast.')
        self._motor_pfwd.coast()
        self._motor_sfwd.coast()
        self._motor_paft.coast()
        self._motor_saft.coast()
        self._motor_pfwd_speed = 0.0
        self._motor_sfwd_speed = 0.0
        self._motor_paft_speed = 0.0
        self._motor_saft_speed = 0.0

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def brake(self):
        self._log.info('brake.')
        self._motor_pfwd.brake()
        self._motor_sfwd.brake()
        self._motor_paft.brake()
        self._motor_saft.brake()
        self._motor_pfwd_speed = 0.0
        self._motor_sfwd_speed = 0.0
        self._motor_paft_speed = 0.0
        self._motor_saft_speed = 0.0

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def slow_decay(self):
        self._log.info('set slow-decay mode.')
        self._motor_pfwd.decay_mode(SLOW_DECAY)
        self._motor_sfwd.decay_mode(SLOW_DECAY)
        self._motor_paft.decay_mode(SLOW_DECAY)
        self._motor_saft.decay_mode(SLOW_DECAY)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def fast_decay(self):
        self._log.info('set fast-decay mode.')
        self._motor_pfwd.decay_mode(FAST_DECAY)
        self._motor_sfwd.decay_mode(FAST_DECAY)
        self._motor_paft.decay_mode(FAST_DECAY)
        self._motor_saft.decay_mode(FAST_DECAY)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def accelerate(self, speed=DEFAULT_SPEED):
        '''
        Accelerate from zero to the provided speed. This assumes the current
        speed of all motors is zero.
        '''
        self._log.info('accelerate to speed: {}.'.format(speed))
        for _speed in MotorController._frange(0.0, speed, self._delta):
            self._log.debug('> speed: {}'.format(_speed))
            self.set_speed(MotorController.PFWD, _speed)
            self.set_speed(MotorController.SFWD, -1.0 * _speed)
            self.set_speed(MotorController.PAFT, _speed)
            self.set_speed(MotorController.SAFT, -1.0 * _speed)
            utime.sleep(self._acceleration_delay)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def decelerate(self, target_speed=0.0):
        '''
        Decelerate from the current speed down to the target speed (if
        unspecified the default is zero). This assumes all motors are
        currently operating at the same speed (we use PFWD as the exemplar).
        '''
        _current_speed = self._motor_pfwd_speed
        _delta = -1.0 * self._delta
        self._log.info('decelerate from current speed {} to target speed {} with delta {}.'.format(_current_speed, target_speed, _delta))
        for _speed in MotorController._frange(_current_speed, target_speed, _delta):
            self._log.debug('decelerate _speed: {}.'.format(_speed))
            self.set_speed(MotorController.PFWD, _speed)
            self.set_speed(MotorController.SFWD, -1.0 * _speed)
            self.set_speed(MotorController.PAFT, _speed)
            self.set_speed(MotorController.SAFT, -1.0 * _speed)
            utime.sleep(self._deceleration_delay)
        # just to be safe, end at stopped
        self.stop()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def all(self, speed=DEFAULT_SPEED, duration=None):
        self._log.info('all: speed={}; duration={}.'.format(speed, duration))
        self.set_speed(MotorController.PFWD, speed)
        self.set_speed(MotorController.SFWD, -1.0 * speed)
        self.set_speed(MotorController.PAFT, speed)
        self.set_speed(MotorController.SAFT, -1.0 * speed)
        self._execute_duration(duration)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def crab(self, speed=DEFAULT_SPEED, duration=None):
        self._log.info('crab: speed={}; duration={}.'.format(speed, duration))
        self.set_speed(MotorController.PFWD, speed)
        self.set_speed(MotorController.SFWD, speed)
        self.set_speed(MotorController.PAFT, -1.0 * speed)
        self.set_speed(MotorController.SAFT, -1.0 * speed)
        self._execute_duration(duration)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def rotate(self, speed=DEFAULT_SPEED, duration=None):
        self._log.info('rotate: speed={}; duration={}.'.format(speed, duration))
        # positive is counter-clockwise
        self.set_speed(MotorController.PFWD, -1.0 * speed)
        self.set_speed(MotorController.SFWD, -1.0 * speed)
        self.set_speed(MotorController.PAFT, -1.0 * speed)
        self.set_speed(MotorController.SAFT, -1.0 * speed)
        self._execute_duration(duration)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_speed(self, motor_id):
        if motor_id == MotorController.PFWD:
            return self._motor_pfwd_speed
        elif motor_id == MotorController.SFWD:
            return self._motor_sfwd_speed
        elif motor_id == MotorController.PAFT:
            return self._motor_paft_speed
        elif motor_id == MotorController.SAFT:
            return self._motor_saft_speed
        else:
            raise ValueError("unrecognised motor id '{}'".format(motor_id))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def set_speed(self, motor_id, speed):
        if motor_id == MotorController.PFWD:
            self._log.debug('pfwd motor set to {}'.format(speed))
            self._motor_pfwd_speed = speed
            self._motor_pfwd.speed(speed)
        elif motor_id == MotorController.SFWD:
            self._log.debug('sfwd motor set to {}'.format(speed))
            self._motor_sfwd_speed = speed
            self._motor_sfwd.speed(speed)
        elif motor_id == MotorController.PAFT:
            self._log.debug('paft motor set to {}'.format(speed))
            self._motor_paft_speed = speed
            self._motor_paft.speed(speed)
        elif motor_id == MotorController.SAFT:
            self._log.debug('saft motor set to {}'.format(speed))
            self._motor_saft_speed = speed
            self._motor_saft.speed(speed)
        else:
            raise ValueError("unrecognised motor id '{}'".format(motor_id))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _execute_duration(self, duration=None):
        if duration:
            utime.sleep(duration)
            self._motor_pfwd.speed(0.0)
            self._motor_sfwd.speed(0.0)
            self._motor_paft.speed(0.0)
            self._motor_saft.speed(0.0)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def _frange(start=0.0, stop=1.0, jump=0.1):
        if jump == 0:
            raise ValueError("jump argument cannot be zero.")
        if (start < stop and jump < 0) or (start > stop and jump > 0):
            raise ValueError("jump direction does not match the range.")

        # Calculate the number of steps depending on whether the range is increasing or decreasing
        if jump > 0:
            nsteps = int((stop - start) / jump)
            if (stop - start) % jump != 0:
                nsteps += 1
        else:
            nsteps = int((start - stop) / abs(jump))
            if (start - stop) % abs(jump) != 0:
                nsteps += 1

        # Generate the range with appropriate rounding
        return [round(start + float(i) * jump, 10) for i in range(nsteps)]

#       return [round(start + float(i) * jump, 10) for i in range(nsteps)]
#       return [round(start + float(i) * dy / nsteps, 10) for i in range(nsteps + 1)]
        # use rounding to avoid floating point math issues

#EOF
