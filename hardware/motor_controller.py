#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2020-10-05
# modified: 2024-11-12
#
# A controller for four motors, based on the Pimoroni Inventor HAT Mini.
#

import sys, traceback
import time
import math
import statistics
import itertools
from typing import List, Optional, Union
from math import isclose
from threading import Thread
from colorama import init, Fore, Style
init()

from ioexpander.common import NORMAL_DIR, REVERSED_DIR
from inventorhatmini import InventorHATMini, MOTOR_A, MOTOR_B

import core.globals as globals
globals.init()

from core.component import Component
from core.direction import Direction
from core.orientation import Orientation
from core.rate import Rate
from core.logger import Logger, Level
from hardware.motor import Motor
from hardware.speed_dto_factory import SpeedDTOFactory

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class MotorController(Component):
    '''
    The controller for 4 motors:

        pfwd: Port-Forward        sfwd: Starboard-Forward
        paft: Port-Aft            saft: Starboard-Aft

    These are provided as the Pimoroni Motor implementation, as well as
    wrappers around them.

    Note that currently the Inventor HAT Minis must be supplied in the
    constructor, they oddly don't seem to work otherwise.

    :param config:            the YAML based application configuration
    :param fwd_controller:    if forward Inventor HAT Mini
    :param aft_controller:    if aft Inventor HAT Mini
    :param enable_pid:        if not None will override configuration
    :param suppressed:        if True the configurer is suppressed
    :param enabled:           if True the configurer is enabled upon instantiation
    :param level:             the logging Level
    '''
    def __init__(self, config, fwd_controller=None, aft_controller=None, enable_pid=None, suppressed=False, enabled=True, level=Level.INFO):
        if not isinstance(level, Level):
            raise ValueError('wrong type for log level argument: {}'.format(type(level)))
        self._log = Logger("motor-ctrl", level)
        Component.__init__(self, self._log, suppressed=False, enabled=True)
        if config is None:
            raise ValueError('no configuration provided.')
        _cfg = config['krzos'].get('motor_controller')
        # config ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._loop_freq_hz   = _cfg.get('loop_freq_hz') # main loop frequency
        self._loop_delay_sec = 1 / self._loop_freq_hz
        self._rate           = Rate(self._loop_freq_hz, Level.ERROR)
        self._log.info('loop frequency:\t{}Hz ({:4.2f}s)'.format(self._loop_freq_hz, self._loop_delay_sec))
        self._accel_decel_steps = _cfg.get('accel_decel_steps') # 50
        self._coasting_steps = _cfg.get('coasting_steps') # 150
        self._braking_steps  = _cfg.get('braking_steps')  # 50
        self._stopping_steps = _cfg.get('stopping_steps') # 10
        self._pause          = _cfg.get('pause') # 0.05
        self._speed_scale    = _cfg.get('speed_scale') # 5.4
        if enable_pid is not None:
            self._enable_pid = enable_pid
        else:
            self._enable_pid = _cfg.get('enable_pid')
        # motor configurer ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._log.info('creating aft motor configurer…')
        if aft_controller is None:
            self._aft_controller  = InventorHATMini(address=0x17, init_motors=True, init_servos=False, init_leds=False)
        else:
            self._aft_controller  = aft_controller
        self._aft_controller.enable_motors()
        self._log.info('creating fwd motor configurer…')
        if fwd_controller is None:
            self._fwd_controller  = InventorHATMini(address=0x16, init_motors=True, init_servos=False, init_leds=False)
        else:
            self._fwd_controller  = fwd_controller
        self._fwd_controller.enable_motors()
        # motors ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        if self._enable_pid:
            self._log.info(Fore.WHITE + 'creating motors and encoders with PID support…')
        else:
            self._log.info(Fore.WHITE + 'creating motors…')
        # pfwd motor ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._pfwd_mtr = self._fwd_controller.motors[MOTOR_A]
        self._pfwd_mtr.direction(REVERSED_DIR)
        if self._enable_pid:
            self._pfwd_enc = self._fwd_controller.encoders[MOTOR_A]
            self._pfwd_enc.direction(REVERSED_DIR) 
#           self._pfwd_mtr.speed_scale(self._speed_scale)
        else:
            self._pfwd_enc = None
        self._pfwd_mtr.enable()
        # sfwd motor ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._sfwd_mtr = self._fwd_controller.motors[MOTOR_B]
        if self._enable_pid:
            self._pfwd_enc = self._fwd_controller.encoders[MOTOR_B]
#           self._sfwd_mtr.speed_scale(self._speed_scale)
        else:
            self._pfwd_enc = None
        self._sfwd_mtr.enable()
        # paft motor ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._paft_mtr = self._aft_controller.motors[MOTOR_A]
        self._paft_mtr.direction(REVERSED_DIR)
        if self._enable_pid:
            self._paft_enc = self._aft_controller.encoders[MOTOR_A]
            self._paft_enc.direction(REVERSED_DIR) 
#           self._paft_mtr.speed_scale(self._speed_scale)
        else:
            self._paft_enc = None
        self._paft_mtr.enable()
        # saft motor ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._saft_mtr = self._aft_controller.motors[MOTOR_B]
        if self._enable_pid:
            self._saft_enc = self._aft_controller.encoders[MOTOR_B]
#           self._saft_mtr.speed_scale(self._speed_scale)
        else:
            self._saft_enc = None
        self._saft_mtr.enable()
        self._all_mtrs = self._bundle_mtrs()
        # Motor wrappers ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._pfwd_motor = None
        self._sfwd_motor = None
        self._paft_motor = None
        self._saft_motor = None
        self._log.info('assigning motors…')
        self._pfwd_motor = Motor(config, orientation=Orientation.PFWD,
                motor=self._pfwd_mtr, encoder=self._pfwd_enc, enable_pid=self._enable_pid, level=Level.INFO)
        self._sfwd_motor = Motor(config, orientation=Orientation.SFWD,
                motor=self._sfwd_mtr, encoder=self._pfwd_enc, enable_pid=self._enable_pid, level=Level.INFO)
        self._paft_motor = Motor(config, orientation=Orientation.PAFT,
                motor=self._paft_mtr, encoder=self._paft_enc, enable_pid=self._enable_pid, level=Level.INFO)
        self._saft_motor = Motor(config, orientation=Orientation.SAFT,
                motor=self._saft_mtr, encoder=self._saft_enc, enable_pid=self._enable_pid, level=Level.INFO)
        self._all_motors = self._bundle_motors()
        # variables ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._verbose       = False
        self._is_daemon     = True
        self._loop_thread   = None
        self._loop_enabled  = False
        # finish up…
        self._log.info('ready with {} motors.'.format(len(self._all_motors)))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def print_stats(self):
        pass
#       self._aft_controller.
#       self._fwd_controller.
#       voltage = board.read_voltage()
#       current_A = board.read_motor_current(MOTOR_A)
#       current_B = board.read_motor_current(MOTOR_B)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_mtr(self, orientation):
        '''
        Return the underlying motors for the provided Orientation.
        '''
        return self._all_mtrs[orientation]

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_mtrs(self):
        '''
        Returns a dictionary containing all instantiated underlying motors,
        keyed by Orientation.
        '''
        return self._all_mtrs

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _bundle_mtrs(self):
        '''
        Returns a dictionary of all extant Motors.
        '''
        _all_motors = {
            Orientation.PFWD:  self._pfwd_mtr,
            Orientation.SFWD:  self._sfwd_mtr,
            Orientation.PAFT:  self._paft_mtr,
            Orientation.SAFT:  self._saft_mtr,
        }
        return _all_motors

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_motors(self):
        '''
        Returns a dictionary containing all instantiated Motors, keyed by
        Orientation.
        '''
        return self._all_motors

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _bundle_motors(self):
        '''
        Returns a dictionary of all extant Motors.
        '''
        _all_motors = {
            Orientation.PFWD:  self._pfwd_motor,
            Orientation.SFWD:  self._sfwd_motor,
            Orientation.PAFT:  self._paft_motor,
            Orientation.SAFT:  self._saft_motor,
        }
        return _all_motors

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_motor(self, orientation):
        '''
        Returns a list containing the motor (or motors) corresponding to
        the orientation.
        '''
        if orientation is Orientation.ALL:
            return [ self._pfwd_motor, self._sfwd_motor, self._paft_motor, self._saft_motor ]
        elif orientation is Orientation.PORT:
            return [ self._pfwd_motor, self._paft_motor ]
        elif orientation is Orientation.STBD:
            return [ self._sfwd_motor, self._saft_motor ]
        elif orientation is Orientation.PFWD:
            return [ self._pfwd_motor ]
        elif orientation is Orientation.SFWD:
            return [ self._sfwd_motor ]
        elif orientation is Orientation.PAFT:
            return [ self._paft_motor ]
        elif orientation is Orientation.SAFT:
            return [ self._saft_motor ]
        else:
            raise Exception('unsupported orientation.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def coast(self):
        '''
        Coasts the robot to a stop rather casually.
        '''
        self.set_motor_speed(SpeedDTOFactory.create(Direction.NO_CHANGE, speed=0.0), steps=self._coasting_steps)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def brake(self):
        '''
        Stops the robot quickly but not immediately.
        '''
        self.set_motor_speed(SpeedDTOFactory.create(Direction.NO_CHANGE, speed=0.0), steps=self._braking_steps)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def stop(self):
        '''
        Stops the robot almost immediately.
        '''
        self.set_motor_speed(SpeedDTOFactory.create(Direction.NO_CHANGE, speed=0.0), steps=self._stopping_steps)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def set_motor_speed(self, dto, steps=None):
        # coast self.set_speed(orientation, 0.0, steps=self._coasting_steps)
        if self._enable_pid:
            raise Exception('pid not supported.')
        else:
            if steps is None: # then use default
                steps = self._accel_decel_steps
            _is_no_change = dto.direction == Direction.NO_CHANGE

            # Flag to indicate when to break out of both loops
            break_flag = False

            # determine the difference between target and current speed
            for i in range(steps + 1):
                for _motor in self._all_motors.values():
                    _target_speed = dto.get(_motor.orientation)
                    speed_diff = _target_speed - _motor.current_speed
                    # calculate the smooth transition using a cosine function
                    step_speed = 0.5 * (1 - math.cos(math.pi * i / steps)) * speed_diff + _motor.current_speed
                    if _is_no_change and math.isclose(abs(step_speed), 0.0, abs_tol=1e-2):
                        self._log.info(Fore.MAGENTA + 'break on step_speed isclose: {:6.3f}'.format(step_speed))
                        # FIXME use deadzone instead
                        _motor.speed = 0.0
                        break_flag = True # break out of both loops
                        break
                    else:
                        _motor.speed = step_speed
                    if _is_no_change:
                        self._log.info(Fore.WHITE + f"adjusting speed: {step_speed:.3f}")
                if break_flag:
                    break
                time.sleep(self._pause)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def enable(self):
        '''
        Enables the motors. This issues a warning if already enabled, but
        no harm is done in calling it repeatedly.
        '''
        if self.enabled:
            self._log.warning('already enabled.')
        else:
            Component.enable(self)
            if self._enable_pid and not self.loop_is_running:
                self._start_loop()
            self._log.info('enabled.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _start_loop(self): 
        '''
        Start the loop Thread.

        If we're using an external clock, calling this method throws an exception.
        '''
        self._log.info('start motor control loop…')
        if not self.enabled:
            raise Exception('not enabled.')
        if self.loop_is_running:
            self._log.warning('loop already running.')
        elif self._loop_thread is None:
            if self._external_clock:
                raise Exception('cannot use thread-based loop: external clock enabled.')
            self._loop_enabled = True
            self._loop_thread = Thread(name='motor_loop', target=MotorController._motor_loop, args=[self, lambda: self._loop_enabled], daemon=self._is_daemon)
            self._loop_thread.start()
            self._log.info('loop enabled.')
        else:
            raise Exception('cannot enable loop: thread already exists.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _stop_loop(self):
        '''
        Stop the motor control loop.
        '''
        if self.loop_is_running:
            self._loop_enabled = False
            self._loop_thread  = None
            self._log.info(Style.BRIGHT + 'stopped motor control loop.')
        else:
            self._log.warning('motor control loop already disabled.')
            
    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def loop_is_running(self):
        '''
        Returns true if using an external clock or if the loop thread is alive.
        '''
        return self._loop_enabled and self._loop_thread != None and self._loop_thread.is_alive()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _motor_loop(self, f_is_enabled):
        '''
        The motors loop, which executes while the flag argument lambda is True.
        '''
        self._log.info('loop start.')
        self._log.info(Style.BRIGHT + 'loop start.')
        try:
            while f_is_enabled():
                # execute any callback here…
                for _motor in self._all_motors:
                    self._log.info('updating {} motor…'.format(_motor.orientation.name))
                    _motor.update_target_speed()
                self._state_change_check()
                self._rate.wait()
        except Exception as e:
            self._log.error('error in loop: {}\n{}'.format(e, traceback.format_exc()))
        finally:
            self._log.info(Fore.GREEN + 'exited motor control loop.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _state_change_check(self):
        '''
        Check if the stopped/moving state has changed since the last call.
        '''
        pass

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def disable(self):
        '''
        Disable the motors.
        '''
        if self.enabled:
            Component.disable(self)
            self._log.info('disabled.')
        else:
            self._log.debug('already disabled.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def close(self):
        '''
        Closes the motor configurer.
        '''
        if not self.closed:
            Component.close(self) # calls disable
            self._log.info('motor configurer closed.')
        else:
            self._log.warning('motor configurer already closed.')

#EOF
