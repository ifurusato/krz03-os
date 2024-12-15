#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2020-10-05
# modified: 2024-10-31
#

import sys, traceback
import time
import statistics
import itertools
from typing import List, Optional, Union
from math import isclose
from threading import Thread
from colorama import init, Fore, Style
init()

import core.globals as globals
globals.init()

from core.component import Component
from core.direction import Direction
from core.orientation import Orientation
from core.rate import Rate
from core.logger import Logger, Level
from hardware.picon_zero import PiconZero
from hardware.pz_motor import Motor
from hardware.pigpiod_util import PigpiodUtility as PigUtil

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class MotorController(Component):
    '''
    The controller for 4 motors:

        pfwd: Port-Forward        sfwd: Starboard-Forward
        paft: Port-Aft            saft: Starboard-Aft

    :param config:            the YAML based application configuration
    :param suppressed         if True the controller is suppressed
    :param enabled            if True the controller is enabled upon instantiation
    :param level:             the logging Level
    '''
    def __init__(self, config, suppressed=False, enabled=False, level=Level.INFO):
        if not isinstance(level, Level):
            raise ValueError('wrong type for log level argument: {}'.format(type(level)))
        self._log = Logger("motor-ctrl", level)
        Component.__init__(self, self._log, suppressed, enabled)
        if config is None:
            raise ValueError('no configuration provided.')
        _cfg = config['krzos'].get('motor_controller')
        # config ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._loop_freq_hz   = _cfg.get('loop_freq_hz') # main loop frequency
        self._loop_delay_sec = 1 / self._loop_freq_hz
        self._rate           = Rate(self._loop_freq_hz, Level.ERROR)
        self._log.info('loop frequency:\t{}Hz ({:4.2f}s)'.format(self._loop_freq_hz, self._loop_delay_sec))
        self._verbose        = False
        # make sure pigpiod is running prior to creating decoders
        PigUtil.ensure_pigpiod_is_running()
        # motor controller ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._fwd_pz = PiconZero(orientation=Orientation.FWD)
        self._fwd_pz.initialise()
        self._aft_pz = PiconZero(orientation=Orientation.AFT)
        self._aft_pz.initialise()
        self._pfwd_motor     = Motor(config, self._fwd_pz, Orientation.PFWD, Level.INFO)
        self._sfwd_motor     = Motor(config, self._fwd_pz, Orientation.SFWD, Level.INFO)
        self._paft_motor     = Motor(config, self._aft_pz, Orientation.PAFT, Level.INFO)
        self._saft_motor     = Motor(config, self._aft_pz, Orientation.SAFT, Level.INFO)
        self._all_motors     = self._get_motors()
        self._is_daemon      = True
        self._loop_thread    = None
        self._external_clock = None
        # finish up…
        self._log.info('ready with {} motors.'.format(len(self._all_motors)))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_picon_zero(self, orientation):
        if orientation is Orientation.FWD:
            return self._fwd_pz
        elif orientation is Orientation.AFT:
            return self._aft_pz
        else:
            raise ValueError('expected FWD or AFT orientation.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_motors(self):
        '''
        Returns a dictionary containing all instantiated motors, keyed by
        Orientation.
        '''
        return self._all_motors

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _get_motors(self):
        '''
        Returns a dictionary of all extant motors.
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
        Returns the motor corresponding to the orientation.
        '''
        if orientation is Orientation.PFWD:
            return self._pfwd_motor
        elif orientation is Orientation.SFWD:
            return self._sfwd_motor
        elif orientation is Orientation.PAFT:
            return self._paft_motor
        elif orientation is Orientation.SAFT:
            return self._saft_motor
        else:
            raise Exception('unsupported orientation.')

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
            self.stop() # default
            if self._external_clock:
                self._external_clock.add_callback(self.external_callback_method)
                for _motor in self._all_motors.values():
                    _motor.enable()
            else:
                self._loop_enabled = True
                self._loop_thread = Thread(name='motor_loop', target=MotorController._motor_loop, args=[self, lambda: self._loop_enabled], daemon=self._is_daemon)
                self._loop_thread.start()
                self._log.info(Fore.WHITE + 'motor loop enabled.')
            self._log.info('enabled.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _motor_loop(self, f_is_enabled):
        '''
        The motors loop, which executes while the flag argument lambda is True.
        '''
        self._log.info('loop start.')
        try:
            while f_is_enabled():
                # execute any callback here…
                for _motor in self._all_motors.values():
                    _motor.update_speed()
#               self._state_change_check()
                self._rate.wait()
        except Exception as e:
            self._log.error('error in loop: {}\n{}'.format(e, traceback.format_exc()))
        finally:
            self._log.info(Fore.GREEN + 'exited motor control loop.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def stop(self):
        self._log.info('stop.')
        self._pfwd_motor.speed = 0
        self._sfwd_motor.speed = 0
        self._paft_motor.speed = 0
        self._saft_motor.speed = 0
#       self._pfwd_motor.stop()
#       self._sfwd_motor.stop()
#       self._paft_motor.stop()
#       self._saft_motor.stop()
#       self._fwd_pz.stop()
#       self._aft_pz.stop()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def brake(self):
        _brake_scale = 80
        for _motor in self._all_motors.values():
            _motor.brake()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def reset(self):
        for _motor in self._all_motors.values():
            _motor.reset()

    # step limits ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def set_step_limit(self, orientations: Union[Orientation, List[Orientation]], step_limit: Optional[int]):
        '''
        Set the step_limit for the given Orientation. This allows for a
        single Orientation or a list of Orientations.
        '''
        if isinstance(orientations, Orientation):
            orientations = [orientations]  # convert to list for uniform handling
        if Orientation.ALL in orientations:
            # set or clear the step limit for all orientations except ALL itself
            for orientation in [Orientation.PFWD, Orientation.SFWD, Orientation.PAFT, Orientation.SAFT]:
                _motor = self._all_motors[orientation]
                if step_limit is None:
                    _motor.step_limit = None  # clear step limit
                else:
                    _motor.step_limit = step_limit
        else:
            for orientation in orientations:
                if isinstance(orientation, Orientation):
                    _motor = self._all_motors[orientation]
                    if step_limit is None:
                        # clear step limit for this orientation
                        _motor.step_limit = None
                    else:
                        _motor.step_limit = step_limit
                else:
                    raise ValueError("Invalid motor orientation provided.")
    
    def get_step_limit(self, orientation):
        '''
        Return the step limit for the specified motor, or None if the
        step limit for the motor is undefined.
        ''' 
        if orientation in self._all_motors.keys():
            return self._all_motors[orientation].step_limit
        return None

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def set_direction(self, direction, speed=0):
        if not self.enabled:
            raise Exception('motor controller not enabled.')
        if direction is Direction.STOPPED:             # (  0, 'stopped',           'stop', ' ')
            self._log.info('set direction: STOPPED.')
            self.stop()

        elif direction is Direction.AHEAD:             # (  1, 'ahead',             'ahed', 'w')
            self._log.info('set direction: AHEAD to {}.'.format(speed))
            self._pfwd_motor.speed = speed
            self._sfwd_motor.speed = speed
            self._paft_motor.speed = speed
            self._saft_motor.speed = speed

        elif direction is Direction.ASTERN:            # (  2, 'astern',            'astn', 'z')
            self._log.info('set direction: ASTERN.')
            self._pfwd_motor.speed = -1 * speed
            self._sfwd_motor.speed = -1 * speed
            self._paft_motor.speed = -1 * speed
            self._saft_motor.speed = -1 * speed

        elif direction is Direction.ROTATE_CW:         # (  3, 'rotate-cw',         'rtcw', 's')
            self._log.info('set direction: ROTATE CLOCKWISE.')
            self._pfwd_motor.speed = speed
            self._sfwd_motor.speed = -1 * speed
            self._paft_motor.speed = speed
            self._saft_motor.speed = -1 * speed

        elif direction is Direction.ROTATE_CCW:        # (  4, 'rotate-ccw',        'rtcc', 'a')
            self._log.info('set direction: ROTATE COUNTER-CLOCKWISE.')
            self._pfwd_motor.speed = -1 * speed
            self._sfwd_motor.speed = speed
            self._paft_motor.speed = -1 * speed
            self._saft_motor.speed = speed

        elif direction is Direction.CRAB_PORT:         # (  5, 'crab-port',         'crap', 'c')
            self._log.info('set direction: CRAB PORT.')
            self._pfwd_motor.speed = -1 * speed
            self._sfwd_motor.speed = speed
            self._paft_motor.speed = speed
            self._saft_motor.speed = -1 * speed

        elif direction is Direction.CRAB_STBD:         # (  6, 'crab-stbd',         'cras', 'v')
            self._log.info('set direction: CRAB STBD.')
            self._pfwd_motor.speed = speed
            self._sfwd_motor.speed = -1 * speed
            self._paft_motor.speed = -1 * speed
            self._saft_motor.speed = speed

        elif direction is Direction.DIAGONAL_PORT:     # (  7, 'diagonal-port',     'diap', 'd')
            self._log.info('set direction: DIAGONAL PORT.')
        elif direction is Direction.DIAGONAL_STBD:     # (  8, 'diagonal-stbd',     'dias', 'f')
            self._log.info('set direction: DIAGONAL STBD.')

        elif direction is Direction.PIVOT_FWD_CW:      # (  9, 'pivot-fwd-cw',      'pfcw', 'p')
            self._log.info('set direction: PIVOT FWD CW.')
            self._pfwd_motor.speed = speed
            self._sfwd_motor.speed = -1 * speed
            self._paft_motor.speed = 0
            self._saft_motor.speed = 0

        elif direction is Direction.PIVOT_FWD_CCW:     # ( 10, 'pivot-fwd-ccw',     'pfcc', 'o')
            self._log.info('set direction: PIVOT FWD CCW.')
            self._pfwd_motor.speed = -1 * speed
            self._sfwd_motor.speed = speed
            self._paft_motor.speed = 0
            self._saft_motor.speed = 0

        elif direction is Direction.PIVOT_AFT_CW:      # ( 11, 'pivot-aft-cw',      'pacw', 'l')
            self._log.info('set direction: PIVOT AFT.')
            self._pfwd_motor.speed = 0
            self._sfwd_motor.speed = 0
            self._paft_motor.speed = -1 * speed
            self._saft_motor.speed = -1 * speed

        elif direction is Direction.PIVOT_PORT_CW:     # ( 13, 'pivot-port-cw',     'ppcw', 'i')
            self._log.info('set direction: PIVOT PORT.')
            self._pfwd_motor.speed = -1 * speed
            self._sfwd_motor.speed = -1 * speed
            self._paft_motor.speed = -1 * speed
            self._saft_motor.speed = -1 * speed

        elif direction is Direction.PIVOT_STBD_CW:      #( 15, 'pivot-stbd-cw',     'pscw', 'j')
            self._log.info('set direction: PIVOT STBD.')
            self._pfwd_motor.speed = -1 * speed
            self._sfwd_motor.speed = -1 * speed
            self._paft_motor.speed = -1 * speed
            self._saft_motor.speed = -1 * speed

        else:                                          # ( 99, 'unknown',           'unkn', '?') # n/a or indeterminate
            self._log.info('set direction: UNKNOWN.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def set_motor_speed(self, orientation, value):
        '''
        If provided PORT or STBD, sets the speed of all motors associated with
        the port or starboard orientation, otherwise sets the speed of the
        specified motor. The "speed" here is either a target RPM if the PID
        controller is active, or a motor speed value from -100 to 100,
        effectively the set percentage of motor power.
        '''
        if not self.enabled:
            raise Exception('motor controller not enabled.')
        if not isinstance(value, int):
            raise ValueError('expected target speed as int, not {}'.format(type(value)))
        if orientation is Orientation.ALL:
            if self._verbose:
                self._log.info(Fore.MAGENTA + 'set ALL motor speed: {:5.2f}'.format(value))
            self._pfwd_motor.speed = value
            self._paft_motor.speed = value
            self._sfwd_motor.speed = value
            self._saft_motor.speed = value
        elif orientation is Orientation.PORT:
            if self._verbose:
                self._log.info(Fore.RED + 'set {} motor speed: {:5.2f}'.format(orientation.name, value))
            self._pfwd_motor.speed = value
            self._paft_motor.speed = value
        elif orientation is Orientation.STBD:
            if self._verbose:
                self._log.info(Fore.GREEN + 'set {} motor speed: {:5.2f}'.format(orientation.name, value))
            self._sfwd_motor.speed = value
            self._saft_motor.speed = value
        elif orientation is Orientation.PFWD:
            if self._verbose:
                self._log.info('set {} motor speed: {:5.2f}'.format(orientation.name, value))
            self._pfwd_motor.speed = value
        elif orientation is Orientation.SFWD:
            if self._verbose:
                self._log.info('set {} motor speed: {:5.2f}'.format(orientation.name, value))
            self._sfwd_motor.speed = value
        elif orientation is Orientation.PAFT:
            if self._verbose:
                self._log.info('set {} motor speed: {:5.2f}'.format(orientation.name, value))
            self._paft_motor.speed = value
        elif orientation is Orientation.SAFT:
            if self._verbose:
                self._log.info('set {} motor speed: {:5.2f}'.format(orientation.name, value))
            self._saft_motor.speed = value
        else:
            self._log.warning('expected a motor orientation, not {}'.format(orientation))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def disable(self):
        '''
        Disable the motors.
        '''
        self.stop()
        if self.enabled:
            Component.disable(self)
            self._log.info('disabled.')
        else:
            self._log.debug('already disabled.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def close(self):
        '''
        Closes the motor controller.
        '''
        if not self.closed:
            Component.close(self) # calls disable
            self._pfwd_motor.stop()
            self._sfwd_motor.stop()
            self._paft_motor.stop()
            self._saft_motor.stop()
            self._fwd_pz.close()
            self._aft_pz.close()
            self._log.info('motor controller closed.')
        else:
            self._log.warning('motor controller already closed.')

#EOF
