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

import sys, traceback
import time
import itertools
import argparse
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.direction import Direction
from core.orientation import Orientation
from core.rate import Rate
from core.config_loader import ConfigLoader
from hardware.motor_controller import MotorController
from hardware.digital_pot import DigitalPotentiometer

# constants ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈


# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
class MotorTest(object):
    IN_MIN  = 0.0 # minimum analog value from IO Expander
    IN_MAX  = 3.3 # maximum analog value from IO Expander
    '''
    A suite of motor tests.
    '''
    def __init__(self):

        _level = Level.INFO
        self._log = Logger('motor-test', _level)
        self._deadband = 5 # around zero
    
        # read YAML configuration
        _config = ConfigLoader(Level.INFO).configure()
        self._motor_controller = MotorController(_config)
        self._motor_controller.enable()
        self._delay_sec = 1.5

        self._speed_pot = DigitalPotentiometer(_config, 0x0C, level=Level.INFO)
        self._speed_pot.set_input_range(self.IN_MIN, self.IN_MAX)
        self._speed_pot.set_output_range(-100, 100)

        self._kx_pot = DigitalPotentiometer(_config, 0x0E, level=_level)
        self._kx_pot.set_input_range(self.IN_MIN, self.IN_MAX)
        self._kx_pot.set_output_range(0.0, 1.0)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def functional_test(self):
        self._log.info("Running functional test…")

        _orientations = [ Orientation.PORT, Orientation.STBD, Orientation.PFWD, Orientation.SFWD, Orientation.PAFT, Orientation.SAFT ]

        for _orientation in _orientations:
            if _orientation.side is Orientation.PORT:
                self._log.info(Fore.RED   + 'set motor speed for {} motor(s).'.format(_orientation) + Style.RESET_ALL)
            else:
                self._log.info(Fore.GREEN + 'set motor speed for {} motor(s).'.format(_orientation) + Style.RESET_ALL)
            self._motor_controller.set_motor_speed(_orientation, 50)
            time.sleep(self._delay_sec)
            self._motor_controller.set_motor_speed(_orientation, 0)
            time.sleep(3)
    
        self._motor_controller.stop()

#       self._log.info('set direction to AHEAD.')
#       self._motor_controller.set_direction(Direction.AHEAD, 20)
#       time.sleep(self._delay_sec)
#       self._motor_controller.set_direction(Direction.STOPPED)
#       time.sleep(0.33)
#   
#       self._log.info('set direction to ASTERN.')
#       self._motor_controller.set_direction(Direction.ASTERN, 20)
#       time.sleep(self._delay_sec)
#       self._motor_controller.set_direction(Direction.STOPPED)
#       time.sleep(0.33)
    
    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def encoder_test(self):
        self._log.info("Running encoder test…")

        self._log.info('DISABLING UNUSED MOTORS ------------------------------------------------------------')
        self._motor_controller.get_motor(Orientation.SFWD).disable()
        self._motor_controller.get_motor(Orientation.PAFT).disable()
        self._motor_controller.get_motor(Orientation.SAFT).disable()
        try:    
            self._log.info(Fore.GREEN + 'Starting loop: use pot to change speed, type Ctrl-C to exit.')
            while True:
                _target_rpm  = int(self._speed_pot.get_scaled_value(absolute_tolerance=self._deadband))
                self._motor_controller.set_motor_speed(Orientation.PFWD, _target_rpm)
                time.sleep(1)
        except KeyboardInterrupt:
            print('\n')
            self._log.info('Ctrl-C caught; exiting…')
        finally:
            self._motor_controller.set_motor_speed(Orientation.PFWD, 0)
            time.sleep(0.33)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def movement_test(self):
        self._log.info("Running movement test…")

        _directions = [ Direction.AHEAD ]
#       _directions = [ Direction.ASTERN ]
#       _directions = [ Direction.AHEAD, Direction.ASTERN ]
#           _target_rpm  = int(self._speed_pot.get_scaled_value(absolute_tolerance=self._deadband))
        _orientation = Orientation.ALL
#           self._motor_controller.set_direction(_direction, _target_rpm)
        try:
            for _direction in _directions:
                self._log.info(Fore.GREEN + 'Starting loop: use pot to change speed, type Ctrl-C to exit.')
                while True:
                    _target_rpm = int(self._speed_pot.get_scaled_value(absolute_tolerance=self._deadband))
                    self._motor_controller.set_motor_speed(_orientation, _target_rpm)
                    time.sleep(2)
                self._log.info('stop. ----------------------------------------- ')
                self._motor_controller.stop()
                time.sleep(2.0)
            self._motor_controller.reset()
            self._log.info('end of cycle. ------------------------------------- ')
        except KeyboardInterrupt:
            print('\n')
            self._log.info('Ctrl-C caught; exiting…')
        finally:
            self._motor_controller.set_motor_speed(Orientation.PFWD, 0)
            time.sleep(0.33)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def step_limit_test(self):
        self._log.info("Running step limit test…")

        _one_meter = 461
        _half_meter = int(_one_meter / 2)
        _two_meters = _one_meter * 2
        self._motor_controller.set_step_limit(Orientation.PFWD, _two_meters)
        try:
            self._log.info(Fore.GREEN + 'Starting loop: use pot to change speed, type Ctrl-C to exit.')
            while True:
                _target_rpm = int(self._speed_pot.get_scaled_value(absolute_tolerance=self._deadband))
                self._motor_controller.set_motor_speed(Orientation.PFWD, _target_rpm)
                time.sleep(0.5)
        except KeyboardInterrupt:
            print('\n')
            self._log.info('Ctrl-C caught; exiting…')
        finally:
            self._motor_controller.set_motor_speed(Orientation.PFWD, 0)
            time.sleep(0.33)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def crab_test(self):
        self._log.info("Running crab-wise movement test…")

        _directions = [ Direction.CRAB_PORT, Direction.CRAB_STBD ]
        for _direction in _directions:
            self._log.info('set direction to {}.'.format(_direction))
            self._motor_controller.set_direction(_direction, 90)
            time.sleep(3)
            self._motor_controller.set_direction(Direction.STOPPED)
            time.sleep(0.33)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def rotate_test(self):
        self._log.info("Running rotation test…")

        _directions = [ Direction.ROTATE_CW, Direction.ROTATE_CCW ]
        for _direction in _directions:
            self._log.info('set direction to {}.'.format(_direction))
            self._motor_controller.set_direction(_direction, 90)
            time.sleep(3)
            self._motor_controller.set_direction(Direction.STOPPED)
            time.sleep(0.33)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def pivot_test(self):
        self._log.info("Running pivot test…")

        _directions = [ Direction.PIVOT_FWD_CW, Direction.PIVOT_FWD_CCW ]
        for _direction in _directions:
            self._log.info('set direction to {}.'.format(_direction))
            self._motor_controller.set_direction(_direction, 30)
            time.sleep(3)
            self._motor_controller.set_direction(Direction.STOPPED)
            time.sleep(0.33)
    
    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def tune_pid(self):
        self._log.info("Tune PID controller…")
        try:

            _='''

            1. Set Initial Values

                Set Kp to a small value.
                Set Ki=0 and Kd=0 initially, so you’re only working with proportional control.

            2. Tune Kp (Proportional Gain)

                Gradually increase Kp until the system output starts to oscillate around the setpoint.
                The value of Kp at which the system just starts oscillating is the ultimate gain Ku.
                Measure the period of oscillation, Tu, which is the time between successive peaks.

            3. Apply Ziegler–Nichols Rules

                Use the values of Ku and Tu to set Kp, Ki, and Kd:
                    P Controller:   Kp = 0.5 × Ku
                    PI Controller:  Kp = 0.45 × Ku, Ki = Kp / (0.5 × Tu)
                    PID Controller: Kp = 0.6 × Ku, Ki = 2 × Kp / Tu, Kd = Kp × Tu / 8
            '''

            _orientation = Orientation.PFWD # the motor orientation (port forward)
            _counter = itertools.count()

            _deadband = 4
            _max_change = 5  # maximum change in RPM per update cycle


            _log.info(Fore.CYAN + 'setting speed to 70...')
            _motor_controller.set_motor_speed(_orientation, 70)
            time.sleep(7)

            _rate = Rate(20) # Hz

            self._log.info(Fore.GREEN + 'Starting loop: type Ctrl-C to exit.')
            while True:

                _count = next(_counter)

                # set PID controller constants
#   #           _Ku = _kx_pot.get_scaled_value()
#   #           _Ku = 0.38
#   #           _log.info(Fore.MAGENTA + 'Ku={:5.2f}'.format(_Ku))
#   #           _pid.kp = _Ku * 0.6
#   #           _pid.ki = _Ku * 0.1
#   #           _pid.kd = _Ku * 0.05

#               # constants
#   #           _pid.kp = 0.23 # _Ku * 0.6
#   #           _pid.ki = 0.03 # _Ku * 0.1
#   #           _pid.kd = 0.01 # _Ku * 0.05

#   #           _current_rpm = _motor.rpm
                _target_rpm  = int(self._speed_pot.get_scaled_value(absolute_tolerance=3))

                # apply deadband to the target RPM
#   #           _adjusted_speed = apply_rpm_deadband(_current_rpm, _target_rpm, _deadband)

                # apply slew limiting
#   #           _adjusted_speed = apply_slew_limit(_current_rpm, _adjusted_speed, _max_change)

                # print current RPM and target RPM for debugging
#   #           _log.info(Fore.CYAN + 'Current RPM: {:.2f}, Target RPM: {:.2f}, Deadband: {:.2f}'.format(_motor.rpm, _target_rpm, _deadband))

#   #           _log.info(Fore.MAGENTA + '_target_rpm={:5.2f}; adjusted: {:5.2f}'.format(_target_rpm, _adjusted_speed))

                _motor_controller.set_motor_speed(_orientation, _target_rpm)
                _rate.wait()


        except KeyboardInterrupt:
            print('\n')
            self._log.info('Ctrl-C caught; exiting…')
        finally:
            self._motor_controller.set_motor_speed(Orientation.PFWD, 0)
            time.sleep(0.33)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def nothing_test(self):
        self._log.info("Doing nothing…")
        try:
            self._log.info(Fore.GREEN + 'Starting loop: type Ctrl-C to exit.')
            while True:
                time.sleep(0.33)
        except KeyboardInterrupt:
            print('\n')
            self._log.info('Ctrl-C caught; exiting…')
        finally:
            self._motor_controller.set_motor_speed(Orientation.PFWD, 0)
            time.sleep(0.33)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def close(self):
        self._speed_pot.set_black()
        self._speed_pot.close()
        self._kx_pot.set_black()
        self._kx_pot.close()
        self._motor_controller.stop()
        self._motor_controller.close()
        self._log.info('closed.')


# main ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

_log = Logger('main', Level.INFO)

if __name__ == "__main__":

    _motor_test = None

    try:

        # parse args ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        _parser = argparse.ArgumentParser(description="Select a test to run, or use with -h for help.")
        _parser.add_argument('-c', '--crab',       action="store_true", help='a test of moving crab-wise')
        _parser.add_argument('-e', '--encoder',    action="store_true", help='a test of the motor encoder on PFWD')
        _parser.add_argument('-f', '--functional', action="store_true", help='a simple, functional test of all motors')
        _parser.add_argument('-m', '--movement',   action="store_true", help='a test that involves movement of the robot using all four motors')
        _parser.add_argument('-n', '--nothing',    action="store_true", help='a test that does nothing')
        _parser.add_argument('-p', '--pivot',      action="store_true", help='a test of a pivot movement')
        _parser.add_argument('-r', '--rotate',     action="store_true", help='a test of rotating in place')
        _parser.add_argument('-s', '--steplimit',  action="store_true", help='a test of the step limit feature')
        _parser.add_argument('-t', '--tune',       action="store_true", help='used for tuning the PID controller')

        _args = _parser.parse_args()

        # execute selected method ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        if any(vars(_args).values()):
            _motor_test = MotorTest()
        else:
            print('\n')

        _log.info("begin…")
        if _args.functional:
            _motor_test.functional_test()
        elif _args.encoder:
            _motor_test.encoder_test()
        elif _args.movement:
            _motor_test.movement_test()
        elif _args.steplimit:
            _motor_test.step_limit_test()
        elif _args.crab:
            _motor_test.crab_test()
        elif _args.rotate:
            _motor_test.rotate_test()
        elif _args.pivot:
            _motor_test.pivot_test()
        elif _args.tune:
            _motor_test.tune_pid()
        elif _args.nothing:
            _motor_test.nothing_test()
        else:
            _log.info("No test selected; use -h for help.")

    except KeyboardInterrupt:
        print('\n')
        _log.info('Ctrl-C caught from main; exiting…')
    except Exception as e:
         _log.error('{} encountered, exiting: {}\n{}'.format(type(e), e, traceback.format_exc()))
    finally:
        if _motor_test:
            _motor_test.close()
        print('\n')
        _log.info("complete.")

#EOF
