#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-05-25
# modified: 2025-05-25
#

import sys
import utime
from math import ceil as ceiling
from machine import Timer
import uasyncio as asyncio
from motor import FAST_DECAY, SLOW_DECAY, Motor, motor2040
from colorama import Fore, Style

from colors import*
from core.logger import Level, Logger
from controller import Controller
from response import*

class MotorController(Controller):
    NORMAL_DIR   = 0
    REVERSED_DIR = 1
    PFWD = 0
    SFWD = 1
    PAFT = 2
    SAFT = 3
    DEFAULT_SPEED = 0.5 # if speed is not specified on motor commands
    '''
    A motor controller for four motors.

    Extends Controller with motor-specific and other commands.
    '''
    def __init__(self, display=None, level=Level.INFO):
        super().__init__(display=display, level=level)
        self._log = Logger('motor', level)
        self._motor_speed = 0
        self._motor_enabled = False

        # create motors
        self._motor_pfwd = Motor(motor2040.MOTOR_A, direction=MotorController.NORMAL_DIR)
        self._motor_sfwd = Motor(motor2040.MOTOR_B, direction=MotorController.REVERSED_DIR)
        self._motor_paft = Motor(motor2040.MOTOR_C, direction=MotorController.NORMAL_DIR)
        self._motor_saft = Motor(motor2040.MOTOR_D, direction=MotorController.REVERSED_DIR)

        self._motor_pfwd.speed_scale(1.5) # this motor is a bit slow

        self._motor_pfwd_scale = self._motor_pfwd.speed_scale()
        self._motor_sfwd_scale = self._motor_sfwd.speed_scale()
        self._motor_paft_scale = self._motor_paft.speed_scale()
        self._motor_saft_scale = self._motor_saft.speed_scale()
#       self._log.info("speed scale; pfwd: '{}'; sfwd: '{}'; paft: '{}'; saft: '{}'".format(self._motor_pfwd_scale, self._motor_sfwd_scale, self._motor_paft_scale, self._motor_saft_scale))

        self._motor_pfwd_zeropoint = self._motor_pfwd.zeropoint()
        self._motor_sfwd_zeropoint = self._motor_sfwd.zeropoint()
        self._motor_paft_zeropoint = self._motor_paft.zeropoint()
        self._motor_saft_zeropoint = self._motor_saft.zeropoint()
#       self._log.info("zero point; pfwd: '{}'; sfwd: '{}'; paft: '{}'; saft: '{}'".format(self._motor_pfwd_zeropoint, self._motor_sfwd_zeropoint, self._motor_paft_zeropoint, self._motor_saft_zeropoint))

        # motor.deadzone(deadzone)
        # _deadzone = motor.deadzone() # default 0.05
#       self._motor_pfwd.zeropoint(0.0) 
#       self._motor_sfwd.zeropoint(0.0)
#       self._motor_paft.zeropoint(0.0)
#       self._motor_saft.zeropoint(0.0)

        self._motor_pfwd_speed   = 0.0
        self._motor_sfwd_speed   = 0.0 
        self._motor_paft_speed   = 0.0
        self._motor_saft_speed   = 0.0
        self._acceleration_delay = 0.08  # for acceleration or any loops
        self._deceleration_delay = 0.15  # for acceleration or any loops
        self._delta              = 0.020 # iterative delta
        self._processing_task    = None
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def enable(self):
        self._enabled = True
        self.motor_enable()
        super().enable()
        self._log.info('enabled.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def disable(self):
        self._log.info('shutting down motor controller…')
        self._enabled = False
        self.motor_disable()
        super().disable()
        self._log.info('disabled.')

    async def handle_command(self, command):
        '''
        Extended async processor for motor-specific commands.
        '''
#       self._log.debug("handling command: '{}'".format(command))
        try:

            # asynchronous wait ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
            if command.startswith('wait'):
                self.show_color(COLOR_VIOLET)
                _duration = self._parse_duration(command, default=5)
                self._log.info("waiting for {:.2f} seconds.".format(_duration))
                await asyncio.sleep(_duration)
                self.show_color(COLOR_DARK_VIOLET)
            else:
                # parse command into arguments
                command, _port_speed, _stbd_speed, _duration = self.parse_command(command)
                if command.startswith('help'):
                    self.help()
                elif command.startswith('enab'):
                    self.enable()
                elif command.startswith('disa'):
                    self.disable()
                elif command.startswith('stop'):
                    self.stop()
                elif command.startswith('coas'):
                    self.coast()
                elif command.startswith('brak'):
                    self.brake()
                elif command.startswith('slow'):
                    self.slow_decay()
                elif command.startswith('fast'):
                    self.fast_decay()
                elif command.startswith('acce'):
                    self.accelerate(_port_speed)
                elif command.startswith('dece'):
                    self.decelerate(0.0)
                elif command.startswith('go'):
                    self.go(_port_speed, _stbd_speed)
                elif command.startswith('crab'):
                    self.crab(_port_speed)
                elif command.startswith('rota'):
                    self.rotate(_port_speed)

                # set some colors ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
                elif command == 'red':
                    self.show_color(COLOR_RED)
                elif command == 'green':
                    self.show_color(COLOR_GREEN)
                elif command == 'blue':
                    self.show_color(COLOR_BLUE)
                elif command == 'cyan':
                    self.show_color(COLOR_CYAN)
                elif command == 'magenta':
                    self.show_color(COLOR_MAGENTA)
                elif command == 'yellow':
                    self.show_color(COLOR_YELLOW)
                elif command == 'black':
                    self.show_color(COLOR_BLACK)

                # start and stop a timer ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
                elif command == 'start-timer':
                    self.startTimer()
                elif command == 'stop-timer':
                    self.stopTimer()
                else:
                    # delegate to base class if not processed ┈┈┈┈┈┈┈┈
                    await super().handle_command(command)
                if _duration != None and _duration > 0.0:
#                   self._log.info("processing duration: {:.2f}s".format(_duration))
                    await asyncio.sleep(_duration)
                    self.stop()

        except Exception as e:
            self._log.error("MotorController error: {}".format(e))
            sys.print_exception(e)
            self.show_color(COLOR_RED)
            return RESPONSE_UNKNOWN_ERROR
        finally:
            self._processing_task = None

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def parse_command(self, command):
        """
        Parses a command string into:
        - base command (str),
        - port_speed (float),
        - stbd_speed (float),
        - duration (float)

        Returns a tuple: (command, port_speed, stbd_speed, duration)
        """
        DEFAULT_SPEED    = 0.5
        DEFAULT_DURATION = 0.0
        parts = command.strip().split()
        if not parts:
            raise ValueError("Empty command")
        # base command like "go", "stop", etc.
        _command = parts[0].lower()
        # parse optional arguments if present
        try:
            _port_speed = float(parts[1]) if len(parts) > 1 else DEFAULT_SPEED
            _stbd_speed = float(parts[2]) if len(parts) > 2 else DEFAULT_SPEED
            _duration   = float(parts[3]) if len(parts) > 3 else DEFAULT_DURATION
        except ValueError:
            raise ValueError("Command arguments must be valid float values")
        # validate ranges
        if not -1.0 <= _port_speed <= 1.0:
            raise ValueError("port speed out of range (-1.0 to 1.0)")
        if not -1.0 <= _stbd_speed <= 1.0:
            raise ValueError("starboard speed out of range (-1.0 to 1.0)")
        if not 0.0 <= _duration <= 99.0:
            raise ValueError("duration out of range (0.0 to 99.0)")
        return _command, _port_speed, _stbd_speed, _duration

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _parse_duration(self, arg, default=5):
        try:
            parts = arg.strip().split()
            if len(parts) >= 2:
                return int(parts[1])
        except ValueError:
            pass
        return default

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def validated(self):
#       self._log.debug("validated.")
        if self._timer:
            self.stopTimer()
        super().validated()

    # help ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def help(self):
        print(Fore.CYAN + '''
motor controller commands:

    help                  prints this help
   
    enable                enable all motors
    disable               disable all motors
    stop                  stop all motors
    coast                 coast all motors to stop
    brake                 brake all motors to stop
    slow-decay            mode change
    fast-decay            mode change
    accelerate            accelerate from zero to speed
    decelerate            decelerate from speed to zero
    go [speed] [dur]      set differential speed of all motors
    crab [speed] [dur]    set crab speed of all motors
    rotate [speed] [dur]  set rotation speed of all motors

    start                 start a timer that blinks the LED
    stop                  stop the timer
    red                   set the RGB LED to red
    green                 set the RGB LED to green
    blue                  set the RGB LED to blue
    black                 set the RGB LED to black (off)
    wait [n]              asynchronously wait n seconds (default 5)

    ''' + Style.RESET_ALL)

    # motor control ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def motor_enable(self):
        self._motor_pfwd.enable()
        self._motor_sfwd.enable()
        self._motor_paft.enable()
        self._motor_saft.enable()
        self._log.info('enabled.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def motor_disable(self):
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
            self.set_speed(MotorController.SFWD, _speed)
            self.set_speed(MotorController.PAFT, _speed)
            self.set_speed(MotorController.SAFT, _speed)
            utime.sleep(self._acceleration_delay)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def decelerate(self, target_speed=0.0):
        '''
        Decelerate from the current speed down to the target speed (if
        unspecified the default is zero). This assumes all motors are
        currently operating at the same speed (we use PFWD as the exemplar).
        '''
#       self._log.info("decel speeds; pfwd: '{:.2f}'; sfwd: '{:.2f}'; paft: '{:.2f}'; saft: '{:.2f}'".format(self._motor_pfwd_speed, self._motor_sfwd_speed, self._motor_paft_speed, self._motor_saft_speed))
        _current_speed = self._motor_pfwd_speed # we just choose one arbitrarily
        self._log.info('decelerate from current speed {:.2f} to target speed {:.2f} with delta {:.2f}.'.format(_current_speed, target_speed, (-1.0 * self._delta)))
        for _speed in MotorController._frange(_current_speed, target_speed, -1.0 * self._delta):
#           self._log.info('decelerate _speed: {}.'.format(_speed))
            self.set_speed(MotorController.PFWD, _speed)
            self.set_speed(MotorController.SFWD, _speed)
            self.set_speed(MotorController.PAFT, _speed)
            self.set_speed(MotorController.SAFT, _speed)
            utime.sleep(self._deceleration_delay)
        # just to be safe, end at stopped
#       self._log.info('calling stop from decel.')
        self.stop()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def all(self, speed=DEFAULT_SPEED):
        self._log.info('all: speed={}.'.format(speed))
        self.set_speed(MotorController.PFWD, speed)
        self.set_speed(MotorController.SFWD, speed)
        self.set_speed(MotorController.PAFT, speed)
        self.set_speed(MotorController.SAFT, speed)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def go(self, port_speed=DEFAULT_SPEED, stbd_speed=DEFAULT_SPEED):
        self._log.info('go: port speed={}; stbd speed: {}.'.format(port_speed, stbd_speed))
        self.set_speed(MotorController.PFWD, port_speed)
        self.set_speed(MotorController.SFWD, stbd_speed)
        self.set_speed(MotorController.PAFT, port_speed)
        self.set_speed(MotorController.SAFT, stbd_speed)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def crab(self, speed=DEFAULT_SPEED):
        self._log.info('crab: speed={}.'.format(speed))
        self.set_speed(MotorController.PFWD, speed)
        self.set_speed(MotorController.SFWD, -1.0 * speed)
        self.set_speed(MotorController.PAFT, -1.0 * speed)
        self.set_speed(MotorController.SAFT, speed)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def rotate(self, speed=DEFAULT_SPEED):
        self._log.info('rotate: speed={}.'.format(speed))
        # positive is counter-clockwise
        self.set_speed(MotorController.PFWD, -1.0 * speed)
        self.set_speed(MotorController.SFWD, speed)
        self.set_speed(MotorController.PAFT, -1.0 * speed)
        self.set_speed(MotorController.SAFT, speed)

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
#           self._log.info('pfwd motor set to {:4.2f}'.format(speed))
            self._motor_pfwd_speed = speed
            self._motor_pfwd.speed(speed)
        elif motor_id == MotorController.SFWD:
#           self._log.info('sfwd motor set to {:4.2f}'.format(speed))
            self._motor_sfwd_speed = speed
            self._motor_sfwd.speed(speed)
        elif motor_id == MotorController.PAFT:
#           self._log.info('paft motor set to {:4.2f}'.format(speed))
            self._motor_paft_speed = speed
            self._motor_paft.speed(speed)
        elif motor_id == MotorController.SAFT:
#           self._log.info('saft motor set to {:4.2f}'.format(speed))
            self._motor_saft_speed = speed
            self._motor_saft.speed(speed)
        else:
            raise ValueError("unrecognised motor id '{}'".format(motor_id))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    @staticmethod
    def _frange(start=0.0, stop=1.0, step=0.1):
        if step == 0:
            raise ValueError("step argument cannot be zero.")
        if (start < stop and step < 0) or (start > stop and step > 0):
            raise ValueError("step direction does not match the range.")
        # Use math.ceil to ensure we include the stop (or slightly exceed it due to rounding)
        nsteps = int(ceiling((stop - start) / step))
        # generate list with rounding to avoid floating point precision issues
        return [round(start + i * step, 10) for i in range(nsteps)]

    # start and stop a timer ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def startTimer(self):
        if not self._timer:
            self._log.info('timer: ' + Fore.GREEN + 'start')
            self._timer = Timer()
            self._timer.init(period=1000, mode=Timer.PERIODIC, callback=self._toggle_led)
        else:
            self._log.warning('timer already started.')

    def _toggle_led(self, arg):
        self._on = not self._on
        if self._on:
            self.show_color(COLOR_DARK_CYAN)
            utime.sleep_ms(50)
            self.show_color(COLOR_BLACK)
        else:
            pass

    def stopTimer(self):
        if self._timer:
            self._log.info('timer: ' + Fore.GREEN + 'stop')
            self._timer.deinit()
        else:
            self._log.warning('timer already stopped.')
        self._timer = None

#EOF
