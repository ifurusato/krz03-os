#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-04-25
# modified: 2025-05-07
#

import uasyncio as asyncio

import utime
from motor import FAST_DECAY, SLOW_DECAY, Motor, motor2040
from colorama import Fore, Style

from colors import*
from core.logger import Level, Logger
from response import (
    RESPONSE_INIT, RESPONSE_OKAY, RESPONSE_BAD_REQUEST, RESPONSE_OUT_OF_SYNC,
    RESPONSE_INVALID_CHAR, RESPONSE_PAYLOAD_TOO_LARGE, 
    RESPONSE_BUSY, RESPONSE_RUNTIME_ERROR, RESPONSE_UNKNOWN_ERROR
)

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class MotorController:
    NORMAL_DIR   = 0
    REVERSED_DIR = 1
    PFWD = 0
    SFWD = 1
    PAFT = 2
    SAFT = 3
    DEFAULT_SPEED = 0.5 # if speed is not specified on motor commands
    '''
    A motor controller for four motors.

    :param led:         the optional RGB LED
    :param level:       the log level
    '''
    def __init__(self, led=None, level=Level.INFO):
        super().__init__()
        self._log = Logger('motor_ctrl', level)
        self._led = led
        # create motors
        self._motor_pfwd = Motor(motor2040.MOTOR_A)
        self._motor_sfwd = Motor(motor2040.MOTOR_B, direction=MotorController.REVERSED_DIR)
        self._motor_paft = Motor(motor2040.MOTOR_C)
        self._motor_saft = Motor(motor2040.MOTOR_D, direction=MotorController.REVERSED_DIR)

        self._motor_pfwd.speed_scale(1.5)

        self._motor_pfwd_scale = self._motor_pfwd.speed_scale()
        self._motor_sfwd_scale = self._motor_sfwd.speed_scale()
        self._motor_paft_scale = self._motor_paft.speed_scale()
        self._motor_saft_scale = self._motor_saft.speed_scale()
        print("🌸 speed scale; pfwd: '{}'; sfwd: '{}'; paft: '{}'; saft: '{}'".format(self._motor_pfwd_scale, self._motor_sfwd_scale, self._motor_paft_scale, self._motor_saft_scale))

        self._motor_pfwd_zeropoint = self._motor_pfwd.zeropoint()
        self._motor_sfwd_zeropoint = self._motor_sfwd.zeropoint()
        self._motor_paft_zeropoint = self._motor_paft.zeropoint()
        self._motor_saft_zeropoint = self._motor_saft.zeropoint()
        print("🌸 zero point; pfwd: '{}'; sfwd: '{}'; paft: '{}'; saft: '{}'".format(self._motor_pfwd_zeropoint, self._motor_sfwd_zeropoint, self._motor_paft_zeropoint, self._motor_saft_zeropoint))

        # motor.deadzone(deadzone)
        # _deadzone = motor.deadzone() # default 0.05

#       self._motor_pfwd.zeropoint(0.0) 
#       self._motor_sfwd.zeropoint(0.0)
#       self._motor_paft.zeropoint(0.0)
#       self._motor_saft.zeropoint(0.0)

        self._motor_pfwd_speed = 0.0
        self._motor_sfwd_speed = 0.0 
        self._motor_paft_speed = 0.0
        self._motor_saft_speed = 0.0
        self._acceleration_delay = 0.1   # for acceleration or any loops
        self._deceleration_delay = 0.2   # for acceleration or any loops
        self._delta              = 0.025 # iterative delta
        self._processing_task    = None
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def enable(self):
        self._enabled = True
        self.motor_enable()
        self._log.info('enabled.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def disable(self):
        self._log.info('shutting down motor controller…')
        self._enabled = False
        self.motor_disable()
        self._log.info('disabled.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def show_color(self, color):
        '''
        Display the color on the RGB LED in GRB order.
        '''
        if self._led:
            self._led.set_rgb(0, color[0], color[1], color[2])

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def process_payload(self, payload):
        '''
        Immediately returns and delegates payload processing to an async task.
        '''
        if self._processing_task is None:
            self._processing_task = asyncio.create_task(self._async_process_payload(payload))
            self._log.debug('task created.')
            # ensure the event loop is running
            asyncio.get_event_loop().run_forever() # keep the event loop running
            self._log.info(Style.DIM + 'payload processing complete (0x4F).')
            return RESPONSE_OKAY #                 0x4F
        elif self._processing_task.done():
            self._processing_task = None
            print('🤖 TASK IS DONE; returning RESPONSE_OUT_OF_SYNC 0x73 --------------------------------- ')
            return RESPONSE_OUT_OF_SYNC #          0x73
        else:
            self._log.warning("Another task is already running. Skipping new payload (0x79).")
            return RESPONSE_BUSY #                 0x79

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    async def _async_process_payload(self, payload):
        '''
        Async motor command processor. Dispatches commands and handles delays.
        '''
        self._log.debug("process payload '{}'…".format(payload.to_string()))
        try:
            self.show_color(COLOR_YELLOW)
            _command, _speed, _stbd_speed, _duration = payload.values
            self._log.info("payload: cmd: '{}'; port: {}; stbd: {}; duration: {}".format(_command, _speed, _stbd_speed, _duration))
            if _command.startswith('enab'):
                self.enable()
            elif _command.startswith('disa'):
                self.disable()
            elif _command.startswith('stop'):
                self.stop()
            elif _command.startswith('coas'):
                self.coast()
            elif _command.startswith('brak'):
                self.brake()
            elif _command.startswith('slow'):
                self.slow_decay()
            elif _command.startswith('fast'):
                self.fast_decay()
            elif _command.startswith('acce'):
                self.accelerate(_speed)
            elif _command.startswith('dece'):
                self.decelerate(0.0)
            elif _command.startswith('forw'):
                self.forward(_speed, _stbd_speed)
            elif _command.startswith('reve'):
                self.reverse(_speed, _stbd_speed)
            elif _command.startswith('crab'):
                self.crab(_speed)
            elif _command.startswith('rota'):
                self.rotate(_speed)
            elif _command.startswith('wait'):
                self._log.info("waiting for {:.2f} seconds.".format(_duration))
                await asyncio.sleep(_duration)
                _duration = None # not again later
            elif _command == 'pfwd':
                self.pfwd(_speed)
            elif _command == 'sfwd':
                self.sfwd(_speed)
            elif _command == 'paft':
                self.paft(_speed)
            elif _command == 'saft':
                self.saft(_speed)
            else:
                self._log.error("unknown command: '{}'".format(_command))

            if _duration != None and _duration > 0.0:
                await asyncio.sleep(_duration)
                self.stop()
#               self._motor_pfwd.speed(0.0)
#               self._motor_sfwd.speed(0.0)
#               self._motor_paft.speed(0.0)
#               self._motor_saft.speed(0.0)

            self.show_color(COLOR_GREEN)

        except Exception as e:
            self._log.error("motor task error: {}".format(e))
            self.show_color(COLOR_RED)
            return RESPONSE_UNKNOWN_ERROR
        finally:
            self._processing_task = None

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def help(self):
        print(Fore.CYAN + '''
motor2040 commands:

    enable            enable all motors
    disable           disable all motors
    stop              stop all motors
    coast             coast all motors to stop
    brake             brake all motors to stop
    slow-decay        mode change
    fast-decay        mode change
    accelerate        accelerate from zero to speed
    decelerate        decelerate from speed to zero
    all [speed] [duration]     set speed of all motors
    crab [speed] [duration]    set crab speed
    rotate [speed] [duration]  set rotation speed
    pfwd [speed] [duration]    set speed of port-forward motor
    sfwd [speed] [duration]    set speed of stbd-forward motor
    paft [speed] [duration]    set speed of port-aft motor
    saft [speed] [duration]    set speed of stbd-aft motor

where 'speed' is 0.0-1.0 and 'duration' is the optional duration in seconds.
    ''' + Style.RESET_ALL)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
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
        _current_speed = self._motor_pfwd_speed
        _delta = -1.0 * self._delta
        self._log.info('decelerate from current speed {} to target speed {} with delta {}.'.format(_current_speed, target_speed, _delta))
        for _speed in MotorController._frange(_current_speed, target_speed, _delta):
            self._log.debug('decelerate _speed: {}.'.format(_speed))
            self.set_speed(MotorController.PFWD, _speed)
            self.set_speed(MotorController.SFWD, _speed)
            self.set_speed(MotorController.PAFT, _speed)
            self.set_speed(MotorController.SAFT, _speed)
            utime.sleep(self._deceleration_delay)
        # just to be safe, end at stopped
        self.stop()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def all(self, speed=DEFAULT_SPEED):
        self._log.info('all: speed={}.'.format(speed))
        self.set_speed(MotorController.PFWD, speed)
        self.set_speed(MotorController.SFWD, speed)
        self.set_speed(MotorController.PAFT, speed)
        self.set_speed(MotorController.SAFT, speed)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def forward(self, port_speed=DEFAULT_SPEED, stbd_speed=DEFAULT_SPEED):
        self._log.info('forward: port speed={}; stbd speed: {}.'.format(port_speed, stbd_speed))
        self.set_speed(MotorController.PFWD, port_speed)
        self.set_speed(MotorController.SFWD, stbd_speed)
        self.set_speed(MotorController.PAFT, port_speed)
        self.set_speed(MotorController.SAFT, stbd_speed)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def reverse(self, port_speed=DEFAULT_SPEED, stbd_speed=DEFAULT_SPEED):
        self._log.info('reverse: port speed={}; stbd speed: {}.'.format(port_speed, stbd_speed))
        self.set_speed(MotorController.PFWD, -1.0 * port_speed)
        self.set_speed(MotorController.SFWD, -1.0 * stbd_speed)
        self.set_speed(MotorController.PAFT, -1.0 * port_speed)
        self.set_speed(MotorController.SAFT, -1.0 * stbd_speed)

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
    @staticmethod
    def _frange(start=0.0, stop=1.0, jump=0.1):
        if jump == 0:
            raise ValueError("jump argument cannot be zero.")
        if (start < stop and jump < 0) or (start > stop and jump > 0):
            raise ValueError("jump direction does not match the range.")

        # calculate the number of steps depending on whether the range is increasing or decreasing
        if jump > 0:
            nsteps = int((stop - start) / jump)
            if (stop - start) % jump != 0:
                nsteps += 1
        else:
            nsteps = int((start - stop) / abs(jump))
            if (start - stop) % abs(jump) != 0:
                nsteps += 1

        # generate the range with appropriate rounding
        return [round(start + float(i) * jump, 10) for i in range(nsteps)]

#EOF
