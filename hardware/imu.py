#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020 by Murray Altheim. All rights reserved. This file is part of
# the Robot Operating System project and is released under the "Apache Licence,
# Version 2.0". Please see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2020-05-19
# modified: 2024-10-31
#
# _Getch at bottom.
#

import asyncio
from datetime import datetime as dt
from colorama import init, Fore, Style
init()

import core.globals as globals
globals.init()

from core.message_factory import MessageFactory
from core.logger import Logger, Level
from core.event import Event
from core.publisher import Publisher

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class IMU(Publisher):

    CLASS_NAME = 'imu'
    _LISTENER_LOOP_NAME = '__imu_listener_loop'

    '''
    The IMU operates in an active mode, using an asyncio loop, as a publisher
    for events derived from the values of an IMU, or in a passive mode, where
    it can be queried for the general data output from the IMU.

    :param config:            the application configuration
    :param message_bus:       the asynchronous message bus
    :param message_factory:   the factory for creating messages
    :param level:             the log level
    '''
    def __init__(self, config, icm20948, message_bus, message_factory, level=Level.INFO):
        if not isinstance(level, Level):
            raise ValueError('wrong type for log level argument: {}'.format(type(level)))
        self._level = level
        if icm20948 is None:
            raise ValueError('null icm20948 argument.')
        self._icm20948 = icm20948
        if message_bus is None:
            raise ValueError('null message bus argument.')
        self._message_bus = message_bus
        if message_factory is None:
            raise ValueError('null message factory argument.')
        self._message_factory = message_factory
        Publisher.__init__(self, IMU.CLASS_NAME, config, message_bus, message_factory, level=self._level)
        # configuration ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        _cfg = config['krzos'].get('publisher').get('imu')
        self._active_mode         = _cfg.get('active_mode')
        _loop_freq_hz             = _cfg.get('loop_freq_hz')
        self._publish_delay_sec   = 1.0 / _loop_freq_hz
        self._pitch_threshold     = _cfg.get('pitch_threshold')
        self._roll_threshold      = _cfg.get('roll_threshold')
        self._initial_calibration = _cfg.get('initial_calibration')
        self._log.info('ready with loop frequency of {:d}Hz.'.format(_loop_freq_hz))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _suppress_sensors(self, suppress):
        '''
        Suppress bumper/IR sensors during manual calibration.
        '''
        _component_registry = globals.get('component-registry')
        _distance_sensors_publisher = _component_registry.get('pub:distance')
        if _distance_sensors_publisher:
            if suppress:
                self._log.info(Fore.WHITE + Style.BRIGHT + 'suppressing distance sensors publisher…')
                _distance_sensors_publisher.suppress()
            else:
                self._log.info(Fore.WHITE + Style.BRIGHT + 'releasing distance sensors publisher…')
                _distance_sensors_publisher.release()
        else:
            self._log.warning('could not find distance sensors publisher.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def enable(self):
        Publisher.enable(self)
        if self.enabled:
            if self._initial_calibration:
                self._log.info('calibrating…')
                self._suppress_sensors(True)
                self._calibrate_imu()
                self._log.info('finished calibration.')
                self._suppress_sensors(False)
            if not self._icm20948.is_calibrated:
                self._log.warning('cannot enable IMU: icm20948 has not been calibrated.')
                self.disable()
                return
            if self._active_mode:
                if self._message_bus.get_task_by_name(IMU._LISTENER_LOOP_NAME):
                    self._log.warning('already enabled.')
                else:
                    self._log.info('creating task for imu listener loop…')
                    self._message_bus.loop.create_task(self._imu_listener_loop(lambda: self.enabled), name=IMU._LISTENER_LOOP_NAME)
                    self._log.info('enabled in active mode.')
            else:
                self._log.info('enabled in passive mode.')

        else:
            self._log.warning('failed to enable publisher.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _calibrate_imu(self):
        '''
        Calibrate the IMU now…
        '''
        if not self._icm20948.enabled:
            self._icm20948.enable()
        if not self._icm20948.is_calibrated:
            # TODO motion calibrate?
            self._log.info('calibrating IMU…')
            self._icm20948.calibrate()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def icm20948(self):
        '''
        Return the backing IMU.
        '''
        return self._icm20948

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    async def _imu_listener_loop(self, f_is_enabled):
        self._log.info('starting imu listener loop.')
        while f_is_enabled():
            _message = None
            # read heading, pitch and roll from IMU
            _heading, _pitch, _roll = self._icm20948.poll()
            if _heading is not None:
                if abs(_roll) > self._roll_threshold:
                    self._log.info('heading: {:.2f}°; pitch: {:.2f}°; '.format(_heading, _pitch) + Style.BRIGHT + ' roll: {:.2f}°'.format(_roll))
                    _message = self.message_factory.create_message(Event.IMU_OVER_ROLL, (_roll))
                elif abs(_pitch) > self._pitch_threshold:
                    self._log.info('imu heading: {:.2f}°;'.format(_heading) + Style.BRIGHT + ' pitch: {:.2f}°;'.format(_pitch) + Style.NORMAL + ' roll: {:.2f}°'.format(_roll))
                    _message = self.message_factory.create_message(Event.IMU_OVER_PITCH, (_pitch))
                else:
#                   self._log.info(Style.DIM + 'imu heading: {:.2f}°; pitch: {:.2f}°; roll: {:.2f}°'.format(_heading, _pitch, _roll))
                    pass
            if _message is not None:
                self._log.info(Style.BRIGHT + 'imu-publishing message:' + Fore.WHITE + Style.NORMAL + ' {}'.format(_message.name)
                        + Fore.CYAN + ' event: {}; '.format(_message.event.name) + Fore.YELLOW + 'timestamp: {}'.format(_message.value))
                await Publisher.publish(self, _message)
            await asyncio.sleep(self._publish_delay_sec)
        self._log.info('imu publish loop complete.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def disable(self):
        '''
        Disable this publisher.
        '''
        Publisher.disable(self)
        self._log.info('disabled publisher.')

#EOF
