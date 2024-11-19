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

import sys
import asyncio
from datetime import datetime as dt
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.event import Event
from core.orientation import Orientation
from core.message_factory import MessageFactory
from core.publisher import Publisher
from hardware.distance_sensor import DistanceSensor

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class DistanceSensors(Publisher):
    CLASS_NAME = 'dists'
    _LISTENER_LOOP_NAME = '__dist_listener_loop'
    '''
    A publisher for events from a trio of DistanceSensors.

    :param config:            the application configuration
    :param message_bus:       the asynchronous message bus
    :param message_factory:   the factory for creating messages
    :param level:             the log level
    '''
    def __init__(self, config, message_bus, message_factory, level=Level.INFO):
        if not isinstance(level, Level):
            raise ValueError('wrong type for log level argument: {}'.format(type(level)))
        self._level = level
        Publisher.__init__(self, DistanceSensors.CLASS_NAME, config, message_bus, message_factory, level=self._level)
        # configuration ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        if config is None:
            raise ValueError('no configuration provided.')
        _cfg = config['krzos'].get('publisher').get('distance_sensors')
        _loop_freq_hz         = _cfg.get('loop_freq_hz')
        self._publish_delay_sec = 1.0 / _loop_freq_hz
        self._sense_threshold = _cfg.get('sense_threshold')
        self._bump_threshold  = _cfg.get('bump_threshold')
        self._exit_on_cancel  = True # FIXME
        # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._port_sensor = DistanceSensor(config, Orientation.PORT)
        self._cntr_sensor = DistanceSensor(config, Orientation.CNTR)
        self._stbd_sensor = DistanceSensor(config, Orientation.STBD)
        self._sensors = [ self._port_sensor, self._cntr_sensor, self._stbd_sensor ]
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def enable(self):
        Publisher.enable(self)
        if self.enabled:
            if self._message_bus.get_task_by_name(DistanceSensors._LISTENER_LOOP_NAME):
                self._log.warning('already enabled.')
            else:
                self._log.info('creating task for distance sensor listener loop…')
                self._message_bus.loop.create_task(self._dist_listener_loop(lambda: self.enabled), name=DistanceSensors._LISTENER_LOOP_NAME)
                self._log.info('enabled.')
        else:
            self._log.warning('failed to enable publisher.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _get_bumper_event(self, orientation):
        match orientation:
            case Orientation.PORT:
                return Event.BUMPER_PORT 
            case Orientation.CNTR:
                return Event.BUMPER_CNTR 
            case Orientation.STBD:
                return Event.BUMPER_STBD 

    def _get_infrared_event(self, orientation):
        match orientation:
            case Orientation.PORT:
                return Event.INFRARED_PORT
            case Orientation.CNTR:
                return Event.INFRARED_CNTR 
            case Orientation.STBD:
                return Event.INFRARED_STBD  

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    async def _dist_listener_loop(self, f_is_enabled):
        self._log.info('starting distance sensor listener loop.')
        try:
            # start all sensors
            for _sensor in self._sensors:
                _sensor.start()
            while f_is_enabled():
                for _sensor in self._sensors:
                    _distance_mm = _sensor.get_distance()
                    if _distance_mm is not None:
                        if _distance_mm < self._bump_threshold:
                            self._log.info(Fore.WHITE + Style.BRIGHT + "bumper: {} {:10.1f}mm".format(_sensor.orientation, _distance_mm))
                            _message = self.message_factory.create_message( self._get_bumper_event(_sensor.orientation), (_distance_mm))
                            await Publisher.publish(self, _message)
                        elif _distance_mm < self._sense_threshold:
                            self._log.info(Fore.WHITE + "infrared: {} {:10.1f}mm".format(_sensor.orientation, _distance_mm))
                            _message = self.message_factory.create_message( self._get_infrared_event(_sensor.orientation), (_distance_mm))
                            await Publisher.publish(self, _message)
                await asyncio.sleep(self._publish_delay_sec)
        except asyncio.CancelledError:
            self._log.error('caught cancellation: system exit!')
            if self._exit_on_cancel:
                sys.exit(0)
        finally:
            for _sensor in self._sensors:
                _sensor.stop()
        self._log.info('distance sensors publish loop complete.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def disable(self):
        '''
        Disable this publisher.
        '''
        Publisher.disable(self)
        self._log.info('disabled publisher.')

#EOF
