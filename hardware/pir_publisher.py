#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-04-21
# modified: 2025-04-21
#

import asyncio
import random # TEMP
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
class PirPublisher(Publisher):

    CLASS_NAME = 'pir'
    _LISTENER_LOOP_NAME = '__pir_listener_loop'

    '''
    The PirPublisher operates in an active mode, using an asyncio loop, as a publisher
    for events polled from the TinyFX's PIR sensor.

    :param config:            the application configuration
    :param message_bus:       the asynchronous message bus
    :param message_factory:   the factory for creating messages
    :param level:             the log level
    '''
    def __init__(self, config, message_bus, message_factory, level=Level.INFO):
        if not isinstance(level, Level):
            raise ValueError('wrong type for log level argument: {}'.format(type(level)))
        self._level = level
        if message_bus is None:
            raise ValueError('null message bus argument.')
        self._message_bus = message_bus
        if message_factory is None:
            raise ValueError('null message factory argument.')
        self._message_factory = message_factory
        Publisher.__init__(self, PirPublisher.CLASS_NAME, config, message_bus, message_factory, level=self._level)
        # configuration ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        _cfg = config['krzos'].get('publisher').get('pir')
        self._active_mode         = _cfg.get('active_mode')
        _loop_freq_hz             = _cfg.get('loop_freq_hz')
        self._pir_reset_delay_sec = 4
        self._publish_delay_sec   = 1.0 / _loop_freq_hz
        self._message_bus.add_callback_on_start(self.enable)
        self._log.info('ready with loop frequency of {:d}Hz.'.format(_loop_freq_hz))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def enable(self):
        '''
        Do not call this directly when using the MessageBus.
        '''
        Publisher.enable(self)
        if self.enabled:
            if self._active_mode:
                if self._message_bus.get_task_by_name(PirPublisher._LISTENER_LOOP_NAME):
                    self._log.warning('already enabled.')
                else:
                    self._log.info('creating task for pir listener loop…')
                    self._message_bus.loop.create_task(self._pir_listener_loop(lambda: self.enabled), name=PirPublisher._LISTENER_LOOP_NAME)
                    self._log.info('enabled in active mode.')
            else:
                self._log.info('enabled in passive mode.')
        else:
            self._log.warning('failed to enable publisher.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    async def _pir_listener_loop(self, f_is_enabled):
        self._log.info('starting pir listener loop.')
        while f_is_enabled():
            _pir_active = random.random() < 0.1 # TODO get actual value
            if _pir_active:
                _message = self.message_factory.create_message(Event.PIR_ACTIVE, ())
                self._log.info(Style.BRIGHT + 'pir-publishing message: {}'.format(_message.name) + Fore.YELLOW + ' event: {}'.format(_message.event.name))
                # additional delay to wait for PIR to reset
                await asyncio.sleep(self._pir_reset_delay_sec)
            else:
                _message = self.message_factory.create_message(Event.PIR_INACTIVE, ())
                self._log.info(Style.NORMAL + 'pir-publishing message: {}'.format(_message.name) + Style.DIM + ' event: {}'.format(_message.event.name))
            await Publisher.publish(self, _message)
            await asyncio.sleep(self._publish_delay_sec)
        self._log.info('pir publish loop complete.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def disable(self):
        '''
        Disable this publisher.
        '''
        Publisher.disable(self)
        self._log.info('disabled publisher.')

#EOF
