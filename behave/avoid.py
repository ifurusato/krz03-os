#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2021 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2020-05-19
# modified: 2021-06-26
#

from abc import ABC, abstractmethod
from datetime import datetime as dt
from colorama import init, Fore, Style
init()

import core.globals as globals
globals.init()

from core.logger import Logger, Level
from core.event import Event, Group
from core.util import Util
from core.subscriber import Subscriber
from behave.behaviour import Behaviour
from behave.roam import Roam # for STOPPED constant

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Avoid(Behaviour):

    CLASS_NAME = 'avoid'
    '''
    Implements an avoidance behaviour, using a macro to execute a ballistic
    series of movements. This subscribes to Event.ROAM, awaiting a 'stopped'
    message, upon which the behaviour is released.

    :param config:          the application configuration
    :param message_bus:     the asynchronous message bus
    :param message_factory: the factory for messages
    :param level:           the optional log level
    '''
    def __init__(self, config, message_bus=None, message_factory=None, level=Level.INFO):
        Behaviour.__init__(self, 'avoid', config, message_bus, message_factory, suppressed=True, enabled=True, level=level)
        self.add_event(Event.ROAM)
        _cfg = self._config['krzos'].get('behaviour').get('avoid')
        # any config here
        _component_registry = globals.get('component-registry')
        self._queue_publisher = _component_registry.get('pub:queue')
        if self._queue_publisher is None:
            raise Exception('queue publisher not available.')
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def name(self):
        return Avoid.CLASS_NAME

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def is_ballistic(self):
        return True

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def callback(self):
        print('💮 avoid callback.')
        self._log.info('avoid callback.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def name(self):
        return 'avoid'

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def release(self):
        '''
        Releases (un-suppresses) this Component.
        '''
        Subscriber.release(self)
        self._log.info('🎲 released.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    async def process_message(self, message):
        '''
        Process the message. If it's not an IDLE message this indicates activity.

        A Subscriber method.

        :param message:  the message to process.
        '''
        if message.gcd:
            raise GarbageCollectedError('cannot process message: message has been garbage collected.')
        _event = message.event
        if _event is Event.ROAM:
            self._log.info(Fore.WHITE + '💮 ROAM message {}; '.format(message.name) + Fore.YELLOW + "event: '{}'; value: {}".format(_event.name, _event.value))
            if _event.value == Roam.STOPPED:
                # send suppress message to Roam
                _message = self.message_factory.create_message(Event.AVOID, 'suppress')
                self._queue_publisher.put(_message)
                self._log.info(Fore.WHITE + "💮 💮 💮  published AVOID message: {}".format(_message))
                self.release()
            else:
                self._log.info(Fore.BLUE + "ignored ROAM message with value: '{}'".format(_event.value))
        await Subscriber.process_message(self, message)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def execute(self, message):
        '''
        The method called upon each loop iteration.

        :param message:  an optional Message passed along by the message bus
        '''
        print('💮 execute message {}.'.format(message))
        if self.suppressed:
            self._log.info(Style.DIM + 'avoid suppressed; message: {}'.format(message.event.label))
        else:
            self._log.info('avoid released; message: {}'.format(message.event.label))
            _payload = message.payload
            _event   = _payload.event
            _timestamp = self._message_bus.last_message_timestamp
            if _timestamp is None:
                self._log.info('avoid loop execute; no previous messages.')
            else:
                _elapsed_ms = (dt.now() - _timestamp).total_seconds() * 1000.0
                self._log.info('avoid loop execute; {}'.format(Util.get_formatted_time('message age:', _elapsed_ms)))
            if self.enabled:
                self._log.info('avoid enabled, execution on message {}; '.format(message.name) + Fore.YELLOW + ' event: {};'.format(_event.label))
            else:
                self._log.info('avoid disabled, execution on message {}; '.format(message.name) + Fore.YELLOW + ' event: {};'.format(_event.label))

#EOF
