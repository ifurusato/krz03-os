#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-05-06
# modified: 2025-05-06
#

import sys, traceback
import time
from threading import Thread
from math import isclose as isclose
import asyncio
from datetime import datetime, timedelta
from colorama import init, Fore, Style
init()

import core.globals as globals
globals.init()

from core.component import Component
from core.config_loader import ConfigLoader
from core.event import Event, Group
from core.logger import Logger, Level
from core.orientation import Orientation
from core.speed import Speed
from core.subscriber import Subscriber
from behave.behaviour import Behaviour
from hardware.distance_sensors import DistanceSensors
from hardware.differential_drive import DifferentialDrive

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Roam(Behaviour):
    '''
    Implements a roaming behaviour. The end result of this Behaviour is to
    provide a forward speed limit for both motors based on a distance value
    provided by the center infrared sensor, i.e., the distance to an obstacle
    in cm. If no obstacle is perceived within the range of the sensor, the
    velocity limit is removed.

    Because we only know how far the obstacle is based on incoming events,
    if we haven't seen an event in awhile we may assume there is nothing
    there and start moving again, at "cruising" speed. But we need to wait
    a bit after reacting to an obstacle before attempting to start moving
    again.

    The Roam behaviour is by default suppressed.
    '''
    def __init__(self, config, message_bus=None, message_factory=None, differential_drive=None, distance_sensors=None, level=Level.INFO):
        '''
        :param config:              the application configuration
        :param message_bus:         the asynchronous message bus
        :param message_factory:     the factory for creating messages
        :param differential_drive:  the optional DifferentialDrive object (will create if not provided)
        :param distance_sensors:    the optional DistanceSensors object (will create if not provided)
        :param level:               the log level
        '''
        self._log = Logger('roam', level)
        Behaviour.__init__(self, 'roam', config, message_bus, message_factory, suppressed=False, enabled=True, level=level)
        self.add_events(Event.by_groups([Group.BEHAVIOUR, Group.BUMPER, Group.INFRARED]))
        _cfg = config['krzos'].get('behaviour').get('roam')
        self._loop_delay_ms  = _cfg.get('loop_delay_ms', 50) # 50ms
        self._cruising_speed = Speed.from_string(_cfg.get('cruising_speed'))
        self._log.info(Style.BRIGHT + 'cruising speed: \t{} ({:5.2f}cm/sec)'.format(self._cruising_speed.label, self._cruising_speed.velocity))
        self._default_speed  = self._cruising_speed.proportional
        self._zero_tolerance = 0.2
        self._log.info(Style.BRIGHT + 'default speed: \t{}'.format(self._default_speed))
        self._post_delay     = 500
        self._last_drive_update = datetime.min
        self._task           = None 
        _component_registry = globals.get('component-registry')
        self._queue_publisher = _component_registry.get('pub:queue')
        if self._queue_publisher is None:
            raise Exception('queue publisher not available.')
        # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        if distance_sensors:
            self._sensors = distance_sensors
        else:
            self._sensors = DistanceSensors(config)
        if differential_drive:
            self._differential = differential_drive
        else:
            self._differential = DifferentialDrive(config, level)
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def name(self):
        return Roam.CLASS_NAME

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def trigger_behaviour(self):
        return TriggerBehaviour.EXECUTE

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def trigger_event(self):
        '''
        This returns the event used to enable/disable the behaviour manually.
        '''
        return Event.ROAM

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def callback(self):
        print('🍀 roam callback.')
        self._log.info('roam callback.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def name(self):
        return 'roam'

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
        if _event.group is Group.BEHAVIOUR:
            if _event is Event.AVOID:
                self._log.info(Fore.WHITE + '🍀 AVOID message {}; '.format(message.name) + Fore.YELLOW + "event: '{}'; value: {}".format(_event.name, _event.value))
                if _event.value == 'suppress':
                    self.suppress()
                # TODO what to do?
            else:
                self._log.info(Fore.WHITE + '🍀 {} message {}; '.format(message.event.group.name, message.name) + Fore.YELLOW + 'event: {}'.format(_event.name))
        elif _event.group is Group.INFRARED:
                self._log.info(Style.DIM + '🍀 INFRARED message {}; '.format(message.name) + Fore.YELLOW + 'event: {}'.format(_event.name))
        elif _event.group is Group.BUMPER:
                self._log.info(Fore.RED + '🍀🍀🍀 BUMPER message {}; '.format(message.name) + Fore.YELLOW + 'event: {}'.format(_event.name))
                self._handle_stoppage()
        await Subscriber.process_message(self, message)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def execute(self, message):
        '''
        The method called upon each loop iteration.

        :param message:  an optional Message passed along by the message bus
        '''
        print('🍀 execute message {}.'.format(message))
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

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    async def _loop_main(self):
        self._log.info("roam loop started with {}ms delay…".format(self._loop_delay_ms))
        try:
            self._accelerate()

            self._differential.set_speeds(self._default_speed, self._default_speed, save=True)
            while self.enabled:
                if not self.suppressed:
                    await self._poll()
                else:
                    self._log.info(Fore.WHITE + "suppressed…")
                await asyncio.sleep(self._loop_delay_ms / 1000)
                # add a safe exit condition for testing
                if not self.enabled:
                    break
        except asyncio.CancelledError:
            self._log.info("roam loop cancelled.")
        except Exception as e:
            self._log.error('🌭 {} encountered in roam loop: {}\n{}'.format(type(e), e, traceback.format_exc()))
            self.disable()
        finally:
            self._log.info("roam loop stopped.")
            self._decelerate()
            self._stop()

    async def _poll(self):
        if not self.enabled or self._differential is None:
            self._log.warning("roam has been disabled.")
            return
        try: # TEMP
            self._log.debug("polling…")
            # get weighted averages for each motor
            port_avg, stbd_avg = self._sensors.get_weighted_averages()
            self._log.info(Style.DIM + "port: {:4.2f};\t stbd: {:4.2f}".format(port_avg, stbd_avg))
            # get current speeds
            current_port_speed, current_stbd_speed = self._differential.get_speeds()
            # apply motor commands
            port_speed = current_port_speed * port_avg
            stbd_speed = current_stbd_speed * stbd_avg
            # apply motor commands HERE
            now = datetime.now()
            if now - self._last_drive_update >= timedelta(milliseconds=self._post_delay):
                self._log.info("current: " 
                    + Style.BRIGHT
                    + Fore.RED + "port: {:4.2f};\t".format(current_port_speed)
                    + Fore.GREEN + "stbd: {:4.2f}".format(current_stbd_speed)
                    + Style.NORMAL
                    + Fore.CYAN + " averages: "
                    + Fore.RED + "port: {:4.2f};\t".format(port_avg)
                    + Fore.GREEN + "stbd: {:4.2f}".format(stbd_avg)
                    + Fore.CYAN + " to set: "
                    + Fore.RED + "port: {:4.2f};\t".format(port_speed)
                    + Fore.GREEN + "stbd: {:4.2f}".format(stbd_speed)
                )
                if isclose(port_speed, 0.0, abs_tol=self._zero_tolerance) or isclose(stbd_speed, 0.0, abs_tol=self._zero_tolerance):
                    # there's no point in setting a speed the motors cannot fulfill
                    self._differential.set_speeds(0.0, 0.0, save=False)
                    self._handle_stoppage()
                else:
                    self._differential.set_speeds(port_speed, stbd_speed, save=False)
                self._last_drive_update = now
        except Exception as e:
            self._log.error("{} thrown while polling: {}".format(type(e), e))
            self.disable()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _accelerate(self):
        self._log.info("accelerate…")
        self._differential.send_payload('acce', self._default_speed, self._default_speed, 2.0)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _decelerate(self):
        self._log.info("decelerate…")
        self._differential.send_payload('dece', 0.0, 0.0, 2.0)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _stop(self):
        self._log.info("stop…")
        self._differential.send_payload('stop', 0.0, 0.0, 0.0)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _handle_stoppage(self):
        self._log.info(Fore.WHITE + "🍀🌼 stoppage")
        self._differential.play('boink')
        # notify Roam that the robot has stopped
        _message = self.message_factory.create_message(Event.ROAM, 'stopped')
        self._queue_publisher.put(_message)
        self._log.info(Fore.WHITE + "published ROAM message: {}".format(_message))
#       self.suppress() # TODO this should be done by a takeover Behaviour

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def enable(self):
        '''
        Enable the roaming behaviour.
        '''
        self._sensors.enable()
        self._differential.enable()

#       self._task = asyncio.create_task(self._loop_main())
        self._loop_instance = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop_instance)
        self._task = self._loop_instance.create_task(self._loop_main())
        # start the loop in a background thread
        self._thread = Thread(target=self._loop_instance.run_forever, daemon=True)
        self._thread.start()

        Component.enable(self)
        self._log.info("roam enabled.")

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def suppress(self):
        '''
        Suppresses this Component.
        '''
        print('🐱🐱🐱🐱🐱 ROAM suppress ............................')
        Behaviour.suppress(self)

    def disable(self):
        '''
        Disable the roaming behaviour.
        '''
        print('🐱🐱 ROAM disable ............................')
        if not self.enabled:
            self._log.warning("already disabled.")
            return
        self._log.info(Fore.YELLOW + "roam disabling…")
        # cancel the async task
        if self._task and not self._task.done():
            self._task.cancel()

        # Wait for task cancellation and shutdown properly
        if self._task:
            try:
                asyncio.get_event_loop().run_until_complete(self._task)  # await cancellation cleanly
            except Exception:
                pass # suppress expected cancellation errors

        self._sensors.disable()
        self._differential.disable()
        Component.disable(self)
        self._log.info("disabled.")

#   async def wait_until_finished(self):
#       if self._task:
#           await self._task

    def wait_until_finished(self):
        '''
        Wait synchronously for the async loop to complete, handling Ctrl-C gracefully.
        '''
        if self._task:
            try:
                loop = asyncio.get_event_loop()
                if loop.is_running():
                    # If an event loop is already running, we need to await the task without creating a new loop
                    while not self._task.done():
                        time.sleep(0.1) # allow other tasks to run (including awareness of Ctrl-C) while we wait for the task to finish
                else:
                    # if no event loop is running, we can safely use `run_until_complete`
                    loop.run_until_complete(self._task)
            except KeyboardInterrupt:
                self._log.warning("interrupted by user (Ctrl-C); stopping roam.")
                self.disable()
            except Exception as e:
                self._log.error("{} raised while waiting for the loop to finish: {}".format(type(e), e))
                raise e

#EOF
