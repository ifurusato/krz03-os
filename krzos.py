#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2019-12-23
# modified: 2024-10-31
#
# The KRZ03 Robot Operating System (KRZOS), including its command line
# interface (CLI).
#
#        1         2         3         4         5         6         7         8         9         C
#234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890
#

print('🤖 import begin.')
import os, sys, signal, time, traceback
print('🤖 a.')
import argparse
from pathlib import Path
from colorama import init, Fore, Style
init()

import core.globals as globals
globals.init()

from core.logger import Logger, Level
from core.event import Event, Group
from core.component import Component
from core.fsm import FiniteStateMachine
from core.orientation import Orientation
from core.util import Util
from core.message import Message, Payload
from core.message_bus import MessageBus
from core.message_factory import MessageFactory
from core.config_loader import ConfigLoader
from core.config_error import ConfigurationError
from core.controller import Controller

from core.publisher import Publisher
from core.queue_publisher import QueuePublisher
#from core.macro_publisher import MacroPublisher
from hardware.system_publisher import SystemPublisher

from core.subscriber import Subscriber, GarbageCollector
#from hardware.system_subscriber import SystemSubscriber
#from core.macro_subscriber import MacroSubscriber
from hardware.distance_sensors_subscriber import DistanceSensorsSubscriber
from hardware.sound_subscriber import SoundSubscriber

from hardware.system import System
from hardware.rtof import RangingToF
from hardware.rgbmatrix import RgbMatrix # used for ICM20948
from hardware.icm20948 import Icm20948
from hardware.imu import IMU
from hardware.task_selector import TaskSelector
from hardware.i2c_scanner import I2CScanner
from hardware.distance_sensors_publisher import DistanceSensorsPublisher
print('🤖 s.')
#from hardware.button import Button
from hardware.killswitch import Killswitch
print('🤖 t.')
from hardware.pigpiod_util import PigpiodUtility as PigUtil

from hardware.sound import Sound
from hardware.player import Player

#from behave.behaviour_manager import BehaviourManager
#from behave.avoid import Avoid
#from behave.roam import Roam
#from behave.swerve import Swerve
#from behave.moth import Moth
#from behave.sniff import Sniff
#from behave.idle import Idle

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class KRZOS(Component, FiniteStateMachine):
    '''
    Extends Component and Finite State Machine (FSM) as a basis of a K-Series
    Robot Operating System (KRZOS) or behaviour-based system (BBS), including
    spawning the various tasks and starting up the Subsumption Architecture,
    used for communication between Components over a common message bus.

    The MessageBus receives Event-containing messages from sensors and other
    message sources, which are passed on to the Arbitrator, whose job it is
    to determine the highest priority action to execute for that task cycle,
    by passing it on to the Controller.

    There is also a krzosd linux daemon, which can be used to start, enable and
    disable krzos.
    '''
    def __init__(self, level=Level.INFO):
        '''
        This initialises KRZOS and calls the YAML configurer.
        '''
        _name = 'krzos'
        self._level = level
        self._log = Logger(_name, self._level)
        self._print_banner()
        self._log.info('…')
        Component.__init__(self, self._log, suppressed=False, enabled=False)
        FiniteStateMachine.__init__(self, self._log, _name)
        self._system                      = System(self, level)
        self._system.set_nice()
        # configuration…
        self._config                      = None
        self._component_registry          = None
        self._controller                  = None
        self._message_bus                 = None
#       self._gamepad_publisher           = None
#       self._macro_publisher             = None
        self._queue_publisher             = None
        self._rtof_publisher              = None
        self._distance_sensors_publisher  = None
        self._system_publisher            = None
        self._system_subscriber           = None
        self._distance_sensors_subscriber = None
        self._sound_subscriber            = None
        self._task_selector               = None
        self._rgbmatrix                   = None
        self._icm20948                    = None
        self._imu                         = None
        self._behaviour_mgr               = None
#       self._gamepad_controller          = None
        self._motor_controller            = None
        self._tinyfx                      = None
        self._killswitch                  = None
        self._closing                     = False
        self._log.info('oid: {}'.format(id(self)))
        self._log.info('initialised.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def configure(self, arguments):
        '''
        Provided with a set of configuration arguments, configures KRZOS based on
        both MR01 hardware as well as optional features, the latter based on
        devices showing up (by address) on the I²C bus. Optional devices are only
        enabled at startup time via registration of their feature availability.
        '''
        self._log.heading('configuration', 'configuring krzos…',
            '[1/2]' if arguments.start else '[1/1]')
        self._log.info('application log level: {}'.format(self._log.level.name))

        # read YAML configuration ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        _loader = ConfigLoader(self._level)
        _config_filename = arguments.config_file
        _filename = _config_filename if _config_filename is not None else 'config.yaml'
        self._config = _loader.configure(_filename)
        self._is_raspberry_pi = self._system.is_raspberry_pi()

        _i2c_scanner = I2CScanner(self._config, level=Level.INFO)

        # configuration from command line arguments ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

        _args = self._config['krzos'].get('arguments')
        # copy argument-based configuration over to _config (changing the names!)

        _args['log_enabled']    = arguments.log
        self._log.info('write log enabled:    {}'.format(_args['log_enabled']))

        _args['json_dump_enabled'] = arguments.json
        self._log.info('json enabled:         {}'.format(_args['json_dump_enabled']))

#       self._log.info('argument gamepad:     {}'.format(arguments.gamepad))
        _args['gamepad_enabled'] = arguments.gamepad and self._is_raspberry_pi
        self._log.info('gamepad enabled:      {}'.format(_args['gamepad_enabled']))

        # print remaining arguments
        self._log.info('argument config-file: {}'.format(arguments.config_file))
        self._log.info('argument level:       {}'.format(arguments.level))

        # confirm external services are running ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

        PigUtil.ensure_pigpiod_is_running()

        # establish basic subsumption components ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

        self._log.info('configure subsumption components…')

        self._message_bus = MessageBus(self._config, self._level)
        self._message_factory = MessageFactory(self._message_bus, self._level)

        self._controller = Controller(self._message_bus, self._level)

        # JSON configuration dump ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        if _args['json_dump_enabled']:
            print('exporting JSON configuration.')
            # TODO

        # create components ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

        _cfg = self._config['krzos'].get('component')
        self._component_registry = globals.get('component-registry')

        # basic hardware ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

        # create subscribers ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

        _subs = arguments.subs if arguments.subs else ''

        if _cfg.get('enable_system_subscriber') or 's' in _subs:
            self._system_subscriber = SystemSubscriber(self._config, self, self._message_bus, level=self._level)

#       if _cfg.get('enable_macro_subscriber'):
#           if not self._macro_publisher:
#               raise ConfigurationError('macro subscriber requires macro publisher.')
#           self._macro_subscriber = MacroSubscriber(self._config, self._message_bus, self._message_factory, self._macro_publisher, self._level)
#       if _cfg.get('enable_omni_subscriber') or 'o' in _subs:
#           self._omni_subscriber = OmniSubscriber(self._config, self._message_bus, level=self._level) # reacts to IR sensors

        if _cfg.get('enable_distance_subscriber'):
            self._distance_sensors_subscriber = DistanceSensorsSubscriber(self._config, self._message_bus, level=self._level) # reacts to IR sensors

        if _cfg.get('enable_sound_subscriber'):
            self._sound_subscriber = SoundSubscriber(self._config, self._message_bus, level=self._level)

        # create publishers  ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

        _pubs = arguments.pubs if arguments.pubs else ''

        if _cfg.get('enable_queue_publisher') or 'q' in _pubs:
            self._queue_publisher = QueuePublisher(self._config, self._message_bus, self._message_factory, self._level)

        _enable_system_publisher = _cfg.get('enable_system_publisher')
        if _enable_system_publisher:
            self._system_publisher = SystemPublisher(self._config, self._message_bus, self._message_factory, self._system, level=self._level)

        _enable_distance_sensors = _cfg.get('enable_distance_sensors')
        if _enable_distance_sensors:
            self._distance_sensors_publisher = DistanceSensorsPublisher(self._config, self._message_bus, self._message_factory, level=self._level)

#       if _cfg.get('enable_macro_publisher') or 'm' in _pubs:
#           _callback = None
#           self._macro_publisher = MacroPublisher(self._config, self._message_bus, self._message_factory, self._queue_publisher, _callback, self._level)
#           _library = KR01MacroLibrary(self._macro_publisher)
#           self._macro_publisher.set_macro_library(_library)

        # optionally used by ICM20948
        self._rgbmatrix = RgbMatrix(enable_port=True, enable_stbd=True, level=self._level)

        _enable_imu_publisher = _cfg.get('enable_imu_publisher')
        if _enable_imu_publisher:
            if _i2c_scanner.has_hex_address(['0x69']):
                self._icm20948 = Icm20948(self._config, self._rgbmatrix, level=self._level)
                self._imu = IMU(self._config, self._icm20948, self._message_bus, self._message_factory, level=self._level)
            else:
                self._log.warning('no IMU available.')

        _enable_rtof_publisher = _cfg.get('enable_rtof_publisher')
        if _enable_rtof_publisher:
            if _i2c_scanner.has_hex_address(['0x29']):
                self._rtof_publisher = RangingToF(self._config, self._message_bus, self._message_factory, level=self._level)
            else:
                self._log.warning('rtof disabled: no VL53L5CX found.')

        _enable_tinyfx_controller = _cfg.get('enable_tinyfx_controller')
        if _enable_tinyfx_controller:
            from hardware.tinyfx_i2c_controller import TinyFxController

            self._log.info('configure tinyfx controller…')
            self._tinyfx = TinyFxController(self._config)

        _enable_killswitch= _cfg.get('enable_killswitch') or 'k' in _pubs
        if _enable_killswitch:
#           self._killswitch = Button(self._config)
#           self._killswitch.add_callback(self.shutdown)
            self._killswitch = Killswitch(config=self._config, callback=self.shutdown, level=self._level)

        # add task selector ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

        # and finally, the garbage collector:
        self._garbage_collector = GarbageCollector(self._config, self._message_bus, level=self._level)

        # create behaviours ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        _enable_behaviours = _cfg.get('enable_behaviours') or Util.is_true(arguments.behave)
        if _enable_behaviours:
            _ = '''
            self._behaviour_mgr = BehaviourManager(self._config, self._message_bus, self._level) # a specialised subscriber
            self._log.info(Style.BRIGHT + 'behaviour manager enabled.')
            _bcfg = self._config['krzos'].get('behaviour')
            # create and register behaviours (listed in priority order)
            if _bcfg.get('enable_avoid_behaviour'):
                self._avoid  = Avoid(self._config, self._message_bus, self._message_factory, self._motor_controller,
                        external_clock=self._external_clock, level=self._level)
            if _bcfg.get('enable_roam_behaviour'):
                self._roam   = Roam(self._config, self._message_bus, self._message_factory, self._motor_controller,
                        external_clock=self._external_clock, level=self._level)
            if _bcfg.get('enable_swerve_behaviour'):
                self._swerve = Swerve(self._config, self._message_bus, self._message_factory, self._motor_controller,
                        self._external_clock, level=self._level)
            if _bcfg.get('enable_moth_behaviour'):
                self._moth   = Moth(self._config, self._message_bus, self._message_factory, self._motor_controller, self._level)
            if _bcfg.get('enable_sniff_behaviour'):
                self._sniff  = Sniff(self._config, self._message_bus, self._message_factory, self._motor_controller, self._level)
            if _bcfg.get('enable_idle_behaviour'):
                self._idle   = Idle(self._config, self._message_bus, self._message_factory, self._level)
            '''

        if _args['gamepad_enabled'] or _cfg.get('enable_gamepad_publisher') or 'g' in _pubs:
            from hardware.gamepad_publisher import GamepadPublisher
            from hardware.gamepad_controller import GamepadController

            self._gamepad_publisher = GamepadPublisher(self._config, self._message_bus, self._message_factory, exit_on_complete=True, level=self._level)
#           self._gamepad_controller = GamepadController(self._message_bus, self._level)

        # finish up ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

        self._export_config = False
        if self._export_config:
            self.export_config()
        self._log.info('configured.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def start(self):
        '''
        This first disables the Pi's status LEDs, establishes the message bus,
        arbitrator, controller, enables the set of features, then starts the main
        OS loop.
        '''
        self._log.heading('starting', 'starting m-series robot operating system (krzos)…', '[2/2]' )
        FiniteStateMachine.start(self)

#       atexit.register(self._cleanup)

        if self._system_subscriber:
            self._log.info('enabling system subscriber…')
            self._system_subscriber.enable()

        if self._tinyfx: # turn on running lights
            _cfg = self._config.get('krzos').get('hardware').get('tinyfx-controller')
            _enable_mast_light = _cfg.get('enable_mast_light')
            if _enable_mast_light:
                self._tinyfx.channel_on(Orientation.MAST)
            _enable_nav_light = _cfg.get('enable_nav_lights')
            if _enable_nav_light:
                self._tinyfx.channel_on(Orientation.PORT)
                time.sleep(0.1)
                self._tinyfx.channel_on(Orientation.STBD)

        # begin main loop ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

        self._log.notice('Press Ctrl-C to exit.')
        self._log.info('begin main os loop.\r')

        if self._motor_controller:
            self._motor_controller.enable()

        # we enable ourself if we get this far successfully
        Component.enable(self)
        FiniteStateMachine.enable(self)

        # print registry of components
        self._component_registry.print_registry()

        if self._tinyfx:
            # instantiate singleton with existing TinyFX
            Player.instance(self._tinyfx).play(Sound.BEEP)

        # ════════════════════════════════════════════════════════════════════
        # now in main application loop until quit or Ctrl-C…
        self._log.info('enabling message bus…')
        self._message_bus.enable()
        # that blocks so we never get here until the end…
        self._log.info('main loop closed.')

        # end main loop ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_config(self):
        '''
        Returns the application configuration.
        '''
        return self._config

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_component_registry(self):
        '''
        Return the registry of all instantiated Components.
        '''
        return self._component_registry

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_logger(self):
        '''
        Returns the application-level logger.
        '''
        return self._log

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_level(self):
        '''
        Returns the log level of the application.
        '''
        return self._level

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_message_bus(self):
        '''
        Returns the MessageBus.
        '''
        return self._message_bus

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_message_factory(self):
        '''
        Returns the MessageFactory.
        '''
        return self._message_factory

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_behaviour_manager(self):
        '''
        Returns the BehaviourManager, None if not used.
        '''
        return self._behaviour_mgr

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_gamepad_publisher(self):
        '''
        Returns the GamepadPublisher, None if not used.
        '''
        return self._gamepad_publisher

#   # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
#   def get_macro_publisher(self):
#       '''
#       Returns the MacroPublisher, None if not used.
#       '''
#       return self._macro_publisher

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_queue_publisher(self):
        '''
        Returns the QueuePublisher, None if not used.
        '''
        return self._queue_publisher

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_distance_sensors(self):
        '''
        Returns the DistanceSensorsPublisher, None if not used.
        '''
        return self._distance_sensors_publisher

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_motor_controller(self):
        '''
        Returns the motor controller.
        '''
        return self._motor_controller

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _set_pi_leds(self, enable):
        '''
        Enables or disables the Raspberry Pi's board LEDs.
        '''
        sudo_name = self._config['pi'].get('sudo_name')
        _led_0_path = self._config['pi'].get('led_0_path')
        _led_0 = Path(_led_0_path)
        _led_1_path = self._config['pi'].get('led_1_path')
        _led_1 = Path(_led_1_path)
        if _led_0.is_file() and _led_0.is_file():
            if enable:
                self._log.info('re-enabling LEDs…')
                os.system('echo 1 | {} tee {}'.format(sudo_name,_led_0_path))
                os.system('echo 1 | {} tee {}'.format(sudo_name,_led_1_path))
            else:
                self._log.debug('disabling LEDs…')
                os.system('echo 0 | {} tee {}'.format(sudo_name,_led_0_path))
                os.system('echo 0 | {} tee {}'.format(sudo_name,_led_1_path))
        else:
            self._log.warning('could not change state of LEDs: does not appear to be a Raspberry Pi.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def shutdown(self, arg=None):
        '''
        This halts any motor activity, demands a sudden halt of all tasks,
        then shuts down the OS.
        '''
        self._log.info(Fore.MAGENTA + 'shutting down…')
        self.close()
        # we never get here if we shut down properly
        self._log.error('shutdown error.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def disable(self):
        '''
        This permanently disables the KRZOS.
        '''
        if self.closed:
            self._log.warning('already closed.')
        elif self._closing:
            self._log.warning('already closing.')
        elif self.enabled:
            if self._tinyfx:
                Player.instance().play(Sound.HZAH)
            self._log.info('disabling…')
            if self._task_selector:
                self._task_selector.close()
            if self._queue_publisher:
                self._queue_publisher.disable()
            Component.disable(self)
            FiniteStateMachine.disable(self)
            self._log.info('disabled.')
        else:
            self._log.warning('already disabled.')
        return True

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def closing(self):
        return self._closing

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
#   def _cleanup(self):
#       try:
#           print('🧡 cleanup a.')
#           main_thread = threading.main_thread()

#           for thread_id, frame in sys._current_frames().items():
#               thread = next((t for t in threading.enumerate() if t.ident == thread_id), None)
#               if thread and thread.name.startswith("Thread-4"):
#                   self._log.info(Fore.MAGENTA + "Inspecting Thread-4 (held): {}".format(thread))
#                   traceback.print_stack(frame)
#           print('🧡 cleanup a.')
#           if threading.active_count() > 0:
#               for thread in threading.enumerate():
#                   if thread is main_thread:
#                       self._log.warning("unclosed main thread: {}/{}; alive={}, daemon={}".format(thread.name, thread.ident, thread.is_alive(), thread.daemon ))
#                   else:
#                       self._log.warning("unclosed resource thread: {}/{}; alive={}, daemon={}".format(thread.name, thread.ident, thread.is_alive(), thread.daemon ))
#                   if thread.daemon:
#                       thread.join(timeout=1)
#                       self._log.info(Fore.MAGENTA + "thread {} stopped.".format(thread.name))
#                   if thread.ident == thread_id and thread.name.startswith("Thread"):
#                       self._log.info("💔 stack trace for Thread '{}':".format(thread.name))
#                       traceback.print_stack(frame)
#           print('🧡 cleanup z.')
#       except Exception as e:
#           self._log.error('error cleaning up application: {}\n{}'.format(e, traceback.format_exc()))
#       finally:
#           print('🧡 cleanup finally.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def close(self):
        '''
        This closes KRZOS and sets the robot to a passive, stable state
        following a session.
        '''
        if self.closed:
            self._log.warning('already closed.')
        elif self.closing:
            self._log.warning('already closing.')
        else:
            try:
                self._log.info('closing…')
                Component.close(self) # will call disable()
                self._closing = True
                _registry = self._component_registry.get_registry()
                # closes all components that are not a publisher, subscriber, the message bus or krzos itself…
                while len(_registry) > 0:
                    _name, _component = _registry.popitem(last=True)
                    if not isinstance(_component, Publisher) and not isinstance(_component, Subscriber) \
                            and _component != self and _component != self._message_bus:
                        self._log.info(Style.DIM + 'closing component \'{}\' ({})…'.format(_name, _component.classname))
                        _component.close()
                time.sleep(0.1)
                if self._message_bus and not self._message_bus.closed:
                    self._log.info('closing message bus from krzos…')
                    self._message_bus.close()
                    self._log.info('message bus closed.')
                while not Component.close(self): # will call disable()
                    self._log.info('closing component…')
                FiniteStateMachine.close(self)
                self._log.info('application closed.')
            except Exception as e:
                self._log.error('error closing application: {}\n{}'.format(e, traceback.format_exc()))
            finally:
                print('close finally 1.')
                self._log.close()
                print('close finally 2.')
                self._closing = False
                _threads = sys._current_frames().items()
                self._log.info(Fore.WHITE + 'close finally: {} threads remain.'.format(len(_threads)))
                if len(_threads) > 1:
                    frames = sys._current_frames()
                    for thread_id, frame in frames.items():
                        print(Fore.YELLOW + "🥃 Thread ID: {}, Frame: {}".format(thread_id, frame) + Style.RESET_ALL)

                sys.exit(0)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def export_config(self):
        '''
        Exports the current configuration to a YAML file named ".config.yaml".
        '''
        self._log.info('exporting configuration to file…')
        _loader.export(self._config, comments=[ \
            '┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈', \
            '      YAML configuration for K-Series Robot Operating System (KRZOS)          ', \
            '┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈', \
            '', \
            'exported: {}'.format(Util.get_timestamp()) ])

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _print_banner(self):
        '''
        Display banner on console.
        '''
        self._log.info(' ')
        self._log.info(' ')
        self._log.info('      ┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓ ')
        self._log.info('      ┃                                                                 ┃ ')
        self._log.info('      ┃    █▒▒    █▒▒  █▒▒▒▒▒▒▒    █▒▒▒▒▒▒▒   █▒▒▒▒▒▒    █▒▒▒▒▒▒   █▒▒  ┃ ')
        self._log.info('      ┃    █▒▒  █▒▒    █▒▒   █▒▒       █▒▒   █▒▒   █▒▒  █▒▒        █▒▒  ┃ ')
        self._log.info('      ┃    █▒▒▒▒▒▒     █▒▒▒▒▒▒▒      █▒▒     █▒▒   █▒▒   █▒▒▒▒▒▒   █▒▒  ┃ ')
        self._log.info('      ┃    █▒▒  █▒▒    █▒▒   █▒▒    █▒▒      █▒▒   █▒▒        █▒▒       ┃ ')
        self._log.info('      ┃    █▒▒    █▒▒  █▒▒    █▒▒  █▒▒▒▒▒▒▒   █▒▒▒▒▒▒    █▒▒▒▒▒▒   █▒▒  ┃ ')
        self._log.info('      ┃                                                                 ┃ ')
        self._log.info('      ┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛ ')
        self._log.info(' ')
        self._log.info(' ')

    # end of KRZOS class  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━


# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
def print_documentation(console=True):
    '''
    Print the extended documentation as imported from the help.txt file. If
    'console' is false, just return its contents.
    '''
    _help_file = Path("help.txt")
    if _help_file.is_file():
        with open(_help_file) as f:
            _content = f.read()
            if console:
                return _content
            else:
                print(_content)
    else:
        if console:
            return 'help file not found.'
        else:
            print('{} not found.'.format(_help_file))

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
def parse_args(passed_args=None):
    '''
    Parses the command line arguments and return the resulting args object.
    Help is available via '--help', '-h', or '--docs', '-d' (for extended help),
    or calling the script with no arguments.

    This optionally permits arguments to be passed in as a list, overriding
    sys.argv.
    '''
    _log = Logger('parse-args', Level.INFO)
    _log.debug('parsing…')
#   formatter = lambda prog: argparse.HelpFormatter(prog,max_help_position=60)
    formatter = lambda prog: argparse.RawTextHelpFormatter(prog)
    parser = argparse.ArgumentParser(formatter_class=formatter,
            description='Provides command line control of the K-Series Robot OS application.',
            epilog='This script may be executed by krzosd (krzos daemon) or run directly from the command line.')

    parser.add_argument('--docs',         '-d', action='store_true', help='show the documentation message and exit')
    parser.add_argument('--configure',    '-c', action='store_true', help='run configuration (included by -s)')
    parser.add_argument('--start',        '-s', action='store_true', help='start krzos')
    parser.add_argument('--json',         '-j', action='store_true', help='dump YAML configuration as JSON file')
    parser.add_argument('--gamepad',      '-g', action='store_true', help='enable bluetooth gamepad control')
    parser.add_argument('--pubs',         '-P', help='enable publishers as identified by first character')
    parser.add_argument('--subs',         '-S', help='enable subscribers as identified by first character')
    parser.add_argument('--behave',       '-b', help='override behaviour configuration (1, y, yes or true, otherwise false)')
    parser.add_argument('--config-file',  '-f', help='use alternative configuration file')
    parser.add_argument('--log',          '-L', action='store_true', help='write log to timestamped file')
    parser.add_argument('--level',        '-l', help='specify logging level \'DEBUG\'|\'INFO\'|\'WARN\'|\'ERROR\' (default: \'INFO\')')

    try:
        print('')
        args = parser.parse_args() if passed_args is None else parser.parse_args(passed_args)
        if args.docs:
            print(Fore.CYAN + '{}\n{}'.format(parser.format_help(), print_documentation(True)) + Style.RESET_ALL)
            return -1
        elif not args.configure and not args.start:
            print(Fore.CYAN + '{}'.format(parser.format_help()) + Style.RESET_ALL)
            return -1
        else:
            globals.put('log-to-file', args.log)
            return args


    except NotImplementedError as nie:
        _log.error('unrecognised log level \'{}\': {}'.format(args.level, nie))
        _log.error('exit on error.')
        sys.exit(1)
    except Exception as e:
        _log.error('error parsing command line arguments: {}\n{}'.format(e, traceback.format_exc()))
        _log.error('exit on error.')
        sys.exit(1)

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

_krzos = None

# execution handler ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
def signal_handler(signal, frame):
    print(Fore.MAGENTA + '\nsignal handler    :' + Style.BRIGHT + ' INFO  : Ctrl-C caught: exiting…' + Style.RESET_ALL)
    try:
        if _krzos and not ( _krzos.closing or _krzos.closed ):
            _krzos.close()
        print(Fore.MAGENTA + 'exiting…' + Style.RESET_ALL)
#       close_daemon_threads()
    except Exception as e:
        print(Fore.RED + "error during shutdown: {}".format(e) + Style.RESET_ALL)
    finally:
        try:
            sys.stdout.flush()
            sys.stdout.close()
        except Exception:
            pass
        sys.exit(0)

#def close_daemon_threads():
#    print(Fore.MAGENTA + 'closing daemon threads…' + Style.RESET_ALL)
#    try:
#        # wait for all non-daemon threads to exit
#        for thread in threading.enumerate():
#            if thread is not threading.main_thread() and not thread.daemon:
#                print(Fore.MAGENTA + 'waiting for thread {} to finish…'.format(thread.name) + Style.RESET_ALL)
#                thread.join(timeout=5)
#            else:
#                print(Fore.MAGENTA + 'thread {} seems okay…'.format(thread.name) + Style.RESET_ALL)
#
#        for thread_id, frame in sys._current_frames().items():
#            thread = next((t for t in threading.enumerate() if t.ident == thread_id), None)
#            if thread:
#                print("stack trace for thread (held):")
#                traceback.print_stack(frame)
#    except Exception as e:
#        print(Fore.RED + "error closing daemon threads: {}".format(e) + Style.RESET_ALL)
#    finally:
#        print(Fore.MAGENTA + 'daemon threads closed.' + Style.RESET_ALL)

# main ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
def main(argv):
    global _krzos

#   print("😻 a. Threads at start:", threading.enumerate())

    _log = Logger("main", Level.INFO)
#   if threading.current_thread() is threading.main_thread():
#       signal.signal(signal.SIGINT, signal_handler)
#       _log.info("signal handler registered.")
#   else:
#       _log.warning("not running in the main thread: signal handling will not work.")

#   print("😻 b. Threads at start:", threading.enumerate())

    _suppress = False
    try:
        _args = parse_args()
        if _args == None:
            print('')
            _log.info('arguments: no action.')
        elif _args == -1:
            _suppress = True # help or docs
        else:
            print('😻 c.')
            # write log_to_file to global symbol table
            _level = Level.from_string(_args.level) if _args.level != None else Level.INFO
            _log.level = _level
            _log.debug('arguments: {}'.format(_args))
            print('😻 d.')
            _krzos = KRZOS(level=_level)
            print('😻 e.')
            if _args.configure or _args.start:
                _krzos.configure(_args)
                if not _args.start:
                    _log.info('configure only: ' + Fore.YELLOW + 'specify the -s argument to start krzos.')
            print('😻 f.')
            if _args.start:
                _krzos.start()
            print('😻 g.')
            # krzos is now running…
    except KeyboardInterrupt:
        print('\n')
        print(Fore.MAGENTA + Style.BRIGHT + 'caught Ctrl-C; exiting…' + Style.RESET_ALL)
    except Exception:
        print(Fore.RED + Style.BRIGHT + 'error starting krzos: {}'.format(traceback.format_exc()) + Style.RESET_ALL)
    finally:
        if _krzos and not _krzos.closed:
            _krzos.close()
        print('😻 z.')

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
if __name__== "__main__":
    main(sys.argv[1:])

# prevent Python script from exiting abruptly
#signal.pause()

#EOF
