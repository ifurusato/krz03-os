#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-04-24
# modified: 2025-05-10
#

import time
from datetime import datetime as dt
from colorama import init, Fore, Style
init()

from core.component import Component
from core.logger import Logger, Level

from hardware.eyeballs import PalpebralMovement

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class MacroProcessor(Component):
    '''
    A processor that imports a text file and processes it as a macro,
    line-by-line executed as commands.

    :param config:            the application configuration
    :param motor_controller:  the optional robot's motor controller
    :param eyeballs:          the eyeballs display
    :param level:             the logging level
    '''
    def __init__(self, config=None, motor_controller=None, eyeballs=None, level=Level.INFO):
        self._log = Logger('macro', level)
        Component.__init__(self, self._log, suppressed=False, enabled=True)
        _cfg = config['krzos'].get('behaviour').get('macro_processor') 
        self._motor_ctrl = motor_controller
        self._eyeballs = eyeballs
        self._post_delay_ms = _cfg.get('post_delay_ms', 100) # after every payload sent, delay by this much
        self._lines = None
        self._log.info('ready.')

    def load_macro(self, filename):
        if not filename.lower().endswith('.txt'):
            raise ValueError("invalid file extension: expected a '.txt' file, got '{}'.".format(filename))
        self._log.info("loading macro file: '{}'…".format(filename))
        self._lines = None # reset
        try:
            with open(filename, 'r') as file:
                lines = [line.strip() for line in file if line.strip() and not line.strip().startswith('#')]
            self._log.info("loaded {} lines from macro file.".format(len(lines)))
            self._lines = lines
        except FileNotFoundError:
            self._log.error("macro file '{}' not found.".format(filename))
            return None

    def execute(self):
        if self._lines == None:
            self._log.warning('cannot continue: no macro file loaded.')
        elif len(self._lines) == 0:
            self._log.warning('cannot continue: macro file contained no actionable lines.')
        else:
            start_time = dt.now()
            line_number = 0
            try:
                for line in self._lines:
                    line_number += 1
                    parts = line.split()
                    command = parts[0]
                    self._log.info(Style.DIM + "processing command '{}' with {} parts…".format(command, len(parts)))
                    if command == 'wait':
                        duration = parts[1]
                        self._log.info(Style.DIM + "wait {}…".format(duration))
                        time.sleep(float(duration))
                    elif command == 'play':
                        sound = parts[1]
                        self._log.info(Style.DIM + "play '{}'…".format(sound))
                        if self._motor_ctrl:
                            self._motor_ctrl.play(parts[1])
                    elif command.startswith("eyeballs"):
                        pm = PalpebralMovement.from_name(parts[1])
                        arg1 = None if len(parts) < 2 else parts[1]
                        arg2 = None if len(parts) < 3 else parts[2]
                        self._log.info(Style.DIM + "eyeballs '{}'…".format(pm.name))
    #                   self._log.info("eyeballs arg1 '{}'…; arg2: '{}'".format(arg1, arg2))
                        if arg1 == 'dead' and arg2 == 'fade': # special case
                            self._eyeballs.dead(include_fade=True)
                        else:
                            self._eyeballs.show(pm)

                    elif command.startswith("lights"):
                        if parts[1] == 'on':
                            self._log.info(Style.DIM + "lights on…")
                            if self._motor_ctrl:
                                self._motor_ctrl.lights(True)
                        elif parts[1] == 'off':
                            self._log.info(Style.DIM + "lights off…")
                            if self._motor_ctrl:
                                self._motor_ctrl.lights(False)
                        else: # ignored
                            self._log.warning("unrecognised command '{}'.".format(command))
                    elif self._motor_ctrl:
                        try:
                            port_speed = float(parts[1]) if len(parts) > 1 else 0.0
                            stbd_speed = float(parts[2]) if len(parts) > 2 else 0.0
                            duration   = float(parts[3]) if len(parts) > 3 else 0.0
                            payload    = self._motor_ctrl.get_payload(command, port_speed, stbd_speed, duration)
                            self._log.info("sending payload with command: '{}'; port: {}; stbd: {}; duration: {}…".format(command, port_speed, stbd_speed, duration))
                            _start_time = dt.now()
                            response = self._motor_ctrl.write_payload(payload)
                            elapsed_ms = (dt.now() - _start_time).total_seconds() * 1000.0
                            if response.value <= response.OKAY.value:
                                self._log.info("response: {}; {} ms elapsed.".format(response.name, int(elapsed_ms)))
                            else:
                                self._log.error("error: {}; {} ms elapsed.".format(response.name, int(elapsed_ms)))
                            time.sleep(self._post_delay_ms / 1000.0)

                        except Exception as e:
                            self._log.error("{} raised parsing '{}' of macro on line {}: {}".format(type(e), line, line_number, e))

            except Exception as e:
                self._log.error("{} raised processing macro on line {}: {}".format(type(e), line_number, e))

            # complete
            elapsed_seconds = (dt.now() - start_time).total_seconds()
            if elapsed_seconds < 2:
                self._log.info("macro complete: {:.2f}ms elapsed.".format(elapsed_seconds * 1000))
            else:
                self._log.info("macro complete: {:.2f}s elapsed.".format(elapsed_seconds))

#EOF
