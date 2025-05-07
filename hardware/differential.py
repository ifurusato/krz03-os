#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#   
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-05-07
# modified: 2025-05-07
#
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from hardware.motor_controller import MotorController

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class DifferentialDrive(MotorController):
    '''
    Wraps a MotorController to treat the four motors as a
    differential drive, port and starboard, maintaining a
    record of the past target speeds sent to the Motoro 2040.

    Currently mocked.
    '''
    def __init__(self, config=None, level=Level.INFO):
        MotorController.__init__(self, config, level=level)
        self._log = Logger('differ', level)
        self._port_speed = 0.0
        self._stbd_speed = 0.0
        self._log.info('ready.')

    def enable(self):
        MotorController.enable(self)
        self._log.info('enable.')

    def get_speeds(self):
        return self._port_speed, self._stbd_speed

    def set_speeds(self, port_speed, stbd_speed):
        self._port_speed = port_speed
        self._stbd_speed = stbd_speed
        self._log.info(Fore.WHITE + 'set speeds; port: {:4.2f}; stbd: {:4.2f}'.format(port_speed, stbd_speed))
        self.send_payload('forw', port_speed, stbd_speed, 0.0)

    def print(self):
        print(
            "    {}{:<8}{} {}{:>5.2f}{}   {}{:<8}{} {}{:>5.2f}{}".format(
                Fore.CYAN, "port:", Style.RESET_ALL, Fore.RED, port_speed, Style.RESET_ALL,
                Fore.CYAN, "stbd:", Style.RESET_ALL, Fore.GREEN, stbd_speed, Style.RESET_ALL
            )
        )

    def disable(self):
        MotorController.disable(self)
        self._log.info('disable.')

#EOF
