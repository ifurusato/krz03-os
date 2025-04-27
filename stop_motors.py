#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-11-07
# modified: 2024-11-07
#

from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.orientation import Orientation
from core.config_loader import ConfigLoader
from hardware.pz_motor_controller import MotorController

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

try:
    # read YAML configuration
    _config = ConfigLoader(Level.INFO).configure()
    _motor_controller = MotorController(_config)
    _motor_controller.enable()
    _delay_sec = 1.5

#       _orientations = [ Orientation.PORT, Orientation.STBD, Orientation.PFWD, Orientation.SFWD, Orientation.PAFT, Orientation.SAFT ]
    _orientations = [ Orientation.PORT, Orientation.STBD ]
    for _orientation in _orientations:
        _motor_controller.set_motor_speed(_orientation, 0)
        time.sleep(0.33)

    _motor_controller.disable()

except Exception as e:
    print(Fore.RED + 'error:{}'.format(e) + Style.RESET_ALL)
finally:
    print(Fore.CYAN + 'complete.' + Style.RESET_ALL)
#EOF
