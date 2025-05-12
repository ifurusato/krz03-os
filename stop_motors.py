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
from core.config_loader import ConfigLoader
from hardware.motor_controller import MotorController

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

_log = Logger('main', Level.INFO)
_motor_controller = None

try:
    # read YAML configuration
    _config = ConfigLoader(Level.INFO).configure()
    _motor_controller = MotorController(_config)
    _motor_controller.enable()

    _motor_controller.stop()

except Exception as e:
    _log.error('error:{}'.format(e))
finally:
    if _motor_controller:
        _motor_controller.disable()
        _motor_controller.close()
    _log.info('complete.')

#EOF
