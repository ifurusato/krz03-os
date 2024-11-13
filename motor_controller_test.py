#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-11-07
# modified: 2024-11-07
#
# Sets the I2C address of an IO Expander device to a new address.
#
# See: https://github.com/pimoroni/ioe-python/blob/master/REFERENCE.md#function-reference
#

import sys, traceback
import time
from colorama import init, Fore, Style
init()

from inventorhatmini import InventorHATMini, MOTOR_A, MOTOR_B
from ioexpander.common import NORMAL_DIR, REVERSED_DIR

from core.logger import Logger, Level
from core.util import Util
from core.direction import Direction
from core.orientation import Orientation
from core.config_loader import ConfigLoader
from hardware.motor_controller import MotorController
from hardware.speed_dto_factory import SpeedDTOFactory
from hardware.button import Button

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

WAIT_FOR_BUTTON   = True
_motor_controller = None

_log = Logger('motor-test', Level.INFO)

try:

    _config = ConfigLoader(Level.INFO).configure()
    _cfg = _config.get('krzos').get('motor')
    _motor_gear_ratio = _cfg.get('gear_ratio')
    _log.info(Fore.GREEN + 'gear ratio: {}'.format(_motor_gear_ratio))

    _btn = Button(_config, level=Level.INFO)

    if WAIT_FOR_BUTTON:
        _log.info(Fore.GREEN + 'waiting for button push…')
        while not _btn.pushed():
            time.sleep(0.1)

    aft_controller = InventorHATMini(address=0x17, motor_gear_ratio=_motor_gear_ratio, init_servos=False, init_leds=False)
    fwd_controller = InventorHATMini(address=0x16, motor_gear_ratio=_motor_gear_ratio, init_servos=False, init_leds=False)

    _enable_pid = False
    _log.info('creating motor controller…')
    _motor_controller = MotorController(_config, fwd_controller, aft_controller, enable_pid=_enable_pid, level=Level.INFO)


    _log.info('accelerate to 0.7 speed…')
    _motor_controller.set_motor_speed(SpeedDTOFactory.create(Direction.AHEAD, speed=0.7))

    time.sleep(1)

#   print('brake…')
#   _motor_controller.brake()

#   _log.info('crab-port to 0.7 speed…')
#   _motor_controller.set_motor_speed(SpeedDTOFactory.create(Direction.CRAB_PORT, speed=0.7))

    print('coast to stop…')
    _motor_controller.coast()

#   time.sleep(1)

#   _log.info('accelerate to -0.6 speed…')
#   _motor_controller.set_motor_speed(SpeedDTOFactory.create(Direction.ASTERN, speed=0.6))

#   time.sleep(1)

#   print('brake…')
#   _motor_controller.brake()

except KeyboardInterrupt:
    print('\n')
    _log.info('Ctrl-C caught; exiting…')
except Exception as e:
     _log.error('{} encountered, exiting: {}\n{}'.format(type(e), e, traceback.format_exc()))
finally:
    if _motor_controller:
        _motor_controller.close()
    
#EOF
