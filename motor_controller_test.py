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
from core.directive import Directive
from core.orientation import Orientation
from core.config_loader import ConfigLoader
from hardware.motor_controller import MotorController
from hardware.motor_directive_factory import MotorDirectiveFactory
from hardware.button import Button
from controllers import*

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

WAIT_FOR_BUTTON   = True
_motor_controller = None
_wait_button      = None

_log = Logger('motor-test', Level.INFO)

try:

    _config = ConfigLoader(Level.INFO).configure()
#   _motor_gear_ratio = _cfg.get('gear_ratio').get('krzos').get('motor').get('gear_ratio')
#   _log.info(Fore.GREEN + 'gear ratio: {}'.format(_motor_gear_ratio))

    if WAIT_FOR_BUTTON:
        _wait_button = Button(_config, waitable=True)
        _wait_button.wait()
    time.sleep(2)

#   aft_controller = InventorHATMini(address=0x17, motor_gear_ratio=_motor_gear_ratio, init_servos=False, init_leds=False)
#   fwd_controller = InventorHATMini(address=0x16, motor_gear_ratio=_motor_gear_ratio, init_servos=False, init_leds=False)

    _enable_pid = False
    _log.info('creating motor controller…')
    _motor_controller = MotorController(_config, fwd_controller, aft_controller, enable_pid=_enable_pid, level=Level.INFO)

    _directives = [
            MotorDirectiveFactory.create(Directive.AHEAD, speed=0.7), 
            MotorDirectiveFactory.create(Directive.WAIT, duration=1.0), 
            MotorDirectiveFactory.create(Directive.BRAKE), 
            MotorDirectiveFactory.create(Directive.CRAB_PORT, speed=0.7),
            MotorDirectiveFactory.create(Directive.WAIT, duration=1.0), 
            MotorDirectiveFactory.create(Directive.BRAKE), 
            MotorDirectiveFactory.create(Directive.WAIT, duration=1.0), 
            MotorDirectiveFactory.create(Directive.ASTERN, speed=0.7),
            MotorDirectiveFactory.create(Directive.WAIT, duration=1.0), 
            MotorDirectiveFactory.create(Directive.CRAB_STBD, speed=0.7),
            MotorDirectiveFactory.create(Directive.WAIT, duration=1.0), 
            MotorDirectiveFactory.create(Directive.BRAKE)
        ]
    _motor_controller.execute(_directives)

except KeyboardInterrupt:
    print('\n')
    _log.info('Ctrl-C caught; exiting…')
except Exception as e:
     _log.error('{} encountered, exiting: {}\n{}'.format(type(e), e, traceback.format_exc()))
finally:
    if _motor_controller:
        _motor_controller.close()
    if _wait_button:
        _wait_button.close()
    
#EOF
