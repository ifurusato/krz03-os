#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-11-15
# modified: 2024-11-15
#
# This script seems necessary as defining the Inventor HAT Mini within a class
# doesn't seem to work.
#

from inventorhatmini import InventorHATMini

from colorama import init, Fore, Style
init()

from core.config_loader import ConfigLoader
from core.logger import Level

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

_config = ConfigLoader(Level.INFO).configure()
_motor_gear_ratio = _config.get('krzos').get('motor').get('gear_ratio')

aft_controller = InventorHATMini(address=0x17, motor_gear_ratio=_motor_gear_ratio, init_servos=False, init_leds=False)
fwd_controller = InventorHATMini(address=0x16, motor_gear_ratio=_motor_gear_ratio, init_servos=False, init_leds=False)

#EOF
