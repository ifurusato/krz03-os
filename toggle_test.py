#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-05-22
# modified: 2024-10-31
#
# A test script for the toggle switch on GPIO 18.
#

import sys
import time
from colorama import init, Fore, Style
init()

try:
    import RPi.GPIO as GPIO
except Exception:
    print('This script requires the RPi.GPIO module.\nInstall with: sudo pip3 install RPi.GPIO')
    sys.exit(1)

from core.logger import Level
from core.config_loader import ConfigLoader
from hardware.button import Button

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

try:

    print('begin')

    _config = ConfigLoader(Level.INFO).configure()
    _pin = 18
    _btn = Button(_config, pin=_pin, level=Level.INFO)

    _old_value = None
    while True:
        _value = _btn.pushed()
        if _value != _old_value:
            if _value:
                print(Fore.GREEN + 'button: {}'.format(_value))
            else:
                print(Fore.RED   + 'button: {}'.format(_value))
            _old_value = _value
        time.sleep(0.1)

except KeyboardInterrupt:
    print('\n')
    print('Ctrl-C caught, exiting...')

#EOF
