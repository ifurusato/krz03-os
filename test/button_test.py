#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-05-22
# modified: 2024-10-31
#
# A test script for the Button class.
#

import sys, traceback
import time
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.config_loader import ConfigLoader
from hardware.button import Button

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

def shutdown(arg):
    global enabled, count
    count += 1
    _log.info(Fore.BLUE + 'pushed? {}; arg: {}; count: {}'.format(_button.pushed(), arg, count) + Style.RESET_ALL)
    if count > 2:
        _log.info(Fore.GREEN + 'shutdown!' + Style.RESET_ALL)
        enabled = False

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

_log   = Logger('main', Level.INFO)
_button = None
enabled = True
count   = 0

try:

    _momentary = True

    # lever/pushbutton = GPIO17
    # toggle 1         = GPIO18
    # toggle 2         = GPIO21
    _pin = 17

    _log.info('button begin…')
    _config = ConfigLoader(Level.INFO).configure()
    _button = Button(config=_config, pin=_pin, momentary=_momentary, level=Level.INFO)
    _button.add_callback(shutdown, 700)

    _old_value = None
    while enabled:
        time.sleep(0.1)

except KeyboardInterrupt:
    print('\n')
    print('Ctrl-C caught, exiting…')
except RuntimeError as rte:
    _log.error('runtime error in test: {}'.format(rte))
except Exception as e:
    _log.error('error in test: {}'.format(e))
    traceback.print_exc(file=sys.stdout)
finally:
    pass
        
#EOF
