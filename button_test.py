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
# A test script for the Button class.
#

import sys, traceback
import time
from colorama import init, Fore, Style
init()

from core.logger import Level
from core.config_loader import ConfigLoader
from hardware.button import Button

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
enabled = True
count = 0

def shutdown(arg):
    global enabled, count
    count += 1
    print('arg: {}; count: {}'.format(arg, count))
    if count > 2:
        print('shutdown!')
        enabled = False

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
def main():
    global enabled
    try:

        _momentary = True
        _pin = 17

        print('button begin…')
        _config = ConfigLoader(Level.INFO).configure()
        _button = Button(config=_config, pin=_pin, momentary=_momentary, level=Level.INFO)
        _button.add_callback(shutdown, 500)

        _old_value = None
        while enabled:
            time.sleep(0.1)

    except KeyboardInterrupt:
        print('\n')
        print('Ctrl-C caught, exiting…')
    except RuntimeError as rte:
        print(Fore.RED + 'runtime error in test: {}'.format(rte) + Style.RESET_ALL)
    except Exception as e:
        print(Fore.RED + 'error in test: {}'.format(e) + Style.RESET_ALL)
        traceback.print_exc(file=sys.stdout)
    finally:
        pass
        
if __name__== "__main__":
    main()
    
#EOF
