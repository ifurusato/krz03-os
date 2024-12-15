#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-05-22
# modified: 2024-12-15
#
# A test script for the Killswitch class.
#

import sys, traceback
import time
from colorama import init, Fore, Style
init()

from core.logger import Level
from core.config_loader import ConfigLoader
#from hardware.button import Button
from hardware.killswitch import Killswitch

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
enabled = True

def shutdown():
    global enabled
    print('shutdown!')
    enabled = False

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
def main():
    global enabled
    try:

        print('killswitch begin…')

        _config = ConfigLoader(Level.INFO).configure()
        _killswitch = Killswitch(config=_config, callback=shutdown, level=Level.INFO)

        _old_value = None
        while enabled:
            time.sleep(0.1)

    except KeyboardInterrupt:
        print('\n')
        print('Ctrl-C caught, exiting…')

    except Exception as e:
        print(Fore.RED + 'error in motor test: {}'.format(e) + Style.RESET_ALL)
        traceback.print_exc(file=sys.stdout)
    finally:
        pass
        
if __name__== "__main__":
    main()
    
#EOF
