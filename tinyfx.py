#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-08-28
# modified: 2025-04-22
#
# Sends data to the TinyFX, using the TinyFxController.
#

import sys, time, traceback
from colorama import init, Fore, Style
init()

from core.orientation import Orientation
from core.logger import Logger, Level
from core.config_loader import ConfigLoader
from hardware.tinyfx_driver import TinyFxDriver

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
def main():

    _loop   = False
    driver  = None
    _log = Logger('test', Level.INFO)

    setting = sys.argv[1]
    _log.info(Fore.CYAN + Style.DIM + "-- setting '{}'…".format(setting) + Style.RESET_ALL)

    try:

        _log.info('starting test…')

        # read YAML configuration
        _level = Level.INFO
        _config = ConfigLoader(Level.INFO).configure()
        driver = TinyFxDriver(_config, _level)
        driver.command(setting)

    except KeyboardInterrupt:
        _log.info('Ctrl-C caught; exiting…')
    except Exception as e:
        _log.error('error in motor test: {}'.format(e))
        traceback.print_exc(file=sys.stdout)
    finally:
        if driver:
            driver.close()

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

if len(sys.argv) != 2:
    print(Fore.RED
            + "\n  ERROR: expected 1 command line argument: 'on' | 'off' | 'mast' | 'port' | 'stbd' | 'pir get' | 'pir on' | 'pir off' | 'play {key}' | ram | flash"
            + Style.RESET_ALL)
    sys.exit(1)

if __name__== "__main__":
    main()

#EOF
