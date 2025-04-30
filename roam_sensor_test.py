#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-04-02
# modified: 2025-04-29
#

import sys, traceback
import time
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.config_loader import ConfigLoader
from hardware.button import Button
from hardware.roam_sensor import RoamSensor

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

enabled = True
_log = Logger('main', Level.INFO)

def shutdown(arg):
    global enabled
    _log.info(Fore.GREEN + 'shutdown!' + Style.RESET_ALL)
    enabled = False

# Assuming self._mean is a list of 8 numeric values
def print_colored_mean(mean):
    formatted_values = []
    for val in mean:
        if val < 300:
            # Highlight with RED if less than 64
            formatted_values.append(Style.BRIGHT + f"{val:<4}" + Style.NORMAL)
        else:
            formatted_values.append(f"{val:<4}")
    print(Fore.CYAN + "mean dist mm:  " + Fore.WHITE + "  ".join(formatted_values) + Style.RESET_ALL)

def main(argv):
    global enabled

    _roams = None
 
    try:

        _log.info('begin…')
        _config = ConfigLoader(Level.INFO).configure()

        # set up button for shut down
        _pin = 17
        _button = Button(config=_config, pin=_pin, momentary=True, level=Level.INFO)
        _button.add_callback(shutdown, 500)

        # set up sensor
        _skip_init = True
        _roams = RoamSensor(_config, skip_init=_skip_init, level=Level.INFO)

        _log.info('begin loop… (press button to exit)')
        _roams.enable()

        while enabled:
#           _mean = _roams.mean
#           print_colored_mean(_mean)
#           print(Fore.WHITE + "mean dist mm:  {:<4}  {:<4}  {:<4}  {:<4}  {:<4}  {:<4}  {:<4}  {:<4}".format(*_mean) + Style.RESET_ALL)
            time.sleep(0.1)

        _roams.disable()

    except KeyboardInterrupt:
        _log.info('caught Ctrl-C; exiting…')
    except Exception:
        _log.error('error in script: {}'.format(traceback.format_exc()))
    finally:
        if _roams:
            _roams.close()
        _log.info('complete.')

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
if __name__== "__main__":
    main(sys.argv[1:])

#EOF
