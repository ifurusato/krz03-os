#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-11-05
# modified: 2024-11-05
#

import sys, traceback
import time
from colorama import init, Fore, Style
init()

from core.config_loader import ConfigLoader
from core.logger import Logger, Level
from hardware.tof import TimeOfFlight

# main ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

_tof = None
_log = Logger('test', Level.INFO)

try:

    # read YAML configuration
    _config = ConfigLoader(Level.INFO).configure()

    _tof = TimeOfFlight(_config, level=Level.INFO)
    _tof.enable()

    # main loop
    while _tof.enabled:
        _distance_in_mm = _tof.poll()
        _log.info(Fore.GREEN + "distance: {}mm".format(_distance_in_mm))
        time.sleep(0.1)

except KeyboardInterrupt:
    print('\n')
    _log.info('Ctrl-C caught; exiting…')
    _tof.disable()
except Exception as e:
     _log.error('{} encountered, exiting: {}\n{}'.format(type(e), e, traceback.format_exc()))
finally:
    if _tof:
        _tof.close()

#EOF
