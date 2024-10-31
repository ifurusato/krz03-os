#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2024 by Murray Altheim. All rights reserved. This file is part of
# the Robot Operating System project and is released under the "Apache Licence,
# Version 2.0". Please see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2020-10-26
# modified: 2024-10-26
#
# 10 runs, average 1228 steps over 200mm, so 614 steps per 100mm or 6.14 steps/mm

import time
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.config_loader import ConfigLoader
from hardware.nofs import NearOpticalFlowSensor

# main ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

_log = Logger('test', Level.INFO)

try:

    _config = ConfigLoader(Level.INFO).configure()
    _nofs = NearOpticalFlowSensor(_config, level=Level.INFO)

    _log.info("""Detect flow/motion in front of the PAA5100JE sensor.

Press Ctrl+C to exit!
""")

    while True:
        try:

            x, y = _nofs.absolute()
            x_mm, y_mm = _nofs.millimeters()
            perc = _nofs.y_variance()
            _log.info("absolute: x {:03d} y {:03d}; ".format(x, y) 
                    + Fore.WHITE + ' {:d}%; '.format(perc)
                    + Fore.GREEN + "dist: x {:03d}mm, y {:03d}mm; ".format(x_mm, y_mm))

        except RuntimeError:
            continue
        time.sleep(0.01)

except KeyboardInterrupt:
    pass

#EOF
