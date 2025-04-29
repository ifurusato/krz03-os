#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-11-16
# modified: 2024-11-18
#

import time
from hardware.distance_sensor import DistanceSensor
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.orientation import Orientation
from core.config_loader import ConfigLoader

# begin ....................................................

_config = ConfigLoader(Level.INFO).configure()

_message_bus = None # can't use in tests

_cntr_sensor = DistanceSensor(_config, Orientation.CNTR, _message_bus)
_cntr_sensor.enable()

try:
    print("measuring distances…")
    while True:
        distance_mm = _cntr_sensor.get_distance()
        if distance_mm > 0:
            print("distance: {}mm".format(distance_mm))
        else:
            print(Style.DIM + "distance: out of range" + Style.RESET_ALL)
        time.sleep(0.1)
#       await asyncio.sleep(0.1)  # Adjust delay as needed
except KeyboardInterrupt:
    _cntr_sensor.disable()
except Exception as e:
    print(e)
finally:
    _cntr_sensor.stop()
    print('complete.')

