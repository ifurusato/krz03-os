#!/usr/bin/env python
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

import asyncio
import time
from hardware.distance_sensor import DistanceSensor

from core.logger import Logger, Level
from core.orientation import Orientation
from core.config_loader import ConfigLoader

async def main():

    _config = ConfigLoader(Level.INFO).configure()
    _port_sensor = DistanceSensor(_config, Orientation.PORT)
    _cntr_sensor = DistanceSensor(_config, Orientation.CNTR)
    _stbd_sensor = DistanceSensor(_config, Orientation.STBD)
    _sensors = [ _port_sensor, _cntr_sensor, _stbd_sensor ]

    # start all sensors
    for _sensor in _sensors:
        _sensor.enable()

    try:
        print("measuring distances…")
        while True:
            distances = []
            for _sensor in _sensors:
                distance_mm = _sensor.get_distance()
                if distance_mm > 0:
                    distances.append("{:>8}mm".format(distance_mm))
                else:
                    distances.append("         ·")

            result_line = " | ".join(distances)
            print(f"\r{' ' * 100}", end="")  # Clear the current line
            print(f"\r\t{result_line}", end="")  # Print the new result

            time.sleep(0.1) # adjust delay as needed
#           await asyncio.sleep(0.1)

    except asyncio.CancelledError:
        # handle task cancellation
        pass
    finally:
        for _sensor in _sensors:
            _sensor.stop()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nCtrl-C caught, exiting…")

