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
from hardware.distance_sensors import DistanceSensors
from hardware.distance_sensor import DistanceSensor

from core.logger import Logger, Level
from core.orientation import Orientation
from core.config_loader import ConfigLoader

RAW      = False
WEIGHTED = True

async def main():

    _config = ConfigLoader(Level.INFO).configure()
    _sensors = DistanceSensors(_config)

    _port_sensor = _sensors.get(Orientation.PORT)
    _cntr_sensor = _sensors.get(Orientation.CNTR)
    _stbd_sensor = _sensors.get(Orientation.STBD)

    # start all sensors
    _sensors.enable()

    print("found {} sensors.".format(len(_sensors.sensors)))  # This should print 3 if you have 3 sensors

    try:
        print("measuring distances…")
        while True:
            if RAW:

#               _port, _cntr, _stbd = _sensors.all
                trio = _sensors.all
#               print("port: {:10.2f};   cntr: {:10.2f};   stbd: {:10.2f}".format(*trio))
                print("port: {};   cntr: {};   stbd: {}".format(*trio))

            elif WEIGHTED:

                _port, _stbd = _sensors.get_weighted_averages()
                print("port: {:4.2f};\t stbd: {:4.2f}".format(_port, _stbd))

            else:
                distances = []
                for _sensor in _sensors:
                    distance_mm = _sensor.distance
                    if distance_mm is None:
                        distances.append("       eor")
                    elif distance_mm > 0:
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

