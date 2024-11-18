#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-11-16
# modified: 2024-11-18
#

import time
from hardware.distance_sensor import DistanceSensor

# main ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
def main():

    # GPIO pins for sensors
    pins = [16, 20, 21]
    sensors = [DistanceSensor(pin, smoothing=True, smoothing_window=5) for pin in pins]

    # define fixed column widths
    distance_column_width     = 10  # Width for distance values
    out_of_range_column_width = 10  # Width for "out of range" messages

    try:

        print("Measuring distances...")
        while True:
            distances = []
            for sensor in sensors:
                distance_mm = sensor.get_distance()
                if distance_mm is not None:
                    distances.append("{:>{}.1f}mm".format(distance_mm, distance_column_width-2))
                else:
                    distances.append("{:>{}}".format("----", distance_column_width))

            # Print distances
            result_line = " | ".join(distances)
            print(f"\r{' ' * 100}", end="")  # Clear the current line
            print(f"\r\t{result_line}", end="")  # Print the new result

            time.sleep(0.1)  # Adjust delay as needed

    except KeyboardInterrupt:
        print("\nCtrl-C caught, exiting…")
    finally:
        for sensor in sensors:
            sensor.stop()

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
if __name__ == "__main__":
    main()

