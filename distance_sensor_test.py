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

import asyncio
from hardware.distance_sensor import DistanceSensor

async def main():
    # GPIO pins for sensors
    pins = [16, 20, 21]
    sensors = [DistanceSensor(pin, smoothing=True, smoothing_window=5) for pin in pins]

    # Start all sensors
    for sensor in sensors:
        sensor.start()

    try:
        print("Measuring distances...")
        while True:
            distances = []
            for sensor in sensors:
                distance_mm = sensor.get_distance()
                if distance_mm is not None:
                    distances.append(f"{distance_mm:10.1f}mm")
                else:
                    distances.append(f"{'----':>10}")

            result_line = " | ".join(distances)
            print(f"\r{' ' * 100}", end="")  # Clear the current line
            print(f"\r\t{result_line}", end="")  # Print the new result

            await asyncio.sleep(0.1)  # Adjust delay as needed

    except asyncio.CancelledError:
        # Handle task cancellation (if needed)
        pass
    finally:
        for sensor in sensors:
            sensor.stop()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nCtrl-C caught, exiting…")

