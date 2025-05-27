#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2020-10-26
# modified: 2024-10-31
#
# 10 runs, average 1228 steps over 200mm, so 614 steps per 100mm or 6.14 steps/mm

import sys, signal
import asyncio
from datetime import datetime as dt
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.orientation import Orientation
from core.config_loader import ConfigLoader
from hardware.i2c_scanner import I2CScanner, DeviceNotFound
from hardware.matrix import Matrix
from hardware.nofs import NearOpticalFlowSensor

# main ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

_log = Logger('test', Level.INFO)

# Constants for matrix dimensions
MATRIX_WIDTH = 11
MATRIX_HEIGHT = 7
CENTER_X = MATRIX_WIDTH // 2
CENTER_Y = MATRIX_HEIGHT // 2
DECAY_RATE = 5  # rate at which values decay towards zero (units per loop)
DECAY_INTERVAL = 0.1  # Time interval (seconds) to wait before applying decay
MAX_SENSOR_VALUE = 100  # maximum sensor value range

def map_value_to_matrix(value, max_value, center, matrix_size):
    '''
    Map a value from -max_value to max_value to a matrix index.
    '''
    normalized = value / (2 * max_value)
    index = round(normalized * (matrix_size - 1)) + center
    return max(0, min(index, matrix_size - 1))

def plot_on_matrix(xd, yd, swap_axes=False):
    '''
    Map xd, yd values to matrix indices and swap axes if needed.
    '''
    if swap_axes:
        xd, yd = yd, xd  # Swap X and Y if needed
    x_index = map_value_to_matrix(xd, 100, CENTER_X, MATRIX_WIDTH)
    y_index = map_value_to_matrix(yd, 100, CENTER_Y, MATRIX_HEIGHT)
    return x_index, y_index

def decay_to_zero(value, decay_rate):
    '''
    Gradually reduce a value to zero by the decay_rate.
    '''
    if value > 0:
        return max(0, value - decay_rate)
    elif value < 0:
        return min(0, value + decay_rate)
    return 0

def accumulate_and_clamp(value, delta, max_value):
    '''
    Accumulate the value and clamp it to the range of -max_value to max_value.
    '''
    value += delta
    return max(-max_value, min(value, max_value))



# .............................................................................
async def main():

    USE_MATRIX = False
    _nofs = None

    try:
        print('🦊 a.')

        _config = ConfigLoader(Level.INFO).configure()
        _nofs = NearOpticalFlowSensor(_config, level=Level.INFO)
        _matrix = None
        print('🦊 b.')
        _i2c_scanner = I2CScanner(_config, level=Level.INFO)
        _brightness = 0.7
        print('🦊 c.')
        if USE_MATRIX and _i2c_scanner.has_hex_address(['0x75']):
            print('🦊 c1.')
            _matrix = Matrix(Orientation.STBD)
            print('🦊 c2.')
            _matrix.set_brightness(_brightness)
            _matrix.pixel(CENTER_X, CENTER_Y, _brightness)
            _matrix.show()
            print('🦊 c3.')
        print('🦊 d.')

        _log.info("""Detect flow/motion in front of the PAA5100JE sensor.

    Press Ctrl+C to exit!
    """)

        print('🦊 e.')
        _swap_axes = True
        x_max_mm = 0.0
        y_max_mm = 0.0
        xd, yd = 0, 0  # Initialize display values
        last_move_time = dt.now().timestamp()  # Track the last time the robot moved

        print('🦊 f.')
        _log.info(Fore.GREEN + 'starting nofs loop…')
        await _nofs.start()

        print('🦊 g.')
        while True:
            try:

                x, y = _nofs.absolute()
                x_mm, y_mm = _nofs.millimeters()

                x_max_mm = max(abs(x_mm), x_max_mm)
                y_max_mm = max(abs(y_mm), y_max_mm)
                perc = _nofs.y_variance()
                _log.info("absolute: x {:03d} y {:03d}; ".format(x, y) 
                        + Fore.WHITE + ' {:d}%; '.format(perc)
                        + Fore.GREEN + "dist: x {:03d}mm, y {:03d}mm; ".format(x_mm, y_mm)
                        + Style.DIM + "max: x {:d}mm, y {:d}mm; ".format(x_max_mm, y_max_mm))

                current_time = dt.now().timestamp()
     
                if x_mm != 0 or y_mm != 0:
                    # Accumulate and clamp the sensor data
                    xd = accumulate_and_clamp(xd, x_mm, MAX_SENSOR_VALUE)
                    yd = accumulate_and_clamp(yd, y_mm, MAX_SENSOR_VALUE)
                    
                    # Update the last move time when movement is detected
                    last_move_time = current_time
                else:
                    # Apply decay if no movement has been detected for a while
                    if current_time - last_move_time > DECAY_INTERVAL:
                        xd = decay_to_zero(xd, DECAY_RATE)
                        yd = decay_to_zero(yd, DECAY_RATE)

                xy = plot_on_matrix(xd, yd, _swap_axes)
                if _matrix:
                    _log.info(Fore.MAGENTA + 'plot: {: 2d}, {: 2d}'.format(xd, yd))
                    _matrix.pixel(xy[0], xy[1], _brightness, update=True)

            except RuntimeError as re:
                _log.error('runtime error in nofs loop: {}\n{}'.format(e, traceback.format_exc()))
                continue
            finally:
                await asyncio.sleep(1)

        print('🦊 h.')
    except KeyboardInterrupt:
        print('\n')
        _log.info('caught Ctrl-C: exiting…')
    except Exception as e:
        _log.error('{} thrown executing nofs: {}\n{}'.format(type(e), e, traceback.format_exc()))
    finally:
        if _nofs:
            _nofs.close()
            await _nofs.stop()
        pass

# main ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

def handle_exit_signal(loop, tasks):
    print(Fore.BLUE + 'handle_exit_signal.')
    # cancel the running tasks gracefully
    for task in tasks:
        task.cancel()

async def shutdown(loop, tasks):
    print(Fore.BLUE + 'shutdown.')
    # wait for tasks to complete their cancellation
    await asyncio.gather(*tasks, return_exceptions=True)
    loop.stop()

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    try:
        tasks = []
        tasks.append(loop.create_task(main()))
        signal.signal(signal.SIGINT, lambda sig, frame: handle_exit_signal(loop, tasks))
        loop.run_until_complete(shutdown(loop, tasks))
    except KeyboardInterrupt:
        print("Program interrupted by user (Ctrl+C). Exiting gracefully.")
    finally:
        # Close the loop after tasks are finished
        loop.close()
        sys.exit(0)

#EOF
