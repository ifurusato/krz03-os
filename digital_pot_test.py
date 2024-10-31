#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2020-10-05
# modified: 2024-05-17
#
# Tests the digital potentiometer.
#

import pytest
import sys, traceback
from datetime import datetime as dt
from math import isclose
from colorama import init, Fore, Style
init()

from core.rate import Rate
from core.logger import Logger, Level
from core.config_loader import ConfigLoader
from hardware.i2c_scanner import I2CScanner, DeviceNotFound
from hardware.digital_pot import DigitalPotentiometer
    
_log = Logger('test', Level.INFO)
    
# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
def key_callback(event):
    _log.info('callback on event: {}'.format(event))

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
@pytest.mark.unit
def test_digital_potentiometer():

    _start_time = dt.now()
    
    MIN_RANGE =   0.0
    MAX_RANGE =   0.2
    _i2c_address = 0x0C

    try:

        # read YAML configuration
        _level = Level.INFO
        _loader = ConfigLoader(_level)
        filename = 'config.yaml'
        _config = _loader.configure(filename)
        
#       _i2c_scanner = I2CScanner(_config, level=_level)
#       if _i2c_scanner.has_hex_address([_i2c_address]):
#       if _i2c_scanner.has_address([_i2c_address]):

        _log.info('using digital potentiometer...')
        # configure digital potentiometer for motor speed
        _pot = DigitalPotentiometer(_config, i2c_address=None, level=_level)

#           _pot.set_output_range(-180, 180) 
        _pot.set_output_range(MIN_RANGE, MAX_RANGE)

#       else:
#           raise Exception('no digital potentiometer available.')
#           _log.info('using mock potentiometer...')
#           _pot = MockPotentiometer(_config, key_callback, Level.INFO)

#       sys.exit(0)
        _last_scaled_value = 0.0
        _log.info('starting test...')
        _hz = 20
        _rate = Rate(_hz, Level.ERROR)
        while True:
            _scaled_value = _pot.get_scaled_value(False)
            if _scaled_value != _last_scaled_value: # if not the same as last time
                # math.isclose(3, 15, abs_tol=0.03 * 255) # 3% on a 0-255 scale
#               if isclose(_scaled_value, 0.0, abs_tol=0.05 * 90):
#                   _pot.set_black()
#                   _log.info(Fore.YELLOW + Style.DIM + 'scaled value: {:9.6f}'.format(_scaled_value))
#               else:
                _pot.set_rgb(_pot.value)
                _log.info(Fore.YELLOW + Style.NORMAL + 'output: {:7.4f}'.format(_scaled_value))
            _last_scaled_value = _scaled_value
            _rate.wait()
    
    except KeyboardInterrupt:
        _log.info('Ctrl-C caught; exiting...')
    except DeviceNotFound as e:
        _log.error('no potentiometer found, exiting.')
    except Exception as e:
        _log.error('{} encountered, exiting: {}'.format(type(e), e))
        traceback.print_exc(file=sys.stdout)
    finally:
        pass
    
    _elapsed_ms = round(( dt.now() - _start_time ).total_seconds() * 1000.0)
    _log.info(Fore.YELLOW + 'complete: elapsed: {:d}ms'.format(_elapsed_ms))
    
# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
def main():
    try:
        test_digital_potentiometer()
    except Exception as e:
        print(Fore.RED + 'error in motor test: {}'.format(e) + Style.RESET_ALL)
        traceback.print_exc(file=sys.stdout)
    finally:
        pass
        
if __name__== "__main__":
    main()
    
#EOF
