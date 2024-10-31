#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-05-20
# modified: 2024-10-24
#

import sys, time, traceback
from math import pi as π
from colorama import init, Fore, Style
init()

from core.cardinal import Cardinal
from core.convert import Convert
from core.orientation import Orientation
from core.logger import Logger, Level
from hardware.i2c_scanner import I2CScanner
from core.config_loader import ConfigLoader
from hardware.icm20948 import Icm20948
from hardware.rgbmatrix import RgbMatrix
from hardware.digital_pot import DigitalPotentiometer # for calibration only

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

CALIBRATE       = True
HEADING_TEST    = False
ACCEL_GYRO_TEST = True

HALF_PI = π / 2.0

_icm20948 = None
_log = Logger('test', Level.INFO)
_cardinal = Cardinal.NORTH
_threshold = 4
_digital_pot = None

try:
    # read YAML configuration
    _config = ConfigLoader(Level.INFO).configure()

    _i2c_scanner = I2CScanner(_config, level=Level.WARN)
    if not _i2c_scanner.has_address([0x74, 0x77]):
        _log.warning('test ignored: no rgbmatrix displays found.')
        sys.exit(1) 
    _addresses = _i2c_scanner.get_int_addresses()
    _enable_port = 0x77 in _addresses
    _enable_stbd = 0x74 in _addresses
 
    _rgbmatrix = RgbMatrix(_enable_port, _enable_stbd, Level.INFO)
    _log.info('starting test...')
#   _port_rgbmatrix = _rgbmatrix.get_rgbmatrix(Orientation.PORT)
#   _stbd_rgbmatrix = _rgbmatrix.get_rgbmatrix(Orientation.STBD)

    if _i2c_scanner.has_address([0x0C]):
        _digital_pot = DigitalPotentiometer(_config, i2c_address=0x0C, level=Level.INFO)
    elif _i2c_scanner.has_address([0x0E]):
        _digital_pot = DigitalPotentiometer(_config, i2c_address=0x0E, level=Level.INFO)
    if _digital_pot:
        _digital_pot.set_output_range(-90.0, 90.0)

#   _rgbmatrix = None

    _icm20948 = Icm20948(_config, rgbmatrix=_rgbmatrix, level=Level.INFO)
    _icm20948._show_console = True
    _icm20948.include_accel_gyro(ACCEL_GYRO_TEST)
    if CALIBRATE:
        if not _icm20948.is_calibrated:
            _icm20948.calibrate()
    _icm20948.include_heading(HEADING_TEST)
    _icm20948.set_poll_rate_hz(2)

    # just scan continually...
    _icm20948.scan(enabled=True, callback=None)

except KeyboardInterrupt:
    _log.info('Ctrl-C caught; exiting…')
except Exception as e:
    _log.error('{} encountered, exiting: {}\n{}'.format(type(e), e, traceback.format_exc()))
finally:
    pass

#EOF
