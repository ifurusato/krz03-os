#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-06-02
# modified: 2025-06-08
#
# A test for the controller for a DFRobot Brushless DC Motor.
#

import time
import traceback
from datetime import datetime as dt
from colorama import init, Fore, Style
init()

import pigpio

from core.logger import Logger, Level
from hardware.pigpiod_util import PigpiodUtility
from hardware.digital_potentiometer import DigitalPotentiometer # TEMP replace with digital_pot.py
from hardware.brushless_motor import BrushlessMotor

# support methods ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

def read_scaled_speed(digital_pot):
    normalized_value = (digital_pot.value * 2) - 1  # scale 0..1 to -1..+1
    speed = round(BrushlessMotor.MOTOR_MAX_RPM * normalized_value)
    return speed

# main ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

_log = Logger('main', level=Level.INFO)
_pi          = None
_motor       = None
_digital_pot = None

try:
    if not PigpiodUtility.is_pigpiod_running():
        PigpiodUtility.ensure_pigpiod_is_running()
        time.sleep(1)
    _pi = pigpio.pi()
    if not _pi.connected:
        _log.info("Failed to connect to pigpio daemon")
        exit()

    _digital_pot = DigitalPotentiometer()
    _digital_pot.start()
    _motor = BrushlessMotor(_pi)

    _target_distance_mm = 500
    if _target_distance_mm > 0:
        speed = read_scaled_speed(_digital_pot)
        if _motor.is_closed_loop_enabled:
            _motor.set_target_rpm(speed, _target_distance_mm)
        else:
            _motor.set_speed(speed, _target_distance_mm)

    while True:
        speed = read_scaled_speed(_digital_pot)
        if speed == 0.0:
            _digital_pot.off()
        if _motor.is_closed_loop_enabled:
            _motor.set_target_rpm(speed)
        else:
            _motor.set_speed(speed)
        time.sleep(0.2)

except Exception as e:
    _log.error('{} raised by motor controller: {}\n{}'.format(type(e), e, traceback.format_exc()))
finally:
    if _digital_pot:
        _digital_pot.stop()
    if _motor:
        _log.info("stopping motor…")
        _motor._stop_rpm_control()
        _motor.stop()
        _motor.close()
    if _pi:
        _pi.stop()
        _log.info("pigpio connection stopped.")
    _log.info("complete.")

#EOF
