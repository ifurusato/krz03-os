#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-11-07
# modified: 2024-11-07
#
# Sets the I2C address of an IO Expander device to a new address.
#
# See: https://github.com/pimoroni/ioe-python/blob/master/REFERENCE.md#function-reference
#

import time
import math
from inventorhatmini import InventorHATMini, MOTOR_A, MOTOR_B
from ioexpander.common import NORMAL_DIR, REVERSED_DIR

"""
Demonstrates how to control a motor on Inventor HAT Mini.
"""
SIMPLE_TEST  = False
FIRST_SWEEP  = True
SECOND_SWEEP = False

SWEEPS = 2              # How many speed sweeps of the motor to perform
STEPS  = 10             # The number of discrete sweep steps
STEPS_INTERVAL = 0.5    # The time in seconds between each step of the sequence
SPEED_EXTENT   = 1.0    # How far from zero to drive the motor when sweeping

# Create a new InventorHATMini
# def __init__(self, address=IOE_ADDRESS, motor_gear_ratio=50, init_motors=True, init_servos=True, init_leds=True, start_muted=False):

print('create aft controller...')
aft_controller = InventorHATMini(address=0x17, init_servos=False, init_leds=False)
print('created aft controller.')

print('create fwd controller...')
fwd_controller = InventorHATMini(address=0x16, init_servos=False, init_leds=False)
print('created fwd controller.')

# forward motors ...........................................
pfwd_motor = fwd_controller.motors[MOTOR_A]
sfwd_motor = fwd_controller.motors[MOTOR_B]
pfwd_motor.enable()
pfwd_motor.direction(REVERSED_DIR)
sfwd_motor.enable()
# aft motors ...............................................
paft_motor = aft_controller.motors[MOTOR_A]
saft_motor = aft_controller.motors[MOTOR_B]
paft_motor.enable()
paft_motor.direction(REVERSED_DIR)
saft_motor.enable()

time.sleep(1)

if SIMPLE_TEST:
    print('fwd_port')
    # Drive at full positive
    pfwd_motor.full_positive()
    time.sleep(2)

    # Stop moving
    pfwd_motor.stop()
    time.sleep(2)

    # Drive at full negative
    pfwd_motor.full_negative()
    time.sleep(2)

    # Coast to a gradual stop
    pfwd_motor.coast()
    time.sleep(2)

    print('fwd_stbd')
    # Drive at full positive
    sfwd_motor.full_positive()
    time.sleep(2)

    # Stop moving
    sfwd_motor.stop()
    time.sleep(2)

    # Drive at full negative
    sfwd_motor.full_negative()
    time.sleep(2)

    # Coast to a gradual stop
    sfwd_motor.coast()
    time.sleep(2)

# ..........................................................................

if FIRST_SWEEP:
    # Do a sine speed sweep
    for j in range(SWEEPS):
        for i in range(360):
            pfwd_motor.speed(math.sin(math.radians(i)) * SPEED_EXTENT)
            pfwd_current = fwd_controller.read_motor_current(MOTOR_A)
            print('pfwd current: {:2.1f}A.'.format(pfwd_current))
            time.sleep(0.02)
        for i in range(360):
#           print('sfwd .................')
            sfwd_motor.speed(math.sin(math.radians(i)) * SPEED_EXTENT)
            sfwd_current = fwd_controller.read_motor_current(MOTOR_B)
            print('sfwd current: {:2.1f}A.'.format(sfwd_current))
            time.sleep(0.02)
        for i in range(360):
#           print('paft .................')
            paft_motor.speed(math.sin(math.radians(i)) * SPEED_EXTENT)
            paft_current = aft_controller.read_motor_current(MOTOR_A)
            print('paft current: {:2.1f}A.'.format(paft_current))
            time.sleep(0.02)
        for i in range(360):
            saft_motor.speed(math.sin(math.radians(i)) * SPEED_EXTENT)
            saft_current = aft_controller.read_motor_current(MOTOR_B)
            print('saft current: {:2.1f}A.'.format(saft_current))
            time.sleep(0.02)

if SECOND_SWEEP:
    # Do a stepped speed sweep
    for j in range(SWEEPS):
        for i in range(0, STEPS):
            print('fwd_port')
            pfwd_motor.to_percent(i, 0, STEPS, 0.0 - SPEED_EXTENT, SPEED_EXTENT)
            time.sleep(STEPS_INTERVAL)
        for i in range(0, STEPS):
            print('fwd_port')
            pfwd_motor.to_percent(i, STEPS, 0, 0.0 - SPEED_EXTENT, SPEED_EXTENT)
            time.sleep(STEPS_INTERVAL)
        for i in range(0, STEPS):
            print('fwd_stbd')
            sfwd_motor.to_percent(i, 0, STEPS, 0.0 - SPEED_EXTENT, SPEED_EXTENT)
            time.sleep(STEPS_INTERVAL)
        for i in range(0, STEPS):
            print('fwd_stbd')
            sfwd_motor.to_percent(i, STEPS, 0, 0.0 - SPEED_EXTENT, SPEED_EXTENT)
            time.sleep(STEPS_INTERVAL)

# Disable the motor
pfwd_motor.disable()
