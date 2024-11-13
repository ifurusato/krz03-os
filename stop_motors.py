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

from colorama import init, Fore, Style
init()

from inventorhatmini import InventorHATMini, MOTOR_A, MOTOR_B

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

print(Fore.CYAN + 'create aft controller…' + Style.RESET_ALL)
aft_controller = InventorHATMini(address=0x17, init_servos=False, init_leds=False)

print(Fore.CYAN + 'create fwd controller…' + Style.RESET_ALL)
fwd_controller = InventorHATMini(address=0x16, init_servos=False, init_leds=False)

# forward motors ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
pfwd_motor = fwd_controller.motors[MOTOR_A]
sfwd_motor = fwd_controller.motors[MOTOR_B]
pfwd_motor.enable()
sfwd_motor.enable()
# aft motors ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
paft_motor = aft_controller.motors[MOTOR_A]
saft_motor = aft_controller.motors[MOTOR_B]
paft_motor.enable()
saft_motor.enable()

print(Fore.GREEN + 'stopping motors…' + Style.RESET_ALL)
pfwd_motor.speed(0.0)
sfwd_motor.speed(0.0)
paft_motor.speed(0.0)
saft_motor.speed(0.0)

# Disable the motors
pfwd_motor.disable()
sfwd_motor.disable()
paft_motor.disable()
saft_motor.disable()

print(Fore.CYAN + 'complete.' + Style.RESET_ALL)
#EOF
