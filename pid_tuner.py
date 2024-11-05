#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# A test/tuner of the pid controller.

_info='''
1. Set Initial Values

    Set Kp to a small value.
    Set Ki=0 and Kd=0 initially, so you’re only working with proportional control.

2. Tune Kp (Proportional Gain)

    Gradually increase Kp until the system output starts to oscillate around the setpoint.
    The value of Kp at which the system just starts oscillating is the ultimate gain Ku.
    Measure the period of oscillation, Tu, which is the time between successive peaks.

3. Apply Ziegler–Nichols Rules

    Use the values of Ku and Tu to set Kp, Ki, and Kd:
        P Controller:   Kp = 0.5 × Ku
        PI Controller:  Kp = 0.45 × Ku, Ki = Kp / (0.5 × Tu)
        PID Controller: Kp = 0.6 × Ku, Ki = 2 × Kp / Tu, Kd = Kp × Tu / 8
'''

import sys, traceback, time, itertools
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.config_loader import ConfigLoader
from core.orientation import Orientation
from core.rate import Rate
from hardware.color import Color
#from hardware.slew_limiter import SlewLimiter
from hardware.i2c_scanner import I2CScanner, DeviceNotFound
from hardware.motor_controller import MotorController
#from hardware.motor_configurer import MotorConfigurer
from hardware.motor import Motor
from hardware.rotary_encoder import RotaryEncoder, Selector
from hardware.digital_pot import DigitalPotentiometer
from hardware.button import Button

USE_PID = True   # set target RPM for PID, or set speed directly
IN_MIN  = 0.0    # minimum analog value from IO Expander
IN_MAX  = 3.3    # maximum analog value from IO Expander

_motor_controller = None

def button_pushed():
    global _motor_controller
    if _motor_controller:
        _motor_controller.reset()
        print(Fore.CYAN + '-- motor controller reset.' + Style.RESET_ALL)
    else:
        print(Fore.RED + '-- no motor controller available.' + Style.RESET_ALL)

#def apply_rpm_deadband(current_rpm, target_rpm, deadband):
#    if target_rpm - deadband < current_rpm < target_rpm + deadband:
#        return current_rpm
#    else:
#        return target_rpm
#
#def apply_slew_limit(current_speed, target_speed, slew_rate):
#    if target_speed > current_speed + slew_rate:
#        return current_speed + slew_rate
#    elif target_speed < current_speed - slew_rate:
#        return current_speed - slew_rate
#    return target_speed

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
def main():
    global _motor_controller

    _level = Level.INFO
    _log = Logger('main', _level)

    _rate = Rate(20) # Hz

    _speed_pot = None
    _kx_pot    = None

    try:

        _config = ConfigLoader(Level.INFO).configure()

        _pin = 23
        _btn = Button(_config, pin=_pin, impl='gpiozero', level=Level.INFO)

#       _value = _btn.pushed()
        _btn.add_callback(button_pushed, bouncetime_ms=300)

        _log.info('creating IRQ clock…')

        _speed_pot = DigitalPotentiometer(_config, 0x0C, level=_level)
        _speed_pot.set_input_range(IN_MIN, IN_MAX)
        _speed_pot.set_output_range(-100, 100)

        _kx_pot = DigitalPotentiometer(_config, 0x0E, level=_level)
        _kx_pot.set_input_range(IN_MIN, IN_MAX)
        _kx_pot.set_output_range(0.0, 1.0)

        _motor_controller = MotorController(_config)
        _motor = _motor_controller.get_motor(Orientation.PFWD)
        _pid = _motor.pid

        # motors ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

        _orientation = Orientation.PFWD # the motor orientation (port forward)
        _counter = itertools.count()

        _deadband = 4
        _max_change = 5  # maximum change in RPM per update cycle


        _log.info(Fore.CYAN + 'setting speed to 70...')
        _motor_controller.set_motor_speed(_orientation, 70)
        time.sleep(7)

        while True:

            _count = next(_counter)

            # set PID controller constants
#           _Ku = _kx_pot.get_scaled_value()
#           _Ku = 0.38
#           _log.info(Fore.MAGENTA + 'Ku={:5.2f}'.format(_Ku))
#           _pid.kp = _Ku * 0.6
#           _pid.ki = _Ku * 0.1
#           _pid.kd = _Ku * 0.05

            # constants
#           _pid.kp = 0.23 # _Ku * 0.6
#           _pid.ki = 0.03 # _Ku * 0.1
#           _pid.kd = 0.01 # _Ku * 0.05

#           _current_rpm = _motor.rpm
            _target_rpm  = int(_speed_pot.get_scaled_value(absolute_tolerance=3))

            # apply deadband to the target RPM
#           _adjusted_speed = apply_rpm_deadband(_current_rpm, _target_rpm, _deadband)

            # apply slew limiting
#           _adjusted_speed = apply_slew_limit(_current_rpm, _adjusted_speed, _max_change)

            # print current RPM and target RPM for debugging
#           _log.info(Fore.CYAN + 'Current RPM: {:.2f}, Target RPM: {:.2f}, Deadband: {:.2f}'.format(_motor.rpm, _target_rpm, _deadband))

#           _log.info(Fore.MAGENTA + '_target_rpm={:5.2f}; adjusted: {:5.2f}'.format(_target_rpm, _adjusted_speed))

            _motor_controller.set_motor_speed(_orientation, _target_rpm)
            _rate.wait()

    except KeyboardInterrupt:
        print('\n')
        _log.info(Fore.CYAN + Style.BRIGHT + 'Ctrl-C captured: exiting test…')
    except Exception as e:
        _log.info(Fore.RED + Style.BRIGHT + '{} raised by PID tuner: {}'.format(type(e), e))
        traceback.print_exc(file=sys.stdout)
    finally:
        _log.info('finally.')
        if _speed_pot:
            _speed_pot.set_black()
            _speed_pot.close()
        if _kx_pot:
            _kx_pot.set_black()
            _kx_pot.close()
        if _motor_controller:
            _motor_controller.stop()
            _motor_controller.close()

if __name__== "__main__":
    main()

#EOF
