#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2020-08-01
# modified: 2024-12-03
#
# A test/tuner of the pid controller.

import sys, traceback, time, itertools
from math import isclose
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.config_loader import ConfigLoader
from core.orientation import Orientation
from core.rate import Rate
from hardware.color import Color
#from hardware.slew_limiter import SlewLimiter
from hardware.irq_clock import IrqClock
from hardware.i2c_scanner import I2CScanner, DeviceNotFound
from hardware.motor_controller import MotorController
#from hardware.motor_configurer import MotorConfigurer
from hardware.motor import Motor
from hardware.rotary_encoder import RotaryEncoder, Selector
from hardware.digital_pot import DigitalPotentiometer

IN_MIN  = 0.0    # minimum analog value from IO Expander
IN_MAX  = 3.3    # maximum analog value from IO Expander

#           kp             ki            kd
COLORS  = [ Color.RED, Color.GREEN, Color.BLUE ]
FORES   = [ Fore.RED,  Fore.GREEN,  Fore.BLUE ]
VARS    = ["kp",          "ki",         "kd" ]

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
def brake(motor):
    motor.target_velocity = 0.0
    motor.stop()
    time.sleep(1.0)

    
# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
def main():

    MODIFY_PID_KP = True
    MODIFY_PID_KI = False
    MODIFY_PID_KD = False
    if MODIFY_PID_KP:
        OUT_MAX   = 0.2
    else:
        OUT_MAX   = 0.1
        
    KP_FIXED      = 0.050
    KI_FIXED      = 0.015
    KD_FIXED      = 0.008
    
    _level = Level.INFO
    _log = Logger('main', _level)
    
    _rate = Rate(20) # Hz
    _orientation = Orientation.SAFT
    _motor = None
    _speed_pot = None
    _pid_pot   = None
    _rot   = None
    
    try:
    
        _config = ConfigLoader(Level.INFO).configure()
    
        # external clock ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        _log.info('creating IRQ clock…')
        _irq_clock = IrqClock(_config, level=Level.INFO)
    
        _speed_pot = DigitalPotentiometer(_config, 0x0C, level=_level)
        _speed_pot.set_input_range(IN_MIN, IN_MAX) 
        _speed_pot.set_output_range(0.0, 1.0)
    
        _pid_pot = DigitalPotentiometer(_config, 0x0E, level=_level)
        _pid_pot.set_input_range(IN_MIN, IN_MAX) 
        _pid_pot.set_output_range(0.0, OUT_MAX)
    
        _rot = RotaryEncoder(Level.INFO)
        _selector = Selector(2, Level.INFO)
    
        _log.info('creating message bus…')
        _i2c_scanner = I2CScanner(_config, level=_level)

    
        # motors ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        _motor_controller = MotorController(_config, external_clock=_irq_clock, level=_level)
        _motor = _motor_controller.get_motor(_orientation)
    
#       _motor_configurer = MotorConfigurer(_config, _i2c_scanner, motors_enabled=True, level=_level)
#       _fwd_tb = _motor_configurer.get_thunderborg(Orientation.FWD)
#       _motor = _motor_configurer.get_motor(_orientation)
#       _motor.enable()
        _pid_ctrl = _motor.pid_controller
        
        # pid ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        _pid_config = _config['krzos'].get('motor').get('pid_controller')
        
        _pid = _pid_ctrl.pid
        
#       _pid.auto_mode = True
#       _pid.proportional_on_measurement = True
#       _pid.set_auto_mode(False)
#       _pid.tunings = ( _kp, _ki, _kd )
#       _pid.output_limits = ( -10.0, 10.0 )
#       _pid.sample_time = _rate.get_period_sec()
#       _log.info(Fore.GREEN + 'sample time: {:>6.3f} sec.'.format(_pid.sample_time))

#       _clip_max = 1.0
#       _clip_min = -1.0 * _clip_max
#       _clip = lambda n: _clip_min if n <= _clip_min else _clip_max if n >= _clip_max else n
#       _limiter = SlewLimiter(_config, None, Level.WARN)
#       _limiter.enable()

#           pid_controller:
#               kp:                               0.09500      # proportional gain
#               ki:                               0.00000      # integral gain
#               kd:                               0.00000      # derivative gain
#               minimum_output:                 -10.0          # lower output limit
#               maximum_output:                  10.0          # upper output limit
#               sample_freq_hz:                  20            # 20Hz equiv to 50ms/0.05s
#               hyst_queue_len:                  20            # size of queue used for running average for hysteresis

        _kp = _pid_config.get('kp')
        _ki = _pid_config.get('ki')
        _kd = _pid_config.get('kd')
        _power = 0.0
        _limit = -1
        _count = 0
        _counter = itertools.count()
        _fore = Fore.BLACK

        _irq_clock.enable()
        _motor_controller.enable()
        while _limit < 0 or _count < _limit:

            _selected = _selector.get_value(_rot.value())
            _color = COLORS[_selected]
            _rot.set_rgb(_color.rgb)
            _var = VARS[_selected]
            _fore = FORES[_selected]

            _count = next(_counter)
            _pid_pot_value = _pid_pot.get_scaled_value()

            if _selected == 0: # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
                _log.debug(_fore + 'editing kp: {:6.3f} from {:7.4f}'.format(_kp, _pid_pot_value))
                if MODIFY_PID_KP:
                    _pid.kp = _pid_pot_value
                else:
                    _pid.kp = KP_FIXED
                    
            if _selected == 1: # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
                _log.debug(_fore + 'editing ki: {:6.3f} from {:7.4f}'.format(_ki, _pid_pot_value))
                if MODIFY_PID_KI:
                    _pid.ki = _pid_pot_value
                else:
                    _pid.ki = KI_FIXED
                    
            if _selected == 2: # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
                _log.debug(_fore + 'editing kd: {:6.3f} from {:7.4f}'.format(_kd, _pid_pot_value))
                if MODIFY_PID_KD:
                    _pid.kd = _pid_pot_value
                else:
                    _pid.kd = KD_FIXED

        
            # speed pot ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

            _target_speed = _speed_pot.get_scaled_value() # values 0.0-1.0
#           _target_speed *= _scale_factor
            if isclose(_target_speed, 0.0, abs_tol=1e-4):
                _motor_controller.set_motor_speed(Orientation.SAFT, 0.0)
            else:
                _target_speed = _motor_controller._clamp(_target_speed)
                _motor_controller.set_motor_speed(Orientation.SAFT, _target_speed)
                
#           _pid.tunings = ( _kp, _ki, _kd )
#           _current_velocity = _motor.velocity
#           _power += _pid(_current_velocity) / 100.0
#           _set_power = _power / 100.0
#           _motor.set_motor_power(_power)
#           _log.info(_fore + 'editing velocity: {:6.3f} from {:5.2f}'.format(_current_velocity, _pot_value))

            # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
            kp, ki, kd = _pid.constants
            _log.info(_fore + '[{:d}] selected: {} pid: {:6.3f}|{:6.3f}|{:6.3f};\tset: {:>7.4f}; \tvel: {:>6.3f}; spd: {:>6.3f}'.format(\
                    _count, _var, kp, ki, kd, _motor.get_current_power(), _motor.velocity, _target_speed))
                    
            _rate.wait()

        '''
            Kpro = 0.380
            Kdrv = 0.0
            last_proportional_error = 0.0
            power = 0.0
            while True:
                Kdrv = _pid_pot.get_scaled_value()
                _current_velocity = _motor.get_velocity()
                proportional_error = _target_velocity - _current_velocity
                derivative_error = proportional_error - last_proportional_error
                last_proportional_error = proportional_error
                power += (proportional_error * Kpro) + (derivative_error * Kdrv)
                _motor.set_motor_power( power / 100.0 )
                _log.info(Fore.GREEN + ' P:{:6.3f}; D:{:6.3f};'.format(Kpro, Kdrv) \
                        + Fore.YELLOW + ' power: {:>8.5f};'.format(power) \
                        + Fore.MAGENTA + ' velocity: {:>6.3f}/{:>6.3f}'.format(_current_velocity, _target_velocity))
                _rate.wait()
        '''


    except KeyboardInterrupt:
        _log.info(Fore.CYAN + Style.BRIGHT + 'B. motor test complete.')
    except Exception as e:
        _log.info(Fore.RED + Style.BRIGHT + 'error in PID controller: {}'.format(e))
        traceback.print_exc(file=sys.stdout)
    finally:
        _log.info('finally.')
        if _pid_pot:
            _pid_pot.set_black()
            _pid_pot.close()
        if _speed_pot:
            _speed_pot.set_black()
            _speed_pot.close()
        if _rot:
            _rot.close()
        if _motor:
            brake(_motor)
            _motor.close()
#       _pid.disable()
#       _pid.close()

if __name__== "__main__":
    main()
    
#EOF
