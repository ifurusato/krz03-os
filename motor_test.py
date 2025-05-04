#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-08-13
# modified: 2025-05-03
#

import argparse
import sys, traceback
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.config_loader import ConfigLoader

from hardware.motor_controller import MotorController
from motor2040.payload import Payload
from hardware.response import Response

HELP = '''
motor2040 commands:

    enable                     enable all motors
    disable                    disable all motors
    stop                       stop all motors
    coast                      coast all motors to stop
    brake                      brake all motors to stop
    slow-decay                 mode change
    fast-decay                 mode change
    all_SPEED[_DURATION]       set speed of all motors
    crab_SPEED[_DURATION]      set crab speed
    rotate_SPEED[_DURATION]    set rotation speed
    pfwd_SPEED[_DURATION]      set speed of port-forward motor
    sfwd_SPEED[_DURATION]      set speed of stbd-forward motor
    paft_SPEED[_DURATION]      set speed of port-aft motor
    saft_SPEED[_DURATION]      set speed of stbd-aft motor

where SPEED (0.0 - 1.0) and DURATION (seconds) are real numbers, e.g., "all_0.6_4.2"
'''

def parse_args():
    # create the argument parser
    parser = argparse.ArgumentParser(description="Process command and optional speeds/duration.")
    
    # command (required) and optional arguments with defaults
    parser.add_argument("command", type=str, help="The command to execute.")
    parser.add_argument("port", type=float, nargs='?', default=Payload.DEFAULT_SPEED, help="Port motor speed (0.0 to 1.0, default: 0.5)")
    parser.add_argument("stbd", type=float, nargs='?', default=Payload.DEFAULT_SPEED, help="Starboard motor speed (0.0 to 1.0, default: 0.5)")
    parser.add_argument("duration", type=float, nargs='?', default=Payload.DEFAULT_DURATION, help="Duration of the command (0.0 to 99.0, default: 0.0)")

    return parser.parse_args()

def main():

    _motor_ctrl = None
    _log = Logger('main', Level.INFO)

    try:

        # parse the arguments
        _args = parse_args()

        _log.info('motor controller begin…')
        _config = ConfigLoader(Level.INFO).configure()

        _motor_ctrl = MotorController(_config, Level.INFO)
        _motor_ctrl.enable()

        _payload = _motor_ctrl.get_payload(_args.command, _args.port, _args.stbd, _args.duration)

        _response = _motor_ctrl.send_payload(_payload)

        if _response.value <= Response.OKAY.value:
            _log.info("response: {}".format(_response.name))
        else:
            _log.error("error response: {}".format(_response.name))

    except ValueError as e:
        # Handle any validation errors
        print("ERROR: parsing command line: {}".format(e))
    except TimeoutError as te:
        print('ERROR: transfer timeout: {}'.format(te))
    except Exception as e:
        print('ERROR: {} encountered: {}\n{}'.format(type(e), e, traceback.format_exc()))
    finally:
        print('complete.')
        if _motor_ctrl:
            _motor_ctrl.close()

if __name__ == "__main__":
    main()

#EOF
