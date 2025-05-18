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
import time
from datetime import datetime as dt
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.config_loader import ConfigLoader

from hardware.motor_controller import MotorController
from motor2040.payload import Payload
from hardware.response import Response

import time
from datetime import datetime as dt

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

    _log = Logger('main', Level.INFO)
    _motor_ctrl = None

    try:

        _log.info('motor controller begin…')
        _config = ConfigLoader(Level.INFO).configure()

        _motor_ctrl = MotorController(_config, Level.INFO)
        _motor_ctrl.enable()

        # parse the arguments
        _args = parse_args()

        start_time = dt.now()
        _response = _motor_ctrl.send_payload(_args.command, _args.port, _args.stbd, _args.duration)
        elapsed_ms = (dt.now() - start_time).total_seconds() * 1000.0

        if _response.value <= Response.OKAY.value:
            _log.info("response: {}; {:5.2f}ms elapsed.".format(_response.name, elapsed_ms))
        else:
            _log.error("error response: {}; {:5.2f}ms elapsed.".format(_response.name, elapsed_ms))

    except KeyboardInterrupt:
        print("Program interrupted by user (Ctrl+C). Exiting gracefully.")
    except ValueError as e:
        # handle any CLI validation errors
        _log.error("parsing command line: {}".format(e))
    except TimeoutError as te:
        _log.error('transfer timeout: {}'.format(te))
    except Exception as e:
        _log.error('{} encountered: {}\n{}'.format(type(e), e, traceback.format_exc()))
    finally:
        _log.info('complete.')
        if _motor_ctrl:
#           _motor_ctrl.stop()
            _motor_ctrl.close()

if __name__ == "__main__":
    main()

#EOF
