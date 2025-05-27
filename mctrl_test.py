#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-08-13
# modified: 2025-05-26
#

import argparse
import itertools
import sys, traceback
import time
from datetime import datetime as dt
from colorama import init, Fore, Style
init()

from core.component import Component
from core.logger import Logger, Level
from core.config_loader import ConfigLoader
from hardware.controller import Controller
from hardware.payload import Payload
from hardware.response import*
from hardware.micro_controller import MicroController

STARTUP_TEST    = False

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
def parse_args(micro_controller):
    parser = argparse.ArgumentParser(description="Send command to a specific I2C controller or run a demo.")
    parser.add_argument(
        "controller",
        type=str,
        help="Either 'demo' to run the demo or a controller ID ('{}', '{}', '{}').".format(
                micro_controller.get_name_0(), micro_controller.get_name_1(), micro_controller.get_name_2())
    )
    parser.add_argument("command", type=str, nargs="?", help="The command to execute.")
    parser.add_argument("args", nargs="*", help="Optional arguments for the command (e.g., speeds, duration).")
    return parser.parse_args()

def demo(log, micro_controller):
    log.info('sending initial payloads…')
    if STARTUP_TEST:
        _response = micro_controller.send_payload(micro_controller.get_name_0(), 'black')
        time.sleep(0.3)
        _response = micro_controller.send_payload(micro_controller.get_name_1(), 'black')
        time.sleep(0.3)
        _response = micro_controller.send_payload(micro_controller.get_name_2(), 'black')
        time.sleep(0.3)
        log.info('test payloads complete.')
    _enabled = True
    _delay_sec = 0.40
    while _enabled:
        for _color in [ 'red', 'green', 'blue', 'cyan', 'magenta', 'yellow', 'black' ]:
            log.info("color: '{}'".format(_color))
            for _name in micro_controller.get_names():
                log.info("controller name: '{}'".format(_name))
                _response = micro_controller.send_payload(_name, _color)
                if _response != RESPONSE_OKAY:
                    log.warning("response on controller '{}' with command '{}': 0x{:02X}".format(_name, _color, _response.value))
                    _enabled = False
                else:
                    log.info("response on controller " + Style.BRIGHT + "'{}'".format(_name) + Style.NORMAL
                            + " with command '{}': 0x{:02X} with delay: {:4.2f}s".format(_color, _response.value, _delay_sec))
                    time.sleep(_delay_sec)
        _delay_sec -= 0.02 # gradually speed up

# main ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
def main():

    _micro_controller = None
    _log = Logger('main', Level.INFO)

    try:


        _log.info('micro-controller begin…')

        _config = ConfigLoader(Level.INFO).configure()
        _micro_controller = MicroController(_config)

        # parse the arguments
        _args = parse_args(_micro_controller)

        if _args.controller == "demo":
            demo(_log, _micro_controller)
        else:
            _name = _args.controller
            _log.info("controller: '{}'".format(_name))
            # combine into a single command string
            _command_string = ' '.join([_args.command] + _args.args)
            _response = _micro_controller.send_payload(_name, _command_string)
            _log.info('response: {}'.format(_response))

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
        if _micro_controller:
            _log.info('complete.')
            _micro_controller.close()

if __name__ == "__main__":
    main()

#EOF
