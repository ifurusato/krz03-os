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
# A script that sends the command line argument as a packet to an I2C slave.
#
# see smbus2
# https://smbus2.readthedocs.io/en/latest/#smbus2.SMBus.write_block_data
#

import time
import argparse
import sys, traceback
from smbus import SMBus
from enum import Enum

from hardware.payload import Payload 
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

# constants ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

I2C_SLAVE_ADDRESS = 0x44
CONFIG_REGISTER   = 1

# main ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

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

    try:

        # parse the arguments
        _args = parse_args()

        # create the Payload object using the parsed arguments
        _payload = Payload(
            command  = _args.command,
            port     = _args.port,
            stbd     = _args.stbd,
            duration = _args.duration
        )
        print("payload: {}".format(_payload.to_string()))


        # send over I2C
        _i2cbus = SMBus(1)

        print("writing payload: '{}'…".format(_payload.to_string()))

        _i2cbus.write_block_data(I2C_SLAVE_ADDRESS, CONFIG_REGISTER, list(_payload.to_bytes()))
     
        print('payload written.')

        # Read 1-byte response
        _read_data = _i2cbus.read_byte_data(I2C_SLAVE_ADDRESS, CONFIG_REGISTER)

        # Convert response byte to status enum or meaning
        _response = Response.from_value(_read_data)

        print("response: '{}'".format(_response))

        if _response.value <= Response.OKAY.value:
            print("response: {}".format(_response.name))
        else:
            print("ERROR response: {}".format(_response.name))

    except ValueError as e:
        # Handle any validation errors
        print("ERROR: parsing command line: {}".format(e))
    except TimeoutError as te:
        print('ERROR: transfer timeout: {}'.format(te))
    except Exception as e:
        print('ERROR: {} encountered: {}\n{}'.format(type(e), e, traceback.format_exc()))
    finally:
        print('complete.')

if __name__ == "__main__":
    main()

#EOF
