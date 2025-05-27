#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-05-22
# modified: 2025-05-22
#

import traceback
from smbus import SMBus
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level

# functions ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

def crc8_ccitt(data):
    '''
    Compute CRC-8-CCITT checksum over the given byte sequence.
    Polynomial: x^8 + x^2 + x + 1 (0x07)
    '''
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc

def encode_with_crc(message: str) -> bytearray:
    '''
    Takes a string, computes CRC, appends it as a byte, returns a bytearray.
    '''
    data_bytes = message.encode('ascii')
    crc = crc8_ccitt(data_bytes)
    return bytearray(data_bytes + bytes([crc]))

def decode_and_verify_crc(data: bytearray) -> str:
    '''
    Takes a bytearray, checks CRC, raises exception if invalid,
    returns the original string if valid.
    '''
    if not isinstance(data, (bytes, bytearray)):
        raise TypeError("Expected bytearray or bytes input")
    if len(data) < 2:
        raise ValueError("Data too short to contain a CRC")
    msg, received_crc = data[:-1], data[-1]
    computed_crc = crc8_ccitt(msg)
    if computed_crc != received_crc:
        raise ValueError("CRC check failed: expected 0x{:02X}, got 0x{:02X}".format(received_crc, computed_crc))
    return msg.decode('ascii')  # or 'utf-8' if needed

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

log = Logger('motor_ctrl', Level.INFO)

i2c_bus_number    = 1
i2c_slave_address = 0x43
config_register   = 1 
i2cbus            = None

log.info('ready.')

def to_bytes(data):
    '''
    Encode data to bytes as ASCII characters + 1 CRC byte.
    '''
    payload  = data.encode()
    crc = crc8_ccitt(payload)
    return payload + bytes([crc])

try:

    data = "0123456789ABCDEF"
    encoded = encode_with_crc(data)

    i2cbus = SMBus(i2c_bus_number)

    log.info("writing data: " + Fore.WHITE + "'{}'".format(encoded))

    # send over I2C
    i2cbus.write_block_data(i2c_slave_address, config_register, list(encoded))
    log.info("data written: " + Fore.WHITE + "'{}'".format(encoded))

    # read response
    _read_data = i2cbus.read_byte_data(i2c_slave_address, config_register)
    response = decode_and_verify_crc(_read_data)
    log.info("response: {}".format(response))
  
except KeyboardInterrupt:
    log.info('Ctrl-C caught; exiting…')
except TimeoutError as te:
    log.error("transfer timeout: {}".format(te))
except Exception as e:
    log.error('{} raised writing data: {}\n{}'.format(type(e), e, traceback.format_exc()))
finally:
    if i2cbus:
        i2cbus.close()

log.info('disabled.')

#EOF
