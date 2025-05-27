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
    """
    Takes a string, computes CRC, appends it as a byte, returns a bytearray.
    """
    data_bytes = message.encode('ascii')  # or 'utf-8' if wider range needed
    crc = crc8_ccitt(data_bytes)
    return bytearray(data_bytes + bytes([crc]))

def decode_and_verify_crc(data: bytearray) -> str:
    """
    Takes a bytearray, checks CRC, raises exception if invalid,
    returns the original string if valid.
    """
    if not isinstance(data, (bytes, bytearray)):
        raise TypeError("Expected bytearray or bytes input")

    if len(data) < 2:
        raise ValueError("Data too short to contain a CRC")

    msg, received_crc = data[:-1], data[-1]
    computed_crc = crc8_ccitt(msg)
    
    if computed_crc != received_crc:
        raise ValueError("CRC check failed: expected 0x{:02X}, got 0x{:02X}".format(received_crc, computed_crc))

    return msg.decode('ascii')  # or 'utf-8' if needed


msg = "hello"
packet = encode_with_crc(msg)
print(packet)  # e.g. bytearray(b'hello<crc-byte>')

try:
    original = decode_and_verify_crc(packet)
    print("Valid message:", original)
except ValueError as e:
    print("CRC error:", e)

