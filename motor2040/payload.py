#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-05-01
# modified: 2025-05-03

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

class Payload:
    PACKET_LENGTH    = 12  # 11-byte payload + 1-byte CRC
    DEFAULT_SPEED    = 0.5
    DEFAULT_DURATION = 0.0
    '''
    The Payload class encapsulates a fixed-length binary message format designed
    for efficient, error-checked communication between devices, particularly over
    constrained interfaces like I2C.

    It is intended for transmitting movement or control commands in a compact and
    reliable form, where fixed packet size, structure, and validation are critical.

    -- Packet Format

    Each packet is exactly 12 bytes (96 bits):

      * Command (4 chars): ASCII string identifying the command (e.g., 'fore', 'stop')
      * Port speed (2 digits): Encoded from a float in the range 0.0–1.0, as "00"–"99"
      * Starboard speed (2 digits): Same as port speed
      * Duration (3 digits): Optional; float 0.0–99.9, encoded as "000"–"999" (tenths of a second)
      * CRC (1 byte): A CRC-8-CCITT checksum for error detection

    All values are encoded into an 11-character ASCII payload, followed by a 1-byte
    checksum.
    '''
    def __init__(self, command, port=None, stbd=None, duration=None):
        if len(command) != 4:
            raise ValueError("Command must be exactly 4 characters.")
        self._command = command
        self._port = self._validate_speed(port, "port")
        self._stbd = self._validate_speed(stbd, "stbd")
        self._duration = self._validate_duration(duration)

    @property
    def command(self):
        return self._command

    @property
    def port(self):
        return self._port

    @property
    def stbd(self):
        return self._stbd

    @property
    def duration(self):
        return self._duration

    @property
    def values(self):
        return self._command, self._port, self._stbd, self._duration

    def _validate_speed(self, value, label):
        if value is None:
            value = Payload.DEFAULT_SPEED
        if not (0.0 <= value <= 1.0):
            raise ValueError(f"{label}_speed must be between 0.0 and 1.0, got {value}")
        return value

    def _validate_duration(self, value):
        if value is None:
            value = Payload.DEFAULT_DURATION
        if not (0.0 <= value <= 99.0):
            raise ValueError(f"Duration must be between 0.0 and 99.0, got {value}")
        return value

    def _format_speed(self, speed):
        if not (0.0 <= speed <= 1.0):
            raise ValueError("Speed must be between 0.0 and 1.0")
        return f"{int(round(speed * 100)):02d}"

    def _format_duration(self, duration):
        if duration is None:
            return "000"
        if not (0.0 <= duration <= 99.9):
            raise ValueError("Duration must be between 0.0 and 99.9")
        return f"{int(round(duration * 10)):03d}"

    @staticmethod
    def _crc8_ccitt(data):
        crc = 0
        for b in data:
            crc ^= b
            for _ in range(8):
                if crc & 0x80:
                    crc = ((crc << 1) ^ 0x07) & 0xFF
                else:
                    crc = (crc << 1) & 0xFF
        return crc

    def to_bytes(self):
        '''
        Compose the 12-byte packet (payload + CRC).
        '''
        port_str = self._format_speed(self._port)
        stbd_str = self._format_speed(self._stbd)
        dur_str  = self._format_duration(self._duration)
        payload  = (self._command + port_str + stbd_str + dur_str).encode()
        crc = self._crc8_ccitt(payload)
        return payload + bytes([crc])

    @classmethod
    def from_bytes(cls, packet_bytes):
        '''
        Decode a 12-byte packet and return a Payload instance.
        '''
        if len(packet_bytes) != cls.PACKET_LENGTH:
            raise ValueError("Expected 12-byte packet")
        payload = packet_bytes[:11]
        received_crc = packet_bytes[11]
        expected_crc = cls._crc8_ccitt(payload)
        if received_crc != expected_crc:
            raise ValueError(f"CRC mismatch: got {received_crc:02X}, expected {expected_crc:02X}")
        try:
            payload_str = payload.decode()
        except UnicodeDecodeError:
            raise ValueError("Payload is not valid ASCII.")
        command = payload_str[:4]
        port = int(payload_str[4:6]) / 100
        stbd = int(payload_str[6:8]) / 100
        dur_raw = payload_str[8:11]
        duration = int(dur_raw) / 10 if dur_raw != "000" else None
        return cls(command, port, stbd, duration)

    def to_string(self) -> str:
        duration_str = f"{self._duration:.1f}s" if self._duration is not None else "None"
        return (
            f"Command: '{self._command}', "
            f"Port Speed: {self._port:.2f}, "
            f"Starboard Speed: {self._stbd:.2f}, "
            f"Duration: {duration_str}"
        )

    def __repr__(self):
        return f"Payload(command='{self._command}', port={self._port}, stbd={self._stbd}, duration={self._duration})"

    # Optional: for I2C 4-bit nibble use
    def to_nibbles(self):
        '''
        Convert 12-byte packet to 24 nibbles (ints 0–15).
        '''
        pkt = self.to_bytes()
        return [(byte >> 4) & 0xF for byte in pkt] + [byte & 0xF for byte in pkt]

    @classmethod
    def from_nibbles(cls, nibbles):
        '''
        Reconstruct packet from 24 nibbles.
        '''
        if len(nibbles) != 24:
            raise ValueError("Expected 24 nibbles (4-bit values)")
        bytes_out = bytearray((nibbles[i] << 4) | nibbles[i + 12] for i in range(12))
        return cls.from_bytes(bytes_out)

#EOF
