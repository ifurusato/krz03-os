#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-05-01
# modified: 2025-05-18

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Payload:
    VALIDATE_ARGS      = True
    PACKET_LENGTH      = 15  # 14-byte payload + 1-byte CRC
    DEFAULT_SPEED      = 0.5
    DEFAULT_DURATION   = 0.0
    SPEED_TOLERANCE    = 0.05
    DURATION_TOLERANCE = 0.1
    '''
    The Payload class encapsulates a fixed-length binary message format designed
    for efficient, error-checked communication between devices, particularly over
    constrained interfaces like I2C.

    It is intended for transmitting movement or control commands in a compact and
    reliable form, where fixed packet size, structure, and validation are critical.

    -- Packet Format

    Each packet is exactly 15 bytes (120 bits):

      * Command (4 chars): ASCII string identifying the command (e.g., 'fore', 'stop')
      * Port speed (3 digits): Encoded from a float in the range 0.0–1.0, as "000"–"100"
      * Starboard speed (3 digits): Same as port speed
      * Duration (4 digits): Optional; float 0.0–999.9, encoded as "0000"–"9999" (tenths of a second)
      * CRC (1 byte): A CRC-8-CCITT checksum for error detection

    All values are encoded into a 14-character ASCII payload, followed by a 1-byte
    checksum. This format ensures the message is compact, human-readable for debugging,
    and robust against transmission errors.
    '''
    def __init__(self, command, port=None, stbd=None, duration=None):
        '''
        Initialize a Payload instance with a command, port and starboard speeds, and an optional duration.
        '''
        if len(command) != 4:
            raise ValueError("Command must be exactly 4 characters.")
        self._command = command
        self._port = self._validate_speed(port, "port")
        self._stbd = self._validate_speed(stbd, "stbd")
        self._duration = self._validate_duration(duration)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    @staticmethod
    def create(command, port_speed, stbd_speed, duration):
        '''
        Factory method that generates a payload provides the requisite arguments.
        '''
        def _or_default(value, default):
            return default if value is None else value

        port_speed = _or_default(port_speed, Payload.DEFAULT_SPEED)
        stbd_speed = _or_default(stbd_speed, Payload.DEFAULT_SPEED)
        duration   = _or_default(duration,   Payload.DEFAULT_DURATION)
        if Payload.VALIDATE_ARGS:
            if not isinstance(command, str):
                raise ValueError('expected string for command, not {}'.format(type(command)))
            if not isinstance(port_speed, float):
                raise ValueError('expected float for port_speed, not {}'.format(type(port_speed)))
            if not isinstance(stbd_speed, float):
                raise ValueError('expected float for stbd_speed, not {}'.format(type(stbd_speed)))
            if not isinstance(duration, float):
                raise ValueError('expected float for duration, not {}'.format(type(duration)))
        return Payload(command, port_speed, stbd_speed, duration)

    @property
    def command(self):
        '''
        Return the 4-character command string.
        '''
        return self._command

    @property
    def port(self):
        '''
        Return the port motor speed as a float (0.0–1.0).
        '''
        return self._port

    @property
    def stbd(self):
        '''
        Return the starboard motor speed as a float (0.0–1.0).
        '''
        return self._stbd

    @property
    def duration(self):
        return self._duration

    @property
    def values(self):
        '''
        Return all values (command, port, starboard, duration) as a tuple.
        '''
        return self._command, self._port, self._stbd, self._duration

    def _validate_speed(self, value, label):
        '''
        Ensure speed is within valid range or assign default.
        '''
        if value is None:
            value = Payload.DEFAULT_SPEED
        if not (-1.0 <= value <= 1.0):
            raise ValueError(f"{label}_speed must be between -1.0 and 1.0, got {value}")
        return value

    def _validate_duration(self, value):
        '''
        Ensure duration is within valid range or assign default.
        '''
        if value is None:
            value = Payload.DEFAULT_DURATION
        if not (0.0 <= value <= 999.9):
            raise ValueError(f"Duration must be between 0.0 and 999.9, got {value}")
        return value

    def _format_speed(self, speed):
        '''
        Convert float speed to 3-digit ASCII string (e.g., 0.5 → '050').
        '''
        return f"{int(round(speed * 100)):03d}"

    def _format_duration(self, duration):
        '''
        Convert float duration to 4-digit ASCII string in tenths of a second.
        '''
        return f"{int(round(duration * 10)):04d}"

    @staticmethod
    def _crc8_ccitt(data):
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

    def to_bytes(self):
        '''
        Encode payload to bytes: 14 ASCII characters + 1 CRC byte = 15 bytes.
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
        Decode a 15-byte packet (14 bytes ASCII + 1 CRC) into a Payload instance.
        Validates CRC and formats fields back to floats.
        '''
        if len(packet_bytes) != cls.PACKET_LENGTH:
            raise ValueError("Expected 15-byte packet")
        payload = packet_bytes[:14]
        received_crc = packet_bytes[14]
        expected_crc = cls._crc8_ccitt(payload)
        if received_crc != expected_crc:
            raise ValueError(f"CRC mismatch: got {received_crc:02X}, expected {expected_crc:02X}")
        try:
            payload_str = payload.decode()
        except UnicodeDecodeError:
            raise ValueError("Payload is not valid ASCII.")
        command = payload_str[:4]
        port = int(payload_str[4:7]) / 100
        stbd = int(payload_str[7:10]) / 100
        dur_raw = payload_str[10:14]
        duration = int(dur_raw) / 10 if dur_raw != "0000" else None
        return cls(command, port, stbd, duration)

    def to_string(self) -> str:
        '''
        Return a human-readable string representation of the payload.
        '''
        duration_str = f"{self._duration:.1f}s" if self._duration is not None else "None"
        return (
            f"Command: '{self._command}', "
            f"Port Speed: {self._port:.2f}, "
            f"Starboard Speed: {self._stbd:.2f}, "
            f"Duration: {duration_str}"
        )

    def __repr__(self):
        '''
        Return a concise string representation for debugging.
        '''
        return f"Payload(command='{self._command}', port={self._port}, stbd={self._stbd}, duration={self._duration})"

    def to_nibbles(self):
        '''
        Convert 15-byte packet to 30 nibbles (4-bit integers).
        Useful for 4-bit I2C or SPI protocols.
        '''
        pkt = self.to_bytes()
        return [(byte >> 4) & 0xF for byte in pkt] + [byte & 0xF for byte in pkt]

    @classmethod
    def from_nibbles(cls, nibbles):
        '''
        Reconstruct Payload from 30 nibbles (integers 0–15).
        '''
        if len(nibbles) != 30:
            raise ValueError("Expected 30 nibbles (4-bit values)")
        bytes_out = bytearray((nibbles[i] << 4) | nibbles[i + 15] for i in range(15))
        return cls.from_bytes(bytes_out)

    def values_equal(self, command, port, stbd, duration):
        '''
        Compare the current values to a set of incoming values, using the same tolerances.
        '''
        return (
            self._command == command
            and self.isclose(self._port, port, abs_tol=self.SPEED_TOLERANCE)
            and self.isclose(self._stbd, stbd, abs_tol=self.SPEED_TOLERANCE)
            and self.isclose(self._duration, duration, abs_tol=self.DURATION_TOLERANCE)
        )

    def __eq__(self, other):
        '''
        Check if another Payload instance is equal to this one.
        Uses tolerances for float comparisons: 0.05 for speeds, 0.1 for duration.
        '''
        if not isinstance(other, Payload):
            return NotImplemented
        def floats_equal(a, b, tol):
            if a is None or b is None:
                return a is b  # True if both are None, otherwise False
            return self.isclose(a, b, abs_tol=tol)
        return (
            self._command == other._command
            and floats_equal(self._port, other._port, self.SPEED_TOLERANCE)
            and floats_equal(self._stbd, other._stbd, self.SPEED_TOLERANCE)
            and floats_equal(self._duration, other._duration, self.DURATION_TOLERANCE)
        )

    @staticmethod
    def isclose(a, b, abs_tol=1e-9):
        return abs(a - b) <= abs_tol

#EOF
