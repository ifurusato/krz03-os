#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-08-14
# modified: 2025-04-26
#
# control for TinyFX

import machine
import _thread
import utime
from rp2040_slave import RP2040_Slave

#from plasma import WS2812
#from motor_controller import MotorController
#from motor import motor2040

from colors import*
from colorama import Fore, Style
from stringbuilder import StringBuilder
from core.logger import Level, Logger

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

# constants ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

ID                =  0
SDA               = 20
SCL               = 21
ADDRESS           = 0x45

# response codes:
INIT              = 0x10
OKAY              = 0x4F # all acceptable responses must be less than this
BAD_ADDRESS       = 0x71
BAD_REQUEST       = 0x72
OUT_OF_SYNC       = 0x73
INVALID_CHAR      = 0x74
SOURCE_TOO_LARGE  = 0x75
UNVALIDATED       = 0x76
EMPTY_PAYLOAD     = 0x77
PAYLOAD_TOO_LARGE = 0x78
RUNTIME_ERROR     = 0x79
UNKNOWN_ERROR     = 0x80

VERBOSE           = False
DEFAULT_SPEED     = 0.5 # if speed is not specified on motor commands
ERROR_LIMIT       = 10  # max errors before exiting main loop

# functions ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

def process_buffer(buffer):
    global enabled
    '''
    Processes the payload to determine the command, and optional speed and
    duration, creating a new Thread to handle the request.
    '''
    try:
        _string = buffer.to_string()
        if _string == 'exit':
            enabled = False
        else:
            thread_id = _thread.start_new_thread(process_payload, (_string,))
    except RuntimeError as rte:
        _log.error("runtime error: '{}'…".format(rte))
        raise I2CSlaveError(RUNTIME_ERROR, "runtime error processing payload: {}".format(rte))
    except Exception as e:
        _log.error("unknown error: '{}'…".format(e))
        raise I2CSlaveError(UNKNOWN_ERROR, "unknown error processing payload: {}".format(e))

def process_payload(payload):
    global is_running
    if is_running:
        _log.error("Error: function is already running, ignoring subsequent call.")
        return False
    _log.info("processing payload: '{}'…".format(payload))
    is_running = True
    try:
        command, speed, duration = parse_payload(payload)
        if command == 'help':
            pass
        elif command.startswith('ena'):
            pass
        return True
    finally:
        is_running = False

def parse_payload(payload):
    '''
    Parses the '_' delimited payload, returning a tuple of
    command, speed and duration; the last two are optional.
    '''
    _split = payload.split("_")
    cmd = None
    speed = None
    duration = None
    if len(_split) >= 1:
         cmd = _split[0]
    if len(_split) >= 2:
         speed = float(_split[1])
    else:
         speed = DEFAULT_SPEED
    if len(_split) >= 3:
         duration = float(_split[2])
    return cmd, speed, duration

def show_color(color):
    '''
    Display the color on the RGB LED in GRB order.
    '''
#   _led.set_rgb(0, color[0], color[1], color[2])
    print('show_color: R{}, G{}, B{}'.format(color[0], color[1], color[2]))
    pass

def reset_transaction():
    global currentTransaction
    currentTransaction.address = 0x00
    currentTransaction.data_byte = []

# classes ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

class I2CSlaveError(Exception):
    def __init__(self, code, message):
        super().__init__(message)
        self._code = code

    @property
    def code(self):
        return self._code

# main ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

_log = Logger('main', Level.INFO)
_log.info(Fore.WHITE + "initialising…")

# RGB LED
#_led = WS2812(motor2040.NUM_LEDS, 1, 0, motor2040.LED_DATA)
#_led.start()

# indicate startup, waiting 5 seconds so it can be interrupted…
_limit = 5
for i in range(_limit):
    show_color(COLOR_CYAN)
    utime.sleep_ms(50)
    show_color(COLOR_BLACK)
    _log.info(Style.DIM + '[{}/{}] starting…'.format(i, _limit))
    utime.sleep_ms(950)
utime.sleep_ms(50)

# initialize an empty buffer list for sequential write sequences
data_buf = []
addr     = 0x00

# create motor controller
#motor_ctrl = MotorController()

# established I2C slave on ID=0; SDA=20; SCL=21 at 0x44
_log.info("I2C slave starting…")
s_i2c = RP2040_Slave(ID,sda=SDA,scl=SCL,slaveAddress=ADDRESS)
state = s_i2c.I2CStateMachine.I2C_START
currentTransaction = s_i2c.I2CTransaction(addr, data_buf)

_log.info(Fore.WHITE + "main loop starting…")
show_color(COLOR_DARK_GREEN)
is_running = False
enabled    = True
errors     = 0

while enabled and errors < ERROR_LIMIT:
    try:
        state = s_i2c.handle_event()

        if state == s_i2c.I2CStateMachine.I2C_START:
            _log.debug('I2C_START')
        if state == s_i2c.I2CStateMachine.I2C_RECEIVE:
            if currentTransaction.address == 0x00:
                # first byte received is the register address
                currentTransaction.address = s_i2c.Read_Data_Received()

            _index = 0
            _expected_length = 0
            _validated = False
            _buffer = StringBuilder()

            # read all data byte received until RX FIFO is empty
            while (s_i2c.Available()):

                _data_rx = s_i2c.Read_Data_Received()
                _int  = int(_data_rx)
                _char = chr(_data_rx)

                if _data_rx == 0x00:
                    _log.debug('[{}] 0x00 start bit.'.format(_index))
                elif _data_rx == 0x01:
                    _log.debug('[{}] 0x01 validated.'.format(_index))
                    _validated = True
                elif _data_rx == 0xFF:
                    _log.debug('[{}] 0xFF end of record.'.format(_index))
                else:
                    if _index == 0:
                        _expected_length = int(_data_rx)
                        _log.debug("[{}] data: '{}'; set expected length to {} chars.".format(_index, _data_rx, _expected_length))
                    else:
                        _log.debug("[{}] data: '{}'; append character: '{}'".format(_index, _data_rx, _char))
                        _buffer.append(_char)
                        currentTransaction.data_byte.append(_data_rx)
                _index += 1
                # end while loop ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

            if _validated:
                if _buffer.length() > 0:
                    if _expected_length == _buffer.length():
                        process_buffer(_buffer)
                    else:
                        raise I2CSlaveError(PAYLOAD_TOO_LARGE,
                                "package failed with expected length: {:d}; actual length: {:d}.".format(_expected_length, _buffer.length()))
                else:
                    raise I2CSlaveError(EMPTY_PAYLOAD, 'empty payload.')

            else:
                raise I2CSlaveError(UNVALIDATED, "unvalidated buffer: '{}'".format(_buffer.to_string()))

        if state == s_i2c.I2CStateMachine.I2C_REQUEST:
            response = OKAY
            _log.debug('sending response: 0x{:02X}…'.format(response))
            while (s_i2c.is_Master_Req_Read()):
                s_i2c.Slave_Write_Data(response)

        if state == s_i2c.I2CStateMachine.I2C_FINISH:
            _log.debug('register: {}; received: {}'.format(currentTransaction.address, currentTransaction.data_byte))
            _log.info('finished.')
            reset_transaction()

    except KeyboardInterrupt:
        pass
    except I2CSlaveError as se:
        _log.error('I2C slave error {} on transaction: {}'.format(se.code, se))
        reset_transaction()
        errors += 1

_log.info(Fore.BLUE + 'loop complete.')
show_color(COLOR_DARK_BLUE)

#EOF
