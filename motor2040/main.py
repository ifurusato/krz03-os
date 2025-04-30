#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-08-14
# modified: 2025-04-30
#
# control for Motor 2040

import machine
import _thread
import utime
from rp2040_slave import RP2040_Slave

from plasma import WS2812
from motor_controller import MotorController
from motor import motor2040

from colors import*
from colorama import Fore, Style
from stringbuilder import StringBuilder
from core.logger import Level, Logger

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

# init ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

_log = Logger('main', Level.INFO)
_log.info(Fore.WHITE + "initialising…")

# RGB LED
_led = WS2812(motor2040.NUM_LEDS, 1, 0, motor2040.LED_DATA)
_led.start()

# constants ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

ID                =  0
SDA               = 20
SCL               = 21
ADDRESS           = 0x44

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

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Controller(object):
    def __init__(self, level=Level.INFO):
        super().__init__()
        self._log = Logger('ctrl', level)
        # initialize an empty buffer list for sequential write sequences
        data_buf = []
        addr     = 0x00
        # create motor controller
        self._motor_ctrl = MotorController()
        # established I2C slave on ID=0; SDA=20; SCL=21 at 0x44
        self._log.info("I2C slave starting…")
        self.s_i2c = RP2040_Slave(ID,sda=SDA,scl=SCL,slaveAddress=ADDRESS)
        self.state = self.s_i2c.I2CStateMachine.I2C_START
        self.currentTransaction = self.s_i2c.I2CTransaction(addr, data_buf)
        self._log.info(Fore.WHITE + "main loop starting…")
        show_color(COLOR_GREEN)
        self.is_running = False
        self.enabled    = True
        self.errors     = 0
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def process_buffer(self, buffer):
        '''
        Processes the payload to determine the command, and optional speed and
        duration, creating a new Thread to handle the request.
        '''
        try:
            _string = buffer.to_string()
            if _string == 'exit':
                self.enabled = False
            else:
                thread_id = _thread.start_new_thread(self.process_payload, (_string,))
        except RuntimeError as rte:
            self._log.error("runtime error: '{}'…".format(rte))
            raise I2CSlaveError(RUNTIME_ERROR, "runtime error processing payload: {}".format(rte))
        except Exception as e:
            self._log.error("unknown error: '{}'…".format(e))
            raise I2CSlaveError(UNKNOWN_ERROR, "unknown error processing payload: {}".format(e))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def process_payload(self, payload):
        if self.is_running:
            self._log.error("Error: function is already running, ignoring subsequent call.")
            return False
        self._log.info("processing payload: '{}'…".format(payload))
        self.is_running = True
        try:
            show_color(COLOR_YELLOW)
            _command, _speed, _duration = self.parse_payload(payload)
            if _command == 'help':
                self._motor_ctrl.help()
            elif _command.startswith('ena'):
                self._motor_ctrl.enable()
            elif _command.startswith('dis'):
                self._motor_ctrl.disable()
            elif _command == 'stop':
                self._motor_ctrl.stop()
            elif _command == 'coast':
                self._motor_ctrl.coast()
            elif _command == 'brake':
                self._motor_ctrl.brake()
            elif _command.startswith('slow'):
                self._motor_ctrl.slow_decay()
            elif _command.startswith('fast'):
                self._motor_ctrl.fast_decay()
            elif _command.startswith('acc'):
                self._motor_ctrl.accelerate(_speed)
            elif _command.startswith('dec'):
                self._motor_ctrl.decelerate(0.0)
            elif _command == 'all':
                self._motor_ctrl.all(_speed, _duration)
            elif _command == 'crab':
                self._motor_ctrl.crab(_speed, _duration)
            elif _command.startswith('rot'):
                self._motor_ctrl.rotate(_speed, _duration)
            elif _command == 'pfwd':
                self._motor_ctrl.pfwd(_speed, _duration)
            elif _command == 'sfwd':
                self._motor_ctrl.sfwd(_speed, _duration)
            elif _command == 'paft':
                self._motor_ctrl.paft(_speed, _duration)
            elif _command == 'saft':
                self._motor_ctrl.saft(_speed, _duration)
            return True
        finally:
            self.is_running = False
            show_color(COLOR_GREEN)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def parse_payload(self, payload):
        '''
        Parses the '_' delimited payload, returning a tuple of
        command, speed and duration; the last two are optional.
        '''
        _split = payload.split("_")
        _command = None
        _speed = None
        _duration = None
        if len(_split) >= 1:
             _command = _split[0]
        if len(_split) >= 2:
             _speed = float(_split[1])
        else:
             _speed = DEFAULT_SPEED
        if len(_split) >= 3:
             _duration = float(_split[2])
        return _command, _speed, _duration

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def start(self):
        while self.enabled and self.errors < ERROR_LIMIT:
            try:
                self.state = self.s_i2c.handle_event()

                if self.state == self.s_i2c.I2CStateMachine.I2C_START:
                    self._log.debug('I2C_START')
                if self.state == self.s_i2c.I2CStateMachine.I2C_RECEIVE:
                    if self.currentTransaction.address == 0x00:
                        # first byte received is the register address
                        self.currentTransaction.address = self.s_i2c.Read_Data_Received()

                    _index = 0
                    _expected_length = 0
                    _validated = False
                    _buffer = StringBuilder()

                    # read all data byte received until RX FIFO is empty
                    while (self.s_i2c.Available()):

                        _data_rx = self.s_i2c.Read_Data_Received()
                        _int  = int(_data_rx)
                        _char = chr(_data_rx)

                        if _data_rx == 0x00:
                            self._log.debug('[{}] 0x00 start bit.'.format(_index))
                        elif _data_rx == 0x01:
                            self._log.debug('[{}] 0x01 validated.'.format(_index))
                            _validated = True
                        elif _data_rx == 0xFF:
                            self._log.debug('[{}] 0xFF end of record.'.format(_index))
                        else:
                            if _index == 0:
                                _expected_length = int(_data_rx)
                                self._log.debug("[{}] data: '{}'; set expected length to {} chars.".format(_index, _data_rx, _expected_length))
                            else:
                                self._log.debug("[{}] data: '{}'; append character: '{}'".format(_index, _data_rx, _char))
                                _buffer.append(_char)
                                self.currentTransaction.data_byte.append(_data_rx)
                        _index += 1
                        # end while loop ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

                    if _validated:
                        if _buffer.length() > 0:
                            if _expected_length == _buffer.length():
                                self.process_buffer(_buffer)
                            else:
                                raise I2CSlaveError(PAYLOAD_TOO_LARGE,
                                        "package failed with expected length: {:d}; actual length: {:d}.".format(_expected_length, _buffer.length()))
                        else:
                            raise I2CSlaveError(EMPTY_PAYLOAD, 'empty payload.')

                    else:
                        raise I2CSlaveError(UNVALIDATED, "unvalidated buffer: '{}'".format(_buffer.to_string()))

                if self.state == self.s_i2c.I2CStateMachine.I2C_REQUEST:
                    response = OKAY
                    self._log.debug('sending response: 0x{:02X}…'.format(response))
                    while (self.s_i2c.is_Master_Req_Read()):
                        self.s_i2c.Slave_Write_Data(response)

                if self.state == self.s_i2c.I2CStateMachine.I2C_FINISH:
                    self._log.debug('register: {}; received: {}'.format(self.currentTransaction.address, self.currentTransaction.data_byte))
                    self._log.info('finished.')
                    self.reset_transaction()

            except KeyboardInterrupt:
                pass
            except I2CSlaveError as se:
                self._log.error('I2C slave error {} on transaction: {}'.format(se.code, se))
                self.reset_transaction()
                self.errors += 1

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def reset_transaction(self):
        self.currentTransaction.address = 0x00
        self.currentTransaction.data_byte = []

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class I2CSlaveError(Exception):
    def __init__(self, code, message):
        super().__init__(message)
        self._code = code

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def code(self):
        return self._code

# functions ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

def show_color(color):
    '''
    Display the color on the RGB LED in GRB order.
    '''
    _led.set_rgb(0, color[0], color[1], color[2])

# main ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

# indicate startup, waiting 5 seconds so it can be interrupted…
_limit = 5
for i in range(_limit):
    show_color(COLOR_CYAN)
    utime.sleep_ms(50)
    show_color(COLOR_BLACK)
    _log.info(Style.DIM + '[{}/{}] starting…'.format(i, _limit))
    utime.sleep_ms(950)
utime.sleep_ms(50)

try:
    _log.info('start controller…')
    controller = Controller()
    controller.start()
    _log.info('controller started.')
except Exception as e:
    _log.error('error in main loop: {}'.format(e))
    # reboot
    machine.reset()

# this only happens following an 'exit', which also disables the I2C slave
_log.info('exit: loop complete.')
show_color(COLOR_DARK_RED)

#EOF
