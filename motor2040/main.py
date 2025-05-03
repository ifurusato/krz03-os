#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-08-14
# modified: 2025-05-03
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
from core.logger import Level, Logger
from payload import Payload

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
MAX_PAYLOAD_LENGTH = 12  # fixed-size payload with CRC

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Controller(object):
    def __init__(self, level=Level.INFO):
        super().__init__()
        self._log = Logger('ctrl', level)
        # initialize an empty buffer list for sequential write sequences
        self.data_buf = []
        self.addr     = 0x00
        # create motor controller
        self._motor_ctrl = MotorController()
        # established I2C slave on ID=0; SDA=20; SCL=21 at 0x44
        self._log.info("I2C slave starting…")
        self.s_i2c = RP2040_Slave(ID,sda=SDA,scl=SCL,slaveAddress=ADDRESS)
        self.state = self.s_i2c.I2CStateMachine.I2C_START
        self.currentTransaction = self.s_i2c.I2CTransaction(self.addr, self.data_buf)
        self._log.info(Fore.WHITE + "main loop starting…")
        show_color(COLOR_GREEN)
        self.is_running = False
        self.enabled    = True
        self.errors     = 0
        self._log.info('ready.')

#   # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
#   def process_buffer(self, buffer):
#       '''
#       Processes the payload to determine the command, and optional speed and
#       duration, creating a new Thread to handle the request.
#       '''
#       try:
#           _string = buffer.to_string()
#           if _command == 'help':
#               self._motor_ctrl.help()
#           elif _string == 'exit':
#               self.enabled = False
#           else:
#               thread_id = _thread.start_new_thread(self.process_payload, (_string,))
#       except RuntimeError as rte:
#           self._log.error("runtime error: '{}'…".format(rte))
#           raise I2CSlaveError(RUNTIME_ERROR, "runtime error processing payload: {}".format(rte))
#       except Exception as e:
#           self._log.error("unknown error: '{}'…".format(e))
#           raise I2CSlaveError(UNKNOWN_ERROR, "unknown error processing payload: {}".format(e))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def process_payload(self, payload):
        if self.is_running:
            self._log.error("Error: function is already running, ignoring subsequent call.")
            return False
        self._log.info("processing payload: '{}'…".format(payload))
        self.is_running = True
        try:
            show_color(COLOR_YELLOW)
            _command, _speed, _stbd_speed, _duration = payload.values
            self._log.info("payload: cmd: '{}'; port: {}; stbd: {}; duration: {}".format(_command, _speed, _stbd_speed, _duration))
            if _command.startswith('enab'):
                self._motor_ctrl.enable()
            elif _command.startswith('disa'):
                self._motor_ctrl.disable()
            elif _command.startswith('stop'):
                self._motor_ctrl.stop()
            elif _command.startswith('coas'):
                self._motor_ctrl.coast()
            elif _command.startswith('brak'):
                self._motor_ctrl.brake()
            elif _command.startswith('slow'):
                self._motor_ctrl.slow_decay()
            elif _command.startswith('fast'):
                self._motor_ctrl.fast_decay()
            elif _command.startswith('acce'):
                self._motor_ctrl.accelerate(_speed)
            elif _command.startswith('dece'):
                self._motor_ctrl.decelerate(0.0)
            elif _command.startswith('forw'):
                self._motor_ctrl.forward(_speed, _stbd_speed, _duration)
            elif _command.startswith('reve'):
                self._motor_ctrl.reverse(_speed, _stbd_speed, _duration)
            elif _command.startswith('crab'):
                self._motor_ctrl.crab(_speed, _duration)
            elif _command.startswith('rota'):
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
    def parse_payload(payload):
        '''
        Parses the '_' and 'd' delimited payload, returning a tuple
        of command, port speed, starboard speed, and duration; all
        but the command are optional.
        '''
        # extract optional duration
        if 'd' in payload:
            main_part, duration_str = payload.split('d', 1)
            try:
                _duration = float(duration_str)
            except ValueError:
                raise ValueError("invalid duration value: '{}'".format(duration_str))
        else:
            main_part = payload
            _duration = None
        # extract parts from remaining string
        parts = main_part.split('_')
        if not parts:
            raise ValueError("input string is empty or malformed.")
        cmd = parts[0]
        # try to extract port speed, default if missing or invalid
        if len(parts) >= 2:
            try:
                _port = float(parts[1])
            except ValueError:
                raise ValueError(f"invalid port value: '{parts[1]}'")
        else:
            _port = DEFAULT_SPEED
        # try to extract STBD, fallback to PORT
        if len(parts) >= 3:
            try:
                _stbd = float(parts[2])
            except ValueError:
                raise ValueError(f"invalid starboard value: '{parts[2]}'")
        else:
            _stbd = _port
        return cmd, _port, _stbd, _duration

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def start(self):
        while self.enabled and self.errors < ERROR_LIMIT:
            try:
                self.state = self.s_i2c.handle_event()

                if self.state == self.s_i2c.I2CStateMachine.I2C_START:
                    self._log.debug('I2C_START')

                if self.state == self.s_i2c.I2CStateMachine.I2C_RECEIVE:

                    rx_step         = 0
                    rx_count        = 0
                    expected_length = 0

                    while self.s_i2c.Available():
                        byte = self.s_i2c.Read_Data_Received()

                        if rx_step == 0:
                            if byte == 0x01:
                                # start marker detected
                                rx_step = 1
                                rx_count = 0
                            else:
                                self._log.debug("ignoring unexpected byte: '{}'".format(byte))

                        elif rx_step == 1:
                            expected_length = byte
                            rx_step = 2
                            self.currentTransaction.reset()  # Safe to call now, after marker and length

                        elif rx_step == 2:
                            self.currentTransaction.append_data_byte(byte)
                            rx_count += 1
                            if rx_count >= expected_length:
                                rx_step = 3  # expect end marker next

                        elif rx_step == 3:
                            if byte == 0x01:
                                # end marker detected, message complete
                                try:
                                    payload = Payload.from_bytes(self.currentTransaction.data_as_bytes())
                                except Exception as e:
                                    self._log.error("error parsing payload: {}".format(e))
                            else:
                                self._log.error("invalid end marker: {}".format(byte))
                            break # stop reading once message is processed

                if self.state == self.s_i2c.I2CStateMachine.I2C_REQUEST:
                    response = OKAY
                    self._log.debug('sending response: 0x{:02X}…'.format(response))
                    while (self.s_i2c.is_Master_Req_Read()):
                        self.s_i2c.Slave_Write_Data(response)

                if self.state == self.s_i2c.I2CStateMachine.I2C_FINISH:

                    #self._log.debug("register address: {}".format(self.currentTransaction.address))
                    #self._log.debug("register received: '{}'".format(self.currentTransaction.data_as_string()))
                    #self._log.debug("register length: {} chars.".format(self.currentTransaction.data_length()))
                    if self.currentTransaction.data_length() == Payload.PACKET_LENGTH:
                        _payload = Payload.from_bytes(self.currentTransaction.data_as_bytes())
                        self.process_payload(_payload)
                    else:
                        raise I2CSlaveError(PAYLOAD_TOO_LARGE,
                                "package failed with length: {:d} chars (expected 12).".format(self.currentTransaction.data_length()))
                    self._log.debug('finished.')
                    self.reset_transaction()

            except KeyboardInterrupt:
                pass
            except Exception as e:
                self._log.error('{} thrown on transaction: {}'.format(type(e), e))
            except I2CSlaveError as se:
                self._log.error('I2C slave error {} on transaction: {}'.format(se.code, se))
                self.reset_transaction()
                self.errors += 1

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def reset_transaction(self):
        self.currentTransaction.reset()

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
