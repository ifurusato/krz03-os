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
import utime
from rp2040_slave import RP2040_Slave

from plasma import WS2812
from motor_controller import MotorController
from motor import motor2040

from colors import*
from colorama import Fore, Style
from core.logger import Level, Logger
from payload import Payload
from response import (
    RESPONSE_INIT, RESPONSE_OKAY, RESPONSE_BAD_REQUEST,
    RESPONSE_INVALID_CHAR, RESPONSE_PAYLOAD_TOO_LARGE,
    RESPONSE_BUSY, RESPONSE_RUNTIME_ERROR, RESPONSE_UNKNOWN_ERROR
)

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

VERBOSE           = False
DEFAULT_SPEED     = 0.5 # if speed is not specified on motor commands
ERROR_LIMIT       = 10  # max errors before exiting main loop
MAX_PAYLOAD_LENGTH = 12  # fixed-size payload with CRC

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Controller(object):
    def __init__(self, level=Level.INFO):
        super().__init__()
        self._log = Logger('ctrl', level)
        self._running_event = False
        self._i2c_thread = None  # For running I2C in a separate thread if needed
        self._motor_controller = MotorController(_led)
        # established I2C slave on ID=0; SDA=20; SCL=21 at 0x44
        self._log.info("I2C slave starting…")
        self.s_i2c = RP2040_Slave(ID, sda=SDA, scl=SCL, slaveAddress=ADDRESS)
        self.state = self.s_i2c.I2CStateMachine.I2C_START
        self.data_buf = []
        self.addr = 0x00
        self.currentTransaction = self.s_i2c.I2CTransaction(self.addr, self.data_buf)
        self._log.info("Main loop initialized.")
        show_color(COLOR_GREEN)
        self.enabled = True
        self.errors = 0
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def enable(self):
        self._log.info("enabling controller…")
        self._running_event = True
        self._motor_controller.enable()
        self._i2c_loop()

    def disable(self):
        self._log.info("disabling controller…")
        self._running_event = False
        self._motor_controller.disable()
        self._log.info("disabled.")

    def _i2c_loop(self):
        self._log.info("Starting I2C main loop…")
        while self._running_event:
            response = RESPONSE_INIT
            try:
                self.state = self.s_i2c.handle_event()
                if self.state == self.s_i2c.I2CStateMachine.I2C_START:
                    self._log.debug("I2C_START")
                if self.state == self.s_i2c.I2CStateMachine.I2C_RECEIVE:
                    self._handle_receive()
                if self.state == self.s_i2c.I2CStateMachine.I2C_REQUEST:
                    self._handle_request(response)
                if self.state == self.s_i2c.I2CStateMachine.I2C_FINISH:
                    response = self._handle_finish()
                    self.reset_transaction()
            except Exception as e:
                self._log.error(f"Error in I2C transaction: {e}")
                self.reset_transaction()
                self.errors += 1
            utime.sleep(0.01) # minimal delay to prevent a tight loop
        self._log.info("I2C main loop stopped.")

    def _handle_receive(self):
        rx_step = 0
        rx_count = 0
        expected_length = 0

        while self.s_i2c.Available():
            byte = self.s_i2c.Read_Data_Received()

            if rx_step == 0:
                if byte == 0x01:  # Start marker
                    rx_step = 1
                    rx_count = 0
                else:
                    self._log.debug(f"Ignoring unexpected byte: {byte}")

            elif rx_step == 1:
                expected_length = byte
                rx_step = 2
                self.currentTransaction.reset()

            elif rx_step == 2:
                self.currentTransaction.append_data_byte(byte)
                rx_count += 1
                if rx_count >= expected_length:
                    rx_step = 3

            elif rx_step == 3:
                if byte == 0x01:  # end marker
                    self._log.info("received end marker.")
                else:
                    self._log.error(f"Invalid end marker: {byte}")
                break

    def _handle_request(self, response):
        self._log.info(Fore.GREEN + "main._handle_REQUEST; sending response: 0x{:02X}".format(response))
        while self.s_i2c.is_Master_Req_Read():
            self.s_i2c.Slave_Write_Data(response)

    def _handle_finish(self):
        try:
            if self.currentTransaction.data_length() == Payload.PACKET_LENGTH:
                payload = Payload.from_bytes(self.currentTransaction.data_as_bytes())
                response = self._motor_controller.process_payload(payload)
                self._log.info(Fore.WHITE + 'main._handle_FINISH; response: 0x{:02X}'.format(response))
                return response
            else:
                raise I2CSlaveError(RESPONSE_PAYLOAD_TOO_LARGE, "Payload too large.")
        except I2CSlaveError as se:
            self._log.error("I2C slave error: {}".format(se))
            return RESPONSE_PAYLOAD_TOO_LARGE

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

controller = None
try:
    _log.info('start controller…')
    controller = Controller()
    _log.info('controller created.')
    controller.enable()
    _log.info('controller started; it should have blocked.')
except Exception as e:
    _log.error('error in main loop: {}'.format(e))
finally:
    if controller:
        controller.disable()

# this only happens following an 'exit', which also disables the I2C slave
_log.info('exit: loop complete.')
show_color(COLOR_DARK_RED)

#EOF
