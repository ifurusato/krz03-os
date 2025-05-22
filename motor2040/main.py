#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-08-14
# modified: 2025-05-22
#
# control for Motor 2040

# If motor_controller currently returns just a byte or string, update it like: 
#
#    return RESPONSE_OKAY, "Speed:45"


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
    RESPONSE_INIT, RESPONSE_PIR_ACTIVE, RESPONSE_PIR_IDLE, RESPONSE_OKAY,
    RESPONSE_BAD_ADDRESS, RESPONSE_BAD_REQUEST, RESPONSE_OUT_OF_SYNC,
    RESPONSE_INVALID_CHAR, RESPONSE_SOURCE_TOO_LARGE, RESPONSE_PAYLOAD_TOO_LARGE,
    RESPONSE_UNVALIDATED, RESPONSE_EMPTY_PAYLOAD, RESPONSE_BUSY, RESPONSE_SKIPPED,
    RESPONSE_CONN_ERROR, RESPONSE_RUNTIME_ERROR, RESPONSE_UNKNOWN_ERROR
)

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

# init ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

_log = Logger('main', Level.INFO)
_log.info(Fore.WHITE + "initialising…")

# RGB LED
_led = WS2812(motor2040.NUM_LEDS, 1, 0, motor2040.LED_DATA)
_led.start()

# constants ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈


ID                  =  0
SDA                 = 20
SCL                 = 21
ADDRESS             = 0x44

TEST_SEND           = False
VERBOSE             = False
DEFAULT_SPEED       = 0.5 # if speed is not specified on motor commands
ERROR_LIMIT         = 10  # max errors before exiting main loop
MAX_PAYLOAD_LENGTH  = 12  # fixed-size payload with CRC
MAX_RESPONSE_LENGTH = 32

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Controller:
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
        # response buffer
        self._response_buffer = bytearray(32)
        self._response_length = 0
        self._response_index  = 0
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

    def build_response(self, code: str, data: str = "") -> bytes:
        '''
        Builds a 32-byte response starting with a 4-char response code, followed by data (up to 28 chars).
        '''
        full_response = (code + data)[:32]  # Ensure response fits within 32 bytes
        self._response_buffer = full_response.encode("ascii", errors="replace")  # Convert to bytes
        self._response_length = len(self._response_buffer)  # Set response length
        self._response_index = 0  # Reset index to start transmission
        return self._response_buffer

    def _i2c_loop(self):
        self._log.info("Starting I2C main loop…")
        while self._running_event:
            response = self.build_response(RESPONSE_INIT)
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

    def _handle_request(self, response=None):
        """
        Send a fixed response if TEST_SEND is True, otherwise proceed with normal response logic.
        """
        if TEST_SEND:
            # create a fixed 32-byte response (4-char response code + payload)
            fixed_response = b"REOKData transmission successful."  # total length = 32 bytes
            self._response_buffer = fixed_response[:32]  # truncate to max length if needed
            self._response_length = len(self._response_buffer)
            self._response_index = 0
            # send the buffer one byte at a time
            while self.s_i2c.is_Master_Req_Read() and self._response_index < self._response_length:
                byte_to_send = self._response_buffer[self._response_index]
                self.s_i2c.Slave_Write_Data(byte_to_send)
                self._log.debug(f"Sent byte {self._response_index + 1}/{self._response_length}: {chr(byte_to_send)}")
                self._response_index += 1
        else:
            # Proceed with the regular logic (use the response argument if given)
            if response is None:
                response = self.build_response()

            # Ensure response is bytes and fits in 32 bytes
            if isinstance(response, str):
                response = response.encode('utf-8')
            self._response_buffer = response[:32]
            self._response_length = len(self._response_buffer)
            self._response_index = 0

            while self.s_i2c.is_Master_Req_Read() and self._response_index < self._response_length:
                byte_to_send = self._response_buffer[self._response_index]
                self.s_i2c.Slave_Write_Data(byte_to_send)
                self._log.debug(f"Sent byte {self._response_index + 1}/{self._response_length}: {chr(byte_to_send)}")
                self._response_index += 1

    def _handle_finish(self) -> bytes:
        try:
            if self.currentTransaction.data_length() == Payload.PACKET_LENGTH:
                payload = Payload.from_bytes(self.currentTransaction.data_as_bytes())
                response_code, response_data = self._motor_controller.process_payload(payload)
                self._log.info(f"main._handle_FINISH; response: {response_code} {response_data}")
                return self.build_response(response_code, response_data)
            else:
                raise I2CSlaveError(RESPONSE_TOO_LARGE, "Payload too large.")
        except I2CSlaveError as se:
            self._log.error(f"I2C slave error: {se}")
            return self.build_response(se.args[0], str(se))
        except Exception as e:
            self._log.error(f"Unexpected error: {e}")
            return self.build_response(RESPONSE_UNKNOWN_ERROR, "Unexpected")

        # Truncate if too long
        encoded_response = response.encode('ascii', errors='replace')[:MAX_RESPONSE_LENGTH]
        self._response_buffer[:len(encoded_response)] = encoded_response
        self._response_length = len(encoded_response)
        self._response_index = 0
        self._log.info(f"Prepared response: {encoded_response}")
        return response

    def reset_transaction(self):
        self.currentTransaction.reset()
        self._response_index = 0
        self._response_length = len(RESPONSE_INIT)
        self._response_buffer[:self._response_length] = RESPONSE_INIT.encode('ascii')

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
