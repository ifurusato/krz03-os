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
import uasyncio as asyncio
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
        # initialize an empty buffer list for sequential write sequences
        self._i2c_task   = None
        self._running_event = asyncio.Event()
        # create motor controller
        self._motor_controller = MotorController(_led)
        # established I2C slave on ID=0; SDA=20; SCL=21 at 0x44
        self._log.info("I2C slave starting…")
        self.s_i2c = RP2040_Slave(ID,sda=SDA,scl=SCL,slaveAddress=ADDRESS)
        self.state = self.s_i2c.I2CStateMachine.I2C_START
        self.data_buf = []
        self.addr     = 0x00
        self.currentTransaction = self.s_i2c.I2CTransaction(self.addr, self.data_buf)
        self._log.info(Fore.WHITE + "main loop starting…")
        show_color(COLOR_GREEN)
        self.enabled    = True
        self.errors     = 0
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    async def _i2c_loop(self):
#       while self.enabled and self.errors < ERROR_LIMIT:
        while self._running_event.is_set() and self.errors < ERROR_LIMIT:
            response = RESPONSE_INIT
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
                                response = RESPONSE_INVALID_CHAR
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
                                    print('🐷 anomalous obtainment of payload.')
                                except Exception as e:
                                    response = RESPONSE_BAD_REQUEST
                                    self._log.error("error parsing payload: {}".format(e))
                            else:
                                response = RESPONSE_BAD_REQUEST
                                self._log.error("invalid end marker: {}".format(byte))
                            break # stop reading once message is processed

                if self.state == self.s_i2c.I2CStateMachine.I2C_REQUEST:
                    self._log.debug('sending response: 0x{:02X}…'.format(response))
                    while (self.s_i2c.is_Master_Req_Read()):
                        self.s_i2c.Slave_Write_Data(response)

                if self.state == self.s_i2c.I2CStateMachine.I2C_FINISH:
                    #self._log.debug("register address: {}".format(self.currentTransaction.address))
                    #self._log.debug("register received: '{}'".format(self.currentTransaction.data_as_string()))
                    #self._log.debug("register length: {} chars.".format(self.currentTransaction.data_length()))
                    if self.currentTransaction.data_length() == Payload.PACKET_LENGTH:
                        payload = Payload.from_bytes(self.currentTransaction.data_as_bytes())
                        print('🐸 anomalous obtainment of payload.')
                        show_color(COLOR_YELLOW)
                        response = self._motor_controller.process_payload(payload)
                        show_color(COLOR_GREEN)
                    else:
                        response = RESPONSE_PAYLOAD_TOO_LARGE
                        raise I2CSlaveError(response,
                                "package failed with length: {:d} chars (expected 12).".format(self.currentTransaction.data_length()))
                    self._log.debug('finished.')
                    self.reset_transaction()

            except Exception as e:
                self._log.error('{} thrown on transaction: {}'.format(type(e), e))
                self.reset_transaction()
                self.errors += 1
            except I2CSlaveError as se:
                self._log.error('I2C slave error {} on transaction: {}'.format(se.code, se))
                self.reset_transaction()
                self.errors += 1

            await asyncio.sleep(0.01)  # Prevents blocking the loop

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def reset_transaction(self):
        self.currentTransaction.reset()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    async def enable(self):
        print('🐥 a. enable')
        self._running_event.set() 
        print('🐥 b. enable')
        self._i2c_task = asyncio.create_task(self._i2c_loop())
        print('🐥 c. enable')
        self._motor_controller.enable()
        print('🐥 d. enable')
        await self._running_event.wait() # wait until explicitly stopped
        print('🐥 e. enable post.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    async def disable(self):
        self._running_event.clear() # signal the controller to stop
        if self._i2c_task:
            self._i2c_task.cancel()
            try:
                await self._i2c_task # ensure proper cancellation
            except asyncio.CancelledError as e:
                self._log.error('task cancelled error: {}'.format(e))
                pass
        await self._motor_controller.disable()
#       self._motor_controller.disable()

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

async def run_controller():
    controller = Controller()
    controller.enable()  # starts I2C main loop, blocking
    try:
        while True:
            await asyncio.sleep(1.0)
    except asyncio.CancelledError:
        _log.info('controller task was cancelled.')
    finally:
        await controller.disable()  # cleanup after enable() exits

try:
    asyncio.run(run_controller())  # event loop starts before controller.enable()
except Exception as e:
    _log.error('error in async main loop: {}'.format(e))

#controller = None
#try:
#    _log.info('start controller…')
#    controller = Controller()
#    controller.enable()
#    asyncio.run(controller._keep_loop_alive())
#    _log.info('controller started.')
#except Exception as e:
#    _log.error('error in main loop: {}'.format(e))
#    # reboot
#    machine.reset()
#finally:
#    if controller:
#        asyncio.run(controller.disable())

# this only happens following an 'exit', which also disables the I2C slave
_log.info('exit: loop complete.')
show_color(COLOR_DARK_RED)

#EOF
