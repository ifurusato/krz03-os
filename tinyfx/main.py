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
# control for TinyFX

import os
import gc
import _thread
import utime
import machine
from machine import Pin, Timer

import itertools
from colorama import Fore, Style
from rp2040_slave import RP2040_Slave

from tiny_fx import TinyFX
from picofx import MonoPlayer
#from picofx import ColourPlayer
from picofx.mono import StaticFX
from triofx import TrioFX
from rgb_blink import RgbBlinkFX
from i2c_settable_blink import I2CSettableBlinkFX
from pir import PassiveInfrared
from sound_dictionary import _sounds

from colors import*
from stringbuilder import StringBuilder
from core.logger import Level, Logger
from response import (
    RESPONSE_INIT, RESPONSE_PIR_ACTIVE, RESPONSE_PIR_IDLE, RESPONSE_OKAY,
    RESPONSE_UNVALIDATED, RESPONSE_EMPTY_PAYLOAD, RESPONSE_PAYLOAD_TOO_LARGE,
    RESPONSE_BUSY, RESPONSE_RUNTIME_ERROR, RESPONSE_UNKNOWN_ERROR
)

# init ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

_log = Logger('main', Level.INFO)
_log.info(Fore.WHITE + "initialising…")

tiny = TinyFX(wav_root='/sounds')       # create a new TinyFX object to interact with the board
rgbled = tiny.rgb                       # get internal RGBLED
pir_sensor = PassiveInfrared()

# constants ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

ID                =  0
SDA               = 16
SCL               = 17
ADDRESS           = 0x45
VERBOSE           = False
ERROR_LIMIT       = 10     # max errors before exiting main loop

# keys of PIR triggered sound
#   arming-tone, beep, beep-hi, blip, boink, buzz, chatter, chirp, chirp-4,
#   chirp-7, cricket, dit_a, dit_b, dit_c, dwerp, earpit, glince, glitch,
#   gwolp, honk, hzah, ippurt, itiz, izit, pew-pew-pew, pizzle, silence,
#   skid-fzzt, sonic-bat, telemetry, tick, tika-tika, tsk-tsk-tsk, tweak,
#   twiddle-pop, twit, wow, zzt
PIR_SOUND          = 'cricket'

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Controller(object):
    def __init__(self, level=Level.INFO):
        super().__init__()
        self._log = Logger('ctrl', level)

        self.player = MonoPlayer(tiny.outputs)       # create a new effect player to control TinyFX's mono outputs
        #rgb_player = ColourPlayer(tiny.rgb)    # create a new effect player to control TinyFX's RGB output

        self.blink_fx     = I2CSettableBlinkFX(1, speed=0.5, phase=0.0, duty=0.015) # ch4
        self.stbd_trio_fx = TrioFX(2, brightness=0.8) # ch5
        self.port_trio_fx = TrioFX(3, brightness=0.8) # ch6

        # set up the effects to play
        self.player.effects = [
            None, #TrioFX(2, brightness=0.5),  # UNUSED
            None, #TrioFX(3, brightness=1.0),  # UNUSED
            None, # StaticFX(0.7),
            self.blink_fx,
            self.stbd_trio_fx,
            self.port_trio_fx
        ]

        # reset rgbled
        show_color(COLOR_BLACK)

        self._log.info("starting player…")
        self.player.start()

        # established I2C slave on ID=0; SDA=20; SCL=21 at 0x44
        self._log.info("I2C slave starting…")
        self.s_i2c = RP2040_Slave(ID,sda=SDA,scl=SCL,slaveAddress=ADDRESS)
        self.state = self.s_i2c.I2CStateMachine.I2C_START
        # initialize an empty buffer list for sequential write sequences
        data_buf = []
        addr     = 0x00
        self.currentTransaction = self.s_i2c.I2CTransaction(addr, data_buf)

        self._log.info(Fore.WHITE + "main loop starting…")
        show_color(COLOR_GREEN)

        self.pir_enabled = False # default disabled
        self.is_running  = False
        self.enabled     = True
        self.errors      = 0

        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def start(self):
        # indicate startup
        tiny.wav.play_wav('arming-tone.wav')

        while self.enabled and self.errors < ERROR_LIMIT and self.player.is_running():
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

                        if self.pir_enabled:
                            if pir_sensor.triggered:
                                self.pir_enabled = False # don't permit reentry
                                show_color(COLOR_ORANGE)
                                _timer = Timer()
                                _timer.init(mode=Timer.ONE_SHOT, period=4000, callback=self.reenable_pir)
                                self.play(PIR_SOUND)
                            utime.sleep(0.05)

                        _index += 1
                        # end while loop ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

                    if _validated:
                        if _buffer.length() > 0:
                            if _expected_length == _buffer.length():
                                self.process_buffer(_buffer)
                            else:
                                raise I2CSlaveError(RESPONSE_PAYLOAD_TOO_LARGE,
                                        "package failed with expected length: {:d}; actual length: {:d}.".format(_expected_length, _buffer.length()))
                        else:
                            raise I2CSlaveError(RESPONSE_EMPTY_PAYLOAD, 'empty payload.')

                    else:
                        raise I2CSlaveError(RESPONSE_UNVALIDATED, "unvalidated buffer: '{}'".format(_buffer.to_string()))

                if self.state == self.s_i2c.I2CStateMachine.I2C_REQUEST:
                    response = RESPONSE_OKAY
                    self._log.debug('sending response: 0x{:02X}…'.format(response))
                    while (self.s_i2c.is_Master_Req_Read()):
                        self.s_i2c.Slave_Write_Data(response)

                if self.state == self.s_i2c.I2CStateMachine.I2C_FINISH:
                    self._log.debug('register: {}; received: {}'.format(self.currentTransaction.address, self.currentTransaction.data_byte))
                    self._log.info('finished processing request.')
                    self.reset_transaction()

            except KeyboardInterrupt:
                pass
            except I2CSlaveError as se:
                self._log.error('I2C slave error {} on transaction: {}'.format(se.code, se))
                self.reset_transaction()
                self.errors += 1
            except OSError as e:
                self._log.error('OS error on transaction: {}; resetting machine!'.format(e))
                # reboot
                machine.reset()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def process_buffer(self, buffer):
        '''
        Processes the payload to determine the command, and optional speed and
        duration, creating a new Thread to handle the request.
        '''
        try:
            _string = buffer.to_string()
            self._log.debug("process_buffer() value: '{}'".format(_string))
            if _string == 'exit':
                self.enabled = False
            elif _string.startswith('play'):
                key = _string[5:]
                self.play(key)
            else:
                thread_id = _thread.start_new_thread(self.process_payload, (_string,))
        except RuntimeError as rte:
            self._log.error("runtime error: '{}'…".format(rte))
            raise I2CSlaveError(RESPONSE_RUNTIME_ERROR, "runtime error processing payload: {}".format(rte))
        except Exception as e:
            self._log.error("{} raised processing buffer: '{}'…".format(type(e), e))
            raise I2CSlaveError(RESPONSE_UNKNOWN_ERROR, "{} raised processing payload: {}".format(type(e), e))
        except OSError as e:
            raise e

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def process_payload(self, payload):
        if self.is_running:
            self._log.error("Error: function is already running, ignoring subsequent call.")
            return False
        self._log.info("processing payload: '{}'…".format(payload))
        self.is_running = True
        try:
            show_color(COLOR_YELLOW)
            utime.sleep(0.3)
            command, speed, duration = self.parse_payload(payload)
            if command == 'help':
                print(Fore.CYAN + '''
tinyfx commands:
    on                turn on port, stbd and mast LEDs
    off               turn off port, stbd and mast LEDs
    mast|ch4          turn on mast LED
    stbd|ch5          turn on stbd LED
    port|ch6          turn on port LED
    flash             show flash memory info
    ram               show free memory info
    play [key]        play sound
    pir get           return pir state
    pir on            enable pir sensor
    pir off           disable pir sensor
    exit              exit processing loop
        ''' + Style.RESET_ALL)
                pass
            elif payload   == 'ch4' or payload == 'mast':
                self.blink_fx.on()
            elif payload == 'ch5' or payload == 'stbd':
                self.stbd_trio_fx.on()
            elif payload == 'ch6' or payload == 'port':
                self.port_trio_fx.on()
            elif payload == 'on':
                self.blink_fx.on()
                self.port_trio_fx.on()
                self.stbd_trio_fx.on()
            elif payload == 'off':
                self.blink_fx.off()
                self.port_trio_fx.off()
                self.stbd_trio_fx.off()
            elif payload == 'flash':
                stat = os.statvfs('/') # get filesystem stats
                total_space = int(( stat[0] * stat[2] ) / 1000) # Block size * Total blocks
                free_space  = int(( stat[0] * stat[3] ) / 1000) # Block size * Free blocks
                print("total flash: {}KB".format(total_space))
                print("free flash:  {}KB".format(free_space))
            elif payload == 'ram':
                gc.collect() 
                ram_mb = gc.mem_free() / 1024
                print("free ram: {:.2f}KB".format(ram_mb))
            elif payload == 'sounds':
                print('sounds:')
                for name in self.list_wav_files_without_extension():
                    print('  {}'.format(name))
            elif payload == 'pir get':
                if pir_sensor.triggered:
                    response = RESPONSE_PIR_ACTIVE
                    show_color(COLOR_ORANGE)
                else:
                    response = RESPONSE_PIR_IDLE
                    show_color(COLOR_VIOLET)
            elif payload == 'pir on':
                self.pir_enabled = True
                show_color(COLOR_GREEN)
            elif payload == 'pir off':
                self.pir_enabled = False
                show_color(COLOR_GREEN)
            else:
                raise ValueError("unrecognised payload: '{}'".format(payload))
            return True
        finally:
            self.is_running = False
            show_color(COLOR_GREEN)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def list_wav_files_without_extension(self, directory="sounds"):
        try:
            files = os.listdir(directory)
            wav_files = [f[:-4] for f in files if f.endswith('.wav')]
            return wav_files
        except OSError as e:
            print("Error accessing directory:", e)
            return []

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def parse_payload(self, payload):
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
             speed = 0.0
        if len(_split) >= 3:
             duration = float(_split[2])
        return cmd, speed, duration

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def reenable_pir(self, arg=None):
        '''
        Disables the PIR sensor so that it doesn't retrigger before it resets.
        '''
        show_color(COLOR_BLACK)
        self.pir_enabled = True

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def play(self, key):
        for name, filename in _sounds:
            if name == key:
                self._log.info("play key: '{}' from file: ".format(key) + Fore.YELLOW + "'{}'".format(filename))
                tiny.wav.play_wav(filename)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def reset_transaction(self):
        self.currentTransaction.address = 0x00
        self.currentTransaction.data_byte = []

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class I2CSlaveError(Exception):
    def __init__(self, code, message):
        super().__init__(message)
        self._code = code

    @property
    def code(self):
        return self._code

# functions ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

def show_color(color):
    '''
    Display the color on the RGB LED in GRB order.
    '''
    _log.debug('show_color: R{}, G{}, B{}'.format(color[0], color[1], color[2]))
    rgbled.set_rgb(*color)
    pass

# main ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

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
    _log.error('error in main loop: {}; resetting machine [from main]'.format(e))
    # reboot
    machine.reset()

# this only happens following an 'exit', which also disables the I2C slave
_log.info('exit: loop complete.')
show_color(COLOR_RED)
if tiny:
    tiny.wav.play_wav('buzz.wav')
utime.sleep(1.0)
# so we reset...
machine.reset()

#EOF
