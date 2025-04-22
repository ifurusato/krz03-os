#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-09-08
# modified: 2024-11-15
#
# Tiny FX main
#
#   channel 4: blink_fx
#   channel 5: stbd_trio_fx
#   channel 6: port_trio_fx

import os
import gc
import utime
from machine import Pin, Timer
from tiny_fx import TinyFX
from picofx import MonoPlayer, ColourPlayer
#from picofx.colour import RainbowFX

from colors import*
import itertools
from pir import PassiveInfrared
from picofx.mono import StaticFX
from triofx import TrioFX
from rgb_blink import RgbBlinkFX
from i2c_settable_blink import I2CSettableBlinkFX

from RP2040_Slave import I2C_Slave
from stringbuilder import StringBuilder
from sound_dictionary import _sounds

# constants ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

PIR_ENABLED        = False # default disabled
COLOR_TEST         = True

REQUEST_OFF        = 'request_off'
REQUEST_PIR_GET    = 'pir_get'
REQUEST_PIR_ON     = 'pir_on'
REQUEST_PIR_OFF    = 'pir_off'
REQUEST_ERROR      = 'error' # unrecognised payload

# keys of PIR triggered sound
#   arming-tone, beep, beep-hi, blip, boink, buzz, chatter, chirp, chirp-4,
#   chirp-7, cricket, dit_a, dit_b, dit_c, dwerp, earpit, glince, glitch, 
#   gwolp, honk, hzah, ippurt, itiz, izit, pew-pew-pew, pizzle, silence, 
#   skid-fzzt, sonic-bat, telemetry, tick, tika-tika, tsk-tsk-tsk, tweak, 
#   twiddle-pop, twit, wow, zzt
PIR_SOUND          = 'cricket'

# I2C slave constants ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

I2C_ID             = 0
SDA_PIN            = 16
SCL_PIN            = 17
I2C_ADDRESS        = 0x45
MAX_CHARS          = 32

# response codes:
INIT               = 0x10
PIR_ACTIVE         = 0x30
PIR_IDLE           = 0x31
BAD_ADDRESS        = 0x41
BAD_REQUEST        = 0x42
OUT_OF_SYNC        = 0x43
INVALID_CHAR       = 0x44
SOURCE_TOO_LARGE   = 0x45
UNVALIDATED        = 0x46
EMPTY_PAYLOAD      = 0x47
PAYLOAD_TOO_LARGE  = 0x48
UNKNOWN_ERROR      = 0x49
OKAY               = 0x4F # all responses must be less than this

# I2C support classes ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

class I2CSlaveError(Exception):
    def __init__(self, code, message):
        super().__init__(message)
        self._code = code

    @property
    def code(self):
        return self._code

# I2C support methods ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

def show_color(title, color):
    '''
    Display the color on the NeoPixel.

      COLOR_BLACK       COLOR_RED         COLOR_GREEN       COLOR_BLUE
      COLOR_CYAN        COLOR_MAGENTA     COLOR_YELLOW      COLOR_YELLOW_GREEN
      COLOR_DARK_CYAN   COLOR_ORANGE      COLOR_INDIGO      COLOR_VIOLET
    '''
    rgbled.set_rgb(*color)
    if title:
#       print('show color: {}'.format(title))
        pass

def process_buffer(buffer):
    '''
    Receives the packet sent by the master, returning the contents as a string.
    This can be expanded to further process the value.
    '''
    _payload = buffer.to_string()
#   print("\nreceived payload: '{}'".format(_payload))
    if _payload   == 'ch4' or _payload == 'mast':
        blink_fx.on()
    elif _payload == 'ch5' or _payload == 'stbd':
        stbd_trio_fx.on()
    elif _payload == 'ch6' or _payload == 'port':
        port_trio_fx.on()
    elif _payload == 'on':
        blink_fx.on()
        port_trio_fx.on()
        stbd_trio_fx.on()
    elif _payload == 'off':
        blink_fx.off()
        port_trio_fx.off()
        stbd_trio_fx.off()
        _payload = REQUEST_OFF
    elif _payload == 'flash':
        stat = os.statvfs('/') # get filesystem stats
        total_space = int(( stat[0] * stat[2] ) / 1000) # Block size * Total blocks
        free_space  = int(( stat[0] * stat[3] ) / 1000) # Block size * Free blocks
        print("total flash: {}KB".format(total_space))
        print("free flash:  {}KB".format(free_space))
    elif _payload == 'ram':
        gc.collect()
        ram_mb = gc.mem_free() / 1024
        print("free ram: {:.2f}KB".format(ram_mb))
    elif _payload.startswith('play '):
        key = _payload[5:]
        play(key)
    elif _payload == 'pir get':
        _payload = REQUEST_PIR_GET
    elif _payload == 'pir on':
        _payload = REQUEST_PIR_ON
    elif _payload == 'pir off':
        _payload = REQUEST_PIR_OFF
    else:
        print("unrecognised payload: '{}'".format(_payload))
        _payload = REQUEST_ERROR
    return _payload

def play(key):
    for name, filename in _sounds:
        if name == key:
            print("play file: '{}'".format(filename))
            tiny.wav.play_wav(filename)

def reset():
    global index, payload, response, currentTransaction, state
    index = 0
    payload = ''
    response = INIT
    currentTransaction.reset()
    state = s_i2c.I2CStateMachine.I2C_START

def i2c_poll(timer):
    global index, _counter, payload, response, currentTransaction, state, pir_sensor, PIR_ENABLED
    try:
        state = s_i2c.handle_event()
        if state == None:
            pass
        elif state == s_i2c.I2CStateMachine.I2C_START:
            show_color('start', COLOR_MAGENTA)
            pass
        elif state == s_i2c.I2CStateMachine.I2C_RECEIVE:
#           print('I2C_RECEIVE: {}'.format(payload))
            show_color('i2c_receive', COLOR_YELLOW)
            '''
            Receive data from the master. The first byte is 0x00, followed by a byte
            indicating the count of bytes in the payload, then the data bytes followed
            by 0x01 to validate, then 0xFF to finish.
            '''
            if currentTransaction.address == 0x00:
                # first byte received is the register address
                _register_address = s_i2c.Read_Data_Received()
                currentTransaction.address = _register_address

            _valid = False
            index = 0
            _sb = StringBuilder()
            _expected_length = 0

            # read all data byte received until Rx FIFO is empty
            while s_i2c.Available():

                _data_rx = s_i2c.Read_Data_Received()
                _int_value = int(_data_rx)
                if _data_rx == 0x00:
                    pass
                elif _data_rx == 0x01:
                    _valid = True
                elif _data_rx == 0xFF:
                    show_color('END 0xFF', COLOR_MAGENTA)
                    break
                else:
                    if index == 0:
                        _expected_length = _int_value
                        if _expected_length > MAX_CHARS:
                            show_color('packet failure', COLOR_RED)
                            raise I2CSlaveError(SOURCE_TOO_LARGE,
                                    "WARNING: packet failed with {:d} chars, exceeded maximum length of {:d}.".format(_expected_length, MAX_CHARS))
                    elif _sb.length() < _expected_length:
                        if ( _int_value >= 32 ) and ( _int_value < 127 ):
                            _sb.append(chr(_data_rx))
                        else:
                            show_color('invalid character received', COLOR_RED)
                            raise I2CSlaveError(INVALID_CHAR,
                                    "invalid character received: '0x{:02X}' (int: '{:d}'); buf length: {:d}; sb: '{}'".format(_data_rx, _int_value, _sb.length(), _sb.to_string()))
                    else:
                        show_color('out of sync', COLOR_RED)
                        raise I2CSlaveError(OUT_OF_SYNC,
                                "out of sync: '0x{:02X}' (int: '{:d}'); buf length: {:d}; sb: '{}'".format(_data_rx, _int_value, _sb.length(), _sb.to_string()))
                index = index + 1
                currentTransaction.data_byte.append(_data_rx)

                # end while ..........................................

            if _sb.length() > 0:
                if _valid:
                    if _expected_length != _sb.length():
                        show_color('package failed', COLOR_RED)
                        raise I2CSlaveError(PAYLOAD_TOO_LARGE, "package failed with expected length: {:d}; actual length: {:d}.".format(_expected_length, _sb.length()))
                    else:
                        # set response based on request here .........
                        payload = process_buffer(_sb)
                        print('receive payload: {}'.format(payload))
                        if payload == REQUEST_ERROR:
                            response = BAD_REQUEST
                        elif payload == REQUEST_OFF:
                            response = REQUEST_OFF
                        else:
                            response = OKAY
                else:
                    show_color('unvalidated buffer', COLOR_RED)
                    raise I2CSlaveError(UNVALIDATED, "unvalidated buffer: '{}'".format(_sb.to_string()))

            # end of receive loop

        elif state == s_i2c.I2CStateMachine.I2C_REQUEST:
#           print('I2C_REQUEST: {}'.format(payload))
            # process payload and set here .........
            if len(payload) > 0:
                if payload == REQUEST_ERROR:
                    response = BAD_REQUEST
                    show_color('bad request', COLOR_RED)
                elif payload == REQUEST_OFF:
                    response = OKAY
                    show_color('okay', COLOR_BLACK) # okay but dim the lights
                elif payload == REQUEST_PIR_ON:
                    PIR_ENABLED = True
                    show_color('pir on', COLOR_GREEN)
                elif payload == REQUEST_PIR_OFF:
                    PIR_ENABLED = False
                    show_color('pir off', COLOR_GREEN)
                elif payload == REQUEST_PIR_GET:
                    if pir_sensor.triggered:
                        response = PIR_ACTIVE
                        show_color('pir active', COLOR_ORANGE)
                    else:
                        response = PIR_IDLE
                        show_color('pir idle', COLOR_VIOLET)
                else:
                    response = OKAY
                    show_color('okay', COLOR_GREEN)
            else:
                show_color('empty payload', COLOR_RED)
            # otherwise use existing response
            while (s_i2c.is_Master_Req_Read()):
                s_i2c.Slave_Write_Data(response)

        elif state == s_i2c.I2CStateMachine.I2C_FINISH:
            reset()

        # now blink
        if next(_counter) % 1000 == 0:
            utime.sleep_ms(4)

    except KeyboardInterrupt:
        print('Ctrl-C caught in callback method, exiting…')
        return
    except I2CSlaveError as se:
        print('I2C slave error {} on transaction: {}'.format(se.code, str(se)))
        # empty buffer
        while s_i2c.Available():
            _data_rx = s_i2c.Read_Data_Received()
        reset()
        while (s_i2c.is_Master_Req_Read()):
            print("sending error response: {}".format(str(se)))
            s_i2c.Slave_Write_Data(se.code)
    except Exception as e:
        print('Exception raised: {}'.format(e))

def reenable_pir(arg=None):
    '''
    Disables the PIR sensor so that it doesn't retrigger before it resets.
    '''
    global PIR_ENABLED
    show_color('pir reset', COLOR_BLACK)
    PIR_ENABLED = True

# main ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

# time allotted to permit interrupt for rshell
for i in range(5):
    print('[{}] starting up…'.format(i))
    utime.sleep(1)

tiny = TinyFX(wav_root='/sounds')       # create a new TinyFX object to interact with the board
tiny.wav.play_wav('arming-tone.wav')

player = MonoPlayer(tiny.outputs)       # create a new effect player to control TinyFX's mono outputs
rgbled = tiny.rgb                       # get internal RGBLED
#rgb_player = ColourPlayer(tiny.rgb)    # create a new effect player to control TinyFX's RGB output

blink_fx     = I2CSettableBlinkFX(1, speed=0.5, phase=0.0, duty=0.015) # ch4
stbd_trio_fx = TrioFX(2, brightness=0.8) # ch5
port_trio_fx = TrioFX(3, brightness=0.8) # ch6

# set up the effects to play
player.effects = [
    None, #TrioFX(2, brightness=0.5),  # UNUSED
    None, #TrioFX(3, brightness=1.0),  # UNUSED
    None, # StaticFX(0.7),
    blink_fx,
    stbd_trio_fx,
    port_trio_fx
]

# set color of RGBLED:
show_color('clear', COLOR_BLACK)

# initial conditions ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

_counter = itertools.count()
s_i2c = None
index = 0
payload = ''
response = INIT

try:
#   global index, payload, response, currentTransaction, state

    pir_sensor = PassiveInfrared()

    # establish I2C slave ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    print("starting I2C slave…")
    s_i2c = I2C_Slave(I2C_ID, sda=SDA_PIN, scl=SCL_PIN, slaveAddress=I2C_ADDRESS)
    currentTransaction = s_i2c.I2CTransaction(0x00, [])
    state = s_i2c.I2CStateMachine.I2C_START

    print("starting player…")
    player.start()

    if COLOR_TEST:
        for _ in range(3):
            show_color(None, COLOR_CYAN)
            utime.sleep(0.2)
            show_color(None, COLOR_BLACK)
            utime.sleep(0.3)

    # start the effects running ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    print("starting loop…")
    _active = True # we must be active on the I2C or we'll mess it up
    # loop until the effect stops or the "Boot" button is pressed
    while player.is_running():
        if tiny.boot_pressed():
            _active = not _active
            if _active:
                stbd_trio_fx.off()
                port_trio_fx.off()
                tiny.wav.play_wav('sonic-bat.wav')
            else:
                tiny.wav.play_wav('hzah.wav')
            utime.sleep(0.5)
        if _active:
            i2c_poll(None)
            if PIR_ENABLED:
                if pir_sensor.triggered:
                    PIR_ENABLED = False # don't permit reentry
                    show_color('pir on…', COLOR_ORANGE)
                    _timer = Timer()
                    _timer.init(mode=Timer.ONE_SHOT, period=4000, callback=reenable_pir)
                    play(PIR_SOUND)
            utime.sleep(0.05)

    print("end of loop.")

except KeyboardInterrupt:
    print('Ctrl-C caught, exiting…')
except Exception as e:
    print('Exception raised: {}'.format(e))
finally:
    player.stop()
    tiny.shutdown()

#EOF
