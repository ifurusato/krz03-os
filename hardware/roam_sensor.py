#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-04-02
# modified: 2025-04-29
#
# Description:
#
# Populates an 8x8 matrix from data polled from a VL53L5CX 8x8 ToF sensor,
# performs some further processing, including a display to the console.
#
# For each column, returns the minimum, maximum, and mean values, as well
# as the index of the largest mean value. This also provides a single bias
# value and a pair of lambda functions to be applied to the motors.
#
# Usage:
#
# The first time the script is been executed it will upload the sensor's
# firmware, which takes quite awhile. Executions of the script may
# thereafter include a 'skip' argument to avoid the firmware upload,
# which survives until power down.
#
# Hardware-wise, this requires a VL53L5CX.
#
# Depencies/requirements include:
#
#  * vl53l5cx library: (https://github.com/pimoroni/vl53l5cx-python),
#  * numpy and colorama
#
# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

import sys, traceback
import threading
import time, datetime
import statistics
import numpy as np
from enum import Enum

import pigpio
import vl53l5cx_ctypes as vl53l5cx
from vl53l5cx_ctypes import STATUS_RANGE_VALID, STATUS_RANGE_VALID_LARGE_PULSE, RANGING_MODE_CONTINUOUS, RANGING_MODE_AUTONOMOUS
from colorama import init, Fore, Style
init()

from core.component import Component
from core.logger import Logger, Level
from hardware.pigpiod_util import PigpiodUtility

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class BiasType(Enum):
    FROM_TARGET      = ( 0, 'bias from target',                       'targ' )
    NO_WEIGHT        = ( 1, 'bias no weight',                         'nowt' )
    EXP_WEIGHT       = ( 2, 'bias exponential weighting',             'expw' )
    INV_PROX_WEIGHT  = ( 3, 'bias inverse obstacle proximity weight', 'ipwt' )

    # ignore the first param since it's already set by __new__
    def __init__(self, num, label, key):
        self._label = label
        self._key = key

    @property
    def label(self):
        return self._label

    @property
    def key(self):
        return self._key

    @classmethod
    def from_key(cls, search_key):
        for member in cls:
            if member.key == search_key:
                return member
        raise ValueError(f"No bias type member with key '{search_key}'")


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class RoamSensor(Component):
    '''
    Wraps a VL53L5CX sensor as a sensor to supply a Roam behaviour, making
    available port and starboard lambdas to be applied to the motor controller.
    '''
    def __init__(self, config, skip_init=False, level=Level.INFO):
        self._log = Logger('roams', level)
        Component.__init__(self, self._log, suppressed=False, enabled=True)
        self._skip_init       = skip_init
        self._pi              = None
        self._vl53            = None
        self._stop_event      = None
        self._read            = 0
        self._count           = 0
        self._indexMax        = 0
        self._startTime       = 0
        self._cols, self._row_count = 8, 8
        self._rows            = 60 # the number of rows to scroll
        self._screen_pad      = '\n' * self._rows # as tall as your screen (you may need to adjust this to suit your terminal)
        self._limit           = [ 200, 300, 400, 500, 1000, 2000, 3000, 4000 ] # mm limits of ea display category
        self._dots            = [ " ⚪ ", " 🔴 ", " 🟠 ", " 🟡 ", " 🟢 ", " 🔵 ", " 🟣 ", " 🟤 ", " ⚫ " ]
        self._min_dist_mm     = 1000 # permit targeting if greater than this distance
        self._matrix          = np.array([[0 for x in range(self._cols)] for y in range(self._row_count)])
        self._mean            = [0 for x in range(self._cols)]
        self._ikon            = [[0 for x in range(self._cols)] for y in range(self._row_count)]
        self._maxDot          = np.array([self._dots[8] for x in range(self._cols)], dtype=object)
        self._target = [False] * 8
        self._verbose         = False
        # visualisation settings
        self._displayData     = False
        self._showMatrix      = False
        self._showSummary     = False
        self._showDistances   = False
        # motor lambda settings
        self._reverse_bias    = False # default: normal behavior
        self._bias            = 0.5 # 0.5 is mid-way
        self._bias_strength   = 0.5 # how much the bias effects the motors
        self._port_speed      = 1.0
        self._stbd_speed      = 1.0
        # define lambdas to calculate the adjusted motor speeds
        self._apply_port_bias = lambda: self._port_speed * (1.0 - max(0.0, (1.0 - self._bias if self._reverse_bias else self._bias) - 0.5) * 2 * self._bias_strength)
        self._apply_stbd_bias = lambda: self._stbd_speed * (1.0 - max(0.0, 0.5 - (1.0 - self._bias if self._reverse_bias else self._bias)) * 2 * self._bias_strength)
        # bias feature choice
        self._bias_type = BiasType.from_key('targ') # TODO from config
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_port_bias_lambda(self):
        '''
        Returns a lambda to be applied to the port motor speed.
        '''
        return self._apply_port_bias

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_stbd_bias_lambda(self):
        '''
        Returns a lambda to be applied to the starboard motor speed.
        '''
        return self._apply_stbd_bias

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def matrix(self):
        return self._matrix

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def mean(self):
        return self._mean

    def update_motor_speeds(self):
        # Get the adjusted motor speeds based on the current bias
        _adjusted_port_speed = self._apply_port_bias()  # Apply bias to port motor and get the adjusted value
        _adjusted_stbd_speed = self._apply_stbd_bias()  # Apply bias to starboard motor and get the adjusted value
        # optionally print the adjusted motor speeds
        if self._verbose:
            print("      adjusted: " + Fore.GREEN + "    stbd speed: {:3.2f} ".format(_adjusted_stbd_speed) + Fore.RED + "{:>11}port speed: {:3.2f}".format("", _adjusted_port_speed))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _getVL53(self):
        '''
        Instantiate the VL53L5CX sensor.
        '''
        _stime = datetime.datetime.now()
        if RoamSensor.hasArg('skip'):
            self._log.info("initialising VL53L5CX…")
        else:
            self._log.info("uploading firmware to VL53L5CX, please wait…")
        _vl53 = vl53l5cx.VL53L5CX(skip_init=self._skip_init)
        _vl53.set_resolution(8 * 8)
        _vl53.set_ranging_frequency_hz(15)
        _vl53.set_integration_time_ms(20)
        _vl53.set_ranging_mode(RANGING_MODE_CONTINUOUS)
        _executionMs = int((datetime.datetime.now() - _stime).total_seconds() * 1000)
        self._log.info(Fore.BLUE + 'get VL53: {}ms elapsed.'.format(_executionMs))
        return _vl53

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def enable(self):
        self._log.info('🌸 enabling…')
        Component.enable(self)
        PigpiodUtility.ensure_pigpiod_is_running()
        self._pi = pigpio.pi()
        if not self._pi.connected:
            raise Exception('cannot continue: pigpiod not running.')
        # set up VL53L5CX
        self._vl53 = self._getVL53()
        self._vl53.start_ranging()

        self._stop_event = threading.Event()
        self._thread = threading.Thread(name='sensor-loop', target=self._sensor_loop, args=(self._stop_event,))
        self._thread.start()
        self._log.info('🌸 enabled.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _sensor_loop(self, stop_event):
        self._log.info('starting loop…')
        while not stop_event.is_set():
            self._startTime = datetime.datetime.now()
            self._count += 1
            _data = self._getData()
            if _data != None:
                _distance = np.array(_data.distance_mm).reshape((8, 8))
                for x in range(0,8):
                    for y in range(0,8):
                        self._matrix[x,y] = _distance[x][y]
                self._processData()
                if self._displayData:
                    self.showMatrix()
                self.update_motor_speeds()
        self._log.info('exited loop.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def showMatrix(self):
        '''
        Displays the matrix and related data to the console.
        '''
        if self._showMatrix:
            # print header .........................................
            print(self._screen_pad)
            print(Fore.BLUE + "  {:07d}   ".format(self._count)
                    + Fore.GREEN + "STBD"
                    + Fore.BLUE + "     0     1     2     3     4     5     6     7     "
                    + Fore.RED + "PORT\n" + Fore.BLUE )
            # print matrix .........................................
            for r in range(7, -1, -1):
                print("                   " + "  ".join(str(val) for val in self._ikon[r]) + " \n")

        if self._showSummary:
            if not self._showMatrix:
                # as tall as your screen plus the missing matrix display
                print(self._screen_pad)
                MMMM = '\n' * 18
                print(MMMM)
            # print minimum distance ...............................
            minV = self._matrix.min(axis=0, keepdims=True)
            print("           min:    " + "  ".join(self._dot(val) for val in minV[0]))
            # print maximum distance ...............................
            maxV = self._matrix.max(axis=0, keepdims=True)
            print("           max:    " + "  ".join(self._dot(val) for val in maxV[0]))
            # print mean distance by column ........................
            print("          mean:    {}".format("  ".join(self._dot(x) for x in self._mean)))
            # print maximum mean distance ..........................
            self._maxDot.fill(self._dots[8])
            _maxValue = self._mean[self._indexMax]
            # clear target array
            self._target[:] = [False] * 8
            self._target[self._indexMax] = True
            if _maxValue > self._min_dist_mm:
                self._maxDot[self._indexMax] = self._dots[0]
            else:
                self._maxDot[self._indexMax] = self._dots[1]
            print("        target:    {}  {}  {}  {}  {}  {}  {}  {}".format(*self._maxDot))

        if self._showDistances:
            '''
            Print numeric distances in mm, plus bias.
            '''
            match self._bias_type:
                case BiasType.FROM_TARGET:
                    self._bias = self.compute_bias_from_target()
                case BiasType.NO_WEIGHT:
                    self._bias = self.compute_bias()
                case BiasType.EXP_WEIGHT:
                    self._bias = self.compute_bias_exp()
                case BiasType.INV_PROX_WEIGHT:
                    self._bias = self.compute_bias_inverse()
            print("  mean dist mm:     {:<4}  {:<4}  {:<4}  {:<4}  {:<4}  {:<4}  {:<4}  {:<4}     bias: {:3.1f}".format(*self._mean, self._bias))

    def _dot(self, value):
        '''
        Return a colored dot indicating the range.
        '''
        if value   <= self._limit[0]:
            return self._dots[0]
        elif value <= self._limit[1]:
            return self._dots[1]
        elif value <= self._limit[2]:
            return self._dots[2]
        elif value <= self._limit[3]:
            return self._dots[3]
        elif value <= self._limit[4]:
            return self._dots[4]
        elif value <= self._limit[5]:
            return self._dots[5]
        elif value <= self._limit[6]:
            return self._dots[6]
        elif value <= self._limit[7]:
            return self._dots[7]
        else: # (out of range)
            return self._dots[8]

    def compute_bias_from_target(self):
        index = self._target.index(True)  # Assumes exactly one True
        return index / (len(self._target) - 1)

    def compute_bias_no_weight(self):
        _indices = np.arange(8)  # [0, 1, 2, ..., 7]
        _weighted_sum = np.dot(self._mean, _indices)
        _total = np.sum(self._mean)
        if _total == 0:
            return 0.5  # no information, center bias
        _center_of_mass = _weighted_sum / _total
        _bias = _center_of_mass / 7
        return _bias

    def compute_bias_exp(self, power=2):
        distances = np.array(self._mean, dtype=float)
        enhanced = distances ** power  # Amplify differences
        indices = np.arange(len(distances))
        weighted_sum = np.dot(enhanced, indices)
        total = np.sum(enhanced)
        if total == 0:
            return 0.5
        center_of_mass = weighted_sum / total
        return center_of_mass / (len(distances) - 1)

    def compute_bias_inverse(self, epsilon=1e-5):
        distances = np.array(self._mean, dtype=float)
        weights = 1.0 / (distances + epsilon)  # Closer = more dangerous
        safety = 1.0 - (weights / np.max(weights))  # Normalize so 1 = safest
        indices = np.arange(len(distances))
        weighted_sum = np.dot(safety, indices)
        total = np.sum(safety)
        if total == 0:
            return 0.5
        center_of_mass = weighted_sum / total
        return center_of_mass / (len(distances) - 1)

    def _processData(self):
        '''
        Processes the data from the matrix.
        '''
        try:
            m = np.array([[0 for x in range(self._cols)] for y in range(self._row_count)])
            for row in range(8):
                # populate ikon and mean arrays
                for col in range(8):
                    n = self._matrix[row,col]
                    self._ikon[row][col] = self._dot(n)
                    # calculate column means
                    self._mean[col] = round(statistics.fmean([
                            self._matrix[0][col], self._matrix[1][col], self._matrix[2][col], self._matrix[3][col],
                            self._matrix[4][col], self._matrix[5][col], self._matrix[6][col], self._matrix[7][col] ]))
                row += 1
            self._indexMax = self._mean.index(max(self._mean))
        except Exception as e:
            self._log.error('error in process data: {}\n{}'.format(e, traceback.format_exc()))
        finally:
            if self._showMatrix or self._showSummary:
                _executionMs = int((datetime.datetime.now() - self._startTime).total_seconds() * 1000)
                print(Fore.BLUE + '\n loop: {}ms elapsed.\n'.format(_executionMs) + Style.RESET_ALL)

    @staticmethod
    def hasArg(arg):
        '''
        Returns true if the command line arguments contains the argument.
        We ignore argument 0 as that is the name of the script.
        '''
        for i in range(1, len(sys.argv)):
            if sys.argv[i] == arg:
                return True
        return False

    def _getData(self):
        '''
        Wait until the data from the vl53 is ready, returning the data, otherwise None upon a timeout.
        '''
        _stime = datetime.datetime.now()
        for i in range(1,20):
            if self._vl53.data_ready():
                data = self._vl53.get_data() # 2d array of distance
                if self._showMatrix or self._showSummary:
                    _executionMs = int((datetime.datetime.now() - _stime).total_seconds() * 1000)
                    print(Fore.CYAN + 'get data: {}ms elapsed.'.format(_executionMs) + Style.RESET_ALL)
                return data
            time.sleep(1 / 1000)

        return None

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def disable(self):
        '''
        Sets the stop event, cancelling the loop thread.
        '''
        self._log.info('disabling…')
        if self._stop_event:
            self._stop_event.set()
            self._log.info('cancelled thread.')
        self._log.info('stopping pi…')
        if self._pi:
            self._pi.stop()
            self._log.info('pi stopped.')
        Component.disable(self)
        self._log.info('disabled.')

#EOF
