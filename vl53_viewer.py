#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-04-02
# modified: 2024-04-06
#
#
# Description:
#
# Populates an 8x8 matrix from data polled from a VL53L5CX 8x8 ToF sensor,
# performs some further processing, including a display to the console.
#
# For each column, returns the minimum, maximum, and mean values, as well
# as the index of the largest mean value.
#
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
import time, datetime
import re
import statistics
import numpy as np


from matrix11x7 import Matrix11x7
import pigpio
import vl53l5cx_ctypes as vl53l5cx
from vl53l5cx_ctypes import STATUS_RANGE_VALID, STATUS_RANGE_VALID_LARGE_PULSE, RANGING_MODE_CONTINUOUS, RANGING_MODE_AUTONOMOUS
from colorama import init, Fore, Style
init()

pi = pigpio.pi()
if not pi.connected:
    exit()

# constants ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

LIMIT      = [ 200, 300, 400, 500, 1000, 2000, 3000, 4000 ] # mm limits of ea display category
DOTS       = [ " ⚪ ", " 🔴 ", " 🟠 ", " 🟡 ", " 🟢 ", " 🔵 ", " 🟣 ", " 🟤 ", " ⚫ " ]
VERBOSE    = True
ROWS       = 60           # the number of rows to scroll
NNNN       = '\n' * ROWS  # as tall as your screen
MIN_DISTANCE_MM = 1000    # permit targeting if greater than this distance
FG_COLOR   = Fore.BLUE  

# variables ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

vl53       = None
read       = 0
okCount    = 0
failCount  = 0
count      = 0
indexMax   = 0
startTime  = 0

COLS, ROWS = 8, 8
matrix     = np.array([[0 for x in range(COLS)] for y in range(ROWS)])
mean       = [0 for x in range(COLS)]
ikon       = [[0 for x in range(COLS)] for y in range(ROWS)]
maxDot     = np.array([DOTS[8] for x in range(COLS)], dtype=object)


# functions ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

def dot(value):
    '''
    Return a colored dot indicating the range.
    '''
    if value   <= LIMIT[0]:
        return DOTS[0]
    elif value <= LIMIT[1]:
        return DOTS[1]
    elif value <= LIMIT[2]:
        return DOTS[2]
    elif value <= LIMIT[3]:
        return DOTS[3]
    elif value <= LIMIT[4]:
        return DOTS[4]
    elif value <= LIMIT[5]:
        return DOTS[5]
    elif value <= LIMIT[6]:
        return DOTS[6]
    elif value <= LIMIT[7]:
        return DOTS[7]
    else: # (out of range)
        return DOTS[8]
        
dLimit = 2500.0


def showMatrix11x7():
    '''
    Displays the matrix and related data to the 11x7 LED matrix display.
    '''
    global mtrx, indexMax, ikon, read, count, okCount, failCount
    
    mtrx.clear()
    # print minimum distance ...............................
    minV = matrix.min(axis=0, keepdims=True)
    print("\n           min:     {}  {}  {}  {}  {}  {}  {}  {}".format(
            minV[0][0], minV[0][1], minV[0][2], minV[0][3], minV[0][4], minV[0][5], minV[0][6], minV[0][7] ))
#           dot(minV[0][0]), dot(minV[0][1]), dot(minV[0][2]), dot(minV[0][3]), dot(minV[0][4]), dot(minV[0][5]), dot(minV[0][6]), dot(minV[0][7]) ))
#   mean[0], mean[1], mean[2], mean[3], mean[4], mean[5], mean[6], mean[7]) 
    for col in range(8):
        val = min(minV[0][col], dLimit)
        bright = min(( val / dLimit), 0.9 )
        mtrx.setMinimumColumn(col, bright)
        
    # print maximum distance ...............................
    maxV = matrix.max(axis=0, keepdims=True)
    print("           max:     {}  {}  {}  {}  {}  {}  {}  {}".format(
            maxV[0][0], maxV[0][1], maxV[0][2], maxV[0][3], maxV[0][4], maxV[0][5], maxV[0][6], maxV[0][7] ))
            
    # print maximum mean distance ..........................
    maxDot.fill(DOTS[8])
    maxValue = mean[indexMax]
    maxDot[indexMax] = DOTS[0]
    print("        target:     {}  {}  {}  {}  {}  {}  {}  {}".format(
            maxDot[0], maxDot[1], maxDot[2], maxDot[3], maxDot[4], maxDot[5], maxDot[6], maxDot[7]))
    mtrx.setTargetColumn(indexMax)
    mtrx.update()
    

def showMatrix():
    '''
    Displays the matrix and related data to the console.
    '''
    global indexMax, ikon, read, count, okCount, failCount
    # print header .........................................
    print(NNNN)
    print(FG_COLOR + "  {:07d}   ".format(count)
            + Fore.RED + "PORT"
            + FG_COLOR + "     0     1     2     3     4     5     6     7     "
            + Fore.GREEN + "STBD \n" + FG_COLOR )
    # print matrix .........................................
    for r in range(8):
        print("                    {}  {}  {}  {}  {}  {}  {}  {} \n".format(
                ikon[r][0], ikon[r][1], ikon[r][2], ikon[r][3], ikon[r][4], ikon[r][5], ikon[r][6], ikon[r][7]))
    # print minimum distance ...............................
    minV = matrix.min(axis=0, keepdims=True)
    print("\n           min:     {}  {}  {}  {}  {}  {}  {}  {}".format(
            dot(minV[0][0]), dot(minV[0][1]), dot(minV[0][2]), dot(minV[0][3]), dot(minV[0][4]), dot(minV[0][5]), dot(minV[0][6]), dot(minV[0][7]) ))
    # print maximum distance ...............................
    maxV = matrix.max(axis=0, keepdims=True)
    print("           max:     {}  {}  {}  {}  {}  {}  {}  {}".format(
            dot(maxV[0][0]), dot(maxV[0][1]), dot(maxV[0][2]), dot(maxV[0][3]), dot(maxV[0][4]), dot(maxV[0][5]), dot(maxV[0][6]), dot(maxV[0][7]) ))
    # print mean distance by column ........................
    print("          mean:     {}  {}  {}  {}  {}  {}  {}  {}".format(
            dot(mean[0]), dot(mean[1]), dot(mean[2]), dot(mean[3]), dot(mean[4]), dot(mean[5]), dot(mean[6]), dot(mean[7])))
    # print maximum mean distance ..........................
    maxDot.fill(DOTS[8])
    maxValue = mean[indexMax]
    if maxValue > MIN_DISTANCE_MM:
        maxDot[indexMax] = DOTS[0]
    else:
        maxDot[indexMax] = DOTS[1]
    print("        target:     {}  {}  {}  {}  {}  {}  {}  {}".format(
            maxDot[0], maxDot[1], maxDot[2], maxDot[3], maxDot[4], maxDot[5], maxDot[6], maxDot[7]))
    # print numeric distances in mm ........................
    print("  mean dist mm:     {:<4}  {:<4}  {:<4}  {:<4}  {:<4}  {:<4}  {:<4}  {:<4}".format(
            mean[0], mean[1], mean[2], mean[3], mean[4], mean[5], mean[6], mean[7]) + Style.RESET_ALL)
            
            


            
def processData():
    '''
    Processes the data from the matrix.
    '''
    global pi, matrix, mean, indexMax, startTime, count, read, okCount, failCount
    try:
        m = np.array([[0 for x in range(COLS)] for y in range(ROWS)])
        for row in range(8): 
            # populate ikon and mean arrays
            for col in range(8):
                n = matrix[row,col]
                ikon[row][col] = dot(n)
                # calculate column means
                mean[col] = round(statistics.fmean([
                        matrix[0][col], matrix[1][col], matrix[2][col], matrix[3][col],
                        matrix[4][col], matrix[5][col], matrix[6][col], matrix[7][col] ]))
            row += 1    
        indexMax = mean.index(max(mean))
#       showMatrix()
        print('\n' * 52)
        showMatrix11x7()
    except:
        print(Fore.RED + Style.BRIGHT + 'error in process data: {}'.format(traceback.format_exc()) + Style.RESET_ALL)
    finally:
        if read > 0:
            okRate = int(okCount / read * 100)
            failRate = int(failCount / read * 100)
        else:
            okRate = 0
            failRate = 0
        executionMs = int( (datetime.datetime.now() - startTime).total_seconds() * 1000 )
        print(FG_COLOR + '\n    ok: {}%; fail: {}%; loop: {}ms elapsed.\n'.format(okRate, failRate, executionMs) + Style.RESET_ALL)
        
        
def hasArg(arg):
    '''
    Returns true if the command line arguments contains the argument.
    We ignore argument 0 as that is the name of the script.
    '''
    for i in range(1, len(sys.argv)):
        if sys.argv[i] == arg:
            return True
    return False

# ..............................................................................

def getVL53():
    '''
    Instantiate the sensor.
    '''
    stime = datetime.datetime.now()
#   if len(sys.argv) > 1 and sys.argv[1] == 'skip':
    if hasArg('skip'):
        print("initialising VL53L5CX…")
        vl53 = vl53l5cx.VL53L5CX(skip_init=True)
    else:
        print("uploading firmware to VL53L5CX, please wait…")
        vl53 = vl53l5cx.VL53L5CX()
    vl53.set_resolution(8 * 8)
    vl53.set_ranging_frequency_hz(15)
    vl53.set_integration_time_ms(20)
    vl53.set_ranging_mode(RANGING_MODE_CONTINUOUS)
    executionMs = int((datetime.datetime.now() - stime).total_seconds() * 1000)
    print(Fore.BLUE + 'get VL53: {}ms elapsed.'.format(executionMs) + Style.RESET_ALL)
    return vl53
    
    
''' 
    Wait until the data from the vl53 is ready, returning the data, otherwise None upon a timeout.
'''
def getData(sensor):
    stime = datetime.datetime.now()
    for i in range(1,20):
        if sensor.data_ready():
            data = sensor.get_data() # 2d array of distance
            executionMs = int((datetime.datetime.now() - stime).total_seconds() * 1000)
            print(Fore.CYAN + 'get data: {}ms elapsed.'.format(executionMs) + Style.RESET_ALL)
            return data
        time.sleep(1 / 1000)
    return None
    

    
''' 
   A wrapper around a Matrix 11x7 display.
'''
class Matrix(object):
    def __init__(self):
        self._matrix = Matrix11x7()
        self._brightness = 0.5
        self._matrix.set_brightness(self._brightness)
        
    def setMinimumColumn(self, col, brightness):
        self._matrix.set_pixel(col + 3, 0, brightness)
        self._matrix.set_pixel(col + 3, 1, brightness)
        self._matrix.set_pixel(col + 3, 2, brightness)
        
    def setTargetColumn(self, col):
        self._matrix.set_pixel(col + 3, 4, self._brightness)
        self._matrix.set_pixel(col + 3, 5, self._brightness)
        self._matrix.set_pixel(col + 3, 6, self._brightness)
        
    def update(self):
        self._matrix.show()
        
    def clear(self):
        self._matrix.clear()

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

'''
    Polls the VL53L5CX sensor for data, populates the matrix,
    then displays the results on the console.
'''
def main(argv):
    global pi, vl53, mtrx, startTime, count, matrix
    
    try:
    
        print(Fore.GREEN + Style.DIM + 'begin…' + Style.RESET_ALL)
    
        mtrx = Matrix()
    
        # set up VL53L5CX
        vl53 = getVL53()
        vl53.start_ranging()
    
        print(Fore.GREEN + 'starting loop…' + Style.RESET_ALL)
        while True:
            startTime = datetime.datetime.now()
            count += 1
            data = getData(vl53)
            if data != None:
                distance = np.array(data.distance_mm).reshape((8, 8))
                for x in range(0,8):
                    for y in range(0,8):
                        matrix[x,y] = distance[x][y]
                processData()
                
    except KeyboardInterrupt:
        print(Style.BRIGHT + 'caught Ctrl-C; exiting…' + Style.RESET_ALL)
    except Exception:
        print(Fore.RED + Style.BRIGHT + 'error in script: {}'.format(traceback.format_exc()) + Style.RESET_ALL)
    finally:
        print(Fore.YELLOW + 'stopping pi…' + Style.RESET_ALL)
        pi.stop()
        print(Fore.YELLOW + 'pi stopped.' + Style.RESET_ALL)
        
    print('complete.')
    
    
# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
if __name__== "__main__":
    main(sys.argv[1:])
    
#EOF
