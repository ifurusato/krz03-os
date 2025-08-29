#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-06-02
# modified: 2025-06-05
#
# This TLC59711 driver has been adapted from (i.e., is not the same as) the
# original source code found at:
#
#    https://github.com/Liriel/tlc59711/tree/master
#
# The BSD license for the original may be found as tlc69711-license.txt
#

import spidev
import time
import RPi.GPIO as GPIO

class TLC59711:
    def __init__(self, clock_pin, data_pin, numDrivers=1, globalBrightness=0x7F, spi_bus=0, spi_device=0, spi_speed=50000):
        # Pins
        self.__clk = clock_pin
        self.__dat = data_pin
        
        # Initialize SPI
        self.__spi = spidev.SpiDev()
        self.__spi.open(spi_bus, spi_device)  # SPI bus and device (e.g., bus 0, device 0)
        self.__spi.max_speed_hz = spi_speed  # Set the SPI speed
        
        # Flags for brightness
        self.__BCr = self.__BCg = self.__BCb = globalBrightness
        
        # Number of drivers
        self.__numDrivers = numDrivers
        
        # Initialize PWM buffer
        self.__pwmBuffer = [0x000 for i in range(0, 12)]

        # Set GPIO mode
        GPIO.setup(self.__clk, GPIO.OUT)
        GPIO.setup(self.__dat, GPIO.OUT)

    def _WriteMSB(self, d):
        b = 0x80
        # 12 bits per channel, send MSB first
        while b:
            GPIO.output(self.__clk, False)
            if (b & d):
                GPIO.output(self.__dat, True)
            else:
                GPIO.output(self.__dat, False)
            GPIO.output(self.__clk, True)
            b = b >> 1

    def _Write(self):
        # Debugging: Log before writing
        print("Starting _Write()")
        
        cmd = 0x25
        cmd <<= 5
        cmd |= 0x16
        
        cmd <<= 7
        cmd |= self.__BCr
        cmd <<= 7
        cmd |= self.__BCb
        cmd <<= 7
        cmd |= self.__BCg
        
        # Prepare data to send over SPI (combining all the command and PWM bytes)
        spi_data = []
        
        # First, send the 4-byte command
        spi_data.append((cmd >> 24) & 0xFF)
        spi_data.append((cmd >> 16) & 0xFF)
        spi_data.append((cmd >> 8) & 0xFF)
        spi_data.append(cmd & 0xFF)
        
        print(f"Command to send: {spi_data}")

        for n in range(self.__numDrivers):
            # Send the PWM values for each driver
            for c in range(11, -1, -1):
                pwm_value = self.__pwmBuffer[n * 12 + c]
                spi_data.append((pwm_value >> 8) & 0xFF)  # High byte
                spi_data.append(pwm_value & 0xFF)  # Low byte
                print(f"PWM Channel {n * 12 + c}: {pwm_value:04x}")

        print("Sending SPI data:", spi_data)
        
        # Send the data over SPI
        response = self.__spi.xfer(spi_data)
        time.sleep(0.01)  # Add a small delay for processing
        
        # Log the response
        print("SPI response:", response)

    def x_Write(self):
        cmd = 0x25
        cmd <<= 5
        cmd |= 0x16
        
        cmd <<= 7
        cmd |= self.__BCr
        cmd <<= 7
        cmd |= self.__BCb
        cmd <<= 7
        cmd |= self.__BCg
        
        # Prepare data to send over SPI (combining all the command and PWM bytes)
        spi_data = []

        # First, send the 4-byte command
        spi_data.append((cmd >> 24) & 0xFF)
        spi_data.append((cmd >> 16) & 0xFF)
        spi_data.append((cmd >> 8) & 0xFF)
        spi_data.append(cmd & 0xFF)
    
        # log the command and PWM data being sent
        print("Command to be sent:", spi_data)

        for n in range(self.__numDrivers):
            # Send the PWM values for each driver
            for c in range(11, -1, -1):
                pwm_value = self.__pwmBuffer[n * 12 + c]
                spi_data.append((pwm_value >> 8) & 0xFF)  # High byte
                spi_data.append(pwm_value & 0xFF)  # Low byte

        # log the entire SPI data being sent
        print("Sending SPI data:", spi_data)

        # send the data over SPI
        response = self.__spi.xfer(spi_data)
        print("SPI response:", response)

    def _SetPWM(self, chan, pwm):
        if chan >= 12 * self.__numDrivers:
            return
        self.__pwmBuffer[chan] = pwm

    def SetPWM(self, chan, pwm):
        self._SetPWM(chan, pwm)
        self._Write()

    def SetLED(self, lednum, r, g, b):
        self._SetPWM(lednum * 3, r)
        self._SetPWM(lednum * 3 + 1, g)
        self._SetPWM(lednum * 3 + 2, b)
        self._Write()

# EOF
