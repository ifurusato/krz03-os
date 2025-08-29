#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#

import time
import RPi.GPIO as GPIO
import math
import datetime as dt

from core.logger import Logger, Level
from hardware.tlc59711 import TLC59711

# Feedback measurement functions
def measure_duty_cycle(feedback_pin, duration=1.0):
    # Measure the duty cycle of the feedback signal on the given GPIO pin
    high_time = 0
    low_time = 0
    start_time = time.time()

    while time.time() - start_time < duration:  # Measure for the given duration
        if GPIO.input(feedback_pin) == GPIO.HIGH:
            high_time += 1
        else:
            low_time += 1

    total_time = high_time + low_time
    duty_cycle = (high_time / total_time) * 100 if total_time > 0 else 0
    return duty_cycle

def measure_pwm_frequency(feedback_pin, duration=1.0):
    # Measure the frequency of the PWM signal on the feedback pin
    last_state = GPIO.input(feedback_pin)
    transitions = 0
    start_time = time.time()

    while time.time() - start_time < duration:
        current_state = GPIO.input(feedback_pin)

        # Detect rising edge (change from LOW to HIGH)
        if last_state == GPIO.LOW and current_state == GPIO.HIGH:
            transitions += 1

        last_state = current_state

    period = (time.time() - start_time) / transitions if transitions > 0 else 0
    frequency = 1 / period if period > 0 else 0
    return frequency

# main ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

def main():
    _log = Logger('main', Level.INFO)
    
    CLK_PIN = 11  # GPIO pin for clock (SCLK) connected to DI
    DAT_PIN = 10  # GPIO pin for data (MOSI) connected to DI
    FEEDBACK_PIN = 13  # GPIO pin for hardware PWM input (GPIO 13, where the TLC59711's R0 output is connected)
    
    GPIO.setmode(GPIO.BCM)
    
    # initialize TLC59711 with 1 driver, global brightness max
    tlc = TLC59711(clock_pin=CLK_PIN, data_pin=DAT_PIN, spi_bus=0, spi_device=0)
    
    # Set up GPIO 13 as an input to measure PWM signal
    GPIO.setup(FEEDBACK_PIN, GPIO.IN)
    
    _log.info('starting test…')
    
    # Define acceptable range for duty cycle (0% - 100%)
    acceptable_duty_cycle_range = (0, 100)  # Allowable duty cycle between 0% and 100%
    
    try:
        while True:
            for speed in range(2048, 4095):
                # set channel 0 (R0) PWM for 12-bit = 0-4096 
                # channels are 0-11, so channel 0 corresponds to output R0
                _log.info("> speed: {}".format(speed))
                tlc.SetPWM(0, speed)  # Set PWM duty cycle for channel 0 (R0)
                
#               # Measure the duty cycle from feedback pin (GPIO 13)
#               duty_cycle = measure_duty_cycle(FEEDBACK_PIN, duration=2.0)
                
                # Measure the PWM frequency from feedback pin (GPIO 13)
                pwm_frequency = measure_pwm_frequency(FEEDBACK_PIN, duration=5.0)
                
#               # Log measured values with tolerance checks using math.isclose
#               expected_duty_cycle = (speed / 4095) * 100
#               if math.isclose(duty_cycle, expected_duty_cycle, abs_tol=5):
#                   _log.info("Measured duty cycle: {:.2f}% (Expected: {:.2f}%)".format(duty_cycle, expected_duty_cycle))
#               else:
#                   _log.info("Measured duty cycle: {:.2f}% (Expected: {:.2f}%)".format(duty_cycle, expected_duty_cycle))
                
                # Log PWM frequency with no upper limit
                _log.info("measured PWM frequency: {:.2f} Hz".format(pwm_frequency))
                
                time.sleep(1.0)
                
    except KeyboardInterrupt:
        _log.info("Ctrl-C caught: exiting test.")
    except Exception as e:
        _log.error("{} raised in test: {}".format(type(e), e))
    finally:
        _log.info("complete.")
        GPIO.cleanup()
        
if __name__ == "__main__":
    main()
    
# EOF
