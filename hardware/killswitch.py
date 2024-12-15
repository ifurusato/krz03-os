#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#

import time

#_USE_GPIO = False
#import RPi.GPIO as GPIO
#import pigpio
from pigpio import pi, INPUT, PUD_UP, EITHER_EDGE

from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.component import Component

class Killswitch(Component):
    '''
    A button configured to either use a GPIO pin as a kill switch. When
    activated it executes the callback, generally KRZOS's shutdown() method.
    '''
    def __init__(self, config=None, callback=None, level=Level.INFO):
        """
        Initializes the Killswitch.

        :param config:     the application configuration providing the pin.
        :param callback:   the function to call when the button is pressed.
        :param level:      the log level
        """
        _cfg = config['krzos'].get('hardware').get('killswitch')
        self._pin = _cfg.get('pin')
        self._log = Logger('killswitch', level)
        Component.__init__(self, self._log, suppressed=False, enabled=True)
        if callback is None:
            raise ValueError('no callback argument provided.')
        self._callback = callback
#       if _USE_GPIO:
#           GPIO.setmode(GPIO.BCM) # use BCM numbering
#           GPIO.setup(self._pin, GPIO.IN, pull_up_down=GPIO.PUD_UP) # enable pull-up resistor
#           # set up an interrupt to detect when the pin goes low
#           GPIO.add_event_detect(self._pin, GPIO.FALLING, callback=self._handle_event, bouncetime=200)
#           self._log.info('ready using GPIO.')
#       else:
        self._pi = pi()
        if not self._pi.connected:
            raise RuntimeError("Unable to connect to pigpio daemon")
        # set GPIO pin as input with pull-up resistor
        self._pi.set_mode(self._pin, INPUT)
        self._pi.set_pull_up_down(self._pin, PUD_UP)
        # set up a callback for the falling edge (button press)
        self._pi.callback(self._pin, EITHER_EDGE, self._handle_event)
        self._log.info('ready using pigpio.')

#   def _handle_event(self, channel):
    def _handle_event(self, gpio, level, tick):
        '''
        This method is called when the button is pressed (falling edge detected).
        It will trigger the registered callback function.

        Args:
        - gpio (int): The GPIO pin number that triggered the event.
        - level (int): The level of the GPIO pin (either HIGH or LOW).
        - tick (int): A timestamp of when the event occurred.
        '''
        self._log.info(Fore.WHITE + Style.BRIGHT + 'kill switch pressed…')
#       if _USE_GPIO:
#           if GPIO.input(self._pin) == GPIO.LOW:
#               self._callback()
#       else:
        self._pi.stop()
        self._callback()

    def close(self):
        '''
        Cleans up connections and resources.
        '''
        self._log.info(Fore.WHITE + Style.BRIGHT + 'closing…')
#       if _USE_GPIO:
#           GPIO.remove_event_detect(self._pin)
#           time.sleep(0.1)
#           GPIO.cleanup(self._pin)
#       else:
            # closes connection to the pigpio daemon and cleans up resources
#       self._pi.stop()
        self._log.info(Fore.WHITE + Style.BRIGHT + 'closed.')

#EOF
