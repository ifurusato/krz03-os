
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2020-03-16
# modified: 2020-06-12
#
#

import sys, itertools, traceback
import asyncio
from enum import Enum
from colorama import init, Fore, Style
init()

from core.logger import Level, Logger
from core.event import Event
from core.publisher import Publisher

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class BatteryCheck(Publisher):

    _LISTENER_LOOP_NAME = '__battery_listener_loop'

    '''
    This uses two channels of an ADS1015 to measure the raw voltage of the 
    battery and that of the 3.3 volt regulator. If either fall below a 
    specified threshold a low battery message is sent to the message bus.

    If unable to establish communication with the ADS1015 this will raise a
    RuntimeError.

    Note that this uses a different version of the Color enum whose values are
    suited to the ThunderBorg's RGB LED inputs.

    Configuration:
    battery_channel:       the ADS1015 channel 0 used to measure the raw battery voltage
    five_volt_channel:     the ADS1015 channel 1 used to measure the 3V3 regulator battery voltage

    Parameters:
    :param message_bus:    the message bus
    :param level:          the logging level

    How many times should we sample before accepting a first value?
    '''
    def __init__(self, config, message_bus, message_factory, level):
        Publisher.__init__(self, 'battery', config, message_bus, message_factory, level=level)
#       self._log = Logger("battery", level)
        if config is None:
            raise ValueError('no configuration provided.')
        _cfg = config['krzos'].get('hardware').get('battery')
        self._counter                    = itertools.count()
        # configuration
        self._enable_battery_messaging   = _cfg.get('enable_battery_messaging')
        self._enable_regulator_messaging = _cfg.get('enable_regulator_messaging')
        _CHANNELS = ['in0/ref', 'in1/ref', 'in2/ref']
        self._battery_channel            = _CHANNELS[_cfg.get('battery_channel')]
        self._regulator_channel          = _CHANNELS[_cfg.get('regulator_channel')]
        self._raw_battery_threshold      = _cfg.get('raw_battery_threshold')
        self._regulator_threshold        = _cfg.get('low_regulator_threshold')
        self._loop_delay_sec             = _cfg.get('loop_delay_sec')
        self._loop_delay_sec_div_10      = self._loop_delay_sec / 10
        self._log.info('battery check loop delay: {:>5.2f} sec'.format(self._loop_delay_sec))
        self._log.info('setting regulator threshold to {:>5.2f}v'.format(self._regulator_threshold))
        self._log.info("channel A from '{}'; raw battery threshold to {:>5.2f}v from '{}'".format(\
                self._regulator_channel, self._raw_battery_threshold, self._battery_channel))

        self._ads1015 = None
        self._message_bus = message_bus
        self._message_factory = message_factory
        self._battery_voltage       = 0.0
        self._regulator_voltage     = 0.0
        try:
            from ads1015 import ADS1015
            self._ads1015 = ADS1015()
            self._ads1015.set_mode('single')
            self._ads1015.set_programmable_gain(2.048)
            self._ads1015.set_sample_rate(1600)
            self._reference = self._ads1015.get_reference_voltage()
            self._log.info('reference voltage: {:6.3f}v'.format(self._reference))
        except ImportError:
            self._log.error("This script requires the ads1015 module\nInstall with: pip3 install --user ads1015")
#           from lib.mock_ads1015 import ADS1015
#           raise ModuleNotFoundError('This script requires the ads1015 module\nInstall with: pip3 install --user ads1015')
        except Exception as e:
            raise RuntimeError('error configuring AD converter: {}'.format(traceback.format_exc()))
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def name(self):
        return 'battery'

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def set_enable_messaging(self, enable):
        '''
        If true we enable all battery and regulator messages to be sent.
        This overrides the configured values.
        '''
        if enable:
            self._log.info('enable battery check messaging.')
        else:
            self._log.info('disable battery check messaging.')
        self._enable_battery_messaging   = enable
        self._enable_regulator_messaging = enable

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def set_raw_battery_threshold(self, threshold):
        self._log.info('set raw battery threshold to {:>5.2f}v'.format(threshold))
        self._raw_battery_threshold = threshold

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def set_five_volt_threshold(self, threshold):
        self._log.info('set five volt threshold to {:>5.2f}v'.format(threshold))
        self._regulator_threshold = threshold

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_raw_battery_voltage(self):
        return self._battery_voltage

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_regulator_voltage(self):
        return self._regulator_voltage

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def enable(self):
        Publisher.enable(self)
        if self.enabled:
            if not self._ads1015:
                self._log.warning('cannot enable battery check: no ADC available.')
            elif self._message_bus.get_task_by_name(BatteryCheck._LISTENER_LOOP_NAME):
                self._log.warning('already enabled.')
            else:
                self._log.info('creating task for battery listener loop…')
                self._message_bus.loop.create_task(self._battery_listener_loop(lambda: self.enabled), name=BatteryCheck._LISTENER_LOOP_NAME)
                self._log.info('enabled.')
            self._log.info('enabled.')
        else:
            self._log.warning('cannot enable: already enabled.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _check_battery_voltage(self):
       '''
       Checks the battery voltage (default on channel 0), returning a message
       if the battery is low, otherwise None.
       '''
#      self._log.debug('checking battery voltage {:5.2f} against threshold {:5.2f}…'.format(self._battery_voltage, self._raw_battery_threshold))
       if self._battery_voltage < self._raw_battery_threshold:
           self._log.warning('battery low: {:>5.2f}v'.format(self._battery_voltage))
           if self._enable_battery_messaging:
               _message = self._message_factory.create_message(Event.BATTERY_LOW, 'battery low: {:5.2f}V'.format(self._battery_voltage))
               return _message
       else:
           return None

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _check_regulator_channel(self):
       '''
       Checks the voltage on channel 1, returning a message if below the
       configured threshold, otherwise None.
       '''
#      self._log.debug('checking channel a voltage {:5.2f} against threshold {:5.2f}…'.format(self._regulator_voltage, self._regulator_threshold))
       if self._regulator_voltage < self._regulator_threshold:
           self._log.warning('regulator low:  {:>5.2f}v'.format(self._regulator_voltage))
           if self._enable_regulator_messaging:
               _message = self._message_factory.create_message(Event.BATTERY_LOW, 'regulator low: {:5.2f}V'.format(self._regulator_voltage))
               return _message
       else:
           return None

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    async def _battery_listener_loop(self, f_is_enabled):
        '''
        The function that checks the raw battery and 3V3 regulator voltages in
        a very slow loop. Note that this doesn't immediately send BATTERY_LOW
        messages until after the loop has run a few times, as it seems the
        first check after starting tends to measure a bit low.
        '''
        self._log.info('starting battery check loop.')
        try:
            while f_is_enabled():
                _count = next(self._counter)
                self._battery_voltage = self._ads1015.get_compensated_voltage(channel=self._battery_channel, reference_voltage=self._reference)
                self._regulator_voltage = self._ads1015.get_compensated_voltage(channel=self._regulator_channel, reference_voltage=self._reference)

                # publish message if exceeds threshold…
                _message = None
                if self._enable_battery_messaging:
                    _message = self._check_battery_voltage()
                if not _message and self._enable_regulator_messaging:
                    _message = self._check_regulator_channel()
                if not _message:
                    self._log.info(Style.DIM + self.get_battery_info())
                if _message:
#                   _message = self._message_factory.create_message(_event, True)
#                   self._log.debug('battery-publishing message:' + Fore.WHITE + ' {}; event: {}'.format(_message.name, _message.event.label))
                    await Publisher.publish(self, _message)
#                   self._log.debug('battery-published message:' + Fore.WHITE + ' {}.'.format(_message.name))
                else:
                    # nothing happening
#                   self._log.debug('[{:03d}] waiting for battery event…'.format(_count))
                    pass
                await asyncio.sleep(self._loop_delay_sec)
#               self._log.debug('[{:03d}] battery check ended.'.format(_count))

            self._log.info('battery check loop complete.')
        finally:
            pass

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_battery_info(self):
        return 'battery: {:>5.2f}v; regulator: {:>5.2f}v'.format(self._battery_voltage, self._regulator_voltage)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def disable(self):
        if self.enabled:
            Publisher.disable(self)
            self._log.info('disabled publisher.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def clamp(n, minn, maxn):
        return max(min(maxn, n), minn)

#EOF
