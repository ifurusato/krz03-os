#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2021 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2020-03-16
# modified: 2021-08-11
#
# Tests the ADS1015 ADC as a battery check device.
#

import sys, traceback
from datetime import datetime as dt
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.config_loader import ConfigLoader
from core.message_bus import MessageBus
from core.message_factory import MessageFactory
from hardware.i2c_scanner import I2CScanner, DeviceNotFound
from hardware.battery import BatteryCheck

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
def main():

    _log = Logger('test', Level.INFO)
    _message_bus = None

    try:

        # read YAML configuration
        _level = Level.INFO
        _loader = ConfigLoader(_level)
        filename = 'config.yaml'
        _config = _loader.configure(filename)

        _i2c_scanner = I2CScanner(_config, level=_level)
        if not _i2c_scanner.has_hex_address(['0x48']):
            raise DeviceNotFound('no ADS1015 available.')

        _log.info('creating message bus...')
        _message_bus = MessageBus(_config, _level)
        _log.info('creating message factory...')
        _message_factory = MessageFactory(_message_bus, _level)

        _start_time = dt.now()

        _log.info('using ADS1015 ADC...')
        _battery = BatteryCheck(_config, _message_bus, _message_factory, Level.INFO)

        _log.info('starting test...')

        # now in main application loop until quit or Ctrl-C...
        _log.info(Fore.YELLOW + 'enabling message bus...')
        _message_bus.enable()
#       _battery.enable()

        if _message_bus and _message_bus.enabled:
            _log.info(Fore.YELLOW + 'disabling and closing message bus...')
            _message_bus.disable()
            _message_bus.close()

        _elapsed_ms = round(( dt.now() - _start_time ).total_seconds() * 1000.0)
        _log.info(Fore.YELLOW + 'complete: elapsed: {:d}ms'.format(_elapsed_ms))

    except KeyboardInterrupt:
        print(Style.BRIGHT + 'caught Ctrl-C; exiting...')
    except DeviceNotFound as e:
        _log.error('no potentiometer found, exiting.')
    except Exception as e:
        print(Fore.RED + '{} encountered, exiting: {}'.format(type(e), e) + Style.RESET_ALL)
        traceback.print_exc(file=sys.stdout)
    finally:
        _log.info('exit.')
        if _message_bus:
            _log.info('finally calling close...')
            _message_bus.close()

if __name__== "__main__":
    main()

#EOF
