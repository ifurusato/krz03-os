#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-11-16
# modified: 2024-11-19
#

import sys, traceback
import time
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.orientation import Orientation
from core.config_loader import ConfigLoader
from core.message_bus import MessageBus
from core.message_factory import MessageFactory
from hardware.distance_sensors import DistanceSensors

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
def main():

    _level = Level.INFO
    _distance_sensors = None
    _log = Logger("dist-test", _level)

    try:

        _config = ConfigLoader(Level.INFO).configure()

        _log.info('creating message bus…')
        _message_bus = MessageBus(_config, _level)
        _log.info('creating message factory…')
        _message_factory = MessageFactory(_message_bus, _level)

        _log.info('creating distance sensors…')
        _distance_sensors = DistanceSensors(_config, _message_bus, _message_factory, level=_level)
#       _distance_sensors.enable()

        _log.info("starting message bus...")
        _message_bus.enable()

        _log.info("Measuring distances...")
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        _log.info("\nCtrl-C caught, exiting…")
    except Exception as e:
        _log.error('error in motor test: {}'.format(e))
        traceback.print_exc(file=sys.stdout)
    finally:
        if _distance_sensors:
            _distance_sensors.close()

if __name__ == "__main__":
    main()

