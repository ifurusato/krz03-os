#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-05-06
# modified: 2025-05-10
#

import sys, time
import traceback

import core.globals as globals
globals.init()

from core.logger import Logger, Level
from core.config_loader import ConfigLoader
from core.queue_publisher import QueuePublisher
from core.message_bus import MessageBus
from core.message_factory import MessageFactory
from hardware.distance_sensors import DistanceSensors
from hardware.differential_drive import DifferentialDrive
from behave.avoid import Avoid
from behave.roam import Roam

# main ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

class MockMessageBus():
    def __init__(self):
        pass


def main():

    _log = Logger('main', Level.INFO)
    roam = None
    try:
        config = ConfigLoader(Level.INFO).configure()

        message_bus = MessageBus(config, Level.INFO)
        message_factory = MessageFactory(message_bus, Level.INFO)
        queue_publisher = QueuePublisher(config, message_bus, message_factory, Level.INFO)

        sensors = DistanceSensors(config, Level.INFO)
        differential = DifferentialDrive(config, Level.INFO)

        avoid = Avoid(config, message_bus, message_factory)
        roam = Roam(config, message_bus, message_factory, differential, sensors)

        # print registry of components
        _component_registry = globals.get('component-registry')
        _component_registry.print_registry()

        avoid.enable()
        roam.enable()

        message_bus.enable() # blocks

        # wait until Roam finishes
        roam.wait_until_finished()

    except KeyboardInterrupt:
        _log.info('caught Ctrl-C; exiting…')
    except Exception as e:
        _log.error('{} raised during roam: {}'.format(type(e), e))
        traceback.print_exc(file=sys.stdout)
    finally:
        if roam:
            roam.disable()
        if message_bus:
            message_bus.disable()
        _log.info('roam complete.')

if __name__ == "__main__":
    main()

#EOF
