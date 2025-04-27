#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-04-21
# modified: 2025-04-21
#

import time
from hardware.pir_publisher import PirPublisher
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.config_loader import ConfigLoader
from core.message_bus import MessageBus
from core.message_factory import MessageFactory

# begin ....................................................

_level = Level.INFO
_log = Logger('test', _level)

_message_bus = None # can't use in tests
_pir_publisher = None

try:

    _config = ConfigLoader(Level.INFO).configure()

    _log.info('creating message bus...')
    _message_bus = MessageBus(_config, _level)
    _log.info('creating message factory...')
    _message_factory = MessageFactory(_message_bus, _level)

    _pir_publisher = PirPublisher(_config, _message_bus, _message_factory)
#   _pir_publisher.enable()

    # now in main application loop until quit or Ctrl-C...
    _log.info(Fore.YELLOW + 'enabling message bus...')
    _message_bus.enable()

    _log.info("polling pir sensor…")

#   while True:
#       _log.info("tick.")
#       time.sleep(0.1)
#       await asyncio.sleep(0.1)  # Adjust delay as needed

except KeyboardInterrupt:
    _pir_publisher.disable()
except Exception as e:
    print(e)
finally:
    if _pir_publisher:
        _pir_publisher.disable()
    print('complete.')

