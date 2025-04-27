#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-10-23
# modified: 2024-10-31
#

import pytest
import sys, time
from enum import Enum
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.config_loader import ConfigLoader
from hardware.button import Button
from hardware.eyeballs import Eyeballs

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
@pytest.mark.unit
def eyeballs_test():

    _wait_for_button = True
    _eyeballs = None

    try:

        _log = Logger("eyeballs-test", Level.INFO)
        _eyeballs = Eyeballs(Level.INFO)

        if _wait_for_button:
            _config = ConfigLoader(Level.INFO).configure()
            _btn = Button(_config, level=Level.INFO)
            _count = 0
            while not _btn.pushed():
                if _count % 10 == 0:
                    _log.info(Style.DIM + 'waiting for button…')
                _count += 1
                time.sleep(0.01)
            time.sleep(1)

        _log.info(Fore.GREEN + 'starting… (type Ctrl-C to exit)')

        _eyeballs.normal()
        time.sleep(2)
        _eyeballs.clear()

        _eyeballs.happy()
        time.sleep(2)
        _eyeballs.clear()

        _eyeballs.wink()
        time.sleep(2)
        _eyeballs.clear()

        _eyeballs.blush()
        time.sleep(2)
        _eyeballs.clear()

        _eyeballs.look_port()
        time.sleep(2)
        _eyeballs.clear()

        _eyeballs.look_stbd()
        time.sleep(2)
        _eyeballs.clear()

        _eyeballs.confused()
        time.sleep(2)
        _eyeballs.clear()

#       _eyeballs.drugged()
#       time.sleep(2)
#       _eyeballs.clear()

        _eyeballs.sad()
        time.sleep(2)
        _eyeballs.clear()

        _eyeballs.blank()
        time.sleep(2)
        _eyeballs.clear()

        _eyeballs.wow(7)
        time.sleep(2)

        _eyeballs.dead(include_fade=True)
        time.sleep(2)
        _eyeballs.clear()

        _log.info('eyeballs complete.')

    except KeyboardInterrupt:
        _log.info('eyeballs    :' + Fore.YELLOW + ' INFO  : Ctrl-C caught: exiting…' + Style.RESET_ALL)
    finally:
        if _eyeballs:
            _eyeballs.close()

# main ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

def main():
    eyeballs_test()

if __name__== "__main__":
    main()

#EOF
