#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-11-23
# modified: 2024-11-23
#

import sys, time
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.config_loader import ConfigLoader
from hardware.tinyfx_i2c_controller import TinyFxController

if __name__ == "__main__":

    if len(sys.argv) != 2:
        print(Fore.RED + "\n  ERROR: expected 1 command line argument: 'on' | 'off' | 'ch1' | 'ch2' | 'ch3' | 'port' | 'stbd' | 'mast' | 'play <key>'" + Style.RESET_ALL)
        sys.exit(1)
    value = sys.argv[1]

    _config = ConfigLoader(Level.INFO).configure()
    tinyfx_controller = TinyFxController(_config)
#   tinyfx_controller.play('beep')
#   time.sleep(2)
    print(Fore.GREEN + "value: '{}'".format(value) + Style.RESET_ALL)
    tinyfx_controller.send_data(value)

#EOF
