#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-04-18
# modified: 2025-05-10
#

import argparse
import sys

from core.logger import Logger, Level
from core.config_loader import ConfigLoader
from hardware.eyeballs import Eyeballs
from hardware.motor_controller import MotorController
from behave.macro_processor import MacroProcessor

def main():
    parser = argparse.ArgumentParser(description="execute a macro using the MacroProcessor")
    parser.add_argument('-m', '--macro', required=True, help="path to the macro file (*.txt)")
    args = parser.parse_args()

    log = Logger('macro', Level.INFO)
    log.info('macro loader begin…')

    try:
        config = ConfigLoader(Level.INFO).configure()
        motor_controller = MotorController(config, Level.INFO)
        motor_controller.enable()
        eyeballs = Eyeballs(Level.INFO)

        macro_processor = MacroProcessor(config, motor_controller, eyeballs, Level.INFO)

        macro_processor.load_macro(args.macro)
        macro_processor.execute()

    except Exception as e:
        log.error("failed to load macro: {}".format(e))
        sys.exit(1)

if __name__ == '__main__':
    main()

#EOF
