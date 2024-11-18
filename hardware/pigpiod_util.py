#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   altheim
# created:  2020-11-18
# modified: 2024-11-18
#

import psutil
import subprocess
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class PigpiodUtility:
    '''
    A simple class to determine if pigpiod is running, and if not, start it.
    '''
    @staticmethod
    def is_pigpiod_running():
        '''
        Check if the pigpiod process is running.

        :return: True if pigpiod is running, False otherwise.
        '''
        for process in psutil.process_iter(['name']):
            if process.info['name'] == 'pigpiod':
                return True
        return False

    @staticmethod
    def ensure_pigpiod_is_running():
        '''
        Ensure the pigpiod service is running, starting it if necessary.
        '''
        _log = Logger('pig-util', Level.INFO)
        if PigpiodUtility.is_pigpiod_running():
            _log.info("pigpiod is already running.")
        else:
            _log.info("pigpiod is not running. Attempting to start it…")
            PigpiodUtility.start_pigpiod(skip_check=True)


    @staticmethod
    def start_pigpiod(skip_check=False):
        '''
        Start the pigpiod service using systemctl.

        :param skip_check: If True, skip the check for whether pigpiod is already running.
        '''
        _log = Logger('pig-util', Level.INFO)
        if not skip_check and PigpiodUtility.is_pigpiod_running():
            _log.info("pigpiod is already running.")
            return
        try:
            subprocess.run(['sudo', 'systemctl', 'start', 'pigpiod'], check=True)
            _log.info(Fore.GREEN + "pigpiod service started successfully.")
        except subprocess.CalledProcessError as e:
            _log.info("failed to start pigpiod service: {}".format(e))
        except FileNotFoundError:
            _log.info("The 'systemctl' command is not found. Ensure it is installed and accessible.")

#EOF
