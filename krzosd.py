#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2020-08-01
# modified: 2025-05-17
#
# KRZ03 Robot Operating System Daemon (krzosd). This also uses the krzosd.service.
#
# see: https://dpbl.wordpress.com/2017/02/12/a-tutorial-on-python-daemon/
#
# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

import importlib
import os
import sys, traceback
from threading import Thread
from pathlib import Path
import time
import itertools
import subprocess
import signal

try:
    import RPi.GPIO as GPIO
except Exception:
    sys.exit('This script requires the RPi.GPIO module.\nInstall with: sudo pip3 install RPi.GPIO')
try:
    import daemon
    from daemon import pidfile
except Exception:
    sys.exit("This script requires the python-daemon module.\nInstall with: pip3 install --user python-daemon")

from core.util import Util
from datetime import datetime as dt
from colorama import init, Fore, Style
init()

import core.globals as globals
globals.init()

from core.logger import Level, Logger
#from krzos import KRZOS
from krzos import parse_args
from hardware.player import Player
from hardware.sound import Sound

# constants ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

USE_DAEMON            = True # set False to run in foreground
GPIO_PIN              = 21
#MODULE_NAME           = 'xrzos'
MODULE_NAME           = 'krzos'
CLASS_NAME            = MODULE_NAME.upper()
WORK_DIR              = '/home/pi/workspaces/workspace-krzos/krzos/'
APPLICATION_FILENAME  = '{}.py'.format(MODULE_NAME)
APPLICATION_PATH      = os.path.join(WORK_DIR, APPLICATION_FILENAME)
print(Fore.BLUE + Style.BRIGHT + "application path: '{}'".format(APPLICATION_PATH) + Style.RESET_ALL)
PID_FILE              = WORK_DIR + '.krzosd.pid'

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
class KrzosDaemon(object):
    '''
    Monitors a switch connected to a GPIO pin.

    This state is used to enable or disable the KRZOS.
    '''
    def __init__(self, level):
        self._level = level
        self._log = Logger("krzosd", self._level)
        self._log.info('initialising krzosd…')
        self._toggle_switch  = ToggleSwitch(pin=GPIO_PIN, level=Level.INFO)
        self._counter    = itertools.count()
        self._old_state  = False
        self._krzos      = None
        _rosd_mask       = os.umask(0)
        os.umask(_rosd_mask)
        self._log.info('mask: ' + Fore.GREEN + '{}'.format(_rosd_mask))
        self._log.info('uid:  ' + Fore.GREEN + '{}'.format(os.getuid()))
        self._log.info('gid:  ' + Fore.GREEN + '{}'.format(os.getgid()))
        self._log.info('cwd:  ' + Fore.GREEN + '{}'.format(os.getcwd()))
        self._log.info('pid:  ' + Fore.GREEN + '{}'.format(PID_FILE))
        self._log.info('krzosd ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _get_timestamp(self):
        return dt.utcfromtimestamp(dt.utcnow().timestamp()).isoformat()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def read_state(self):
        '''
        Reads the state of the switch then returns the value.
        This only calls enable() or disable() if the value has changed
        since last reading.
        '''
        self._state = self._toggle_switch.pushed()
        self._log.debug('read state: {}'.format(self._state))
        if self._state is not self._old_state:
            if self._state:
                self._log.info('enabling KRZOS from state: {}'.format(self._state))
                self.enable()
            else:
                self._log.info('disabling KRZOS from state: {}'.format(self._state))
                self.disable()
            self._old_state = self._state

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def enable(self):
        self._enable_krzos()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def disable(self):
        self._disable_krzos()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _enable_krzos(self):
        if Util.already_running(APPLICATION_FILENAME):
            self._log.warning('{} already running…'.format(MODULE_NAME))
        else:
            Player.instance().play(Sound.SONIC_BAT)
            self._log.info('starting {} at {}…'.format(APPLICATION_FILENAME, self._get_timestamp()))
            # dynamically import the module
            _module = importlib.import_module(MODULE_NAME)
            # get the class from the module
            _class = getattr(_module, CLASS_NAME)
            # instantiate and use the class
            self._krzos = _class(level=Level.INFO)
            self._krzos.configure(parse_args(['-s'])) # include -g for gamepad
            # start KRZOS in its own thread
            self._krzos_thread = Thread(target=self._start_krzos, name="monitor-thread")
            self._krzos_thread.start()
            self._log.info('{} enabled; uuid: {}'.format(MODULE_NAME, self._krzos.uuid))

    def _start_krzos(self):
        if self._krzos:
            self._log.info('starting {}…'.format(MODULE_NAME))
            self._krzos.start()
            self._log.info('{} started.'.format(MODULE_NAME))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _disable_krzos(self):
        if self._krzos is not None:
            Player.instance().play(Sound.BOINK)
            self._log.info('shutting down {} at: {}'.format(MODULE_NAME, self._get_timestamp()))
            self._krzos.shutdown()
            self._cleanup();
            self._log.info('{} shut down.'.format(MODULE_NAME))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _cleanup(self):
        '''
        Remove instances of the main application and any singletons from the
        component registry following shutdown, to permit re-starting the
        application.
        '''
        self._log.info('🧈 {} cleanup.'.format(MODULE_NAME))
        _component_registry = globals.get('component-registry')
        if _component_registry:
            _instances = [ MODULE_NAME, 'tinyfx-ctrl' ]
            for _instance in _instances:
                if not _component_registry.has(_instance) or _component_registry.remove(_instance) is None:
                    self._log.info("no previous instance of '{}' in component registry.".format(_instance))
                else:
                    self._log.info("removed previous instance of '{}' from component registry".format(_instance))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def close(self):
        self._log.info(Fore.WHITE + 'closing krzosd…')
        if self._toggle_switch:
            self._toggle_switch.close()
        if self._krzos is not None:
            self._log.info('closing {}…'.format(MODULE_NAME))
            self._krzos.close()
            self._log.info('{} closed.'.format(MODULE_NAME))
        else:
            self._log.warning('{} thread was null.'.format(MODULE_NAME))
        self._log.info(Fore.WHITE + 'krzosd closed.')

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
class ToggleSwitch(object):
    def __init__(self, pin=GPIO_PIN, level=Level.INFO):
        self._log = Logger('switch', level)
        self._pin = pin
        GPIO.setwarnings(False)
        GPIO.cleanup()
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self._pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    def pushed(self):
        _value = not GPIO.input(self._pin)
        time.sleep(0.1)
        self._log.debug('pushed? {}'.format(_value))
        return _value

    def close(self):
        GPIO.cleanup(self._pin)

# main ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

def main():
    _daemon = None
    try:
        _daemon = KrzosDaemon(Level.INFO)
        while True:
            _daemon.read_state()
            time.sleep(0.2)
    except Exception:
        print(Fore.WHITE + 'error starting krzos daemon: {}'.format(traceback.format_exc()) + Style.RESET_ALL)
    finally:
        if _daemon:
            try:
                _daemon.close()
            except Exception:
                print(Fore.RED + 'error closing krzos daemon.' + Style.RESET_ALL)
        print(Fore.WHITE + 'krzosd complete.' + Style.RESET_ALL)

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

if Path(PID_FILE).is_file() and not Util.already_running(APPLICATION_FILENAME):
    os.remove(PID_FILE)
    print(Fore.WHITE + 'deleted previous pid file.' + Style.RESET_ALL)

def shutdown(signum, frame):  # signum and frame are mandatory
    print('krzosd.shutdown')
    sys.exit(0)

if __name__ == '__main__':
    if USE_DAEMON:
        with daemon.DaemonContext(
            stdout=sys.stdout,
            stderr=sys.stderr,
            working_directory=WORK_DIR,
            umask=0o002,
            pidfile=pidfile.TimeoutPIDLockFile(PID_FILE),
            signal_map={
                signal.SIGTERM: shutdown,
                signal.SIGTSTP: shutdown
            }
        ) as context:
            main()
    else:
        print(Fore.YELLOW + "Running in foreground mode (no daemon)" + Style.RESET_ALL)
        main()

#EOF
