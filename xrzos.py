#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-05-16
# modified: 2025-05-17
#
# a mock test app for krzosd.py

import sys, time, traceback
import itertools
from threading import Thread, Event
from colorama import init, Fore, Style
init()

from core.component import Component
from core.fsm import FiniteStateMachine
from core.logger import Logger, Level

class XRZOS(Component, FiniteStateMachine):

    def __init__(self, level=Level.INFO):
        _name = 'xrzos'
        self._level = level
        self._log = Logger(_name, self._level)
        Component.__init__(self, self._log, suppressed=False, enabled=False)
        FiniteStateMachine.__init__(self, self._log, _name)
        self._loop_interval = 2.0
        self._stop_event    = None
        self._log.info('initialised.')

    def configure(self, arguments):
        self._log.info('configured.')

    def start(self):
        self._log.info('starting…')
        self._enabled = False
        self.enable()

    def enable(self):
        if self.enabled:
            self._log.warning('already running.')
            return
        Component.enable(self)
        self._stop_event = Event()
        self._thread = Thread(name='sensor-loop', target=self._loop, args=(self._stop_event,), daemon=False)
        self._thread.start()

    def _loop(self, stop_event):
        _se_set = stop_event.is_set()
        _counter = itertools.count()
        while not stop_event.is_set():
            time.sleep(self._loop_interval)
            count = next(_counter)
            self._log.info(Fore.GREEN + '[{:005d}] loop…'.format(count))
        self._log.info('exited loop.')

    def shutdown(self):
        self._log.info('shutting down…')
        if self._stop_event:
            self._stop_event.set()

    def disable(self):
        self._log.info('disabled.')
        self.shutdown()

    def close(self):
        self._log.info('closing…')
        Component.close(self)
        FiniteStateMachine.close(self)
        self._log.info('closed.')

def main(argv):
    _xrzos = None
    _log = Logger("main", Level.INFO)
    _keep_running = Event()
    try:
        _level = Level.INFO
        _log.level = _level
        _xrzos = XRZOS(level=_level)
        _args = []
        _xrzos.configure(_args)
        _xrzos.start()
        _keep_running.wait()
    except KeyboardInterrupt:
        print('\n')
        print(Fore.MAGENTA + Style.BRIGHT + 'caught Ctrl-C; exiting…' + Style.RESET_ALL)
        if _xrzos:
            _xrzos.disable()
    except RuntimeError as rte:
        _log.error('runtime error starting xrzos: {}'.format(traceback.format_exc()))
    except Exception as e:
        _log.error('{} raised executing xrzos: {}\n{}'.format(type(e), e, traceback.format_exc()))
    finally:
        if _xrzos and not _xrzos.closed:
            _xrzos.close()
        return _xrzos

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
if __name__== "__main__":
    main(sys.argv[1:])

#EOF
