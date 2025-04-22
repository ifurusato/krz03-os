#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-05-20
# modified: 2024-11-23
#

import time
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.component import Component
from core.config_loader import ConfigLoader
from hardware.sound import Sound
from hardware.tinyfx_controller import TinyFxController

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
class Player(Component):
    __instance = None
    '''
    A sound player that plays sounds using the Tiny FX, connected over I2C.

    This is a singleton class; obtain its instance and play a sound via:

        Player.instance().play(Sound.CHIRP_1)

    Unlike the version of this on the MR01 this does not use threads, nor
    does it support looping.
    '''

    def __init__(self):
        raise RuntimeError('singleton: call instance() instead.')

    @classmethod
    def instance(cls, tinyfx=None):
        if cls.__instance is None:
            cls.__instance = cls.__new__(cls)
            # put any initialization here.
            cls.__instance._log = Logger('player', Level.INFO)
            Component.__init__(cls.__instance, cls.__instance._log, suppressed=False, enabled=True)
            _config = ConfigLoader(Level.INFO).configure()
            _cfg = _config['krzos'].get('hardware').get('player')
            if tinyfx is None:
                cls.__instance._tinyfx_controller = TinyFxController(_config)
            else:
                cls.__instance._tinyfx_controller = tinyfx
            cls.__instance._verbose = _cfg.get('verbose')
            cls.__instance._log.info('ready.')
        return cls.__instance

    @staticmethod
    def play(value):
        '''
        Plays a Sound.
        '''
        if isinstance(value, Sound):
            _sound = value
        else:
            raise ValueError('expected a Sound.')
        _name        = _sound.name
        _duration    = _sound.duration
        _description = _sound.description
        _player      = Player.instance()
        if _player._verbose:
            _player._log.info(Fore.MAGENTA + "playing '" + Style.BRIGHT + "{}".format(_name) + Style.NORMAL + "' ('{}') for {} seconds.".format(_description, _duration))
        _player._tinyfx_controller.play(_sound.name)
        time.sleep(_duration)
        time.sleep(0.1)

    # unsupported methods ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def loop(self, sound, count):
        raise NotImplementedError('not implemented.')

    def stop(self):
        raise NotImplementedError('not implemented.')

    def continuous_loop(self, sound):
        raise NotImplementedError('not implemented.')

    @staticmethod
    def play_from_thread(value):
        raise NotImplementedError('not implemented.')

    @staticmethod
    def halt_thread():
        raise NotImplementedError('not implemented.')

#EOF
