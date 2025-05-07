#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-08-28
# modified: 2025-04-22
#
# Sends data to the TinyFX, using the TinyFxController.
#

import sys, time, traceback
from colorama import init, Fore, Style
init()

from core.orientation import Orientation
from core.logger import Logger, Level
from core.config_loader import ConfigLoader
from hardware.tinyfx_controller import TinyFxController

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
def main():
    
    _loop   = False
    _tinyfx = None
    _log = Logger('test', Level.INFO)

    setting = sys.argv[1]
    _log.info(Fore.CYAN + Style.DIM + "-- setting '{}'…".format(setting) + Style.RESET_ALL)
    
    try:
    
        # read YAML configuration
        _level = Level.INFO
        _config = ConfigLoader(Level.INFO).configure()
        _tinyfx = TinyFxController(_config, level=Level.INFO)
        _tinyfx.enable()

        _log.info('starting test…')
    
        if setting.startswith('help'):
            _tinyfx.help()

        elif setting.startswith('exit'):
            _tinyfx.exit()

        elif setting.startswith('on'):
            _log.info(Fore.WHITE + 'setting ON')
            _tinyfx.on()
            
        elif setting.startswith('ram'):
            _tinyfx.ram()

        elif setting.startswith('sounds'):
            _tinyfx.sounds()

        elif setting.startswith('flash'):
            _tinyfx.flash()

        elif setting.startswith('of'):
            _log.info(Fore.WHITE + 'setting OFF')
            _tinyfx.off()
            
        elif setting.startswith('ma'):
            _log.info(Fore.WHITE + 'setting MAST LIGHT')
            _tinyfx.channel_on(Orientation.MAST)
            
        elif setting.startswith('po'):
            _log.info(Fore.WHITE + 'setting PORT LIGHT')
            _tinyfx.channel_on(Orientation.PORT)
            
        elif setting.startswith('st'):
            _log.info(Fore.WHITE + 'setting STBD LIGHT')
            _tinyfx.channel_on(Orientation.STBD)

        elif setting.startswith('pi'):
            if setting == 'pir get':
                _log.info(Fore.WHITE + 'getting PIR')
                _tinyfx.channel_on(Orientation.PIR)
            elif setting == 'pir on':
                _log.info(Fore.WHITE + 'enable PIR')
                _tinyfx.pir(True)
            elif setting == 'pir off':
                _log.info(Fore.WHITE + 'disable PIR')
                _tinyfx.pir(False)
            else:
                _log.warning('unrecognised argument \"{}\"'.format(setting))
        elif setting.startswith('play '):
            _log.info(Fore.WHITE + "play '{}'".format(setting[5:]))
            _tinyfx.play(setting)
        else:   
            _log.warning('unrecognised argument \"{}\"'.format(setting))

    except KeyboardInterrupt:
        _log.info('Ctrl-C caught; exiting…')
    except Exception as e:
        _log.error('error in motor test: {}'.format(e))
        traceback.print_exc(file=sys.stdout)
    finally:
        if _tinyfx and _loop:
            _tinyfx.off()
            _tinyfx.close()
            pass
            
# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

if len(sys.argv) != 2:
    print(Fore.RED + "\n  ERROR: expected 1 command line argument: 'on' | 'off' | 'mast' | 'port' | 'stbd' | 'pir get' | 'pir on' | 'pir off' | 'play {key}' | ram | flash" + Style.RESET_ALL)
    sys.exit(1)
    
if __name__== "__main__":
    main()
    
#EOF
