#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-08-28
# modified: 2024-09-06
#
# Sends 0 to both channels of the TinyFX, turning all LEDs on.
#

import sys, time, traceback
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.config_loader import ConfigLoader
from hardware.tinyfx_gpio_controller import TinyFxGpioController

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
def main():

    _loop   = False
    _tinyfx = None
    _log = Logger('test', Level.INFO)

    setting = sys.argv[1]
    _log.info(Fore.CYAN + Style.DIM + '-- setting \"{}\"...'.format(setting) + Style.RESET_ALL)

    try:
    
        # read YAML configuration
        _level = Level.INFO
        _config = ConfigLoader(Level.INFO).configure()
        _tinyfx = TinyFxGpioController(_config, level=Level.INFO)
        _log.info('starting test…')
    
        if setting.startswith('on'):
            _log.info(Fore.WHITE + 'setting ON')
            _tinyfx.on()
            
        elif setting.startswith('of'):
            _log.info(Fore.WHITE + 'setting OFF')
            _tinyfx.off()
            
        elif setting.startswith('ch1'):
            _log.info(Fore.WHITE + 'setting DOWN LIGHTS')
            _tinyfx.channel_on(TinyFxGpioController.CHANNEL_1)
            
        elif setting.startswith('ch2'):
            _log.info(Fore.WHITE + 'setting HEADLIGHT')
            _tinyfx.channel_on(TinyFxGpioController.CHANNEL_2)
            
        elif setting.startswith('ch3'):
            _log.info(Fore.WHITE + 'setting RUNNING LIGHTS')
            _tinyfx.channel_on(TinyFxGpioController.CHANNEL_3)
            
        elif setting.startswith('lo'):
            _log.info(Fore.WHITE + 'setting LOOP')
            _loop = True
            while True:
                _tinyfx.off()
                time.sleep(0.33)
                _tinyfx.channel_on(TinyFxGpioController.HEADLIGHT)
                time.sleep(1)
                time.sleep(0.33)
                _tinyfx.channel_on(TinyFxGpioController.DOWNLIGHT)
                time.sleep(1)
                time.sleep(0.33)
                _tinyfx.channel_on(TinyFxGpioController.RUNNING)
                time.sleep(1)
                
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
    print(Fore.RED + "\n  ERROR: expected 1 command line argument: 'on' | 'off' | 'ch1' | 'ch2' | 'ch3' | 'loop'" + Style.RESET_ALL)
    sys.exit(1)
    
if __name__== "__main__":
    main()
    
#EOF
