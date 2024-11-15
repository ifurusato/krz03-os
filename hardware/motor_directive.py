#!/usr/bin/env python3 
# -*- coding: utf-8 -*-
#
# Copyright 2020 by Murray Altheim. All rights reserved. This file is part of
# the Robot OS project and is released under the "Apache Licence, Version 2.0".
# Please see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-11-13
# modified: 2024-11-15
#

from enum import Enum
from colorama import init, Fore, Style
init()

from core.directive import Directive
from core.orientation import Orientation

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class MotorDirective:
    '''
    A Data Transfer Object for motor target speeds and durations. If only
    the first 'pfwd' value is provided, it will set all four. This is
    equivalent to using Orientation.ALL. 

    :param directive:   the Directive indicating the action
    :param pfwd:        the speed of the port-forward motor
    :param sfwd:        the speed of the starboard-forward motor
    :param paft:        the speed of the port-aft motor
    :param saft:        the speed of the starboard-aft motor
    :param duration:    optional, for Directives related to time (e.g., WAIT)
    '''
    def __init__(self, directive: Directive, pfwd: float, sfwd: float = None, paft: float = None, saft: float = None, duration: float = None):
        self._directive = directive
        if pfwd is None or not (-1.0 <= pfwd <= 1.0):
            raise ValueError("pfwd must be provided and between 0.0 and 1.0")
        if sfwd is None and paft is None and saft is None:
            # Set all values to pfwd if only pfwd is provided
            self._pfwd = self._sfwd = self._paft = self._saft = pfwd
        else:
            # Ensure other values are provided and within the valid range
            if not all(-1.0 <= v <= 1.0 for v in (sfwd, paft, saft)):
                raise ValueError("All values must be between 0.0 and 1.0")
            self._pfwd = pfwd
            self._sfwd = sfwd
            self._paft = paft
            self._saft = saft
        self._duration  = duration

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def directive(self) -> Directive:
        return self._directive

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def pfwd(self) -> float:
        return self._pfwd

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def sfwd(self) -> float:
        return self._sfwd

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def paft(self) -> float:
        return self._paft

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def saft(self) -> float:
        return self._saft

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def duration(self) -> float:
        return self._duration

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get(self, orientation: Orientation) -> float:
        if orientation is Orientation.PFWD:
            return self._pfwd
        elif orientation is Orientation.SFWD:
            return self._sfwd
        elif orientation is Orientation.PAFT:
            return self._paft
        elif orientation is Orientation.SAFT:
            return self._saft
        else:
            raise ValueError("Invalid orientation")

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def __str__(self):
        return ('MotorDirective: '
                '{:<10} {:<14}  '
                '{:<8} {:>5.2f}  '
                '{:<8} {:>5.2f}  '
                '{:<8} {:>5.2f}  '
                '{:<8} {:>5.2f}'.format(
                    'Directive:', self.directive.name,
                    Fore.RED + 'PFWD:', float(self.pfwd),
                    Fore.GREEN + 'SFWD:', float(self.sfwd),
                    Fore.RED + 'PAFT:', float(self.paft),
                    Fore.GREEN + 'SAFT:', float(self.saft)
                ) + Style.RESET_ALL)

#EOF
