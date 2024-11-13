#!/usr/bin/env python3 
# -*- coding: utf-8 -*-
#
# Copyright 2020 by Murray Altheim. All rights reserved. This file is part of
# the Robot OS project and is released under the "Apache Licence, Version 2.0".
# Please see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-11-13
# modified: 2024-11-13
#

from enum import Enum
from colorama import init, Fore, Style
init()

from core.direction import Direction
from core.orientation import Orientation

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class SpeedDTO:
    '''
    A Data Transfer Object for motor target speeds. If only the first 'pfwd'
    value is provided, it will set all four. This is equivalent to using
    Orientation.ALL.
    '''
    def __init__(self, direction: Direction, pfwd: float, sfwd: float = None, paft: float = None, saft: float = None):
        self._direction = direction
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

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def direction(self) -> Direction:
        return self._direction

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
        return ('SpeedDTO: '
                '{:<10} {:<14}  '
                '{:<8} {:>5.2f}  '
                '{:<8} {:>5.2f}  '
                '{:<8} {:>5.2f}  '
                '{:<8} {:>5.2f}'.format(
                    'Direction:', self.direction.name,
                    Fore.RED + 'PFWD:', float(self.pfwd),
                    Fore.GREEN + 'SFWD:', float(self.sfwd),
                    Fore.RED + 'PAFT:', float(self.paft),
                    Fore.GREEN + 'SAFT:', float(self.saft)
                ) + Style.RESET_ALL)

#EOF
