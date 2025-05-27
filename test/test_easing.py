#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#               
# author:   Murray Altheim
# created:  2025-05-19
# modified: 2025-05-19
#

import time
from colorama import init, Fore, Style
init()

from core.util import Util
from hardware.distance_sensors import Easing

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class DistanceSensors:

    def __init__(self, easing=Easing.SIGMOID):
        self._min_distance = 80
        self._max_distance = 300
        self._easing       = easing

    @property
    def min(self):
        return self._min_distance

    @property
    def max(self):
        return self._max_distance

    @property
    def easing_name(self):
        return self._easing.name

    def normalise_distance(self, dist):
        '''
        Normalize distance to a value between 0.0 and 1.0 using the instance's
        configured easing method and min/max distance settings.
        '''
        dist = max(min(dist, self._max_distance), self._min_distance)
        normalised = (dist - self._min_distance) / (self._max_distance - self._min_distance)
        return self._easing.apply(normalised)


# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
def main():

    ds = DistanceSensors(Easing.LINEAR)

    for f in Util.frange(start=ds.max, stop=ds.min, jump=-20.0): 
        _norm = ds.normalise_distance(f)
        print(Fore.CYAN + "value: '{:4.1f}';".format(f) 
                + Fore.GREEN + "\t    normalised: {:4.2f};".format(_norm)
                + Fore.CYAN  + "\t  using '{}' easing.".format(ds.easing_name) + Style.RESET_ALL)
        time.sleep(0.1)

if __name__ == '__main__':
    main()

#EOF
