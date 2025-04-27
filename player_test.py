#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-05-20
# modified: 2024-11-23
#

import time
from hardware.sound import Sound
from hardware.player import Player

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
def main():

    ALL_SOUNDS = False

    try:
        if ALL_SOUNDS:
            for _sound in Sound:
                Player.instance().play(_sound)
                time.sleep(1.5)
        else:
            Player.instance().play(Sound.SKADOODLE)
    except KeyboardInterrupt:
        print('done.')

if __name__== "__main__":
    main()

#EOF
