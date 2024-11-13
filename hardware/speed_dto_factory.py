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

from core.direction import Direction
from hardware.speed_dto import SpeedDTO

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class SpeedDTOFactory:

    @staticmethod
    def create(direction: Direction, speed: float) -> SpeedDTO:
        # Check speed is within range
        if not (-1.0 <= speed <= 1.0):
            raise ValueError("Speed must be between 0.0 and 1.0")
        # Set up SpeedDTO values based on the direction
        if direction == Direction.STOPPED:
            return SpeedDTO( direction, pfwd=0.0,    sfwd=0.0,    paft=0.0,     saft=0.0 )
        elif direction == Direction.NO_CHANGE:
            return SpeedDTO( direction, pfwd=speed,  sfwd=speed,  paft=speed,   saft=speed )
        elif direction == Direction.AHEAD:
            return SpeedDTO( direction, pfwd=speed,  sfwd=speed,  paft=speed,   saft=speed )
        elif direction == Direction.ASTERN:
            return SpeedDTO( direction, pfwd=-speed, sfwd=-speed, paft=-speed,  saft=-speed )
        elif direction == Direction.ROTATE_CW:
            return SpeedDTO( direction, pfwd=speed,  sfwd=-speed, paft=speed,   saft=-speed )
        elif direction == Direction.ROTATE_CCW:
            return SpeedDTO( direction, pfwd=-speed, sfwd=speed,  paft=-speed,  saft=speed )
        elif direction == Direction.CRAB_PORT:
            return SpeedDTO( direction, pfwd=-speed, sfwd=speed,  paft=speed,   saft=-speed )
        elif direction == Direction.CRAB_STBD:
            return SpeedDTO( direction, pfwd=speed,  sfwd=-speed, paft=-speed,  saft=speed )
        elif direction == Direction.DIAGONAL_PORT:
            return None #SpeedDTO( direction, pfwd=speed,  sfwd=0.0,    paft=speed,   saft=0.0 )
        elif direction == Direction.DIAGONAL_STBD:
            return None #SpeedDTO( direction, pfwd=speed,  sfwd=0.0,    paft=-speed,  saft=0.0 )
        elif direction == Direction.PIVOT_FWD_CW:
            return SpeedDTO( direction, pfwd=speed,  sfwd=-speed, paft=0.0,     saft=0.0 )
        elif direction == Direction.PIVOT_FWD_CCW:
            return SpeedDTO( direction, pfwd=-speed, sfwd=speed,  paft=0.0,     saft=0.0 )
        elif direction == Direction.PIVOT_AFT_CW:
            return SpeedDTO( direction, pfwd=0.0,    sfwd=0.0,    paft=-speed,  saft=-speed )
        elif direction == Direction.PIVOT_AFT_CCW:
            return SpeedDTO( direction, pfwd=0.0,    sfwd=0.0,    paft=speed,   saft=speed )
        elif direction == Direction.PIVOT_PORT_CW:
            return None #SpeedDTO( direction, pfwd=-speed, sfwd=-speed, paft=-speed,  saft=-speed )
        elif direction == Direction.PIVOT_STBD_CW:
            return None #SpeedDTO( direction, pfwd=0.0,    sfwd=0.0,    paft=-speed,  saft=speed )
        elif direction == Direction.PIVOT_PORT_CCW:
            return None #SpeedDTO( direction, pfwd=0.0,    sfwd=0.0,    paft=-speed,  saft=speed )
        elif direction == Direction.PIVOT_STBD_CCW:
            return None #SpeedDTO( direction, pfwd=0.0,    sfwd=0.0,    paft=speed,   saft=-speed )

#EOF
