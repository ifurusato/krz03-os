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

from core.directive import Directive
from hardware.motor_directive import MotorDirective

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class MotorDirectiveFactory:

    @staticmethod
    def create(directive: Directive, speed: float = 0.0, duration: float = None) -> MotorDirective:
        '''
        Create a new MotorDirective.

        For STOP, BRAKE and COAST we rely on the step counter for deceleration
        to reduce the speed rather than returning a 0.0 value (which would stop
        the motors immediately). WAIT is a sleep period with no change to the
        current state of the motors (moving or stopped).

        :param directive:    the Directive of a motion
        :param speed:        the speed of all motors, 0.0-1.0
        :param duration:     the optional duration in seconds
        '''
        # validate speed is within range
        if not (0.0 <= speed <= 1.0):
            raise ValueError("Speed must be between 0.0 and 1.0")
        # Set up MotorDirective values based on the directive
        match directive:
            case Directive.NO_CHANGE:
                return MotorDirective( directive, pfwd=speed,  sfwd=speed,  paft=speed,   saft=speed,  duration=duration )
            case Directive.WAIT:
                if duration is None:
                    raise ValueError('no duration provided.')
                return MotorDirective( directive, pfwd=speed,  sfwd=speed,  paft=speed,   saft=speed,  duration=duration )
            case Directive.STOP:
                return MotorDirective( directive, pfwd=speed,  sfwd=speed,  paft=speed,   saft=speed,  duration=duration )
            case Directive.BRAKE:
                return MotorDirective( directive, pfwd=speed,  sfwd=speed,  paft=speed,   saft=speed,  duration=duration )
            case Directive.COAST:
                return MotorDirective( directive, pfwd=speed,  sfwd=speed,  paft=speed,   saft=speed,  duration=duration )
            case Directive.AHEAD:
                return MotorDirective( directive, pfwd=speed,  sfwd=speed,  paft=speed,   saft=speed,  duration=duration )
            case Directive.ASTERN:
                return MotorDirective( directive, pfwd=-speed, sfwd=-speed, paft=-speed,  saft=-speed, duration=duration )
            case Directive.ROTATE_CW:
                return MotorDirective( directive, pfwd=speed,  sfwd=-speed, paft=speed,   saft=-speed, duration=duration )
            case Directive.ROTATE_CCW:
                return MotorDirective( directive, pfwd=-speed, sfwd=speed,  paft=-speed,  saft=speed,  duration=duration )
            case Directive.CRAB_PORT:
                return MotorDirective( directive, pfwd=-speed, sfwd=speed,  paft=speed,   saft=-speed, duration=duration )
            case Directive.CRAB_STBD:
                return MotorDirective( directive, pfwd=speed,  sfwd=-speed, paft=-speed,  saft=speed,  duration=duration )
            case Directive.DIAGONAL_PORT:
                return None #MotorDirective( directive, pfwd=speed,  sfwd=0.0,    paft=speed,   saft=0.0, duration=duration )
            case Directive.DIAGONAL_STBD:
                return None #MotorDirective( directive, pfwd=speed,  sfwd=0.0,    paft=-speed,  saft=0.0, duration=duration )
            case Directive.PIVOT_FWD_CW:
                return MotorDirective( directive, pfwd=speed,  sfwd=-speed, paft=0.0,     saft=0.0,    duration=duration )
            case Directive.PIVOT_FWD_CCW:
                return MotorDirective( directive, pfwd=-speed, sfwd=speed,  paft=0.0,     saft=0.0,    duration=duration )
            case Directive.PIVOT_AFT_CW:
                return MotorDirective( directive, pfwd=0.0,    sfwd=0.0,    paft=-speed,  saft=-speed, duration=duration )
            case Directive.PIVOT_AFT_CCW:
                return MotorDirective( directive, pfwd=0.0,    sfwd=0.0,    paft=speed,   saft=speed,  duration=duration )
            case Directive.PIVOT_PORT_CW:
                return None #MotorDirective( directive, pfwd=-speed, sfwd=-speed, paft=-speed,  saft=-speed, duration=duration )
            case Directive.PIVOT_STBD_CW:
                return None #MotorDirective( directive, pfwd=0.0,    sfwd=0.0,    paft=-speed,  saft=speed, duration=duration )
            case Directive.PIVOT_PORT_CCW:
                return None #MotorDirective( directive, pfwd=0.0,    sfwd=0.0,    paft=-speed,  saft=speed, duration=duration )
            case Directive.PIVOT_STBD_CCW:
                return None #MotorDirective( directive, pfwd=0.0,    sfwd=0.0,    paft=speed,   saft=-speed, duration=duration )

#EOF
