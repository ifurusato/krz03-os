#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-05-07
# modified: 2025-05-09
#

import math
from enum import Enum
from colorama import init, Fore, Style
init()

from core.component import Component
from core.logger import Logger, Level
from core.orientation import Orientation
from hardware.distance_sensor import DistanceSensor

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Easing(Enum):
    LINEAR      = 'linear'       # direct proportional mapping 
    QUADRATIC   = 'quadratic'    # values decrease quickly near max, then flatten near min 
    CUBIC       = 'cubic'        # decreases very fast initially, then barely changes close to obstacles
    SQUARE_ROOT = 'square_root'  # maintains higher values longer, drops more gradually near obstacles
    LOGARITHMIC = 'logarithmic'  # keeps values high over a wide range, drops off sharply only near minimum
    SIGMOID     = 'sigmoid'      # smooth, balanced transition; avoids abrupt changes

    def apply(self, normalised: float) -> float:
        '''
        Apply the selected easing function to the normalised value.
        '''
        match self:
            case Easing.LINEAR:
                return normalised
            case Easing.QUADRATIC:
                return normalised ** 2
            case Easing.CUBIC:
                return normalised ** 3
            case Easing.SQUARE_ROOT:
                return math.sqrt(normalised)
            case Easing.LOGARITHMIC:
                '''
                log1p(9x)/log1p(9) keeps range [0,1]

                Value  Behaviour       Effect on Curve                            Responsiveness
                1      linear-ish      very shallow log curve, close to linear    fast response — not much easing
                3      mild easing     starts higher, drops gently                medium responsiveness
                5      moderate        flatter curve near 0                       less responsive until mid-range
                9      strong          very flat early, sharp drop at end         conservative; waits to respond
                20     very strong     stays almost flat until ~80%               very late response; very cautious
                '''
                log_scaling_factor = 7
                return math.log1p(normalised * log_scaling_factor) / math.log1p(log_scaling_factor)
            case Easing.SIGMOID:
                sigmoid_sharpness = 6   # reduce sharpness to make the deceleration more gradual (was 4)
                midpoint          = 0.4 # shift the midpoint of the sigmoid to start decelerating earlier (was 0.22)
                return 1 / (1 + math.exp(-sigmoid_sharpness * (normalised - midpoint)))
            case _:
                raise ValueError("Unknown easing type: {}".format(self))

    @classmethod
    def from_string(cls, name: str):
        '''
        Convert a string to an Easing enum member.
        Case-insensitive. Raises ValueError on invalid name.
        '''
        name = name.strip().lower()
        for member in cls:
            if member.value == name:
                return member
        raise ValueError("'{}' is not a valid Easing type. Available options: {}".format(name, [e.value for e in cls]))

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class DistanceSensors(Component):
    '''
    Collects the three IR distance sensors into a class. This is a raw sensing
    class with no timing or integration into the rest of the operating system.
    It does support weighted averages of the center and port, and center and
    starboard sensors.

    :param config:            the application configuration
    :param level:             the log level
    '''
    def __init__(self, config, level=Level.INFO):
        if not isinstance(level, Level):
            raise ValueError('wrong type for log level argument: {}'.format(type(level)))
        self._level = level
        self._log = Logger('dists', level)
        Component.__init__(self, self._log, suppressed=False, enabled=False)
        # configuration ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        if config is None:
            raise ValueError('no configuration provided.')
        _cfg = config['krzos'].get('hardware').get('distance_sensors')
        self._min_distance     = _cfg.get('min_distance', 80)
        self._default_distance = _cfg.get('max_distance', 300)
        _easing_value          = _cfg.get('easing', 'logarithmic')
        self._easing           = Easing.LINEAR # Easing.from_string(_easing_value)
        # sensors ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._port_sensor = DistanceSensor(config, Orientation.PORT)
        self._cntr_sensor = DistanceSensor(config, Orientation.CNTR)
        self._stbd_sensor = DistanceSensor(config, Orientation.STBD)
        self._sensors = {
           Orientation.PORT: self._port_sensor,
           Orientation.CNTR: self._cntr_sensor,
           Orientation.STBD: self._stbd_sensor
        }
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def enable(self):
        '''
        Enable the three sensors as well as this class.
        '''
        for _sensor in self._sensors.values():
            _sensor.enable()
        Component.enable(self)
        self._log.info('enabled.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def sensors(self):
        '''
        Return the Orientation-keyed dictionary of sensors as a property.
        '''
        return self._sensors

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def __getattr__(self, name):
        '''
        Access individual sensors via attributes, e.g.,

            _sensors = DistanceSensors(config)
            _port_sensor = _sensors.PORT
        '''
        if name in Orientation.__members__: # check if the name is a valid orientation
            orientation = Orientation[name]
            if hasattr(self, f"_{orientation.name.lower()}_sensor"):
                return getattr(self, f"_{orientation.name.lower()}_sensor")
        raise AttributeError("'{}' object has no attribute '{}'".format(self.__class__.__name__, name))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def __iter__(self):
        '''
        Iterate over the sensors, permitting this construction:

            for _sensor in _sensors:
                distance_mm = _sensor.distance
        '''
        return iter(self._sensors.values())

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get(self, orientation: Orientation):
        '''
        Get method to retrieve the sensor by Orientation.
        '''
        return self._sensors.get(orientation, None)  # Returns the sensor or None if not found

    # weighted averages support ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def normalise_distance(self, dist):
        '''
        Normalize distance to a value between 0.0 and 1.0 using the instance's
        configured easing method and min/max distance settings.
        '''
        dist = max(min(dist, self._default_distance), self._min_distance)
        normalised = (dist - self._min_distance) / (self._default_distance - self._min_distance)
        return self._easing.apply(normalised)

    def get_weighted_averages(self):
        '''
        Read current distances or substitute default. This returns a tuple
        containing the port and starboard values.
        '''
        port = self._port_sensor.distance or self._default_distance
        cntr = self._cntr_sensor.distance or self._default_distance
        stbd = self._stbd_sensor.distance or self._default_distance
        # compute pairwise averages
        port_avg = (port + cntr) / 2
        stbd_avg = (stbd + cntr) / 2
        # normalise using instance method
        port_norm = self.normalise_distance(port_avg)
        stbd_norm = self.normalise_distance(stbd_avg)
        return port_norm, stbd_norm

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def disable(self):
        '''
        Disable the three sensors as well as this class.
        '''
        Component.disable(self)
        for _sensor in self._sensors.values():
            _sensor.disable()
        self._log.info('disabled.')

#EOF
