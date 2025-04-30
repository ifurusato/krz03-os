#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2020-05-19
# modified: 2025-04-28
#

import itertools
import threading
import time
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.component import Component
from core.event import Event
from core.orientation import Orientation
from core.speed import Speed
from core.util import Util
from behave.behaviour import Behaviour
from behave.trigger_behaviour import TriggerBehaviour
from hardware.roam_sensor import RoamSensor
#from hardware.motor_controller import MotorController

class FakeMotor(object):
    def __init__(self, label):
        self.__target_velocity = 0.0
        self._label = label
        self._velocity = 0.0
    @property
    def label(self):
        return self._label
    @property
    def velocity(self):
        return self._velocity
    @property
    def target_velocity(self):
        return self.__target_velocity
    @target_velocity.setter
    def target_velocity(self, target_velocity):
        self.__target_velocity = target_velocity
    def remove_velocity_multiplier(self, name):
        pass

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Roam(Behaviour):
    CLASS_NAME = 'roam'
    '''
    Implements a roaming behaviour. The end result of this Behaviour is to
    provide a forward speed limit for both motors based on a distance value
    provided by the center infrared sensor, i.e., the distance to an obstacle
    in cm. If no obstacle is perceived within the range of the sensor, the
    velocity limit is removed.

    Because we only know how far the obstacle is based on incoming events,
    if we haven't seen an event in awhile we may assume there is nothing
    there and start moving again, at "cruising" speed. But we need to wait
    a bit after reacting to an obstacle before attempting to start moving
    again.

    The Roam behaviour is by default suppressed.

    NOTES ....................

    This is a Subscriber to INFRARED_CNTR events, altering the usage of the
    center analog IR sensor to no longer function solely for obstacle
    avoidance, but instead set the robot's target velocity as a proportion to
    the perceived distance. I.e., if the sensor sees nothing at its maximum
    range the robot's forward target velocity will be set to its maximum. As
    the sensed distance is lessened the target velocity is likewise, until the
    robot reaches a minimum distance in which it halts and then goes into an
    obstacle avoidance behaviour (handled elsewhere).

    This means that we will in the future need to suppress whatever is the
    normal avoidance behaviour for the center IR sensor when this is active,
    at least up to the minimum roam distance.

    The external clock is required insofar as the Roam behaviour won't
    function in its absence, as it is used for resetting the motor's maximum
    velocity setting.

    This is implemented by adding a lambda function multiplier into the
    Motor's update_target_velocity method. When absent there is no effect;
    when closer than the minimum range a lambda that returns zero is set;
    otherwise a lambda that converts the observed distance (cm) to a ratio
    is used.

    The motor controller and external clock must be added after initialisation.

    :param config:           the application configuration
    :param message_bus:      the asynchronous message bus
    :param message_factory:  the factory for messages (added after init)
    :param exernal_clock:    the external clock (added after init)
    :param suppressed:       suppressed state, default True
    :param enabled:          enabled state, default True
    :param level:            the optional log level
    '''
    def __init__(self, config, message_bus=None, message_factory=None, level=Level.INFO):
        Behaviour.__init__(self, Roam.CLASS_NAME, config, message_bus, message_factory, suppressed=True, enabled=True, level=level)

#       if not isinstance(motor_ctrl, MotorController):
#           raise ValueError('wrong type for motor_ctrl argument: {}'.format(type(motor_ctrl)))
#       self._port_motor   = motor_ctrl.get_motor(Orientation.PORT)
#       self._stbd_motor   = motor_ctrl.get_motor(Orientation.STBD)
#       self._ext_clock    = external_clock
#       if self._ext_clock:
#           self._ext_clock.add_callback(self._tick)
#           pass
#       else:
#           raise Exception('unable to enable roam behaviour: no external clock available.')

        # add VL53L5CX sensor
        _skip_init = False
        self._roam_sensor = RoamSensor(config, _skip_init, level)

      
        # TEMP
        self._stop_event = threading.Event()
        self._thread     = None
        self._port_motor = FakeMotor('port')
        self._stbd_motor = FakeMotor('stbd')

        _cfg = config['krzos'].get('behaviour').get('roam')
        self._modulo        = 5 # at 20Hz, every 20 ticks is 1 second, every 5 ticks 250ms
        self._min_distance  = _cfg.get('min_distance')
        self._max_distance  = _cfg.get('max_distance')
        self._log.info(Style.BRIGHT + 'configured distance:\t{:4.2f} to {:4.2f}cm'.format(self._min_distance, self._max_distance))
        self._min_velocity  = _cfg.get('min_velocity')
        self._max_velocity  = _cfg.get('max_velocity')
        _velocity_km_hr = 36.0 * ( self._max_velocity / 1000 )
        self._log.info(Style.BRIGHT + 'configured speed:    \t{:4.2f} to {:4.2f}cm/sec ({:3.1f}km/hr)'.format(
                self._min_velocity, self._max_velocity, _velocity_km_hr))
        # zero lambda always returns a zero value
        self._zero_velocity_ratio = lambda n: self._min_velocity
        # lambda accepts distance and returns a ratio to multiply against velocity
        self._velocity_ratio = lambda n: ( ( n - self._min_distance ) / ( self._max_distance - self._min_distance ) )
        _ratio               = ( self._max_velocity - self._min_velocity ) / ( self._max_distance - self._min_distance )
        self._log.info(Style.BRIGHT + 'ratio calculation:\t{:4.2f} = ({:4.2f} - {:4.2f}) / ({:4.2f} - {:4.2f})'.format(
                _ratio, self._max_velocity, self._min_velocity, self._max_distance, self._min_distance))
        self._log.info(Style.BRIGHT + 'speed/distance ratio:\t{:4.2f} ({:.0%})'.format(_ratio, _ratio))
        self._cruising_speed = Speed.from_string(_cfg.get('cruising_speed'))
        self._cruising_velocity = float(self._cruising_speed.velocity)
        self._log.info(Style.BRIGHT + 'cruising speed:      \t{} ({:5.2f}cm/sec)'.format(self._cruising_speed.label, self._cruising_speed.velocity))
        self._wait_ticks    = _cfg.get('cruise_wait_ticks') # assumes slow tick at 1Hz
        self._wait_count    = self._wait_ticks
        self._log.info(Style.BRIGHT + 'cruise wait time:    \t{:4.2f} ticks'.format(self._wait_ticks))
        self._counter   = itertools.count()
        self._modulo    = 20 # 100: every 10 ticks 2Hz; 200: 1Hz;
        # .................................
        self.add_event(Event.INFRARED_CNTR)
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _poll_loop(self, stop_event):
        self._log.info('starting poll loop…')
        while not stop_event.is_set(): 
            _mean = self._roam_sensor.mean
            self._print_colored_mean(_mean)
            time.sleep(0.1)
        self._log.info('exited loop.')

    # TEMP Assuming self._mean is a list of 8 numeric values
    def _print_colored_mean(self, mean):
        formatted_values = []
        for val in mean:
            if val < 300:
                # Highlight with RED if less than 64
                formatted_values.append(Style.BRIGHT + f"{val:<4}" + Style.NORMAL)
            else:
                formatted_values.append(f"{val:<4}")
        print(Fore.CYAN + "mean dist mm:  " + Fore.WHITE + "  ".join(formatted_values) + Style.RESET_ALL)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def execute(self, message):
        '''
        The method called by process_message(), upon receipt of a message.
        :param message:  an Message passed along by the message bus
        '''
        if self.suppressed:
            self._log.info(Style.DIM + 'roam suppressed; message: {}'.format(message.event.label))
        elif self.enabled:
            if message.payload.event is Event.INFRARED_CNTR:
                _distance_cm = message.payload.value
                # TODO filter on distance here
#               self._log.info('processing message {}; '.format(message.name)
#                       + Fore.GREEN + ' distance: {:5.2f}cm\n'.format(_distance_cm))
                self._set_max_fwd_velocity_by_distance(_distance_cm)
            else:
                raise ValueError('expected INFRARED_CNTR event not: {}'.format(message.event.label))
#           print(Fore.WHITE + "mean dist mm:  {:<4}  {:<4}  {:<4}  {:<4}  {:<4}  {:<4}  {:<4}  {:<4}".format(*_mean) + Style.RESET_ALL)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _set_max_fwd_velocity_by_distance(self, distance_cm):
        '''
        This sets the velocity limit for both motors based on the distance
        argument. Both motors share the same limit, as there's no reason for
        them to be different.
        '''
        print('')
        self._log.info('setting max fwd velocity from distance of {:<5.2f}cm'.format(distance_cm))
        if distance_cm >= self._max_distance: # when distance >+ max_distance, no speed limit
            self._log.info(Fore.YELLOW + 'no speed limit at distance: {:5.2f} > max: {:5.2f}'.format(distance_cm, self._max_distance))
            self._reset_velocity_multiplier('no obstacle seen at {:>5.2f}cm.'.format(distance_cm))
        elif distance_cm < self._min_distance: # when distance < min_distance, set zero lambda
            self._set_velocity_multiplier(Fore.RED + 'too close', self._zero_velocity_ratio(distance_cm))
            self._wait_count = self._wait_ticks # reset wait
        else: # otherwise set lambda that returns a ratio of distance to speed as the limit
            self._set_velocity_multiplier(Fore.WHITE + 'within range at {:5.2f}'.format(distance_cm), self._velocity_ratio(distance_cm))
            self._wait_count = self._wait_ticks # reset wait

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _get_bar(self):
        return Util.repeat('█', 4 - self._wait_count) + Util.repeat('░', self._wait_count)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _tick(self):
        '''
        This uses a leaky integrator to set a target forward velocity after
        waiting at least 3 seconds (configurable). The trigger occurs on the
        transition of the wait count from 1 to 0, so that at zero it won't
        continually auto-trigger.
        '''
        print('🦋 _tick')
        if not self.suppressed:
            _count = next(self._counter)
            if _count % self._modulo == 0:
                self._log.info('tick; wait: {} ({:d}); suppressed: {};\t'.format(self._get_bar(), self._wait_count, self.suppressed))
                # wait ten counts before trying to move
                if self._wait_count == 0:
                    self._log.info('roaming;\t'
                            + Fore.RED   + 'port: {:5.2f}cm/s;\t'.format(self._port_motor.velocity)
                            + Fore.GREEN + 'stbd: {:5.2f}cm/s'.format(self._stbd_motor.velocity))
                elif self._wait_count == 1:
                    self._log.info('cruise triggered.')
                    self._log.info('cruise triggered at: {} ({:5.2f}cm/sec)'.format(self._cruising_speed.name, self._cruising_velocity))
                    self._wait_count = 0
                    # we change state in the transition from wait count 1 to 0 (0 being a steady state)
                    self._reset_velocity_multiplier('recovered from encounter.')
                    self._port_motor.target_velocity = self._cruising_velocity
                    self._stbd_motor.target_velocity = self._cruising_velocity
                else:
                    self._log.info('counting down from {:d}...'.format(self._wait_count))
                    self._wait_count -= 1
                    pass

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _set_velocity_multiplier(self, reason, lambda_function):
#       if not isinstance(lambda_function, function):
#           raise TypeError('expected lambda function, not {}'.format(type(lambda_function)))
        self._log.info(Fore.GREEN + 'set max fwd velocity: ' + '{}'.format(reason))
        self._port_motor.add_velocity_multiplier(Roam.CLASS_NAME, lambda_function)
        self._stbd_motor.add_velocity_multiplier(Roam.CLASS_NAME, lambda_function)
        pass

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _reset_velocity_multiplier(self, reason):
        self._log.info(Fore.MAGENTA + '😨 reset max fwd velocity: ' + Fore.YELLOW + '{}'.format(reason))
        self._port_motor.remove_velocity_multiplier(Roam.CLASS_NAME)
        self._stbd_motor.remove_velocity_multiplier(Roam.CLASS_NAME)
        pass

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_trigger_behaviour(self, event):
        return TriggerBehaviour.TOGGLE

    @property
    def trigger_event(self):
        '''
        This returns the event used to enable/disable the behaviour manually.
        '''
        return Event.ROAM

    def release(self):
        '''
        Releases (un-suppresses) this Component.
        '''
        Component.release(self)
        self._log.info(Fore.GREEN + '💙 roam released.')

    def suppress(self):
        '''
        Suppresses this Component.
        '''
        Component.suppress(self)
        self._reset_velocity_multiplier('suppressing roam.')
        self._log.info(Fore.BLUE + '💙 roam suppressed.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def enable(self):
        '''
        Enables this behaviour.
        '''
        self._log.info(Fore.BLUE + '💙 enabling roam…')
        Behaviour.enable(self)
        if self._roam_sensor:
            self._roam_sensor.enable()
        self._thread = threading.Thread(name='poll-loop', target=self._poll_loop, args=(self._stop_event,))
        self._thread.start()
        self._log.info(Fore.BLUE + '💙 enabled roam.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def disable(self):
        '''
        Disables this behaviour.
        '''
        self._log.info(Fore.BLUE + '💙 disabling roam…')
        Behaviour.disable(self)
        if self._stop_event:
            self._stop_event.set()
            self._log.info('cancelled thread.')
        if self._roam_sensor:
            self._roam_sensor.disable()
        self._reset_velocity_multiplier('disabling roam.')
        self._log.info(Fore.BLUE + '💙 roam disabled.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def close(self):
        if self._roam_sensor:
            self._roam_sensor.close()
        Behaviour.close(self)
        self._log.info(Fore.BLUE + '💙 roam closed.')

#EOF
