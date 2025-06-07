#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-06-02
# modified: 2025-06-08
#
# A controller for a DFRobot Brushless DC Motor, which contains its own internal
# hardware controller and an 'FG' feedback pin permitting closed-loop control.
# Its PWM is inverted: 100% is stopped, 0% is full speed.
#
# This motor controller includes support for open- or closed-loop control,
# stall-and-recovery, deadband control, and control by target RPM when operating
# in closed-loop mode. Given a wheel diameter this also provides for odometric
# distance and speed measurements.
#
# PWM0 pin:      GPIO 12 | 18
# PWM1 pin:      GPIO 13 | 19
# direction pin: GPIO 23
# FG pin:        GPIO 24
#

import time
import asyncio
import itertools
from threading import Thread
from math import pi as π
from datetime import datetime as dt
from colorama import init, Fore, Style
init()

import pigpio

from core.logger import Logger, Level
from hardware.pwm_controller import HardwarePWMController, SoftwarePWMController

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
class BrushlessMotor:
    PWM_GPIO_PIN      = 18
    DIR_GPIO_PIN      = 23
    FG_GPIO_PIN       = 24
    MOTOR_MAX_RPM     = 159 # from the motor docs
    DIRECTION_FORWARD = 1
    DIRECTION_REVERSE = 0
    # calibration constant: millimeters traveled per 1% speed per second
    CALIBRATION_MM_PER_PERCENT_PER_SEC = 5.48  # based on 10 rotations being 2199mm

    def __init__(self, pi, pwm_pin=None, dir_pin=None, pwm_freq=25000, accel_delay=0.1, max_slew_rate=20, level=Level.INFO):
        self._log = Logger('motor', level)
        self._pi                  = pi
        self._pwm_pin             = self.PWM_GPIO_PIN if pwm_pin is None else pwm_pin
        self._dir_pin             = self.DIR_GPIO_PIN if dir_pin is None else dir_pin
        self._pwm_freq            = pwm_freq
        self._accel_delay         = accel_delay
        self._max_slew_rate       = max_slew_rate
        self._current_speed       = 0        # current speed (-100 to 100)
        self._target_speed        = 0        # target speed (-100 to 100)
        self._kickstart_speed     = 14       # speed threshold to kickstart motor from zero
        self._start_time          = None     # when current motion started
        self._stop_time           = None     # when motor stopped or last speed change
        self._verbose             = False
        # open or closed loop support ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._closed_loop_enabled = True # TODO config
        if self._closed_loop_enabled:
            self._log.info("using closed loop control.")
        else:
            self._log.info("using open loop control.")
        self._last_tick           = None
        self._pulse_count         = 0
        self._pulse_intervals     = []
        self._max_interval_buffer = 10       # average over last N intervals
        self._rpm_errors          = []       # for calculating PID performance
        self._feedback_interval   = 0.05     # seconds between corrections
        self._feedback_task       = None     # async task
        self._direction           = self.DIRECTION_FORWARD
        self._measured_rpm        = 0 
        self._kp                  = 0.1
        self._ki                  = 0.0
        self._kd                  = 0.0
        # hardware or software PWM config ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._pi.set_mode(self._dir_pin, pigpio.OUTPUT) # set motor direction pin as output
        _use_software_pwm         = False   # TODO from config
        # initialize PWM controller (hardware or software)
        self._pwm_controller = self._get_pwm_controller(_use_software_pwm)
        # odometry ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._gear_ratio              = 45 # from motor spec on DFRobot page
        self._pulses_per_motor_rev    = 6 # ditto
        self._pulses_per_output_rev   = self._pulses_per_motor_rev * self._gear_ratio # 270
        self._wheel_diameter_mm       = 48 # TODO from config
        self._circumference_mm        = self._wheel_diameter_mm * π
        self._cumulative_distance_mm  = 0.0
        self._initial_distance        = 0.0
        self._target_distance         = None
        self._distance_target_reached = False
        self._distance_future         = None
        # slew limiter ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._slew_limiter_enabled    = True  # TODO from config
        _max_delta_speed_per_sec      = 100.0 # TODO from config
        _max_delta_rpm_per_sec        = 120.0 # TODO from config
        self._rpm_limiter = SlewLimiter(max_delta_per_sec=_max_delta_rpm_per_sec)
        self._speed_limiter = SlewLimiter(max_delta_per_sec=_max_delta_speed_per_sec)
        # establish callback from FB on GPIO pin using falling edge detection
        self._pi.set_mode(self.FG_GPIO_PIN, pigpio.INPUT)
        self._pi.set_pull_up_down(self.FG_GPIO_PIN, pigpio.PUD_OFF)
        self._callback = self._pi.callback(self.FG_GPIO_PIN, pigpio.FALLING_EDGE, self._fg_callback)
        # stall & recovery management ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._target_rpm              = 0    # target RPM (used in closed-loop mode)
        self._is_stalled              = False
        self._last_target_rpm_set_dt  = None
        self._stall_detection_grace_period_ms = 400 # or tweak to maybe ~500ms
        # pulse timing
        self._last_tick_dt            = None # timestamp of last FG pulse
        self._stall_timeout_ms        = 300  # duration with no pulses to declare stall (in ms)
        # deadband configuration
        self._deadband_rpm            = 6    # don't attempt to move at speed less than this
        self._fixed_deadband          = True # so far dynamic deadband doesn't work
        self._deadband                = 2    # percent
        # callback handlers
        self._on_stall_callback       = None
        self._on_recovery_callback    = None
        self.set_on_stall(self._handle_stall)
        self.set_on_recovery(self._handle_recovery)
        # asyncio event loop in background thread ┈┈┈┈┈┈┈┈┈┈
        self._loop = asyncio.new_event_loop()
        self._loop, self._loop_thread = self._start_event_loop_thread()
        # start periodic stall monitor
        asyncio.run_coroutine_threadsafe(self._stall_monitor(), self._loop)
        self._log.info('ready.')

    def _get_pwm_controller(self, use_software_pwm):
        '''
        Get the appropriate PWM controller based on the configuration.
        '''
        if use_software_pwm:
            self._log.info("using software PWM controller.")
            return SoftwarePWMController(self._pi, self._pwm_pin, self._pwm_freq)
        else:
            self._log.info("using hardware PWM controller.")
            return HardwarePWMController(self._pi, self._pwm_pin, self._pwm_freq)

    def _start_event_loop_thread(self):
        loop = asyncio.new_event_loop()
        thread = Thread(target=loop.run_forever, daemon=True)
        thread.start()
        return loop, thread

    # properties ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    @property
    def is_closed_loop_enabled(self):
        return self._closed_loop_enabled

    @property
    def measured_rpm(self):
        '''
        Returns the RPM indicated by counting the pulses coming from FG,
        the motor feedback pin. Returns None if running in open loop mode.
        '''
        if self._measured_rpm is None:
            self._log.warning("RPM not yet calculated.")
            return 0.0
        # apply direction sign dynamically when reading
        return -abs(self._measured_rpm) if self._direction == self.DIRECTION_REVERSE else abs(self._measured_rpm)

    @property
    def measured_mm_per_sec(self):
        '''
        Returns the current measured speed in mm/sec using the formula:

            linear speed = wheel circumference * RPM / 60

        Returns None if running in open loop mode.
        '''
        return (self._circumference_mm * self.measured_rpm) / 60.0 if self._closed_loop_enabled else None

    @property
    def cumulative_distance_mm(self):
        '''
        Return the cumulative distance in millimeters.
        Returns None if running in open loop mode.
        '''
        return self._cumulative_distance_mm if self._closed_loop_enabled else None

    # stall & recovery support ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def set_on_stall(self, callback):
        self._on_stall_callback = callback

    def set_on_recovery(self, callback):
        self._on_recovery_callback = callback

    def _handle_stall(self):
        self._log.info("stall detected — attempting recovery…")
        self._stop_rpm_control() # pause PID temporarily
        asyncio.run_coroutine_threadsafe(self._attempt_recovery(), self._loop)

    async def _attempt_recovery(self):
        if abs(self._target_rpm) < self._deadband_rpm:
            self._log.warning("recovery aborted — target RPM below minimum threshold.")
            return
        self._log.info("attempting stall recovery…")
        direction = 1 if self._target_rpm >= 0 else -1
        ramp_speeds = range(20, 80, 10)  # Try 20% to 70% in steps
        for s in ramp_speeds:
            speed = direction * s
            self._log.info(f"Trying recovery speed: {speed}%")
            await self._apply_pwm_async(speed)
            await asyncio.sleep(0.3)
            # if FG pulses resume, _handle_recovery() will be called
            # so we just exit and let it resume PID
            now = dt.now()
            if self._last_tick_dt and (now - self._last_tick_dt).total_seconds() < 0.3:
                self._log.info("Motor response detected during recovery.")
                return
        self._log.warning("Recovery ramp failed — no FG detected.")

    def _handle_recovery(self):
        self._log.info("motor has recovered and is moving again.")
        if self._target_rpm > 0.0:
            # smooth re-ramp to target RPM after recovery
            asyncio.run_coroutine_threadsafe(
                self._resume_after_recovery(), self._loop
            )

    async def _resume_after_recovery(self):
        self._log.info("smoothly resuming PID after recovery.")
        await self._apply_pwm_async(0) # reset to zero (optional)
        await asyncio.sleep(0.2)
        self.set_target_rpm(self._target_rpm)

    def _check_stall(self):
        now = dt.now()
        # cooldown window: ignore stall detection immediately after RPM changes
        if self._last_target_rpm_set_dt:
            since_change_ms = (now - self._last_target_rpm_set_dt).total_seconds() * 1000
            if since_change_ms < self._stall_detection_grace_period_ms:
                return  # skip checking
        # determine if we should be moving
        target_moving = (
            (self.is_closed_loop_enabled and self._target_rpm > 0.0 and abs(self._target_rpm) >= self._deadband_rpm)
            or
            (not self.is_closed_loop_enabled and self._target_speed and abs(self._target_speed) >= 10)
        )
        if not target_moving:
            self._is_stalled = False
            return
        if not hasattr(self, '_last_tick_dt') or self._last_tick_dt is None:
            return  # haven’t seen a pulse yet
        elapsed_ms = (now - self._last_tick_dt).total_seconds() * 1000
        if elapsed_ms > self._stall_timeout_ms:
            if not self._is_stalled:
                self._is_stalled = True
                self._log.warning("STALL DETECTED: no FG pulse in {:.1f}ms.".format(elapsed_ms))
                if self._on_stall_callback:
                    self._on_stall_callback()
        else:
            if self._is_stalled:
                self._is_stalled = False
                self._log.info("RECOVERY DETECTED: FG pulses resumed.")
                # reset speed and PID state to prevent jerk on recovery
                self._current_speed = 0
                self._reset_pid_state()
                if self._on_recovery_callback:
                    self._on_recovery_callback()

    def _reset_pid_state(self):
        '''
        Reset PID control state variables.
        '''
        self._integral_error = 0
        self._last_error = 0

    async def _stall_monitor(self):
        while True:
            self._check_stall()
            await asyncio.sleep(0.1) # check every 100 ms

    def enable_closed_loop(self, enable=True):
        self._closed_loop_enabled = enable

    def set_target_rpm(self, rpm, target_mm=0):
        if not self._closed_loop_enabled:
            raise Exception("closed-loop mode must be enabled for RPM control.")
        elif rpm is None:
            raise ValueError("null rpm argument.")
        elif rpm == 0 or abs(rpm) < self._deadband_rpm: # stop if RPM is zero or within deadband
            if self._verbose:
                self._log.info("target RPM ({}) is 0 or below viable threshold.".format(rpm))
            self.stop()
            self._target_rpm = 0.0
            return
        if self._slew_limiter_enabled:
            limited_rpm = int(self._rpm_limiter.limit(rpm))
            if self._verbose and ( rpm != limited_rpm ):
                self._log.info(Style.DIM + "RPM {} was limited to {}.".format(rpm, limited_rpm))
            rpm = limited_rpm
        # setup target distance tracking
        if target_mm > 0:
            self._initial_distance = self.cumulative_distance_mm
            self._target_distance = self._initial_distance + target_mm
        else:
            self._initial_distance = None
            self._target_distance  = None
        self._distance_target_reached = False
        self._target_rpm = rpm
        self._last_target_rpm_set_dt = dt.now()
        if self._verbose:
            self._log.info("closed loop target set to: " + Fore.GREEN + Style.DIM + "{} RPM".format(rpm))
        # apply kickstart if starting from zero and below threshold
        if self.measured_rpm == 0 and abs(rpm) < self._kickstart_speed:
            self._log.info("applying kickstart for low RPM startup…")
            direction = 1 if rpm >= 0 else -1
            kick_speed = direction * self._kickstart_speed
            asyncio.run_coroutine_threadsafe(self._apply_kickstart(kick_speed), self._loop).result()
            self._reset_pid_state()
            time.sleep(0.1)
        # start feedback coroutine if not already running
        if self._feedback_task is None:
            try:
                self._log.debug("scheduling feedback coroutine…")
                future = asyncio.run_coroutine_threadsafe(self._run_rpm_feedback(), self._loop)
                self._feedback_task = future
                self._log.debug("feedback coroutine started successfully.")
            except Exception as e:
                self._log.error("{} raised starting feedback coroutine: {}".format(type(e), e))

    async def _apply_kickstart(self, speed):
        self._log.info(Fore.WHITE + Style.BRIGHT + "apply kickstart.")
        await self._apply_pwm_async(speed)
        await asyncio.sleep(0.3) # brief burst

    async def _run_rpm_feedback(self):
        self._log.info(Fore.YELLOW + "starting PID RPM feedback control loop…")
        self._integral_error = 0
        self._last_error = 0
        self._last_target_rpm = None
        dt = self._feedback_interval
        max_adjustment = 10  # max PID output adjustment (%)
        try:
            counter = itertools.count(1)
            while abs(self._target_rpm) > 0.0:
                # Reset integral if target RPM sign changes
                if self._last_target_rpm is None or (self._last_target_rpm * self._target_rpm < 0):
                    self._integral_error = 0
                self._last_target_rpm = self._target_rpm
                # check distance target if set
                if self._target_distance is not None and not self._distance_target_reached:
                    if self._check_distance_target_reached():
                        self._log.info(
                            Fore.MAGENTA + "Target distance {:.1f} mm reached, stopping motor.".format(
                                abs(self._target_distance - self._initial_distance)
                            )
                        )
                        self.stop()
                        break
                adjustment, error = self._compute_pid_adjustment(dt, max_adjustment)
                new_speed = self._compute_new_speed(adjustment)
                if self._verbose:
                    if next(counter) % 4 == 0:
                        self._log.info(
                            "PID control target set to: "
                            + Fore.GREEN + "{} RPM ({} measured); ".format(self._target_rpm, int(self.measured_rpm))
                            + Fore.YELLOW + "distance: {:.1f}mm".format(self.cumulative_distance_mm)
#                           + Fore.WHITE + "; Kp={:.2f}; Ki={:.2f}; Kd={:.2f}; ".format(self._kp, self._ki, self._kd)
#                           + Fore.CYAN + Style.DIM + "error={:.2f}, i={:.2f}, adj={:.2f}, new_speed={:.2f}".format(error, self._integral_error, adjustment, new_speed)
                        )
                await self._apply_pwm_async(new_speed)
                await asyncio.sleep(dt)
        except asyncio.CancelledError:
            self._log.debug("RPM feedback task cancelled.")

    def _check_distance_target_reached(self):
        current = self.cumulative_distance_mm
        start = self._initial_distance
        target = self._target_distance
        if (self._target_rpm > 0 and current >= target) or (self._target_rpm < 0 and current <= target):
            self._distance_target_reached = True
            return True
        return False

    def _compute_pid_adjustment(self, dt, max_adjustment):
        error = self._target_rpm - self.measured_rpm
        self._integral_error += error * dt
        derivative = (error - self._last_error) / dt if dt > 0 else 0
        self._last_error = error
        adjustment = self._kp * error + self._ki * self._integral_error + self._kd * derivative
        adjustment = self._clamp(adjustment, -max_adjustment, max_adjustment)
        if abs(adjustment) > 20:
            self._log.warning("Huge PID adjustment: {:.2f}".format(adjustment))
        return adjustment, error

    def _compute_new_speed(self, adjustment):
        new_speed = self._clamp(self._current_speed + adjustment, -100, 100)
        if self._fixed_deadband:
            new_speed = self._apply_deadband(new_speed, self._deadband)
        else:
            threshold = max(1, 5 * (self._target_rpm / 100))
            new_speed = self._apply_deadband(new_speed, threshold)
        return new_speed

    def _clamp(self, value, min_val, max_val):
        return max(min_val, min(max_val, value))

    def _apply_deadband(self, value, threshold):
        return 0 if abs(value) < threshold else value

    def _stop_rpm_control(self):
        if self.is_closed_loop_enabled and self._feedback_task:
            self._feedback_task.cancel()
            self._feedback_task = None
            self._log.debug("closed-loop RPM control disabled.")

    def _fg_callback(self, gpio, level, tick):
        '''
        Callback for open or closed loop control.
        '''
        if self._verbose:
            self._log.info(Style.DIM + "tick: {}".format(tick))
        self._pulse_count += 1
        self._last_tick_dt = dt.now()  # record time of this tick
        if self._last_tick is not None:
            interval = pigpio.tickDiff(self._last_tick, tick)  # in microseconds
            self._pulse_intervals.append(interval)
            if len(self._pulse_intervals) > self._max_interval_buffer:
                self._pulse_intervals.pop(0)
            self._calculate_rpm()
        # update cumulative distance based on direction
        distance_per_pulse = self._circumference_mm / self._pulses_per_output_rev
        if self._direction == self.DIRECTION_FORWARD:
            self._cumulative_distance_mm += distance_per_pulse
        else:
            self._cumulative_distance_mm -= distance_per_pulse
        self._check_stall()
        self._last_tick = tick

    def _calculate_rpm(self):
        if not self._pulse_intervals:
            self._measured_rpm = 0.0
            self._log.info("No pulse intervals available, RPM set to 0.")
            return
        avg_interval_us = sum(self._pulse_intervals) / len(self._pulse_intervals)
        if avg_interval_us == 0:
            self._measured_rpm = 0.0
            self._log.info("Average pulse interval is zero, RPM set to 0.")
            return
        pulses_per_minute = 60_000_000 / avg_interval_us
        output_shaft_rpm = pulses_per_minute / self._pulses_per_output_rev
        # adjust RPM sign based on current direction
        if self._direction == self.DIRECTION_REVERSE:
            output_shaft_rpm = -abs(output_shaft_rpm)
        else:
            output_shaft_rpm = abs(output_shaft_rpm)
        self._measured_rpm = output_shaft_rpm
        self._log.debug("measured RPM: {:.2f}".format(self._measured_rpm))

    def get_distance_future(self):
        '''
        Used to create blocking return while running the task.
        '''
        return self._distance_future

    def _start_loop(self):
        self._log.info(Fore.GREEN + 'starting asyncio loop…')
        asyncio.set_event_loop(self._loop)
        self._loop.run_forever()

    def stop_loop(self):
        '''
        Stops the asyncio loop, to be used upon closing.
        '''
        self._log.info(Fore.YELLOW + 'stopping asyncio loop…')
        self._loop.stop()
        self._log.info(Fore.YELLOW + "event loop stopped..")

    def reset_distance(self):
        """
        Reset odometry tracking to zero.
        """
        self._start_time = dt.now()
        self._stop_time = None
        self._pulse_count = 0  # reset session pulse count for get_distance_mm()
        self._log.info("distance reset.")

    def get_distance_mm(self):
        """
        Calculate distance traveled in millimeters based on elapsed time
        and current speed using the calibration constant.
        """
        if self.is_closed_loop_enabled:
            rotations = self._pulse_count / self._pulses_per_output_rev
            self._log.info(Style.DIM + "{:.2f} wheel rotations.".format(rotations))
            return rotations * self._circumference_mm
        else:
            if self._start_time is None:
                return 0.0
            end_time = self._stop_time or dt.now()
            elapsed = (end_time - self._start_time).total_seconds()
            # Distance = speed (%) * elapsed time (sec) * calibration constant (mm/%/sec)
            distance = abs(self._target_speed) * elapsed * self.CALIBRATION_MM_PER_PERCENT_PER_SEC
            return distance

    def set_speed(self, speed, target_mm=0, poll_delay=0.01):
        """
        Set motor speed (-100 to 100) in open loop. If target_mm > 0, run until distance
        reached then stop.
        """
        # clamp speed to [-100, 100]
        speed = max(-100, min(100, speed))
        if speed == 0:
            self._stop_time = dt.now()
            self._target_speed = 0
            self._apply_pwm_sync(0)
            return
        if self._slew_limiter_enabled:
            speed = self._speed_limiter.process(speed)
        if target_mm > 0:
            self.reset_distance()
            self._target_speed = speed
            self._start_time = dt.now()
            self._apply_pwm_sync(speed)
            self._log.info("speed set to {:+.2f}%, running to target {:.1f}mm".format(speed, target_mm))
            # run stop-at-distance coroutine in background
            self._log.info("scheduling feedback coroutine.")
            future = asyncio.run_coroutine_threadsafe(
                self._stop_at_distance_async(target_mm, poll_delay), self._loop
            )
            # holds a concurrent.futures.Future representing the scheduled coroutine.
            self._distance_future = future
        else:
            self._target_speed = speed
            self._start_time = dt.now()
            self._apply_pwm_sync(speed)
            self._distance_future = None
        self._calculate_rpm()

    async def _stop_at_distance_async(self, target_mm, poll_delay):
        self._log.info(Fore.MAGENTA + "monitoring distance to stop at {:.1f}mm".format(target_mm))
        try:
            counter = itertools.count(1)
            while self.get_distance_mm() < target_mm:
                if next(counter) % 10 == 0:  # print every 10th iteration
                    self._log.info(Fore.MAGENTA + "current distance: {:.1f}mm".format(self.get_distance_mm()))
                await asyncio.sleep(poll_delay)
        except Exception as e:
            self._log.error("{} raised by _stop_at_distance_async: {}".format(type(e), e))
        finally:
            self.stop()
            self._log.info(Fore.MAGENTA + "target distance {:.1f}mm reached, motor stopped.".format(target_mm))

    def _apply_pwm_sync(self, speed):
        '''
        Synchronously ramps motor speed to target, sending PWM signals with delay between steps.
        '''
        for _speed in self._ramp_speeds(speed):
            self._send_pwm(_speed)
            if self._verbose:
                self._log.info("_apply_pwm_sync() ramping to: {:3.2f} (direction: {})".format(_speed,
                        "FORWARD" if self._direction == self.DIRECTION_FORWARD else "REVERSE"))
            time.sleep(self._accel_delay)
        if self._verbose:
            self._log.info("_apply_pwm_sync() final speed: {:3.2f} (direction: {})".format(_speed,
                    "FORWARD" if self._direction == self.DIRECTION_FORWARD else "REVERSE"))

    async def _apply_pwm_async(self, speed):
        '''
        Asynchronously ramps motor speed to target, sending PWM signals with async delays between steps.
        This is the older version that uses _ramp_speeds().
        '''
        async def async_sleep():
            await asyncio.sleep(self._accel_delay)
        for _speed in self._ramp_speeds(speed):
            self._send_pwm(_speed)
            if self._verbose:
                self._log.info("_apply_pwm_async() ramping to: {:3.2f} (direction: {})".format(_speed,
                        "FORWARD" if self._direction == self.DIRECTION_FORWARD else "REVERSE"))
            await async_sleep()
        if self._verbose:
            self._log.info("_apply_pwm_async() final speed: {:3.2f} (direction: {})".format(_speed,
                    "FORWARD" if self._direction == self.DIRECTION_FORWARD else "REVERSE"))

    def _send_pwm(self, speed):
        '''
        Send PWM and direction to the motor, using signed speed input.
        '''
        self._direction = self.DIRECTION_FORWARD if speed >= 0 else self.DIRECTION_REVERSE
        self._current_speed = speed
        abs_speed = abs(speed)
        if abs_speed == 0:
            self._pwm_controller.stop_pwm()
            self._pi.write(self._dir_pin, self.DIRECTION_FORWARD)  # default direction when stopped
            if self._verbose:
                self._log.info("motor stopped.")
        else:
            self._pi.write(self._dir_pin, self._direction)
            self._pwm_controller.set_pwm(abs_speed)
            if self._verbose:
                self._log.info("speed set to {speed:+.2f}% (direction: {dir}, duty inverted)".format(
                    speed=speed, dir="FORWARD" if self._direction == self.DIRECTION_FORWARD else "REVERSE"))

    def _ramp_speeds(self, target_speed):
        '''
        Generator yielding intermediate speeds from current to target with slew limiting.
        '''
        target_speed = self._clamp(target_speed, -100, 100)
        if self._max_slew_rate <= 0:
            yield target_speed
            return
        delta = target_speed - self._current_speed
        if abs(delta) <= self._max_slew_rate:
            yield target_speed
            return
        step = self._max_slew_rate if delta > 0 else -self._max_slew_rate
        current = self._current_speed
        while abs(target_speed - current) > abs(step):
            current += step
            current = self._clamp(current, min(target_speed, current), max(target_speed, current))
            yield current
        yield target_speed

    def accelerate(self, start_speed, end_speed, step_speed, delay=None):
        """
        Gradually accelerate from start_speed to end_speed, with set
        delay between steps.
        """
        if delay is None:
            delay = self._accel_delay
        start_speed = max(-100, min(100, start_speed))
        end_speed = max(-100, min(100, end_speed))
        step_speed = abs(step_speed)
        if start_speed == end_speed:
            if self._closed_loop_enabled:
                self.set_target_rpm(start_speed)
            else:
                self.set_speed(start_speed)
            return
        self._log.info("ramp speed from {} to {} with step {} and delay {}.".format(start_speed, end_speed, step_speed, delay))
        direction = 1 if end_speed > start_speed else -1
        current_speed = start_speed
        while (direction == 1 and current_speed <= end_speed) or (direction == -1 and current_speed >= end_speed):
            if self._closed_loop_enabled:
                self.set_target_rpm(current_speed)
            else:
                self.set_speed(current_speed)
            time.sleep(delay)
            current_speed += direction * step_speed
        self.set_speed(end_speed)

    def stop(self):
        '''
        Stop motor immediately.
        '''
        self._log.debug("stopping motor…")
        self._close_distance_future()
        self._stop_rpm_control()
        self._rpm_limiter.reset()
        self._speed_limiter.reset()
        self._reset_pid_state()
        self._target_rpm = 0.0
        self.set_speed(0)
        self._reset_fg_state()
        if self._verbose:
            self._log.info("motor stopped.")

    def _close_distance_future(self):
        if self._distance_future and not self._distance_future.done():
            self._log.info("cancelling distance monitoring task…")
            self._distance_future.cancel()
            self._distance_future = None

    def _reset_fg_state(self):
        """
        Reset all FG (feedback) pulse tracking state.
        """
        self._measured_rpm = 0.0 # reset measured RPM
        self._last_tick = None
        self._pulse_intervals.clear()
        self._pulse_count = 0 # optional, if you track odometry elsewhere

    def close(self):
        """
        Stop motor and clean up resources.
        """
        self.stop()
        if self._callback:
            self._log.info("cancelling callback…")
            self._callback.cancel()

        # cancel all tasks
        for task in asyncio.all_tasks(self._loop):
            task.cancel()

        # stop the event loop
        self._loop.call_soon_threadsafe(self._loop.stop)
        self._loop_thread.join()

        self._pi.write(self._dir_pin, 0)
        self._pwm_controller.stop_pwm()
        self._log.info("motor closed.")

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

class SlewLimiter:
    def __init__(self, max_delta_per_sec, safe_threshold=10):
        '''
        :param max_delta_per_sec: Maximum allowed change per second (e.g. RPM/sec or %/sec)
        :param safe_threshold: Minimum magnitude below which direction changes are allowed
        '''
        self.max_delta_per_sec = float(max_delta_per_sec)
        self.safe_threshold = float(safe_threshold)
        self._last_value = None
        self._last_time = None
        self.reset()

    def reset(self, value=None):
        '''
        Resets the limiter to a specific value (or clears it).
        '''
        self._last_value = float(value) if value is not None else None
        self._last_time = dt.now()

    def limit(self, new_value):
        now = dt.now()
        new_value = float(new_value)
        if self._last_value is None:
            self._last_value = new_value
            self._last_time = now
            return new_value
        elapsed = (now - self._last_time).total_seconds()
        if elapsed <= 0.0:
            return self._last_value  # no time has passed
        # Prevent direction change unless speed is low
        if ( new_value * self._last_value < 0 and abs(self._last_value) > self.safe_threshold ):
            return self._last_value  # block reversal
        # calculate allowed delta
        max_delta = self.max_delta_per_sec * elapsed
        delta = new_value - self._last_value
        if abs(delta) > max_delta:
            delta = max_delta if delta > 0 else -max_delta
            new_value = self._last_value + delta
        self._last_value = new_value
        self._last_time = now
        return new_value

#EOF
