*******************************************
KRZOS: Operating System for the KRZ03 Robot
*******************************************

**KRZOS** provides the core functionality of the *KRZ03 Robot*, a Raspberry
Pi-based robot OS written in Python 3.

Note: this document is a work-in-progress and does not necessarily reflect the
current state of the project.

.. figure:: https://service.robots.org.nz/wiki/attach/KRZ03/krz03.jpg
   :width: 1200px
   :align: center
   :alt: The KRZ03 Robot

   The KRZ03 Robot.

|

Background
**********

The *KRZOS* library provides essential support designed as the basis of a
`Behaviour-Based Systems (BBS) <https://en.wikipedia.org/wiki/Behavior-based_robotics>`_.
This library is relatively "low-level" and, in theory, could be used for any Python 3 
based robot.

The basic function is for sensors to act as "Publishers" in a "Publish-Subscribe" model,
firing event-laden messages onto an asynchronous message bus. Subscribers to the bus can
filter which event types they are interested in. The flow of messages are thus filtered
through the Subscribers, who pass on to an Arbitrator messages they have consumed. Once all
Subscribers have acknowledged a message it is passed to a Garbage Collector (a specialised
Subscriber).

Each event type has a fixed priority. The Arbitrator receives this flow of events and
passes along to a Controller the highest priority event for a given clock cycle (typically
50ms/20Hz). The Controller takes the highest priority event and for that clock cycle
initiates any Behaviours registered for that event type.

For example, a Subscriber that filters on bumper events receives a message whose event
type is Event.BUMPER_PORT (the left/port side bumper has been triggered). This Subscriber
passes the Payload of its Message to the Arbitrator. Since a bumper press is a relatively
high priority event it's likely that it will be the highest priority and is therefore
passed on to the Controller.  If an avoidance Behaviour &mdash; let's call it AVOID_PORT
&mdash; has been registered with the Controller, it is called and the robot will begin
whatever the AvoidPort behaviour entails, perhaps stopping, backing up while turning
clockwise, then proceeding forward again on a new trajectory.


Software Features
*****************

* message and event handling
* an asynchronous message bus that forms the basis of a `Subsumption Architecture <https://en.wikipedia.org/wiki/Subsumption_architecture>`_ [#f1]_, with an "exactly-once' message delivery guarantee
* YAML-based configuration
* timestamped, multi-level, colorised [#f2]_ logging
* written in Python 3 (currently 3.11.2)

.. [#f1] Uses finite state machines, an asynchronous message bus, an arbitrator and controller for task prioritisation.
.. [#f2] Colorised console output tested only on Unix/Linux operating systems.


Hardware Features
*****************

The specific features of the KRZ03 robot's hardware include:

* Raspberry Pi Zero 2 W as main controller
* Pimoroni Motor 2040 as the motor controller
* 4x Pololu N20 250:1 Micro Metal Gearmotors (HPCB 6V #5197) with built-in encoders
* 4x JST SH 6 pin cables to connect the motors to the controllers ("reversed" or "opposite")
* Pimoroni Breakout Garden for Raspberry Pi, 6 slots
* Pimoroni Tiny FX W for control of audio and LED lights
* PIR Stick for Tiny FX
* Pimoroni Adhesive Backed Mini Speaker 8Ω (1W)
* subminiature SPDT toggle switch to switch between an external supply and the battery
* toggle switch and a pushbutton connected to GPIO pins as input devices
* chassis made from 3mm and 5mm black Delrin plastic, using LynxMotion M3 spacers
* set of four Nexus Robot 48mm Steel Mecanum Wheels
* Pololu 5V, 5.5A Step-Down Voltage Regulator D36V50F5
* Makita 12V 2Ah power tool battery
* Makita YL00000003 12V max CXT Power Source (a power clip compatible with the battery)
* 19V laptop power supply as an external power supply

Sensors include:

* 3x Pololu Distance Sensors with Pulse Width Output, 50cm Max (#4064)
* PAA5100JE Near Optical Flow SPI Breakout
* ICM20948 9DoF Motion Sensor Breakout mounted on a 6x10x140mm Delrin mast
* VL53L5CX 8x8 Time of Flight (ToF) Array Sensor Breakout
* VL53L1X Time of Flight (ToF) Sensor Breakout
* pair of 5x5 RGB Matrix Breakouts
* 11x7 LED Matrix Breakout
* ADS1015 used for monitoring the battery and regulators
* Adafruit CH334F Mini 4-Port USB Hub Breakout


Requirements
************

This library requires Python 3.8.5 or newer. It's currently being written using 
Python 3.11.2. Some portions (modules) of the KRZOS code will only run on a 
Raspberry Pi, though KRZOS Core should function independently of the various Pi 
libraries.

KRZOS requires installation of a number of dependencies (support libraries). 
There is currently no dependency management set up for this project.

First:

  sudo apt install python3-pip

then:

* numpy:        https://numpy.org/
    with:         sudo apt install python3-numpy
* psutil:       https://pypi.org/project/psutil/
    with:         sudo apt install python3-psutil
* pyyaml:       https://pypi.org/project/PyYAML/
    with:         sudo apt install python3-yaml
* colorama:     https://pypi.org/project/colorama/
    with:         sudo apt install python3-colorama
* pytest:       https://docs.pytest.org/en/stable/getting-started.html
    with:         sudo apt install python3-pytest
* smbus2:       https://pypi.org/project/smbus2/
    with:         sudo apt install python3-smbus2
* rgbmatrix5x5: https://github.com/pimoroni/rgbmatrix5x5-python.git
    with:         sudo pip3 install rgbmatrix5x5 --break-system-packages
* matrix11x7:   https://github.com/pimoroni/matrix11x7-python/tree/master
    with:         sudo pip3 install matrix11x7 --break-system-packages
* icm20948:     https://pypi.org/project/icm20948/
    with:         sudo pip3 install icm20948 --break-system-packages
* pyquaternion: https://pypi.org/project/pyquaternion/
*   with:         sudo pip3 install pyquaternion --break-system-packages
* IO Expander:  https://pypi.org/project/pimoroni-ioexpander/  
    with:         sudo pip3 install pimoroni-ioexpander --break-system-packages
* gpiodevice:   https://pypi.org/project/gpiodevice/
    with:         sudo pip3 install gpiodevice --break-system-packages
* PAA5100JE:    https://github.com/pimoroni/pmw3901-python
    with:         sudo pip3 install pmw3901 --break-system-packages

for the VL53L5CX and 1.3" TFT display (used for its demo):

* VL53L1CX:     https://github.com/pimoroni/vl53l1x-python
    with:         sudo pip3 install vl53l1cx --break-system-packages
* VL53L5CX:     https://github.com/pimoroni/vl53l5cx-python
    with:         sudo pip3 install vl53l5cx-ctypes --break-system-packages
* ST7789:
    with:        sudo pip3 install st7789 --break-system-packages
* Pyhon Image Library (PIL)
    with:        sudo pip3 install --upgrade Pillow --break-system-packages
* matplotlib
    with:        sudo pip3 install matplotlib --break-system-packages

The pimoroni-iopxpander library supports the Pimoroni Breakout Garden 
IO Expander as well as the Encoder and Potentiometer.

The original PiconZero library has been included and significantly refactored as
a Python class, so it is not an external dependency.


Status
******

Early days. The Phase 0 hardware is largely complete and migration and conversion
of the `KROS-Core <https://github.com/ifurusato/kros-core/tree/main>`_ is being used
as the basis of MROS, which was forked to create KRZOS.

.. note::

   This project is currently in a very preliminary state.

   The project is being exposed publicly so that those interested can follow its progress.


Support & Liability
*******************

This project comes with no promise of support or acceptance of liability. Use at
your own risk.


Copyright & License
*******************

All contents (including software, documentation and images) Copyright 2020-2025
by Murray Altheim. All rights reserved.

Software and documentation are distributed under the MIT License, see LICENSE
file included with project.

