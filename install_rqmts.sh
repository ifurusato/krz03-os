#!/bin/bash

#sudo apt install python3-pip

then:

# numpy:        https://numpy.org/ with:
sudo apt install python3-numpy

# psutil:       https://pypi.org/project/psutil/ with:
sudo apt install python3-psutil

# pyyaml:       https://pypi.org/project/PyYAML/ with:         
sudo apt install python3-yaml

# colorama:     https://pypi.org/project/colorama/ with:
sudo apt install python3-colorama

# pytest:       https://docs.pytest.org/en/stable/getting-started.html with:
sudo apt install python3-pytest

# smbus2:       https://pypi.org/project/smbus2/ with:         
sudo apt install python3-smbus2

# rgbmatrix5x5: https://github.com/pimoroni/rgbmatrix5x5-python.git with:
sudo pip3 install rgbmatrix5x5 --break-system-packages

# matrix11x7:   https://github.com/pimoroni/matrix11x7-python/tree/master with:
sudo pip3 install matrix11x7 --break-system-packages

# icm20948:     https://pypi.org/project/icm20948/ with:
sudo pip3 install icm20948 --break-system-packages

# pyquaternion: https://pypi.org/project/pyquaternion/ with:
sudo pip3 install pyquaternion --break-system-packages

# IO Expander:  https://pypi.org/project/pimoroni-ioexpander/  with:
sudo pip3 install pimoroni-ioexpander --break-system-packages

# gpiodevice:   https://pypi.org/project/gpiodevice/ with:
sudo pip3 install gpiodevice --break-system-packages

# PAA5100JE:    https://github.com/pimoroni/pmw3901-python with:
sudo pip3 install pmw3901 --break-system-packages

# dill:         https://pypi.org/project/dill/ with:
sudo pip3 install dill --break-system-packages

# evdev:        https://pypi.org/project/evdev/  with:
sudo pip3 install evdev --break-system-packages
    
# for the VL53L5CX and 1.3" TFT display (used for its demo):

# VL53L1CX:     https://github.com/pimoroni/vl53l1x-python with:
sudo pip3 install vl53l1cx --break-system-packages

# VL53L5CX:     https://github.com/pimoroni/vl53l5cx-python with:
sudo pip3 install vl53l5cx-ctypes --break-system-packages

# ST7789:         with:
sudo pip3 install st7789 --break-system-packages

# Python Image Library (PIL) with:
sudo pip3 install --upgrade Pillow --break-system-packages

# matplotlib     with:
sudo pip3 install matplotlib --break-system-packages

