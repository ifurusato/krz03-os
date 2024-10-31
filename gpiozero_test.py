#!/usr/bin/env python3

from gpiozero import DigitalInputDevice
from signal import pause

# Define GPIO pins for the Hall Effect encoder
gpio_a = 16  # A channel
gpio_b = 26  # B channel

# Setup DigitalInputDevices for Hall Effect sensor channels
encoder_a = DigitalInputDevice(gpio_a)
encoder_b = DigitalInputDevice(gpio_b)

# Callback functions to handle encoder events
def callback_a():
    print("Encoder A triggered")

def callback_b():
    print("Encoder B triggered")

# Attach edge detection callbacks directly to GPIO pins
encoder_a.when_activated = callback_a
encoder_b.when_activated = callback_b

# Use pause() to keep the program running
try:
    print("Listening for encoder signals. Press Ctrl+C to exit.")
    pause()  # Keep the script running
except KeyboardInterrupt:
    print("Exiting...")

