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

import math

# Step 1: Normalize the distance to a value between 0.0 and 1.0
def normalize_distance(distance, min_distance=7.0, max_distance=30.0):
    return (distance - min_distance) / (max_distance - min_distance)

# Step 2: Apply the sigmoid function to introduce non-linearity
def sigmoid(x, k=10.0, x0=0.5):
    return 1 / (1 + math.exp(-k * (x - x0)))

# Step 3: Main function to compute motor speed based on distance
def motor_speed(distance, min_distance=7.0, max_distance=30.0, k=10.0, x0=0.5):
    # Normalize the distance first
    normalized_distance = normalize_distance(distance, min_distance, max_distance)
    
    # Then apply the sigmoid function to adjust motor speed non-linearly
    speed = sigmoid(normalized_distance, k, x0)
    
    return speed

# Test script to check motor speeds for different distances
def test_motor_speed():
    # Test distances ranging from the minimum to the maximum
    distances = [7, 8, 10, 12, 15, 20, 25, 30]  # You can change or extend this list

    # Parameters for the motor speed calculation
    min_distance = 7.0
    max_distance = 30.0
    k = 10.0   # Adjust the sharpness of the sigmoid curve
    x0 = 0.5   # The midpoint of the sigmoid curve (adjust as needed)

    # Print the results
    print("Testing Motor Speed for Different Distances:")
    print("-------------------------------------------------")
    for dist in distances:
        speed = motor_speed(dist, min_distance, max_distance, k, x0)
        print(f"Distance: {dist} cm -> Motor Speed: {speed:.4f}")

# Run the test
if __name__ == "__main__":
    test_motor_speed()

