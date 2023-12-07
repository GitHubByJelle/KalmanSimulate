"""
Autonomous Robotic Systems Assignment 5
Author: Jelle Jansen
Date: 23-03-2021

Landmarks.py

Landmark class that defines a landmark.

"""

class Landmark(object):
    def __init__(self, position, max_r):
        self.position = position
        self.max_r = max_r
        self.r = max_r