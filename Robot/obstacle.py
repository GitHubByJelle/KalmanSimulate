"""
Autonomous Robotic Systems Assignment 5
Author: Jelle Jansen
Date: 22-02-2021

Obstacle.py

Obstacle class that defines an obstacle, such as a wall.

"""


class Obstacle:
    def __init__(self, params, sort):
        self.sort = sort

        if sort == 'wall':
            self.from_coords = [params[0], params[1]]
            self.to_coords = [params[2], params[3]]
        else:
            raise Exception('There is not obstacle called {}.'.format(sort))