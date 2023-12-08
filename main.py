"""
Autonomous Robotic Systems Assignment 5
Author: Jelle Jansen
Date: 23-03-2021

Main.py

Mobile Robot Simulator in which the robot moves in an
environment with landmarks. The robot has no collision with the walls.

"""
from Simulation.simulation import simulate
from Simulation.simulation import simulate_trajectory
from Robot.obstacle import Obstacle
from Robot.landmarks import Landmark
from Robot.robot import Robot
import numpy as np


if __name__ == '__main__':

    # Create environment
    obstacles = [Obstacle([50, 50, 700, 50], 'wall'),
                 Obstacle([700, 50, 700, 370], 'wall'),
                 Obstacle([700, 370, 50, 370], 'wall'),
                 Obstacle([50, 370, 50, 50], 'wall'),
                 Obstacle([150, 50, 150, 270], 'wall'),
                 Obstacle([250, 370, 250, 170], 'wall'),
                 Obstacle([250, 170, 350, 170], 'wall'),
                 Obstacle([450, 50, 450, 270], 'wall'),
                 Obstacle([350, 270, 550, 270], 'wall')]
    max_r = 125
    landmarks = [Landmark([50, 50], max_r), Landmark([700, 50], max_r), Landmark([700, 370], max_r), Landmark([50, 370], max_r),
                 Landmark([150, 50], max_r), Landmark([150, 270], max_r),
                 Landmark([250, 370], max_r), Landmark([250, 170], max_r), Landmark([350, 170], max_r),
                 Landmark([450, 50], max_r), Landmark([450, 270], max_r), Landmark([350, 270], max_r),
                 Landmark([550, 270], max_r)]

    # Create robot
    robot = Robot([100, 100], np.pi / 2, 75, 20, obstacles, landmarks, 20, [750, 420])

    # Simulate either a robot on a predifined traject or a keyboard-controlled robot
    simulate_trajectory(robot)
    # simulate(robot)