"""
Autonomous Robotic Systems Assignment 5
Author: Jelle Jansen
Date: 23-03-2021

Simulation.py

Simulates a mobile robot moving in an evironment with possible obstacles.
"""

import pygame

from Simulation.visualization import Visualizer
import numpy as np
import time


# Simulate a robot that follows a trajectory
def simulate_trajectory(robot, visualize=True):
    # Delta_t for calculations and visualizations
    delta_t = 1.0 / 7

    trajectory = [[70,0],[40,-np.pi/6], [60,np.pi/100], [25,np.pi/8], [40,np.pi/90], [35,np.pi/6],
                  [20,-np.pi/70], [39,-np.pi/6], [80,0], [42, -np.pi/15], [40, -np.pi/15], [40, -np.pi/11]]

    start_pause = True

    if visualize:
        # Set-up visualizations
        pygame.init()
        pygame.display.set_caption('Simulation Robot | ARS')

        width, height = robot.room_size
        win = pygame.display.set_mode((width, height))

        vis = Visualizer(win, robot, robot.obstacles, robot.landmarks)
        vis.visualize()
        time.sleep(2)
        clock = pygame.time.Clock()

    while len(trajectory) > 0:
        if visualize:
            # clock.tick(int(1 / delta_t))
            clock.tick(20) # For more FPS (faster simulations)

        robot.move_trajectory(delta_t, trajectory[0][1])
        trajectory[0][0] -= 1

        if trajectory[0][0] == 0:
            trajectory.remove(trajectory[0])
            if len(trajectory) == 0:
                time.sleep(10)

        if visualize:
            vis.visualize()

            # temporary, give a chance to start recording
            # if start_pause:
            #  start_pause = False
            #  time.sleep(10)

            for event in pygame.event.get():

                if event.type == pygame.QUIT:
                    running = False

# Simulate a non-automatic robot
def simulate(robot, visualize=True):
    # Delta_t for calculations and visualizations
    delta_t = 1.0 / 7

    start_pause = True

    if visualize:
        # Set-up visualizations
        pygame.init()
        pygame.display.set_caption('Simulation Robot | ARS')

        width, height = robot.room_size
        win = pygame.display.set_mode((width, height))

        vis = Visualizer(win, robot, robot.obstacles, robot.landmarks)
        vis.visualize()

        clock = pygame.time.Clock()

    running = True
    while running:
        if visualize:
            clock.tick(int(1 / delta_t))

        robot.move(delta_t)

        if visualize:
            vis.visualize()

            # temporary, give a chance to start recording
            # if start_pause:
            #  start_pause = False
            #  time.sleep(10)

            for event in pygame.event.get():

                if event.type == pygame.QUIT:
                    running = False
