"""
Autonomous Robotic Systems Assignment 5
Author: Jelle Jansen
Date: 23-03-2021

Visualization.py

Visualizes a mobile robot in an environment with landmarks and
handles keyboard input using Pygame. Therefore the pygame
library must be imported and initialized.

"""

import numpy as np
import pygame


class Visualizer(object):
    def __init__(self, surface, robot, obstacles, landmarks):
        # Initialise, robot, sensors, surface, sensors and obstacles
        self.surface = surface
        self.robot = robot
        self.obstacles = obstacles
        self.landmarks = landmarks

        # Get width and height
        self.width, self.height = surface.get_size()

        # Make font
        pygame.font.init()
        self.sensor_font = pygame.font.SysFont('Calibri', int(robot.r * 0.4))

    def visualize(self):
        # Make the visualization
        self.draw_background()
        self.draw_obstacles()
        self.draw_landmarks()
        self.draw_position_line()
        self.draw_robot()
        self.draw_cov(self.robot.prev_cov[:2, :2], [float(x) for x in self.robot.pred_prev_pos[-1][0]])
        self.draw_cov_past()

        # Update the window
        pygame.display.update()

        # Handle new input
        self.handle_input()

    def draw_background(self):
        self.surface.fill((255, 255, 255))

    # Draw the robot
    def draw_robot(self):
        pygame.draw.circle(self.surface, (100, 149, 237), self.pygame_coords(self.robot.position), self.robot.r)

        pygame.draw.line(self.surface, (0, 0, 0), self.pygame_coords(self.robot.position),
                         self.pygame_coords(self.robot.position + \
                                            np.array([self.robot.r * np.cos(self.robot.theta), \
                                                      self.robot.r * np.sin(self.robot.theta)])))

    def draw_obstacles(self):
        # For every obstacle
        for obstacle in self.obstacles:
            # Draw the obstacle
            if obstacle.sort == 'wall':
                pygame.draw.line(self.surface, (194, 197, 204), self.pygame_coords(obstacle.from_coords),
                                 self.pygame_coords(obstacle.to_coords), 4)

    # Draw all landmarks
    def draw_landmarks(self):
        for landmark in self.landmarks:
            pygame.draw.circle(self.surface, (0, 0, 0), self.pygame_coords(landmark.position), 4)
            if landmark.r < landmark.max_r:
                pygame.draw.line(self.surface, (80, 220, 100), self.pygame_coords(self.robot.position),
                                 self.pygame_coords(landmark.position), 2)

    # Draw the actual and predicted trajectories
    def draw_position_line(self):
        # Actual robot trajectory
        pygame.draw.aalines(self.surface, (0, 0, 0), False, [self.pygame_coords(pos) for pos in self.robot.real_prev_pos])

        # Predicted robot trajectory
        j, k = 0, 0
        for i in range(len(self.robot.pred_prev_pos)-1):
            pos = [int(x) for x in self.robot.pred_prev_pos[i][0]]
            next_pos = [int(x) for x in self.robot.pred_prev_pos[i+1][0]]
            sensor = self.robot.pred_prev_pos[i+1][1]

            if sensor:
                pygame.draw.aaline(self.surface, (80, 220, 100), self.pygame_coords(pos), self.pygame_coords(next_pos), 2)
            else:
                pygame.draw.aaline(self.surface, (255,131,0), self.pygame_coords(pos), self.pygame_coords(next_pos), 2)

    def draw_cov(self, cov, position, n_std = 2):
        # Calculate width and height
        variances = np.array([float(cov[0,0]), float(cov[1,1])])
        width, height = 2 * n_std * np.sqrt(variances)
        width = int(width)
        height = int(height)

        # Rotate with 45 degrees (theta)
        theta = np.pi / 2

        # Draw the actual ellipse
        # Make temporary surface for blit
        temp_sur = pygame.Surface((width, height))
        temp_sur.fill((1,1,1))
        temp_sur.set_colorkey((1,1,1), 1)
        temp_sur.set_alpha(125)

        # Draw and rotate ellipse
        pygame.draw.ellipse(temp_sur, (255, 131, 0), (0,0,width,height), 2)
        pygame.transform.rotate(temp_sur, np.rad2deg(theta))

        # Calculate right position on different surface
        l_u = list([position[0] - .5*width, position[1] + .5*height])

        # Add ellipse on right position
        self.surface.blit(temp_sur, self.pygame_coords(l_u))

    def draw_cov_past(self):
        for cov, position in self.robot.cov_lst:
            self.draw_cov(cov, position)

    # Convert normal coords to pygame coords (y-axes is mirrored)
    def pygame_coords(self, coords):
        return np.array([int(coords[0]), int(self.height - coords[1])])

    # Add text to image
    def add_centered_text(self, text, loc):
        text = self.sensor_font.render(text, False, (0, 0, 0))
        text_rect = text.get_rect(center=loc)
        self.surface.blit(text, text_rect)

    def handle_input(self):
        # Input
        keys = pygame.key.get_pressed()
        speed_modifier = self.robot.v_max / 30
        angle_modifier = 2*np.pi / 30

        # Speed input
        if keys[pygame.K_w]:
            if self.robot.v < self.robot.v_max:
                self.robot.v += speed_modifier
        if keys[pygame.K_s]:
            if self.robot.v > -self.robot.v_max:
                self.robot.v -= speed_modifier
        if keys[pygame.K_a]:
            self.robot.omega += angle_modifier
        if keys[pygame.K_d]:
            self.robot.omega -= angle_modifier
        if keys[pygame.K_x]:
            self.robot.v = 0
        if keys[pygame.K_c]:
            self.robot.omega = 0

