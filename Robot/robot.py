"""
Autonomous Robotic Systems Assignment 5
Author: Elisha A. Nieuwburg, Jelle Jansen
Date: 23-03-2021

Robot.py

Robot class that defines a differential drive robot
with a beam sensor that can locate landmarks.

"""

import sys
import numpy as np

from sympy import solve, Poly, Eq, Function, exp
from sympy.abc import x, y, z, a, b
from scipy.optimize import least_squares
from KF.kalmanfilter import KalmanFilter


class Robot(object):

    def __init__(self, position, theta, v_max, r, obstacles, landmarks, v, room_size):
        # Initialise parameters
        self.position = position
        self.v = v
        self.v_max = v_max
        self.room_size = room_size
        self.omega = 0
        self.theta = theta
        self.r = r
        self.obstacles = obstacles
        self.landmarks = landmarks

        # Parameters needed for Kalman Filter
        self.kf = KalmanFilter(self)
        self.real_prev_pos = [position, position]
        self.pred_prev_pos = [[position, 1], [position, 1]]
        self.prev_cov = np.zeros([3,3])
        self.cov_lst = []
        self.last_cov = 0

    # Move the robot according to a given trajectory
    def move_trajectory(self, delta_t, omega):
        self.omega = omega
        pose = np.array([[self.position[0]],[self.position[1]],[self.theta]])
        m = np.array([[delta_t * np.cos(self.theta), 0],
                      [delta_t * np.sin(self.theta), 0],
                      [0, delta_t]])
        param = np.array([[self.v],
                          [self.omega]])

        new_pose = pose + np.matmul(m, param)

        self.position = [float(new_pose[0]), float(new_pose[1])]
        self.theta = float(new_pose[2])

        # Update landmarks
        self.update_landmarks()

        # Compute sensor estimated position
        sensor_z = self.find_new_pos()

        # KF
        pred_pose, cov = self.kf.predict(self, sensor_z, delta_t)

        # Keep track of positions
        if sensor_z is None:
            self.pred_prev_pos.append([[pred_pose[0], pred_pose[1]], 0])
        else:
            self.pred_prev_pos.append([[pred_pose[0], pred_pose[1]], 1])
        self.real_prev_pos.append(self.position)

        # Keep track of cov
        self.prev_cov = cov
        self.last_cov += 1
        if self.last_cov == 30:
            self.cov_lst.append([cov, [pred_pose[0], pred_pose[1]]])
            self.last_cov = 0

    # Move the robot according to velocity and omega input
    def move(self, delta_t):
        pose = np.array([[self.position[0]],[self.position[1]],[self.theta]])
        m = np.array([[delta_t * np.cos(self.theta), 0],
                      [delta_t * np.sin(self.theta), 0],
                      [0, delta_t]])
        param = np.array([[self.v],
                          [self.omega]])

        new_pose = pose + np.matmul(m, param)

        self.position = [float(new_pose[0]), float(new_pose[1])]
        self.theta = float(new_pose[2])

        # Update landmarks
        self.update_landmarks()

        # Compute sensor estimated position
        sensor_z = self.find_new_pos()

        # Find predicted pose and covariance using Kalman Filter
        pred_pose, cov = self.kf.predict(self, sensor_z, delta_t)

        # Keep track of positions
        if sensor_z is None:
            self.pred_prev_pos.append([[pred_pose[0], pred_pose[1]], 0])
        else:
            self.pred_prev_pos.append([[pred_pose[0], pred_pose[1]], 1])
        self.real_prev_pos.append(self.position)

        # Keep track of covariance
        self.prev_cov = cov
        self.last_cov += 1
        if self.last_cov == 30:
            self.cov_lst.append([cov, [pred_pose[0], pred_pose[1]]])
            self.last_cov = 0

    # Update landmark distances
    def update_landmarks(self):
        for i, landmark in enumerate(self.landmarks, start = 0):
            r = np.linalg.norm(np.array(landmark.position) - np.array(self.position))
            if r < landmark.max_r:
                self.landmarks[i].r = r
            else:
                self.landmarks[i].r = landmark.max_r

    # Find new position according to sensor
    def find_new_pos(self):

        # Compute sensor position if and only if more than 2 landmarks are in range
        if sum([landmark.r < landmark.max_r for landmark in self.landmarks]) > 2:
            points = []
            i = 0

            # Limit of landmarks to compute intersection is 3
            while len(points) < 3:
                if self.landmarks[i].r < self.landmarks[i].max_r:
                    points.append(self.landmarks[i])
                i += 1

            # Compute intersection of three landmark circles
            return self.calculate_intersection(points[0].position, points[0].r,
                                          points[1].position, points[1].r,
                                          points[2].position, points[2].r)

    # Compute intersection of three landmark circles
    def calculate_intersection(self, pos1, r1, pos2, r2, pos3, r3):
        def equation(g):
            x, y = g

            return (
                (x - pos1[0])**2 + (y - pos1[1])**2 - r1**2,
                (x - pos2[0])**2 + (y - pos2[1])**2 - r2**2,
                (x - pos3[0])**2 + (y - pos3[1])**2 - r3**2)

        guess = [float(x) for x in self.pred_prev_pos[-1][0]]

        ans = least_squares(equation, guess, ftol=None, xtol=None)

        return ans.x


