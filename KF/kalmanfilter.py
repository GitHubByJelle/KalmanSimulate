"""
Autonomous Robotic Systems Assignment 5
Author: Sacha L. Sindorf
Date: 23-03-2021

Kalmanfilter.py

KalmanFilter class that defines a Kalman filter.

"""

import numpy as np


class KalmanFilter():
    def __init__(self, robot):

        # Measurement noise
        self.std_sensor_x = 2.0
        self.std_sensor_y = 2.0
        self.std_sensor_theta = 0.2

        # Noise of motion model
        std_Rx = 2
        std_Ry = 2
        std_Rtheta = 0.2

        # Noise of sensor model
        std_Qx = 2
        std_Qy = 2
        std_Qtheta = 0.2

        # Initial values of Sigma
        std_x = 0.2
        std_y = 0.2
        std_theta = 0.1

        # Covariance matrices
        self.R = (np.square(np.array([std_Rx, std_Ry, std_Rtheta])).reshape(1, 3) *
                  np.identity(3))
        self.Q = (np.square(np.array([std_Qx, std_Qy, std_Qtheta])).reshape(1, 3) *
                 np.identity(3))

        # Initialize with known pose
        self.mu_prev = np.array([robot.position[0], robot.position[1], robot.theta]).reshape(3, 1) + \
                       np.array([[np.random.normal(0, self.std_sensor_x)],
                                 [np.random.normal(0, self.std_sensor_y)],
                                 [np.random.normal(0, self.std_sensor_theta)]])

        # Initialize with small estimated errors
        self.Sigma_prev = (np.square(np.array([std_x, std_y, std_theta])).reshape(1, 3) *
                           np.identity(3))

    # Predict new position of the robot using a Kalman Filter
    def predict(self, robot, sensor_z, delta_t):
        # Prediction
        u = np.array([robot.v, robot.omega]).reshape(2, 1)
        B = delta_t*np.array([[np.cos(float(self.mu_prev[2])), 0.],
                                   [np.sin(float(self.mu_prev[2])), 0.],
                                   [0., 1.]])

        mu_bar = self.mu_prev + np.dot(B, u)

        Sigma_bar = self.Sigma_prev + self.R

        # If there is no sensor input, do not do Correction part
        if sensor_z is not None:

            # Add noise to sensor input
            z = np.array([[sensor_z[0]], [sensor_z[1]], [robot.theta]]) + \
                np.array([[np.random.normal(0, self.std_sensor_x)],
                         [np.random.normal(0, self.std_sensor_y)],
                         [np.random.normal(0, self.std_sensor_theta)]])
            K = np.dot(Sigma_bar, np.linalg.inv(Sigma_bar + self.Q))

            mu = mu_bar + np.dot(K, (z - mu_bar))

            Sigma = np.dot((np.identity(3) - K), Sigma_bar)

            # Store state
            self.mu_prev = mu
            self.Sigma_prev = Sigma

            return mu, Sigma

        # Store state
        self.mu_prev = mu_bar
        self.Sigma_prev = Sigma_bar

        return mu_bar, Sigma_bar
