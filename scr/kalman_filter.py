from ulab import numpy as np
import random
import time
from math import sin, cos
from micropython import const

class kalman_filter:

    def __init__(self):
        # Initialize state vector with position, heading
        self.x = np.array([0, 0, 0])  # [x, y, heading]

        # Process noise covariance matrix, reduce to 3x3 for x, y, heading
        self.Q = np.diag([4, 4, 0.05])  # Adjust variances as needed for x, y, heading

        # Initial covariance matrix, reduce to 3x3 for x, y, heading
        self.P = np.eye(3) * 1  # Initial covariance matrix

    def state_transition(self, dt, wheel_speed, heading, ax, ay, pitch):
        vx = wheel_speed * np.cos(heading)
        vy = wheel_speed * np.sin(heading)
        self.x[0] += vx * dt * cos(pitch)
        self.x[1] += vy * dt * cos(pitch)
        return self.x

    def update_R(self, dt_gps, error):
        var = (error**12)+np.exp(dt_gps*10)
        return np.diag([var, var])

    def predict(self, dt, wheel_speed, heading, ax, ay, pitch):
        F = np.array([[1, 0, 0],
                      [0, 1, 0],
                      [0, 0, 1]])
        x_pred = self.state_transition(dt, wheel_speed, heading, ax, ay, pitch)
        self.Q = np.diag([0.5, 0.5, 0.05])  # Adjust variances as needed
        P_pred = np.dot(np.dot(F, self.P), F.T) + self.Q
        return x_pred, P_pred

    def update(self, x_pred, P_pred, gps_data, dt_gps, error):
        updated_R = self.update_R(dt_gps, error)
        H = np.array([[1, 0, 0],
                      [0, 1, 0]])
        innovation = gps_data - ((np.dot(H, x_pred)) + 0.05)
        S = np.dot(np.dot(H, P_pred), H.T) + updated_R
        K = np.dot(np.dot(P_pred, H.T), np.linalg.inv(S))
        self.x = x_pred + np.dot(K, innovation)
        self.P = np.dot((np.eye(3) - np.dot(K, H)), P_pred)

kf = kalman_filter()
