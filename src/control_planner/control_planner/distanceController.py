#!/usr/bin/env python3
# coding: utf-8
import numpy as np

class Distance_Controller:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0
        self.limit = 1.5

    def update(self, error, dt=0.01):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        output = self.saturate(output)
        return output

    def saturate(self, u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)

        return u